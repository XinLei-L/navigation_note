/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <rotate_recovery/rotate_recovery.h>
#include <pluginlib/class_list_macros.hpp>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>


// 将恢复动作行为注册为插件，可以被navigation动态加载
PLUGINLIB_EXPORT_CLASS(rotate_recovery::RotateRecovery, nav_core::RecoveryBehavior)

namespace rotate_recovery
{
RotateRecovery::RotateRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL)
{
}

void RotateRecovery::initialize(std::string name, tf2_ros::Buffer*,
                                costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = local_costmap;

    ros::NodeHandle private_nh("~/" + name);  // 插件专属命名空间参数
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");  // DWA / TrajectoryPlanner 的命名空间，用于加载运动学参数

    // 从参数服务器加载仿真角度分辨率和执行频率(私有句柄加载)
    private_nh.param("sim_granularity", sim_granularity_, 0.017); 
    private_nh.param("frequency", frequency_, 20.0);

    // 加载旋转相关参数
    acc_lim_th_ = nav_core::loadParameterWithDeprecation(blp_nh, "acc_lim_theta", "acc_lim_th", 3.2);
    max_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_vel_theta", "max_rotational_vel", 1.0);
    min_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "min_in_place_vel_theta", "min_in_place_rotational_vel", 0.4);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

RotateRecovery::~RotateRecovery()
{
  delete world_model_;
}

void RotateRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if (local_costmap_ == NULL)
  {
    ROS_ERROR("The costmap passed to the RotateRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Rotate recovery behavior started.");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  geometry_msgs::PoseStamped global_pose;
  local_costmap_->getRobotPose(global_pose);  // 获取机器人当前位姿

  double current_angle = tf2::getYaw(global_pose.pose.orientation); 
  double start_angle = current_angle;  // 起点角度

  bool got_180 = false;  // 是否已经旋转过180°

  while (n.ok() &&
         (!got_180 ||  // 没有旋转180°
          std::fabs(angles::shortest_angular_distance(current_angle, start_angle)) > tolerance_))
  {
    // 更新机器人朝向角yaw
    local_costmap_->getRobotPose(global_pose);
    current_angle = tf2::getYaw(global_pose.pose.orientation); // 当前yaw角

    // 计算剩余旋转角度
    double dist_left;
    if (!got_180)  // 转到180°
    {
      // 先强制让机器人至少旋转半圈(起点朝向 + 180°)
      // distance_to_180表示还差多少角度到180°
      double distance_to_180 = std::fabs(angles::shortest_angular_distance(current_angle, start_angle + M_PI));
      // 再继续旋转直到回到起点
      dist_left = M_PI + distance_to_180;

      if (distance_to_180 < tolerance_)
      {
        got_180 = true;
      }
    }
    else  // 回到起点角度
    {
      // 此时已经旋转半圈，计算当前角度与起点角度的差值
      dist_left = std::fabs(angles::shortest_angular_distance(current_angle, start_angle));
    }

    double x = global_pose.pose.position.x, y = global_pose.pose.position.y;

    double sim_angle = 0.0;
    while (sim_angle < dist_left)
    {
      double theta = current_angle + sim_angle;

      // 每隔sim_granularity_ 检查一次 footprint 是否会碰撞
      double footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
      if (footprint_cost < 0.0)
      {
        ROS_ERROR("Rotate recovery can't rotate in place because there is a potential collision. Cost: %.2f",
                  footprint_cost);
        return;  // 发生碰撞，直接退出
      }

      sim_angle += sim_granularity_;
    }

    // 计算旋转速度，确保能在剩余角度内停下
    double vel = sqrt(2 * acc_lim_th_ * dist_left);

    // 限制在最小、最大旋转速度范围内
    vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = vel;

    vel_pub.publish(cmd_vel);

    r.sleep();
  }
}
};  // namespace rotate_recovery
