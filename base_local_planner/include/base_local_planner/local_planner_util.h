/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef ABSTRACT_LOCAL_PLANNER_ODOM_H_
#define ABSTRACT_LOCAL_PLANNER_ODOM_H_

#include <nav_core/base_local_planner.h>

#include <boost/thread.hpp>

#include <costmap_2d/costmap_2d.h>
#include <tf2_ros/buffer.h>

#include <base_local_planner/local_planner_limits.h>


namespace base_local_planner {

/**
 * @class LocalPlannerUtil
 * @brief Helper class implementing infrastructure code many local planner implementations may need.
 */
class LocalPlannerUtil {

private:
  // things we get from move_base
  std::string name_;
  std::string global_frame_;  // 全局坐标系

  costmap_2d::Costmap2D* costmap_;  // 局部代价地图
  tf2_ros::Buffer* tf_; // 用于坐标变换

  std::vector<geometry_msgs::PoseStamped> global_plan_;  // 当前全局路径


  boost::mutex limits_configuration_mutex_;  // 约束配置互斥锁
  bool setup_;  // 是否设置过局部规划器约束
  LocalPlannerLimits default_limits_; // 默认局部规划器约束
  LocalPlannerLimits limits_;  // 当前局部规划器约束
  bool initialized_; // 是否初始化

public:

  /**
   * @brief  Callback to update the local planner's parameters
   */
  void reconfigureCB(LocalPlannerLimits &config, bool restore_defaults);

  LocalPlannerUtil() : initialized_(false) {}

  ~LocalPlannerUtil() {
  }
  // 初始化局部规划器工具
  void initialize(tf2_ros::Buffer* tf,
      costmap_2d::Costmap2D* costmap,
      std::string global_frame);
  // 获取全局路径中的最后一个点作为目标点
  bool getGoal(geometry_msgs::PoseStamped& goal_pose);
  // 设置新的全局路径
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
  // 获取局部路径：将全局路径转换到机器人坐标系，并裁剪一部分作为局部路径
  bool getLocalPlan(const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& transformed_plan);
  // 获取局部代价地图
  costmap_2d::Costmap2D* getCostmap();
  // 获取当前局部规划器约束
  LocalPlannerLimits getCurrentLimits();
  // 获取全局坐标系
  std::string getGlobalFrame(){ return global_frame_; }
};

};

#endif /* ABSTRACT_LOCAL_PLANNER_ODOM_H_ */
