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
#ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_H_
#define DWA_LOCAL_PLANNER_DWA_PLANNER_H_

#include <vector>
#include <Eigen/Core>


#include <dwa_local_planner/DWAPlannerConfig.h>

//for creating a local cost grid
#include <base_local_planner/map_grid_visualizer.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

#include <nav_msgs/Path.h>

namespace dwa_local_planner {
  /**
   * @class DWAPlanner
   * @brief A class implementing a local planner using the Dynamic Window Approach
   */
  class DWAPlanner {
    public:
      /**
       * @brief  Constructor for the planner
       * @param name The name of the planner 
       * @param costmap_ros A pointer to the costmap instance the planner should use
       * @param global_frame the frame id of the tf frame to use
       */
      DWAPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util);

      /**
       * @brief Reconfigures the trajectory planner
       */
      void reconfigure(DWAPlannerConfig &cfg);

      /**
       * @brief  Check if a trajectory is legal for a position/velocity pair
       * @param pos The robot's position
       * @param vel The robot's velocity
       * @param vel_samples The desired velocity
       * @return True if the trajectory is valid, false otherwise
       */
      bool checkTrajectory(
          const Eigen::Vector3f pos,
          const Eigen::Vector3f vel,
          const Eigen::Vector3f vel_samples);

      /**
       * @brief Given the current position and velocity of the robot, find the best trajectory to exectue
       * @param global_pose The current position of the robot 
       * @param global_vel The current velocity of the robot 
       * @param drive_velocities The velocities to send to the robot base
       * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
       */
      base_local_planner::Trajectory findBestPath(
          const geometry_msgs::PoseStamped& global_pose,
          const geometry_msgs::PoseStamped& global_vel,
          geometry_msgs::PoseStamped& drive_velocities);

      /**
       * @brief  Update the cost functions before planning
       * @param  global_pose The robot's current pose
       * @param  new_plan The new global plan
       * @param  footprint_spec The robot's footprint
       *
       * The obstacle cost function gets the footprint.
       * The path and goal cost functions get the global_plan
       * The alignment cost functions get a version of the global plan
       *   that is modified based on the global_pose 
       */
      void updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose,
          const std::vector<geometry_msgs::PoseStamped>& new_plan,
          const std::vector<geometry_msgs::Point>& footprint_spec);

      /**
       * @brief Get the period at which the local planner is expected to run
       * @return The simulation period
       */
      // 获取仿真周期
      double getSimPeriod() { return sim_period_; }

      /**
       * @brief Compute the components and total cost for a map grid cell
       * @param cx The x coordinate of the cell in the map grid
       * @param cy The y coordinate of the cell in the map grid
       * @param path_cost Will be set to the path distance component of the cost function
       * @param goal_cost Will be set to the goal distance component of the cost function
       * @param occ_cost Will be set to the costmap value of the cell
       * @param total_cost Will be set to the value of the overall cost function, taking into account the scaling parameters
       * @return True if the cell is traversible and therefore a legal location for the robot to move to
       */
      bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);

      /**
       * 设置新的全局路径，并重置状态
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    private:

      base_local_planner::LocalPlannerUtil *planner_util_;// 局部规划器工具

      double stop_time_buffer_; // 步进时间缓冲区: 当机器人接近目标时，允许其在完全停止前继续前进的时间
      double path_distance_bias_;  // 路径距离权重: 鼓励轨迹靠近全局路径的代价权重
      double goal_distance_bias_;  // 目标距离权重: 鼓励轨迹朝向目标的代价权重
      double occdist_scale_;  // 障碍物距离权重: 惩罚靠近障碍物的轨迹的代价权重
      Eigen::Vector3f vsamples_;  // 速度采样: x,y,theta三个方向的速度采样数量

      double sim_period_;// dwa前向模拟时间
      base_local_planner::Trajectory result_traj_; // 最佳轨迹

      double forward_point_distance_; // 前向点距离: 用于调整路径末端的目标点，让机器人“瞄准”目标点前方一小段距离，而不是死死追着目标点

      std::vector<geometry_msgs::PoseStamped> global_plan_;  // 当前全局路径

      boost::mutex configuration_mutex_; // 配置互斥锁
      std::string frame_id_; // 全局坐标系
      ros::Publisher traj_cloud_pub_;  // 轨迹点云发布器
      bool publish_cost_grid_pc_; // 是否创建和发布点云代价地图，pc表示 point cloud
      bool publish_traj_pc_;  // 是否发布轨迹点云：dwa多种轨迹采样的集合

      double cheat_factor_;  // 作弊因子: 用于调整机器人接近目标时的行为，防止其过度追求路径对齐而忽略目标位置

      base_local_planner::MapGridVisualizer map_viz_; // 根据代价函数生成势场地图的可视化工具

      // see constructor body for explanations
      base_local_planner::SimpleTrajectoryGenerator generator_; // 轨迹生成器
      base_local_planner::OscillationCostFunction oscillation_costs_; // 震荡代价函数
      base_local_planner::ObstacleCostFunction obstacle_costs_; // 障碍物代价函数
      base_local_planner::MapGridCostFunction path_costs_; // 路径代价函数
      base_local_planner::MapGridCostFunction goal_costs_; // 目标代价函数
      base_local_planner::MapGridCostFunction goal_front_costs_; // 目标前方代价函数
      base_local_planner::MapGridCostFunction alignment_costs_; // 对齐代价函数
      base_local_planner::TwirlingCostFunction twirling_costs_; // 旋转代价函数

      base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_; // 采样打分规划器
  };
};
#endif
