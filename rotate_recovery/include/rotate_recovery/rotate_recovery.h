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
#ifndef ROTATE_RECOVERY_ROTATE_RECOVERY_H
#define ROTATE_RECOVERY_ROTATE_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <base_local_planner/costmap_model.h>
#include <string>

/**
 * 旋转恢复整体思路：
 * 1. 让机器人原地旋转一整圈
 * 2. 这样它的传感器(激光雷达、深度相机等)可以扫描周围环境，刷新局部costmap
 * 3. 如果之前costmap上残留了错误的障碍物(比如动态障碍物已经走开)，这个旋转动作会帮助清理
 */

namespace rotate_recovery
{
/**
 * @class RotateRecovery
 * @brief 让机器人原地旋转，尝试清理出可行空间的恢复行为
 */
class RotateRecovery : public nav_core::RecoveryBehavior
{
public:
  /**
   * @brief  构造函数，不会完成实际配置，需要后面 initialize()
   */
  RotateRecovery();

  /**
   * @brief  旋转恢复行为初始化函数
   * @param name 命名空间，用于加载参数
   * @param tf tf变化(unused)
   * @param global_costmap 全局costmap(unused)
   * @param local_costmap 局部地图costmap，用来检测碰撞和获取机器人位姿
   */
  void initialize(std::string name, tf2_ros::Buffer*,
                  costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);

  /**
   * @brief  执行恢复动作：机器人原地旋转
   */
  void runBehavior();

  /**
   * @brief  Destructor for the rotate recovery behavior
   */
  ~RotateRecovery();

private:
  costmap_2d::Costmap2DROS* local_costmap_; // 局部代价地图指针
  bool initialized_;  // 标志位，防止多次初始化
  double sim_granularity_; // 仿真角度分辨率，默认值为0.017
  double min_rotational_vel_; // 最小旋转速度，默认值为0.4
  double max_rotational_vel_;  // 最大旋转速度，默认值为1.0
  double acc_lim_th_;  // 角加速度限制，默认值为3.2
  double tolerance_;  // 角度容忍误差，默认值为0.10
  double frequency_;  // 执行频率，默认值为20.0
  base_local_planner::CostmapModel* world_model_;  //代价地图模型，用于检测footprint是否会发生碰撞
};
};  // namespace rotate_recovery
#endif  // ROTATE_RECOVERY_ROTATE_RECOVERY_H
