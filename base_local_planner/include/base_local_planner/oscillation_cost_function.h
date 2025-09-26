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
 * Author: TKruse
 *********************************************************************/

#ifndef OSCILLATION_COST_FUNCTION_H_
#define OSCILLATION_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>
#include <Eigen/Core>

namespace base_local_planner {

class OscillationCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
  OscillationCostFunction();
  virtual ~OscillationCostFunction();

  double scoreTrajectory(Trajectory &traj);

  bool prepare() {return true;};

  /**
   * @brief  重置震荡标志。将所有标志位都设置为false
   */
  void resetOscillationFlags();

  /**
   * @brief 根据当前位置和轨迹，更新震荡标志
   * @param min_vel_trans 最小平移速度阈值
  */
  void updateOscillationFlags(Eigen::Vector3f pos, base_local_planner::Trajectory* traj, double min_vel_trans);
  
  /**
   * @brief 设置震荡重置的距离和角度阈值
   * @param dist 最小移动距离
   * @param angle 最小旋转角度
  */
  void setOscillationResetDist(double dist, double angle);

private:
  /**
   * @brief 检查是否满足条件来重置震荡标志：
   *  当前位置与上次震荡标记位置的距离大于震荡最小移动距离 或者 当前角度与上次震荡角度差大于震荡最小旋转角，那么重置
   * @param pos 当前位置
   * @param prev 之前记录的位置
  */
  void resetOscillationFlagsIfPossible(const Eigen::Vector3f& pos, const Eigen::Vector3f& prev);

  /**
   * @brief  根据选定轨迹来设置震荡标志
   * @param  t 选定轨迹
   * @param min_vel_trans 最小平移速度(针对的是x方向)
   * @return 如果设置了新的限制标志返回true，否则false
   */
  bool setOscillationFlags(base_local_planner::Trajectory* t, double min_vel_trans);

  // 侧向移动（横向运动）相关标志
  bool strafe_pos_only_;  // true: 只能向正方向侧移（之前检测到负向侧移振荡）
  bool strafe_neg_only_;  // true: 只能向负方向侧移（之前检测到正向侧移振荡）
  bool strafing_pos_;     // true: 当前正在向正方向侧移
  bool strafing_neg_;     // true: 当前正在向负方向侧移

  // 旋转运动相关标志
  bool rot_pos_only_;     // true: 只能顺时针旋转（之前检测到逆时针振荡）
  bool rot_neg_only_;     // true: 只能逆时针旋转（之前检测到顺时针振荡）
  bool rotating_pos_;     // true: 当前正在顺时针旋转
  bool rotating_neg_;     // true: 当前正在逆时针旋转

  // 前后移动相关标志
  bool forward_pos_only_; // true: 只能向前移动（之前检测到向后移动振荡）
  bool forward_neg_only_; // true: 只能向后移动（之前检测到向前移动振荡）
  bool forward_pos_;      // true: 当前正在向前移动
  bool forward_neg_;      // true: 当前正在向后移动

  double oscillation_reset_dist_;   // 重置振荡标志所需的最小移动距离（米）
  double oscillation_reset_angle_;  // 重置振荡标志所需的最小旋转角度（弧度）

  Eigen::Vector3f prev_stationary_pos_; // 上次设置振荡标志时的机器人位置 [x, y, theta]
};

} /* namespace base_local_planner */
#endif /* OSCILLATION_COST_FUNCTION_H_ */
