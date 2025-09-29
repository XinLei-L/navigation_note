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

#ifndef SIMPLE_TRAJECTORY_GENERATOR_H_
#define SIMPLE_TRAJECTORY_GENERATOR_H_

#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/local_planner_limits.h>
#include <Eigen/Core>

namespace base_local_planner {

/**
 * generates trajectories based on equi-distant discretisation of the degrees of freedom.
 * This is supposed to be a simple and robust implementation of the TrajectorySampleGenerator
 * interface, more efficient implementations are thinkable.
 *
 * This can be used for both dwa and trajectory rollout approaches.
 * As an example, assuming these values:
 * sim_time = 1s, sim_period=200ms, dt = 200ms,
 * vsamples_x=5,
 * acc_limit_x = 1m/s^2, vel_x=0 (robot at rest, values just for easy calculations)
 * dwa_planner will sample max-x-velocities from 0m/s to 0.2m/s.
 * trajectory rollout approach will sample max-x-velocities 0m/s up to 1m/s
 * trajectory rollout approach does so respecting the acceleration limit, so it gradually increases velocity
 */
class SimpleTrajectoryGenerator: public base_local_planner::TrajectorySampleGenerator {
public:

  SimpleTrajectoryGenerator() {
    limits_ = NULL;
  }

  ~SimpleTrajectoryGenerator() {}

  /**
   * @param pos 当前位置
   * @param vel 当前速度
   * @param goal 目标点
   * @param limits 包含最大/最小速度、加速度等约束
   * @param vsamples 每个维度采样的数量
   * @param additional_samples 额外的静态速度样本
   * @param discretize_by_time 若为 true，按照时间等分轨迹点，否则按距离/角度等分
   */
  void initialise(
      const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel,
      const Eigen::Vector3f& goal,
      base_local_planner::LocalPlannerLimits* limits,
      const Eigen::Vector3f& vsamples,
      std::vector<Eigen::Vector3f> additional_samples,
      bool discretize_by_time = false);

  /**
   * @brief 生成一组速度采样
   * @param pos 当前位置
   * @param vel 当前速度
   * @param limits 包含最大/最小速度、加速度等约束
   * @param vsamples 每个维度采样的数量
   * @param discretize_by_time 若为 true，按照时间等分轨迹点，否则按距离/角度等分
   */
  void initialise(
      const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel,
      const Eigen::Vector3f& goal,
      base_local_planner::LocalPlannerLimits* limits,
      const Eigen::Vector3f& vsamples,
      bool discretize_by_time = false);

  /**
   * This function is to be called only when parameters change
   * @brief 设置仿真/采样相关参数
   * @param sim_time 仿真总时长(每条轨迹模拟的总时间)
   * @param sim_granularity 平移方向仿真步长(用于决定轨迹离散点数)---或是碰撞检测点之间的最大允许距离
   * @param angular_sim_granularity 角度方向仿真步长
   * @param use_dwa 是否使用dwa策略
   * @param sim_period 只在dwa下使用，表示dwa的时间步长(用于限制一步之内能到达的速度和距离)
   */
  void setParameters(double sim_time,
      double sim_granularity,
      double angular_sim_granularity,
      bool use_dwa = false,
      double sim_period = 0.0);

  /**
   * @brief 是否还有更多的轨迹
   */
  bool hasMoreTrajectories();

  /**
   * @brief 获取下一个轨迹
   */
  bool nextTrajectory(Trajectory &traj);

  /**
   * @brief 根据当前位置、速度和时间步长，计算下一时刻的位姿
   */
  static Eigen::Vector3f computeNewPositions(const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel, double dt);
  
  /**
   * @brief 在加速度上限和dt时间内，使当前速度向采样目标速度收敛，返回新的速度
   * @param sample_target_vel 采样目标速度
   * @param vel 当前速度
   * @param acclimits 加速度上限
   * @param dt 时间间隔
   */
  static Eigen::Vector3f computeNewVelocities(const Eigen::Vector3f& sample_target_vel,
      const Eigen::Vector3f& vel, Eigen::Vector3f acclimits, double dt);

  /**
   * @brief 根据位置、速度和采样目标速度，生成轨迹
   * @param pos 位置
   * @param vel 速度
   * @param sample_target_vel 采样目标速度
   * @param traj 生成的轨迹
   */
  bool generateTrajectory(
        Eigen::Vector3f pos,
        Eigen::Vector3f vel,
        Eigen::Vector3f sample_target_vel,
        base_local_planner::Trajectory& traj);

protected:

  unsigned int next_sample_index_; // 下一个样本索引
  std::vector<Eigen::Vector3f> sample_params_;  // 保存速度采样点，每个对包含x_v、y_v、w_v
  base_local_planner::LocalPlannerLimits* limits_; // 线速度、角速度、加速度约束
  Eigen::Vector3f pos_; // 初始位置
  Eigen::Vector3f vel_; // 初始速度

  bool continued_acceleration_;  // 若为 true，轨迹中速度会随时间按照加速度限制变化（连续加速度模型）；若为 false，每步速度相等（DWA 风格）
  bool discretize_by_time_; // 若为 true，轨迹点数由 sim_time / sim_granularity_ 决定；否则按距离/角度粒度决定步数。

  double sim_time_;  // 仿真总时间
  double sim_granularity_; // 平移方向仿真步长
  double angular_sim_granularity_; // 角度仿真步长
  bool use_dwa_;  // 是否使用dwa
  double sim_period_; // 仿真周期(执行一次控制循环的时间间隔)
};

} /* namespace base_local_planner */
#endif /* SIMPLE_TRAJECTORY_GENERATOR_H_ */
