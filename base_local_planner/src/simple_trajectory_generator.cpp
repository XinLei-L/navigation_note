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

#include <base_local_planner/simple_trajectory_generator.h>

#include <cmath>

#include <base_local_planner/velocity_iterator.h>

namespace base_local_planner {

void SimpleTrajectoryGenerator::initialise(
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel,
    const Eigen::Vector3f& goal,
    base_local_planner::LocalPlannerLimits* limits,
    const Eigen::Vector3f& vsamples,
    std::vector<Eigen::Vector3f> additional_samples,
    bool discretize_by_time) {
  initialise(pos, vel, goal, limits, vsamples, discretize_by_time);
  // add static samples if any
  sample_params_.insert(sample_params_.end(), additional_samples.begin(), additional_samples.end());
}

// 对速度点进行采样
void SimpleTrajectoryGenerator::initialise(
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel,
    const Eigen::Vector3f& goal,
    base_local_planner::LocalPlannerLimits* limits,
    const Eigen::Vector3f& vsamples,
    bool discretize_by_time) {
  double max_vel_th = limits->max_vel_theta; 
  double min_vel_th = -1.0 * max_vel_th;
  discretize_by_time_ = discretize_by_time;
  Eigen::Vector3f acc_lim = limits->getAccLimits();
  pos_ = pos;
  vel_ = vel;
  limits_ = limits;
  next_sample_index_ = 0;
  sample_params_.clear(); // 用于存储速度采样点

  double min_vel_x = limits->min_vel_x;
  double max_vel_x = limits->max_vel_x;
  double min_vel_y = limits->min_vel_y;
  double max_vel_y = limits->max_vel_y;

  if (vsamples[0] * vsamples[1] * vsamples[2] > 0) {
    //compute the feasible velocity space based on the rate at which we run
    Eigen::Vector3f max_vel = Eigen::Vector3f::Zero();
    Eigen::Vector3f min_vel = Eigen::Vector3f::Zero();
		// 设置最大线速度和角速度：注意两者使用的加速度时间不同
    if ( ! use_dwa_) {  // 不采用dwa形式
      double dist = hypot(goal[0] - pos[0], goal[1] - pos[1]);
      // 计算x、y方向最大线速度，避免在仿真时间内行走距离大于到目标点距离
      max_vel_x = std::max(std::min(max_vel_x, dist / sim_time_), min_vel_x);
      max_vel_y = std::max(std::min(max_vel_y, dist / sim_time_), min_vel_y);

      // if we use continous acceleration, we can sample the max velocity we can reach in sim_time_
      max_vel[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_time_); // 最大取小
      max_vel[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_time_);
      max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_time_);

      min_vel[0] = std::max(min_vel_x, vel[0] - acc_lim[0] * sim_time_); // 最小取大
      min_vel[1] = std::max(min_vel_y, vel[1] - acc_lim[1] * sim_time_);
      min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_time_);
    } else {
      // with dwa do not accelerate beyond the first step, we only sample within velocities we reach in sim_period
      max_vel[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_period_);
      max_vel[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_period_);
      max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_period_);

      min_vel[0] = std::max(min_vel_x, vel[0] - acc_lim[0] * sim_period_);
      min_vel[1] = std::max(min_vel_y, vel[1] - acc_lim[1] * sim_period_);
      min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_period_);
    }

    Eigen::Vector3f vel_samp = Eigen::Vector3f::Zero();
    VelocityIterator x_it(min_vel[0], max_vel[0], vsamples[0]);
    VelocityIterator y_it(min_vel[1], max_vel[1], vsamples[1]);
    VelocityIterator th_it(min_vel[2], max_vel[2], vsamples[2]);
    for(; !x_it.isFinished(); x_it++) {
      vel_samp[0] = x_it.getVelocity();
      for(; !y_it.isFinished(); y_it++) {
        vel_samp[1] = y_it.getVelocity();
        for(; !th_it.isFinished(); th_it++) {
          vel_samp[2] = th_it.getVelocity();
          sample_params_.push_back(vel_samp);
        }
        th_it.reset();
      }
      y_it.reset();
    }
  }
}

void SimpleTrajectoryGenerator::setParameters(
    double sim_time,
    double sim_granularity,
    double angular_sim_granularity,
    bool use_dwa,
    double sim_period) {
  sim_time_ = sim_time;
  sim_granularity_ = sim_granularity;
  angular_sim_granularity_ = angular_sim_granularity;
  use_dwa_ = use_dwa;
  continued_acceleration_ = ! use_dwa_;
  sim_period_ = sim_period;
}

bool SimpleTrajectoryGenerator::hasMoreTrajectories() {
  return next_sample_index_ < sample_params_.size();
}

/**
 * Create and return the next sample trajectory
 */
bool SimpleTrajectoryGenerator::nextTrajectory(Trajectory &comp_traj) {
  bool result = false;
  if (hasMoreTrajectories()) {
    if (generateTrajectory(
        pos_,
        vel_,
        sample_params_[next_sample_index_],
        comp_traj)) {
      result = true;
    }
  }
  next_sample_index_++;
  return result;
}

bool SimpleTrajectoryGenerator::generateTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f sample_target_vel,
      base_local_planner::Trajectory& traj) {
  double vmag = hypot(sample_target_vel[0], sample_target_vel[1]);  // 计算平移速度
  double eps = 1e-4;
  traj.cost_   = -1.0;
  traj.resetPoints();

  // 检查速度是否满足最小平移速度或最小角速度条件：不满足表示机器人静止或者速度太小，不生成轨迹
  if ((limits_->min_vel_trans >= 0 && vmag + eps < limits_->min_vel_trans) &&
      (limits_->min_vel_theta >= 0 && fabs(sample_target_vel[2]) + eps < limits_->min_vel_theta)) {
    return false;
  }
  // 大于最大平移速度，不生成轨迹
  if (limits_->max_vel_trans >=0 && vmag - eps > limits_->max_vel_trans) {
    return false;
  }

  int num_steps; // 仿真步数
  if (discretize_by_time_) {
    num_steps = ceil(sim_time_ / sim_granularity_);
  } else {
    //compute the number of steps we must take along this trajectory to be "safe"
    double sim_time_distance = vmag * sim_time_; // the distance the robot would travel in sim_time if it did not change velocity
    double sim_time_angle = fabs(sample_target_vel[2]) * sim_time_; // the angle the robot would rotate in sim_time
    num_steps =
        ceil(std::max(sim_time_distance / sim_granularity_,
            sim_time_angle    / angular_sim_granularity_));
  }

  if (num_steps == 0) {
    return false;
  }

  double dt = sim_time_ / num_steps; // 离散时间步长
  traj.time_delta_ = dt;

  Eigen::Vector3f loop_vel;
  if (continued_acceleration_) {  // 连续加速度模型，速度会改变
    loop_vel = computeNewVelocities(sample_target_vel, vel, limits_->getAccLimits(), dt);
    traj.xv_     = loop_vel[0];
    traj.yv_     = loop_vel[1];
    traj.thetav_ = loop_vel[2];
  } else { // dwa模式，使用采样目标速度作为轨迹速度
    loop_vel = sample_target_vel;
    traj.xv_     = sample_target_vel[0];
    traj.yv_     = sample_target_vel[1];
    traj.thetav_ = sample_target_vel[2];
  }

  for (int i = 0; i < num_steps; ++i) {
    traj.addPoint(pos[0], pos[1], pos[2]);
    if (continued_acceleration_) {
      // 更新速度：连续加速度模型，速度会改变
      loop_vel = computeNewVelocities(sample_target_vel, loop_vel, limits_->getAccLimits(), dt);
    }
    pos = computeNewPositions(pos, loop_vel, dt);
  }

  return true;
}

Eigen::Vector3f SimpleTrajectoryGenerator::computeNewPositions(const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel, double dt) {
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}

Eigen::Vector3f SimpleTrajectoryGenerator::computeNewVelocities(const Eigen::Vector3f& sample_target_vel,
    const Eigen::Vector3f& vel, Eigen::Vector3f acclimits, double dt) {
  Eigen::Vector3f new_vel = Eigen::Vector3f::Zero();
  for (int i = 0; i < 3; ++i) {
    if (vel[i] < sample_target_vel[i]) {  // 加速收敛
      new_vel[i] = std::min(double(sample_target_vel[i]), vel[i] + acclimits[i] * dt);
    } else {  // 减速收敛
      new_vel[i] = std::max(double(sample_target_vel[i]), vel[i] - acclimits[i] * dt);
    }
  }
  return new_vel;
}

} /* namespace base_local_planner */
