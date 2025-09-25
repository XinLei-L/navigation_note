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

#include <base_local_planner/obstacle_cost_function.h>
#include <cmath>
#include <Eigen/Core>
#include <ros/console.h>

namespace base_local_planner {

ObstacleCostFunction::ObstacleCostFunction(costmap_2d::Costmap2D* costmap) 
    : costmap_(costmap), sum_scores_(false) {
  if (costmap != NULL) {
    // 创建一个基于costmap的世界模型
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
  }
}

ObstacleCostFunction::~ObstacleCostFunction() {
  if (world_model_ != NULL) {
    delete world_model_;
  }
}

// 设置参数：最大线速度、最大缩放因子、缩放速度阈值
void ObstacleCostFunction::setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed) {
  max_trans_vel_ = max_trans_vel;
  max_scaling_factor_ = max_scaling_factor;
  scaling_speed_ = scaling_speed;  // 超过缩放速度阈值，开始缩放
}

// 设置机器人足迹：即设置机器人轮廓点
void ObstacleCostFunction::setFootprint(std::vector<geometry_msgs::Point> footprint_spec) {
  footprint_spec_ = footprint_spec;
}

bool ObstacleCostFunction::prepare() {
  return true;
}

double ObstacleCostFunction::scoreTrajectory(Trajectory &traj) {
  double cost = 0;
  double scale = getScalingFactor(traj, scaling_speed_, max_trans_vel_, max_scaling_factor_);
  double px, py, pth;
  if (footprint_spec_.size() == 0) {
    // Bug, should never happen
    ROS_ERROR("Footprint spec is empty, maybe missing call to setFootprint?");
    return -9;
  }
  // 根据车辆轮廓以及路径点，计算每个路径点的代价
  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
    traj.getPoint(i, px, py, pth);
    double f_cost = footprintCost(px, py, pth,
        scale, footprint_spec_,
        costmap_, world_model_);

    if(f_cost < 0){
        return f_cost;
    }

    if(sum_scores_)
        cost +=  f_cost;
    else
        cost = std::max(cost, f_cost);
  }
  return cost;
}

// 根据轨迹的线速度计算缩放因子, 从而让机器人在高速时要么减速要么离墙壁更远
// 低速时: 使用正常碰撞检测范围，允许机器人靠近障碍物
// 高速时: 扩大碰撞检测范围，强制机器人要么减速，要么选择离障碍物更远的路径
double ObstacleCostFunction::getScalingFactor(Trajectory &traj, double scaling_speed, double max_trans_vel, double max_scaling_factor) {
  // 计算线速度大小
  double vmag = hypot(traj.xv_, traj.yv_);

  double scale = 1.0;
  // 速度大于缩放速度阈值，进行缩放
  if (vmag > scaling_speed) {
    // 线性插值计算缩放因子
    double ratio = (vmag - scaling_speed) / (max_trans_vel - scaling_speed);
    scale = max_scaling_factor * ratio + 1.0;
  }
  return scale;
}

double ObstacleCostFunction::footprintCost (
    const double& x,
    const double& y,
    const double& th,
    double scale,
    std::vector<geometry_msgs::Point> footprint_spec,
    costmap_2d::Costmap2D* costmap,
    base_local_planner::WorldModel* world_model) {

  std::vector<geometry_msgs::Point> scaled_footprint;
  // 根据缩放因子，调整机器人的轮廓大小
  for(unsigned int i  = 0; i < footprint_spec.size(); ++i) {
    geometry_msgs::Point new_pt;
    new_pt.x = scale * footprint_spec[i].x;
    new_pt.y = scale * footprint_spec[i].y;
    scaled_footprint.push_back(new_pt);
  }

  //check if the footprint is legal
  // TODO: Cache inscribed radius
  // 使用世界模型检测车辆轮廓是否与障碍物发生碰撞
  double footprint_cost = world_model->footprintCost(x, y, th, scaled_footprint);

  if (footprint_cost < 0) { // 表示发生碰撞
    return -6.0; 
  }
  unsigned int cell_x, cell_y;

  // 判断位置 (x, y) 在costmap中的网格坐标 (cell_x, cell_y)。返回false表示 (x, y) 在costmap外
  if ( ! costmap->worldToMap(x, y, cell_x, cell_y)) {
    return -7.0;
  }
  // 计算代价占据代价：取足迹代价、栅格值、0的最大值
  double occ_cost = std::max(std::max(0.0, footprint_cost), double(costmap->getCost(cell_x, cell_y)));

  return occ_cost;
}

} /* namespace base_local_planner */
