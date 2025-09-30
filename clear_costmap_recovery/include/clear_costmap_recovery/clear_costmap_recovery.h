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
#ifndef CLEAR_COSTMAP_RECOVERY_H_
#define CLEAR_COSTMAP_RECOVERY_H_
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>

namespace clear_costmap_recovery{
  /**
   * @class ClearCostmapRecovery
   * @brief A recovery behavior that reverts the navigation stack's costmaps to the static map outside of a user-specified region.
   */
  class ClearCostmapRecovery : public nav_core::RecoveryBehavior {
    public:
      /**
       * @brief  对象构造后还需要调用 initialize() 来完成初始化
       */
      ClearCostmapRecovery();

      /**
       * @brief  初始化函数
       * @param tf tf变换指针
       * @param global_costmap 全局代价地图指针
       * @param local_costmap 局部代价地图指针
       */
      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

      /**
       * @brief  执行地图恢复行为：主要是清除代价地图，包括全局和局部. Reverts the
       * costmap to the static map outside of a user-specified window and
       * clears unknown space around the robot.
       */
      void runBehavior();

    private:
      // 对某个costmap调用层级清除
      void clear(costmap_2d::Costmap2DROS* costmap);
      // 真正执行在一定区域内清除
      void clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap, double pose_x, double pose_y);
      costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_; // 全局地图、局部地图指针
      std::string name_;  // 恢复行为名字
      tf2_ros::Buffer* tf_; // TF 变换缓存
      bool initialized_;  // 是否完成初始化
      bool force_updating_; // 清理后是否强制立即更新，默认值为false
      double reset_distance_; // 清理的区域边长（以机器人为中心的正方形），默认值为3.0
      bool invert_area_to_clear_; // 清除区域（清理内区域还是外区域），默认值为false，清理内区域
      std::string affected_maps_; // 哪个代价地图需要清理 ("local"、"global"、"both")，默认值为both
      std::set<std::string> clearable_layers_; // 可被清理的层，默认只清除obstacle层
  };
};
#endif
