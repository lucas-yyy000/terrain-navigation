/****************************************************************************
 *
 *   Copyright (c) 2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name terrain-navigation nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Motion primtiive based Terrain planner library
 *
 * Motion primitive based terrain planner library
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef TERRAIN_PLANNER_H
#define TERRAIN_PLANNER_H

#include "terrain_planner/common.h"
#include "terrain_planner/maneuver_library.h"
#include "terrain_planner/profiler.h"

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>

#include <Eigen/Dense>

class TerrainPlanner {
 public:
  TerrainPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  virtual ~TerrainPlanner();

 private:
  void cmdloopCallback(const ros::TimerEvent &event);
  void statusloopCallback(const ros::TimerEvent &event);
  void publishTrajectory(std::vector<Eigen::Vector3d> trajectory);
  void mavposeCallback(const geometry_msgs::PoseStamped &msg);
  void mavtwistCallback(const geometry_msgs::TwistStamped &msg);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher vehicle_path_pub_;
  ros::Subscriber mavpose_sub_;
  ros::Subscriber mavtwist_sub_;
  ros::Timer cmdloop_timer_, statusloop_timer_;

  std::shared_ptr<ManeuverLibrary> maneuver_library_;
  std::shared_ptr<Profiler> planner_profiler_;

  std::vector<Eigen::Vector3d> vehicle_position_history_;
  Eigen::Vector3d vehicle_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vehicle_velocity_{Eigen::Vector3d::Zero()};
};

#endif
