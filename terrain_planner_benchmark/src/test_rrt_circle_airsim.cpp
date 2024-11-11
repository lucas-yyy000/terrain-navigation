/****************************************************************************
 *
 *   Copyright (c) 2021 Jaeyoung Lim, Autonomous Systems Lab,
 *  ETH Zürich. All rights reserved.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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
 * @brief ROS Node to test ompl
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */
#include <cstdlib> 
#include <random>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <any>

#include <terrain_navigation/terrain_map.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "terrain_navigation/data_logger.h"
#include "terrain_navigation_ros/visualization.h"
#include "terrain_planner/common.h"
#include "terrain_planner/terrain_ompl_rrt.h"

void publishCircleSetpoints(const ros::Publisher& pub, const Eigen::Vector3d& position, const double radius) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = "map";
  marker.id = 0;
  marker.header.stamp = ros::Time::now();
  std::vector<geometry_msgs::Point> points;
  for (double t = 0.0; t <= 1.0; t += 0.02) {
    geometry_msgs::Point point;
    point.x = position.x() + radius * std::cos(t * 2 * M_PI);
    point.y = position.y() + radius * std::sin(t * 2 * M_PI);
    point.z = position.z();
    points.push_back(point);
  }
  geometry_msgs::Point start_point;
  start_point.x = position.x() + radius * std::cos(0.0);
  start_point.y = position.y() + radius * std::sin(0.0);
  start_point.z = position.z();
  points.push_back(start_point);

  marker.points = points;
  marker.scale.x = 5.0;
  marker.scale.y = 5.0;
  marker.scale.z = 5.0;
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  pub.publish(marker);
}

void getDubinsShortestPath(std::shared_ptr<fw_planning::spaces::DubinsAirplaneStateSpace>& dubins_ss,
                           const Eigen::Vector3d start_pos, const double start_yaw, const Eigen::Vector3d goal_pos,
                           const double goal_yaw, std::vector<Eigen::Vector3d>& path) {
  ompl::base::State* from = dubins_ss->allocState();
  from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setX(start_pos.x());
  from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setY(start_pos.y());
  from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setZ(start_pos.z());
  from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setYaw(start_yaw);

  ompl::base::State* to = dubins_ss->allocState();
  to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setX(goal_pos.x());
  to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setY(goal_pos.y());
  to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setZ(goal_pos.z());
  to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setYaw(goal_yaw);

  ompl::base::State* state = dubins_ss->allocState();
  double dt = 0.02;
  for (double t = 0.0; t <= 1.0 + dt; t += dt) {
    dubins_ss->interpolate(from, to, t, state);
    auto interpolated_state =
        Eigen::Vector3d(state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getX(),
                        state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getY(),
                        state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getZ());
    path.push_back(interpolated_state);
  }
}

bool validatePosition(std::shared_ptr<TerrainMap> map, const Eigen::Vector3d goal, Eigen::Vector3d& valid_goal) {
  double upper_surface = map->getGridMap().atPosition("ics_+", goal.head(2));
  double lower_surface = map->getGridMap().atPosition("ics_-", goal.head(2));
  const bool is_goal_valid = (upper_surface < lower_surface) ? true : false;
  valid_goal(0) = goal(0);
  valid_goal(1) = goal(1);
  valid_goal(2) = (upper_surface + lower_surface) / 2.0;
  return is_goal_valid;
}

double mod2pi(double x) { return x - 2 * M_PI * floor(x * (0.5 / M_PI)); }

Eigen::Vector4d rpy2quaternion(double roll, double pitch, double yaw) {
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  Eigen::Vector4d q;
  q(0) = cr * cp * cy + sr * sp * sy;
  q(1) = sr * cp * cy - cr * sp * sy;
  q(2) = cr * sp * cy + sr * cp * sy;
  q(3) = cr * cp * sy - sr * sp * cy;

  q.normalize();

  return q;
}

PathSegment generateArcTrajectory(Eigen::Vector3d rate, const double horizon, Eigen::Vector3d current_pos,
                                  Eigen::Vector3d current_vel, const double dt = 0.1) {
  PathSegment trajectory;
  trajectory.states.clear();

  double cruise_speed_{20.0};

  double time = 0.0;
  const double current_yaw = std::atan2(-1.0 * current_vel(1), current_vel(0));
  const double climb_rate = rate(1);
  trajectory.flightpath_angle = std::asin(climb_rate / cruise_speed_);
  /// TODO: Fix sign conventions for curvature
  trajectory.curvature = -rate(2) / cruise_speed_;
  trajectory.dt = dt;
  for (int i = 0; i < std::max(1.0, horizon / dt); i++) {
    if (std::abs(rate(2)) < 0.0001) {
      rate(2) > 0.0 ? rate(2) = 0.0001 : rate(2) = -0.0001;
    }
    double yaw = rate(2) * time + current_yaw;

    Eigen::Vector3d pos =
        cruise_speed_ / rate(2) *
            Eigen::Vector3d(std::sin(yaw) - std::sin(current_yaw), std::cos(yaw) - std::cos(current_yaw), 0) +
        Eigen::Vector3d(0, 0, climb_rate * time) + current_pos;
    Eigen::Vector3d vel = Eigen::Vector3d(cruise_speed_ * std::cos(yaw), -cruise_speed_ * std::sin(yaw), -climb_rate);
    const double roll = std::atan(rate(2) * cruise_speed_ / 9.81);
    const double pitch = std::atan(climb_rate / cruise_speed_);
    Eigen::Vector4d att = rpy2quaternion(roll, -pitch, -yaw);  // TODO: why the hell do you need to reverse signs?

    State state_vector;
    state_vector.position = pos;
    state_vector.velocity = vel;
    state_vector.attitude = att;
    trajectory.states.push_back(state_vector);

    time = time + dt;
  }
  return trajectory;
}


PathSegment getLoiterPath(Eigen::Vector3d end_position, Eigen::Vector3d end_velocity, Eigen::Vector3d center_pos) {
  Eigen::Vector3d radial_vector = (end_position - center_pos);
  radial_vector(2) = 0.0;  // Only consider horizontal loiters
  Eigen::Vector3d emergency_rates =
      20.0 * end_velocity.normalized().cross(radial_vector.normalized()) / radial_vector.norm();
  double horizon = 2 * M_PI / std::abs(emergency_rates(2));
  // Append a loiter at the end of the planned path
  PathSegment loiter_trajectory = generateArcTrajectory(emergency_rates, horizon, end_position, end_velocity);
  return loiter_trajectory;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "airsim_rrt_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  // Initialize ROS related publishers for visualization
  auto start_pos_pub = nh.advertise<visualization_msgs::Marker>("start_position", 1, true);
  auto goal_pos_pub = nh.advertise<visualization_msgs::Marker>("goal_position", 1, true);
  auto path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
  auto grid_map_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  auto trajectory_pub = nh.advertise<visualization_msgs::MarkerArray>("tree", 1, true);

  grid_map_msgs::GridMap gridMapMsg;
  boost::shared_ptr<grid_map_msgs::GridMap const> sharedPtr;
  sharedPtr  = ros::topic::waitForMessage<grid_map_msgs::GridMap>("/grid_map", ros::Duration(10));
  if (sharedPtr == NULL) {
      ROS_INFO("No map message received");
  }
  else {
      gridMapMsg = *sharedPtr;
  }

  grid_map::GridMap gridMap;
  grid_map::GridMapRosConverter::fromMessage(gridMapMsg, gridMap);

  std::string output_directory;
  nh_private.param<std::string>("output_directory", output_directory, "");

  double min_height;
  nh_private.param<double>("min_height", min_height, 0.0);
  double max_height;
  nh_private.param<double>("max_height", max_height, 120.0);

  double time_budget;
  nh_private.param<double>("time_budget", time_budget, 30.0);

  // Load terrain map using grid map.
  auto terrain_map = std::make_shared<TerrainMap>();
  terrain_map->setGridMap(gridMap);
  std::cout << "Set Grid Map..." << std::endl;

  terrain_map->AddLayerDistanceTransform(min_height, "distance_surface");
  terrain_map->AddLayerDistanceTransform(max_height, "max_elevation");
  double radius;
  nh_private.param<double>("loiter_radius", radius, 1.0);
  terrain_map->AddLayerHorizontalDistanceTransform(radius, "ics_+", "distance_surface");
  terrain_map->AddLayerHorizontalDistanceTransform(-radius, "ics_-", "max_elevation");

  // Path path;

  // Initialize planner with loaded terrain map
  auto planner = std::make_shared<TerrainOmplRrt>();
  std::cout << "Setting Terrain Map..." << std::endl;
  planner->setMap(terrain_map);
  std::cout << "Terrain Map Set." << std::endl;
  /// TODO: Get bounds from gridmap
  planner->setBoundsFromMap(terrain_map->getGridMap());

  std::cout << "Set up problem with min and max elevations: " << min_height << " " << max_height << std::endl;
  std::cout << "RRT time budget: " << time_budget << std::endl;
  std::cout << "Loiter radius: " << radius << std::endl;

  const double map_width_x = terrain_map->getGridMap().getLength().x();
  std::cout << "Map width x: " << map_width_x << std::endl;
  const double map_width_y = terrain_map->getGridMap().getLength().y();
  std::cout << "Map width y: " << map_width_y << std::endl;
  const Eigen::Vector2d map_pos = terrain_map->getGridMap().getPosition();
  std::cout << "Grid Map position: " << map_pos << std::endl;

  srand (static_cast <unsigned> (time(0)));
  std::random_device rd;
  std::mt19937 gen(rd());
  int iter_num;
  nh_private.param<int>("starting_index", iter_num, 0);
  int mode_repeats;
  nh_private.param<int>("mode_repeats", mode_repeats, 1);
  int max_mode_repeats;
  nh_private.param<int>("max_mode_repeats", max_mode_repeats, 1);
  int max_iter_num;
  nh_private.param<int>("collect_data_num", max_iter_num, 100);
  // Collect training data.
  while (iter_num < max_iter_num) {
    std::cout << "Number of Collected Demonstrations: " << iter_num << std::endl;
    // // Initialize data logger for recording
    // auto data_logger_states = std::make_shared<DataLogger>();
    // data_logger_states->setKeys({"x", "y", "z", "qw", "qx", "qy", "qz", "min_dist", "max_dist"});
    // auto data_logger_attitude = std::make_shared<DataLogger>();
    // data_logger_attitude->setKeys({"w", "x", "y", "z"});

    float start_x_ratio = 0.4*static_cast <float> (rand()) / static_cast <float> (RAND_MAX) + 0.1;
    float start_y_ratio = 0.4*static_cast <float> (rand()) / static_cast <float> (RAND_MAX) + 0.1;
    float end_x_ratio = 0.4*static_cast <float> (rand()) / static_cast <float> (RAND_MAX) + 0.1;
    float end_y_ratio = 0.4*static_cast <float> (rand()) / static_cast <float> (RAND_MAX) + 0.1;

    float start_x_sgn;
    float start_y_sgn;
    float end_x_sgn;
    float end_y_sgn;

    std::uniform_int_distribution<int> distribution(0, 1);
    
    start_x_sgn = distribution(gen) ? 1.0:-1.0;
    start_y_sgn = distribution(gen) ? 1.0:-1.0;
    end_x_sgn = distribution(gen) ? 1.0:-1.0;
    end_y_sgn = distribution(gen) ? 1.0:-1.0;

    // Note that the z coordiante does not matter. validatePosition will set the z coordiante to be in the middle of the two layers: max_elevation, distance_surface.
    Eigen::Vector3d start{Eigen::Vector3d(map_pos(0)  + start_x_sgn * start_x_ratio * map_width_x, map_pos(1) + start_y_sgn * start_y_ratio * map_width_y, 0.0)};
    Eigen::Vector3d goal{Eigen::Vector3d(map_pos(0) + end_x_sgn * end_x_ratio * map_width_x, map_pos(1) + end_y_sgn * end_y_ratio * map_width_y, 0.0)};
    std::cout << "Sampled start position: " << start << std::endl;
    std::cout << "Sampled goal position: " << goal << std::endl;
    if ((goal - start).norm() < 300.0) {
      std::cout << "Start and goal too close" << std::endl;
      continue;
    }
    Eigen::Vector3d updated_start;

    if (validatePosition(terrain_map, start, updated_start)) {
      start = updated_start;
      std::cout << "Specified start position is valid" << std::endl;
    } else {
      // throw std::runtime_error("Specified start position is NOT valid");
      std::cout << "Specified start position is NOT valid" << std::endl;
      continue;
    }
    
    
    Eigen::Vector3d updated_goal;
    if (validatePosition(terrain_map, goal, updated_goal)) {
      goal = updated_goal;
      std::cout << "Specified goal position is valid" << std::endl;
    } else {
      // throw std::runtime_error("Specified goal position is NOT valid");
      std::cout << "Specified goal position is NOT valid" << std::endl;
      continue;
    }
    int num_trials = 0;
    for (int i = 0; i < mode_repeats; i++) {
      // if (iter_num > 0) {
      //   path.resetSegments();
      // }

      // Initialize data logger for recording
      auto data_logger_states = std::make_shared<DataLogger>();
      data_logger_states->setKeys({"x", "y", "z", "qw", "qx", "qy", "qz", "min_dist", "max_dist"});
      Path path;

      num_trials++;
      if (num_trials > max_mode_repeats) {
        continue;
      }
      planner->setupProblem(start, goal, radius);
      if (planner->Solve(time_budget, path)) {
        std::cout << "[TestRRTCircleGoal] Found Solution!" << std::endl;
      } else {
        std::cout << "[TestRRTCircleGoal] Unable to find solution" << std::endl;
        --i;
        continue;
      }

      Eigen::Vector3d start_position = path.firstSegment().states.front().position;
      Eigen::Vector3d start_velocity = path.firstSegment().states.front().velocity;

      PathSegment start_loiter_path = getLoiterPath(start_position, start_velocity, start);
      path.prependSegment(start_loiter_path);

      Eigen::Vector3d end_position = path.lastSegment().states.back().position;
      Eigen::Vector3d end_velocity = path.lastSegment().states.back().velocity;
      PathSegment goal_loiter_path = getLoiterPath(end_position, end_velocity, goal);

      path.appendSegment(goal_loiter_path);

      // Repeatedly publish results
      terrain_map->getGridMap().setTimestamp(ros::Time::now().toNSec());
      grid_map_msgs::GridMap message;
      grid_map::GridMapRosConverter::toMessage(terrain_map->getGridMap(), message);
      grid_map_pub.publish(message);
      publishTrajectory(path_pub, path.position());

      /// TODO: Publish a circle instead of a goal marker!
      publishCircleSetpoints(start_pos_pub, start, radius);
      publishCircleSetpoints(goal_pos_pub, goal, radius);
      publishTree(trajectory_pub, planner->getPlannerData(), planner->getProblemSetup());
      /// TODO: Save planned path into a csv file for plotting
      for (std::size_t i = 1; i < path.segments.size() - 1; ++i) {
        for (auto segment_state: path.segments[i].states) {
          Eigen::Vector3d segment_pos = segment_state.position;
          Eigen::Vector4d segment_att = segment_state.attitude;
          std::unordered_map<std::string, std::any> state;
          state.insert(std::pair<std::string, double>("x", segment_pos(0)));
          state.insert(std::pair<std::string, double>("y", segment_pos(1)));
          state.insert(std::pair<std::string, double>("z", segment_pos(2)));
          state.insert(std::pair<std::string, double>("qw", segment_att(0)));
          state.insert(std::pair<std::string, double>("qx", segment_att(1)));
          state.insert(std::pair<std::string, double>("qy", segment_att(2)));
          state.insert(std::pair<std::string, double>("qz", segment_att(3)));
          state.insert(std::pair<std::string, double>("min_dist", min_height));
          state.insert(std::pair<std::string, double>("max_dist", max_height));
          data_logger_states->record(state);
        }
      }

      data_logger_states->setPrintHeader(true);
      std::string states_output_file_path = output_directory + "/" + std::to_string(iter_num) + "_states" + ".csv";
      data_logger_states->writeToFile(states_output_file_path);
      
      iter_num++;

      ros::Duration(0.01).sleep();
    }

  }

  ros::spin();
  return 0;
}
