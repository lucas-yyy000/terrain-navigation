#ifndef TERRAIN_PLANNER_OMPL_H
#define TERRAIN_PLANNER_OMPL_H

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/goals/GoalStates.h>

#include "terrain_planner/DubinsAirplane.hpp"

#include <grid_map_core/GridMap.hpp>

namespace ompl {

class TerrainValidityChecker : public base::StateValidityChecker {
 public:
  TerrainValidityChecker(const base::SpaceInformationPtr& space_info, const grid_map::GridMap& map,
                         bool check_max_altitude)
      : base::StateValidityChecker(space_info), map_(map), check_collision_max_altitude_(check_max_altitude) {}

  /**
   * @brief State validity check
   *
   * @param state
   * @return true
   * @return false
   */
  virtual bool isValid(const base::State* state) const override;

  /**
   * @brief Check collisions on gridmap
   *
   * @param state
   * @return true
   * @return false
   */
  bool checkCollision(const Eigen::Vector3d state) const;

  /**
   * @brief Check if the state is in collision with a layer
   *
   * @param layer name of the layer in the elevation map
   * @param position position of the state to evaluate
   * @param is_above
   * @return true
   * @return false
   */
  bool isInCollision(const std::string& layer, const Eigen::Vector3d& position, bool is_above) const;

 protected:
  const grid_map::GridMap& map_;
  bool check_collision_max_altitude_{true};
};

class TerrainStateSampler : public base::StateSampler {
 public:
  TerrainStateSampler(const ompl::base::StateSpace* si, const grid_map::GridMap& map, const double max_altitude = 120.0,
                      const double min_altitude = 50.0)
      : StateSampler(si), map_(map), max_altitude_(max_altitude), min_altitude_(min_altitude), iteration_number_(1.0), init_state_(nullptr), goal_state_(nullptr) {
    // name_ = "my sampler";
    std::cout << "Custom sampler initialized with min altitude: " << min_altitude << " max altitude: " << max_altitude << std::endl;
  }


  // const ompl::base::State* getClosestStateToGoal()
  // {
  //   if (!goal_state_ || !state_space_ || planner_data_ || planner_data_->numVertices() == 0)
  //   {
  //     return nullptr; // Handle invalid inputs
  //   }

  //   const ompl::base::State* closestState = nullptr;
  //   double minDistance = std::numeric_limits<double>::infinity();

  //   // Iterate through all states in PlannerData
  //   for (unsigned int i = 0; i < planner_data_->numVertices(); ++i)
  //   {
  //     const ompl::base::State* currentState = planner_data_->getVertex(i).getState();
  //     if (currentState)
  //     {
  //       double distance = state_space_->distance(currentState, goal_state_);
  //       if (distance < minDistance)
  //       {
  //         minDistance = distance;
  //         closestState = currentState;
  //       }
  //     }
  //   }

  //   return closestState;
  // }

  void sampleUniform(ompl::base::State* state) override {
    /// TODO: We don't need to querry this everytime we sample
    // std::cout << "Custom sampler used." << std::endl;
    // const Eigen::Vector2d map_pos = map_.getPosition();
    // const double map_width_x = map_.getLength().x();
    // const double map_width_y = map_.getLength().y();
    // const double map_diag_length = sqrt(map_width_x*map_width_x + map_width_y*map_width_y);
    // double p = rng_.uniformReal(-1.0, 1.0);
    // double yaw = rng_.uniformReal(-M_PI, M_PI);
    // if (p < 1.0) {
    //   // const ompl::base::State* closestState = getClosestStateToGoal();
    //   // Eigen::Vector2d init_loc(init_state_->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getX(), init_state_->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getY());
    //   double init_x = init_state_->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getX();
    //   double init_y = init_state_->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getY();
    //   Eigen::Vector2d goal_direction(goal_state_->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getX() - init_x, 
    //                                 goal_state_->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getY() - init_y);
    //   goal_direction.normalize();
    //   Eigen::Vector2d perpendicular_direction(-goal_direction(1), goal_direction(0));
    //   double search_range = map_diag_length*exp(-1.0/iteration_number_);
    //   double search_range_diag = rng_.uniformReal(0.0, search_range);
    //   double search_range_cross = rng_.uniformReal(0.0, search_range);
      
    //   double x = init_x + search_range_diag*goal_direction(0);
    //   double y = init_y + search_range_diag*goal_direction(1);
    //   double terrain_elevation{0.0};

    //   if (map_.isInside(Eigen::Vector2d(x, y))) {
    //     terrain_elevation = map_.atPosition("elevation", Eigen::Vector2d(x, y));
    //   }
    //   double z = rng_.uniformReal(min_altitude_, max_altitude_) + terrain_elevation;

    //   state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setX(x);
    //   state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setY(y);
    //   state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setZ(z);
    //   state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setYaw(yaw);
    //   std::cout << "Sampled position: " << x << y << z << std::endl;

    // } else {
    //   double x = rng_.uniformReal(map_pos(0) - 0.45 * map_width_x, map_pos(0) + 0.45 * map_width_x);
    //   double y = rng_.uniformReal(map_pos(1) - 0.45 * map_width_y, map_pos(1) + 0.45 * map_width_y);
    //   while (! map_.isInside(Eigen::Vector2d(x, y))) {
    //     x = rng_.uniformReal(map_pos(0) - 0.45 * map_width_x, map_pos(0) + 0.45 * map_width_x);
    //     y = rng_.uniformReal(map_pos(1) - 0.45 * map_width_y, map_pos(1) + 0.45 * map_width_y);
    //   }

    //   /// TODO: Workaround when sampled position is not inside the map
    //   double terrain_elevation{0.0};
    //   if (map_.isInside(Eigen::Vector2d(x, y))) {
    //     terrain_elevation = map_.atPosition("elevation", Eigen::Vector2d(x, y));
    //   }
    //   double z = rng_.uniformReal(min_altitude_, max_altitude_) + terrain_elevation;

    //   state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setX(x);
    //   state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setY(y);
    //   state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setZ(z);
    //   state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setYaw(yaw);
    // }

    // assert(si_->isValid(state));
    return;
  }
  void sampleUniformNear(ompl::base::State* /*state*/, const ompl::base::State* /*near*/,
                         const double /*distance*/) override {
    // std::cout << "Sample Near" << std::endl;
    return;
  }
  void sampleGaussian(ompl::base::State* state, const ompl::base::State* mean, double stdDev) override {
    // std::cout << "Sample Gaussian" << std::endl;
    return;
  }

  // void setIterationNumber(int iteration_number) {
  //   iteration_number_ = iteration_number;
  // }


  // void setInitialState(ompl::base::State* init_state) {
  //   init_state_ = init_state;
  // }

  // void setGoalState(ompl::base::State* goal_state) {
  //   goal_state_ = goal_state;
  // }

  // void setPlannerData(ompl::base::PlannerData* plannerData) {
  //   planner_data_ = plannerData;
  // }

  // void setStateSpace(ompl::base::StateSpace* state_space) {
  //   state_space_ = state_space;
  // }

 protected:
  ompl::RNG rng_;
  const grid_map::GridMap& map_;
  double max_altitude_{120.0};
  double min_altitude_{50.0};

  private:
    int iteration_number_;
    ompl::base::GoalPtr goal_state_;
    ompl::base::State* init_state_;
    ompl::base::PlannerData* planner_data_;
    ompl::base::StateSpace* state_space_;
};
}  // namespace ompl

#endif
