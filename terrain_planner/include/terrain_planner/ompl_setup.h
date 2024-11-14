#ifndef TERRAIN_PLANNER_OMPL_SETUP_H_
#define TERRAIN_PLANNER_OMPL_SETUP_H_


#include <stdexcept>


#include "terrain_planner/DubinsAirplane.hpp"
#include "terrain_planner/terrain_ompl.h"

#include <terrain_navigation/terrain_map.h>
#include <grid_map_core/GridMap.hpp>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include "ompl/base/SpaceInformation.h"

enum PlannerType { RRTSTAR, INFORMED_RRTSTAR, RRTCONNECT, BITSTAR, FMTSTAR };



namespace ompl {
  
class LowAltitudeFlightObjective : public ompl::base::StateCostIntegralObjective
{
public:
    LowAltitudeFlightObjective(const ompl::base::SpaceInformationPtr& si, const grid_map::GridMap& map) : 
    ompl::base::StateCostIntegralObjective(si, true), map_(map) {}
 
    ompl::base::Cost stateCost(const ompl::base::State* s) const
    {   
        double x = s->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getX();
        double y = s->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getY();
        double z = s->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getZ();
        double terrain_elevation = 0;
        // I don't understand why there are points out of the map. Shouldn't the sampler take care of this?
        if (map_.isInside(Eigen::Vector2d(x, y))) {
          terrain_elevation = map_.atPosition("elevation", Eigen::Vector2d(x, y));
        }
        // std::cout << "Terrain Elevation: " << terrain_elevation << std::endl;
        // std::cout << "Relative Elevation: " << z - terrain_elevation << std::endl;
        return ompl::base::Cost(abs(z - terrain_elevation));
        // return ompl::base::Cost(0.0);
    }

private:
  const grid_map::GridMap& map_;
};


class OmplSetup : public geometric::SimpleSetup {
 public:
  OmplSetup(const base::StateSpacePtr& space_ptr) : geometric::SimpleSetup(space_ptr) {}

  ompl::base::OptimizationObjectivePtr getLowAltitudeFlightObjective(const ompl::base::SpaceInformationPtr& si)
  {
    if (map_ != nullptr) {
      return std::make_shared<LowAltitudeFlightObjective>(si, map_->getGridMap());
    }
    else {
      throw std::invalid_argument( "Need to set map before set objective." );
    }
  }

  void setDefaultObjective() {
    auto lengthObj(std::make_shared<ompl::base::PathLengthOptimizationObjective>(getSpaceInformation()));
    auto noeObj = getLowAltitudeFlightObjective(getSpaceInformation());
    auto opt = std::make_shared<ompl::base::MultiOptimizationObjective>(getSpaceInformation());
    opt->addObjective(lengthObj, 1000.0);
    opt->addObjective(noeObj, 1.0);
    setOptimizationObjective(ob::OptimizationObjectivePtr(opt));
  }

  void setDefaultPlanner(PlannerType planner_type = PlannerType::RRTSTAR) {
    switch (planner_type) {
      case PlannerType::RRTSTAR: {
        auto planner = std::make_shared<ompl::geometric::RRTstar>(getSpaceInformation());
        planner->setRange(600.0);
        planner->setGoalBias(planner->getGoalBias());
        setPlanner(planner);
        break;
      }
      case PlannerType::RRTCONNECT: {
        auto planner = std::make_shared<ompl::geometric::RRTConnect>(getSpaceInformation());
        planner->setRange(600.0);
        // planner->setGoalBias(goal_bias);
        setPlanner(planner);
        break;
      }
      case PlannerType::INFORMED_RRTSTAR: {
        auto planner = std::make_shared<ompl::geometric::InformedRRTstar>(getSpaceInformation());
        // planner->setRange(600.0);
        // planner->setGoalBias(goal_bias);
        setPlanner(planner);
        break;
      }
      case PlannerType::BITSTAR: {
        auto planner = std::make_shared<ompl::geometric::BITstar>(getSpaceInformation());
        setPlanner(planner);
        break;
      }
      case PlannerType::FMTSTAR: {
        auto planner = std::make_shared<ompl::geometric::FMT>(getSpaceInformation());
        setPlanner(planner);
        break;
      }
    }
  }

  const base::StateSpacePtr& getGeometricComponentStateSpace() const { return getStateSpace(); }

  void setStateValidityCheckingResolution(double resolution) {
    // This is a protected attribute, so need to wrap this function.
    si_->setStateValidityCheckingResolution(resolution);
  }

  void setTerrainCollisionChecking(const grid_map::GridMap& map, bool check_max_altitude) {
    std::shared_ptr<TerrainValidityChecker> validity_checker(
        new TerrainValidityChecker(getSpaceInformation(), map, check_max_altitude));

    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
  }

  void setMap(std::shared_ptr<TerrainMap> map) { map_ = map; }

  private:
    std::shared_ptr<TerrainMap> map_;
};

}  // namespace ompl

#endif
