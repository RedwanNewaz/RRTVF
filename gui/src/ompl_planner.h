//
// Created by redwan on 4/3/22.
//

#ifndef VFRRT_OMPL_PLANNER_H
#define VFRRT_OMPL_PLANNER_H

#include <utility>
#include <vector>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>

#include <ompl/control/SimpleSetup.h>
#include <boost/math/constants/constants.hpp>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include "ompl/geometric/planners/rrt/VFRRT.h"
#include <Eigen/Dense>

#include "polygonal_obstacles.h"
#include "dataset_parser.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

using namespace std;


class ompl_planner {
    using PATH = pair<vector<float>, vector<float>>;
public:
    ompl_planner(const vector<float> &stat, const vector<float> &goal);
    void setup(ObstclesPtr obstacles);
    PATH get_solution(DatasetPtr dataset, int depth);


private:
    vector<float> start_, goal_;
    ob::ProblemDefinitionPtr pdef_;
    ob::SpaceInformationPtr si_;

protected:
    // Returns a structure representing the optimization objective to use
    // for optimal motion planning. This method returns an objective which
    // attempts to minimize the length in configuration space of computed
    // paths.
    ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);
};



// Our collision checker. For this demo, our robot's state space
// lies in [0,1]x[0,1], with a circular obstacle of radius 0.25
// centered at (0.5,0.5). Any states lying in this circular region are
// considered "in collision".
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si, ObstclesPtr obstacles);

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const override;
    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State* state) const override;

private:
    ObstclesPtr obstacles_;
};



#endif //VFRRT_OMPL_PLANNER_H
