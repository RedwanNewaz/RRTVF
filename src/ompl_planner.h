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


// Our collision checker. For this demo, our robot's state space
// lies in [0,1]x[0,1], with a circular obstacle of radius 0.25
// centered at (0.5,0.5). Any states lying in this circular region are
// considered "in collision".
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si, ObstclesPtr obstacles) :
            ob::StateValidityChecker(si), obstacles_(std::move(obstacles)) {}

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const
    {
        const auto* state2D =
                state->as<ob::RealVectorStateSpace::StateType>();
        auto x = (float) state2D->values[0];
        auto y = (float) state2D->values[1];
        return obstacles_->isValidState(x, y);
    }

    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State* state) const
    {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ob::RealVectorStateSpace::StateType* state2D =
                state->as<ob::RealVectorStateSpace::StateType>();

        // Extract the robot's (x,y) position from its state
        double x = state2D->values[0];
        double y = state2D->values[1];

        // Distance formula between two points, offset by the circle's
        // radius
        return sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5)) - 0.25;
    }



private:
    ObstclesPtr obstacles_;
};

// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration space of computed
// paths.
inline ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}



class ompl_planner {
    using PATH = pair<vector<float>, vector<float>>;
public:
    ompl_planner(const vector<float> &stat, const vector<float> &goal);
    void setup(ObstclesPtr obstacles);
    PATH get_solution(DatasetPtr dataset, int depth);
//    static Eigen::VectorXd vectorField(const ompl::base::State *qnear);

private:
    vector<float> start_, goal_;
    ob::ProblemDefinitionPtr pdef;
    ob::SpaceInformationPtr si;


};


#endif //VFRRT_OMPL_PLANNER_H
