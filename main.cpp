#include <iostream>
#include "src/dataset_parser.h"
#include "matplotlibcpp.h"
#include "cassert"

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>

#include <ompl/control/SimpleSetup.h>
#include <boost/math/constants/constants.hpp>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;


using namespace std;
namespace plt = matplotlibcpp;


void demo_roms_data_reader(const char *filepath)
{
    dataset_parser dataset(filepath);
    int depth = 8;
    if (dataset.open())
    {
        std::vector<float> x, y, u, v;
        auto resU = dataset.retrieve_data(depth, UU);
        auto resV = dataset.retrieve_data(depth, VV);
        for (int i = 0; i < resU.size(); ++i) {
            for (int j = 0; j < resU[i].size(); ++j) {
                //                cout << res[i][j] << " ";
                x.push_back(i);
                u.push_back(resU[j][i]);
                y.push_back(j);
                v.push_back(resV[j][i]);
            }
            //            cout << endl;
        }

        //        TODO update x tick and y tick value with lat and lon
//        auto lat = dataset.retrieve_data(depth, LAT);
//        auto lon = dataset.retrieve_data(depth, LON);

        plt::quiver(x, y, u, v);

    }
}


// Our collision checker. For this demo, our robot's state space
// lies in [0,1]x[0,1], with a circular obstacle of radius 0.25
// centered at (0.5,0.5). Any states lying in this circular region are
// considered "in collision".
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
            ob::StateValidityChecker(si) {}

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const
    {
        const ob::RealVectorStateSpace::StateType* state2D =
                state->as<ob::RealVectorStateSpace::StateType>();
        double x = state2D->values[0];
        double y = state2D->values[1];

        bool inRegion = x >= 10 - 1 && x <= 15 + 1 && y >= 2 - 1 && y <= 15 + 1;
        return !inRegion;
//        return this->clearance(state) > 0.0;
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
};

// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration space of computed
// paths.
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}


int main(int argc, char * argv[]) {

    assert(argc > 1 && "filepath not found in args");
//    demo_roms_data_reader(argv[1]);
// Construct the robot state space in which we're planning. We're
// planning in [0,1]x[0,1], a subset of R^2.
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

// Set the bounds of space to be in [0,1].
    space->as<ob::RealVectorStateSpace>()->setBounds(0.0, 25.0);

// Construct a space information instance for this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

// Set the object used to check which states in the space are valid
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));

    si->setup();

// Set our robot's starting state to be the bottom-left corner of
// the environment, or (0,0).
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.0;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 1.0;

// Set our robot's goal state to be the top-right corner of the
// environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 20.0;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 15.0;

// Create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

// Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);
    pdef->setOptimizationObjective(getPathLengthObjective(si));
    // Construct our optimizing planner using the RRTstar algorithm.
    ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));

// Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
//    optimizingPlanner->setup();


// attempt to solve the planning problem within one second of
// planning time
    ob::PlannerStatus solved = optimizingPlanner->solve(1.0);

    // Output the length of the path found
    std::cout
            << optimizingPlanner->getName()
            << " found a solution of length "
            << pdef->getSolutionPath()->length()
            << " with an optimization objective value of "
            << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) << std::endl;

    pdef->getSolutionPath()->print(cout);

    auto path = pdef->getSolutionPath()->as<og::PathGeometric>();

    vector<double>pathX, pathY;
    for(auto state: path->getStates())
    {
        const ob::RealVectorStateSpace::StateType* state2D =
                state->as<ob::RealVectorStateSpace::StateType>();
        double x = state2D->values[0];
        double y = state2D->values[1];
//        cout << x << ", " << y << endl;
        pathX.push_back(x);
        pathY.push_back(y);

    }
//   cout << path.size() <<endl;

    demo_roms_data_reader(argv[1]);

    vector<double> obX{10, 15, 15, 10, 10}, obY{2, 2, 15, 15, 2};
    plt::plot(obX, obY);

    plt::plot(pathX, pathY);
    plt::show();

    return 0;
}