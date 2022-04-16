//
// Created by redwan on 4/3/22.
//

#include "ompl_planner.h"
#include <memory>
#include <utility>

ompl_planner::ompl_planner(const vector<float> &start, const vector<float> &goal) : start_(start), goal_(goal) {}

void ompl_planner::setup(ObstclesPtr obstacles) {
    // Construct the robot state space in which we're planning. We're
// planning in [0,1]x[0,1], a subset of R^2.
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));


// Set the bounds of space to be in [0,1].
    auto low = obstacles->params->get_min_index();
    auto high = obstacles->params->get_max_index();
    space->as<ob::RealVectorStateSpace>()->setBounds(low, high);

// Construct a space information instance for this state space
    si_ = std::make_shared<ob::SpaceInformation>(space);

// Set the object used to check which states in the space are valid
    si_->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si_, std::move(obstacles))));

    si_->setup();

// Set our robot's starting state to be the bottom-left corner of
// the environment, or (0,0).
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_[0];
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_[1];

// Set our robot's goal state to be the top-right corner of the
// environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_[0];
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_[1];

// Create a problem instance
    pdef_ = std::make_shared<ob::ProblemDefinition>(si_);

// Set the start and goal states
    pdef_->setStartAndGoalStates(start, goal);
    pdef_->setOptimizationObjective(getPathLengthObjective(si_));


}

ompl_planner::PATH ompl_planner::get_solution(DatasetPtr dataset, int depth) {

    //vector field generator
    auto U = dataset->retrieve_data(depth, UU);
    auto V = dataset->retrieve_data(depth, VV);

    // here we generate vector field from the roms dataset. ROMS dataset has 1 km x 1 km resolution
    // we need to convert sample point to integer indices of velocity vectors
    auto vectorField = [=](const ompl::base::State *qnear){
        auto qnear2D = qnear->as<ob::RealVectorStateSpace::StateType>();

        int x = int(qnear2D->values[0]);
        int y = int(qnear2D->values[1]);
        auto qnear_u = U[y][x];
        auto qnear_v = V[y][x];
        Eigen::VectorXd vfield(2);
        vfield << qnear_u, qnear_v;
        return vfield;
    };

    double exploration = dataset->params->get_exploration_const();
    double initial_lambda = dataset->params->get_initial_lambda();
    int update_freq = dataset->params->get_update_freq();

    ob::PlannerPtr optimizingPlanner(new og::VFRRT(si_, vectorField, exploration, initial_lambda, update_freq));

// Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef_);
//    optimizingPlanner->setup();


// attempt to solve the planning problem within one second of
// planning time
    float solve_time = dataset->params->get_solve_time();
    ob::PlannerStatus solved = optimizingPlanner->solve(solve_time);

    ompl_planner::PATH solution;

    // Output the length of the path found
    std::cout
            << optimizingPlanner->getName()
            << " found a solution of length "
            << pdef_->getSolutionPath()->length()
            << " with an optimization objective value of "
            << pdef_->getSolutionPath()->cost(pdef_->getOptimizationObjective()) << std::endl;

//    pdef_->getSolutionPath()->print(cout);

    auto res = pdef_->getSolutionPath()->as<og::PathGeometric>();

    for(auto state: res->getStates())
    {
        const ob::RealVectorStateSpace::StateType* state2D =
                state->as<ob::RealVectorStateSpace::StateType>();
        float x = state2D->values[0];
        float y = state2D->values[1];
//        cout << x << ", " << y << endl;
        solution.first.push_back(x);
        solution.second.push_back(y);

    }

    return solution;
}

ob::OptimizationObjectivePtr ompl_planner::getPathLengthObjective(const ompl::base::SpaceInformationPtr &si) {
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

ValidityChecker::ValidityChecker(const ompl::base::SpaceInformationPtr &si, ObstclesPtr obstacles)  :
        ob::StateValidityChecker(si), obstacles_(std::move(obstacles)) {

}


bool ValidityChecker::isValid(const ob::State *state) const {
    const auto* state2D =
            state->as<ob::RealVectorStateSpace::StateType>();
    auto x = (float) state2D->values[0];
    auto y = (float) state2D->values[1];
    return obstacles_->isValidState(x, y);
}

double ValidityChecker::clearance(const ob::State *state) const {
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

