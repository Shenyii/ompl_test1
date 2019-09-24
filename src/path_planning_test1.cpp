#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include "boost/bind.hpp"
 
namespace ob = ompl::base;
namespace og = ompl::geometric;
 
// Return true if the state is valid, false if the state is invalid
bool isStateValid(const ob::State *state)
{
  const ob::SE2StateSpace::StateType *state_2d = state->as<ob::SE2StateSpace::StateType>();
  const double &x(state_2d->getX()), &y(state_2d->getY());
  // State is invalid when it is inside a 1x1 box
  // centered at the origin:
  // if(std::fabs(x)<0.9 && std::fabs(y)<0.9)
  //   return false;
  if(x < -0.3 && x > -0.6)
  {
      if(y < 0.5 && y > -0.6)
      {
          return false;
      }
  }
  if(x > 0.3 && x < 0.6)
  {
      if(y < 0.5 && y > -0.6)
      {
          return false;
      }
  }
  if(x < 0.3 && x > -0.3)
  {
      if(y < -0.3 && y > -0.6)
      {
          return false;
      }
  }
  // Otherwise, the state is valid:
  return true;
}

void plan()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);

    // create a random start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    //start.random();
    start->setXY(0,0);

    // create a random goal state
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    //goal.random();
    goal->setXY(0,-0.9);

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner(std::make_shared<og::RRTConnect>(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();


    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}
 
void planWithSimpleSetup(void)
{
  // Construct the state space where we are planning
  ob::StateSpacePtr space(new ob::SE2StateSpace());
 
  ob::RealVectorBounds bounds(2);
  bounds.setLow(-1);
  bounds.setHigh(1);
  space->as<ob::SE2StateSpace>()->setBounds(bounds);
 
  // Instantiate SimpleSetup
  og::SimpleSetup ss(space);
 
  // Setup the StateValidityChecker
  ss.setStateValidityChecker(boost::bind(&isStateValid, _1));
 
  // Setup Start and Goal
  ob::ScopedState<ob::SE2StateSpace> start(space);
  start->setXY(0,0);
  std::cout << "start: "; start.print(std::cout);
 
  ob::ScopedState<ob::SE2StateSpace> goal(space);
  goal->setXY(0,-0.9);
  std::cout << "goal: "; goal.print(std::cout);
 
  ss.setStartAndGoalStates(start, goal);
 
  std::cout << "----------------" << std::endl;
 
  // Execute the planning algorithm
  ob::PlannerStatus solved = ss.solve(1.0);
 
  // If we have a solution,
  if (solved)
  {
    // Simplify the solution
    ss.simplifySolution();
    std::cout << "----------------" << std::endl;
    std::cout << "Found solution:" << std::endl;
    // Print the solution path to screen
    ss.getSolutionPath().print(std::cout);
 
    // Print the solution path to a file
    std::ofstream ofs("../plot/path.dat");
    ss.getSolutionPath().printAsMatrix(ofs);
  }
  else
    std::cout << "No solution found" << std::endl;
}
 
int main()
{
  planWithSimpleSetup();
  std::cout << std::endl << std::endl << std::endl;
  plan();
  return 0;
}
// --------------------- 
// 作者：找不到工作的我 
// 来源：CSDN 
// 原文：https://blog.csdn.net/ljq31446/article/details/79714728 
// 版权声明：本文为博主原创文章，转载请附上博文链接！
