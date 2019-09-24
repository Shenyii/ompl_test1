#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <cmath>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <iostream>
#include <fstream>
#include "boost/bind.hpp"
#include <vector>
 
namespace ob = ompl::base;
namespace og = ompl::geometric;
 
 
// Return true if the state is valid, false if the state is invalid
bool isStateValid(const ob::State *state)
{
    const ob::SE2StateSpace::StateType *state_2d = state->as<ob::SE2StateSpace::StateType>();
    const double &x(state_2d->getX()), &y(state_2d->getY());

    int grid[15][15];
    std::ifstream infile;
    //infile.open("~/Desktop/ompl/src/ompl_test1/script/22.txt");
    infile.open("22.txt");
    assert(infile.is_open());
    char c[1000];
    int C_n = 0;
    while(!infile.eof())
    {
        infile >> c[C_n];
        C_n++;
    }
    for(int i=0;i<15;i++)
    {
        for(int j=0;j<15;j++)
        {
            grid[i][j] = c[15 * i + j] - 48;
        }
    }
    infile.close();

    if((grid[int(x)][int(y)]==1))
        return false;
        
    return true;
 
}
 
void planWithSimpleSetup(void)
{
 
  ob::StateSpacePtr space(new ob::SE2StateSpace());
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0);
  bounds.setHigh(15);
  space->as<ob::SE2StateSpace>()->setBounds(bounds);
 
  // Instantiate SimpleSetup
  og::SimpleSetup ss(space);
 
  // Setup the StateValidityChecker
 
 
  // Setup Start and Goal
  ob::ScopedState<ob::SE2StateSpace> start(space);
  start->setXY(0,5);
  std::cout << "start: "; start.print(std::cout);
 
  ob::ScopedState<ob::SE2StateSpace> goal(space);
  goal->setXY(13,3);
  std::cout << "goal: "; goal.print(std::cout);
 
  ss.setStartAndGoalStates(start, goal);
  ob::PlannerPtr planner(new og::RRT(ss.getSpaceInformation()));
  ss.setPlanner(planner);
  ss.setStateValidityChecker(boost::bind(&isStateValid, _1));
  std::cout << "----------------" << std::endl;
 
  // Execute the planning algorithm
  ob::PlannerStatus solved = ss.solve(1.0);
 
  // If we have a solution,
  if (solved)
  {
    std::cout << "----------------" << std::endl;
    std::cout << "Found solution:" << std::endl;
    ss.getSolutionPath().print(std::cout);
    std::ofstream ofs("../script/path.dat");
    ss.getSolutionPath().printAsMatrix(ofs);
  }
  else
    std::cout << "No solution found" << std::endl;
}
 
int main()
{
    planWithSimpleSetup();

    return 0;
}
// --------------------- 
// 作者：找不到工作的我 
// 来源：CSDN 
// 原文：https://blog.csdn.net/ljq31446/article/details/79728814 
// 版权声明：本文为博主原创文章，转载请附上博文链接！
