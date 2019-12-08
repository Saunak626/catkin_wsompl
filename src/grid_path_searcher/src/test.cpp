#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <ostream>
#include <boost/bind.hpp> //绑定函数

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state) {
  //抽象类型转换为我们期望类型
  const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
  //提取第1、2状态的组成，并转换为我们期望的
  const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
  const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

  //确定状态是否可行，这里一直为true，避免编译器警告
  return (const void *)rot != (const void *)pos;
}

void planWithSimpleSetup() {
  //声明我们规划所在的空间维度
  ob::StateSpacePtr space(new ob::SE3StateSpace());
  //设置三维空间的边界
  ob::RealVectorBounds bounds(3);
  bounds.setHigh(1);
  bounds.setLow(-1);
  space->as<ob::SE3StateSpace>()->setBounds(bounds);
  //定义一个简易类
  og::SimpleSetup ss(space);

  //路径约束检查,使用bind绑定函数，参考 https://blog.csdn.net/giepy/article/details/45046737
  ss.setStateValidityChecker(boost::bind(&isStateValid,_1));
  // 随机创建一个起始点和目标点
  ob::ScopedState<> start(space),goal(space);
  start.random();
  goal.random();
  start.print();
  //加入起终点
  ss.setStartAndGoalStates(start, goal);
  //设定规划方法
  ob::PlannerPtr planner(new og::RRT(ss.getSpaceInformation()));
  ss.setPlanner(planner);
  //在规划的时间内解决
  ob::PlannerStatus solved = ss.solve(1.0);

  //解决则导出生成的路径
  if (solved) {
    cout << "Found solution\n" << endl;
    ofstream osf0("path0.txt");
    ss.getSolutionPath().printAsMatrix(osf0);
    ofstream osf1("path1.txt");
    ss.simplifySolution();
    ss.getSolutionPath().printAsMatrix(osf1);
  }
  else
    cout << "No found" << endl;
}

int main(int, char**) {
  cout << "OMPL_VERSION:" << OMPL_VERSION << endl;
  planWithSimpleSetup();
  return 0;
}
