#include <trajectory_based_nav/behavior_manager.h>


using namespace trajectory_based_nav;

int main(int argc, char** argv) {
  ros::init(argc, argv, "behavior_manager_test");
  ros::Time::init();
  
  using namespace trajectory_based_nav;
  BehaviorManager m("manager");
  
  auto b1 = std::make_shared<TestBehavior>("b1");
  auto b2 = std::make_shared<TestBehavior>("b2");
  
  
  BehaviorRequest r1(b1, true);
  m.addRequest(r1);
  
  ros::Duration(1.0).sleep();
  
  BehaviorRequest r2(b1, false);
  BehaviorRequest r3(b2, true);
  
  m.addRequest(r2);
  m.addRequest(r3);
  
  ros::spin();
  
  return 0;
}
