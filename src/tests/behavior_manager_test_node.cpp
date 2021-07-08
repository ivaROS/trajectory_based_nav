#include <trajectory_based_nav/behavior_manager.h>


using namespace trajectory_based_planning;

class TestBehavior : public Behavior
{
    virtual bool start() override
    {
        ROS_INFO_STREAM("[" << name_ << "] started!");
        return true;
    }
    
    virtual bool stop() override
    {
        ROS_INFO_STREAM("[" << name_ << "] stopped!");
        return true;
    }
};






int main(int argc, char** argv) {
  ros::init(argc, argv, "behavior_manager_test");
  
  using namespace trajectory_based_planning;
  BehaviorManager m("manager");
  
  auto b1 = std::make_shared<TestBehavior>("b1");
  auto b2 = std::make_shared<TestBehavior>("b2");
  
  
  BehaviorRequest r1(b1, true);
  m.addRequest(r1);
  
  ros::Time::sleep(1000);
  
  BehaviorRequest r2(b1, false);
  BehaviorRequest r3(b2, true);
  
  m.addRequest(r2);
  m.addRequest(r3);
  
  ros::spin();
  
  return 0;
}
