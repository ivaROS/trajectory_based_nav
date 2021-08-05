#include <trajectory_based_nav/interfaces.h>
#include <limits>

namespace trajectory_based_nav
{
  
  bool TrajectoryVerifier::update(const nav_msgs::Odometry& odom)
  {
    std_msgs::Header header = cc_wrapper_->getCurrentHeader();
    
    if(!cc_wrapper_->isReady(odom.header))
    {
      return false;
    }
    cc_wrapper_->update();
    return true;
  }
  
  
  int TrajectoryVerifier::collisionCheckTrajectory(TrajectoryWrapper::Ptr traj)
  {
    auto cc = cc_wrapper_->getCC();
    
    const auto& poses = traj->getPoseArray().poses;
    size_t unsigned_num = poses.size();
    
    ROS_ASSERT(unsigned_num <= static_cast<size_t>(std::numeric_limits<int>::max()));
    int num_remaining_poses = static_cast<int>(unsigned_num);
    
    int i;
    for(i = 0; i < num_remaining_poses; i++)
    {
      bool res = cc->testCollision(poses[i]);
      if(res)
      {
        break;
      }
    }
    if(i<num_remaining_poses) //Current trajectory collides
    {
      return i;
    }
    else
    {
      return -1;
    }
  }

} //end namespace trajectory_based_nav
    
