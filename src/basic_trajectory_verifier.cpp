#include <trajectory_based_nav/default_implementations.h>



namespace trajectory_based_nav
{
  
//     BasicTrajectoryVerifier::BasicTrajectoryVerifier(std::string name):
//       name_(name)
//       {}
    
    bool BasicTrajectoryVerifier::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      auto ppnh = ros::NodeHandle(pnh, name_);
      
      double min_ttc=3;
      ppnh.getParam("min_ttc", min_ttc);
      ppnh.setParam("min_ttc", min_ttc);
      
      min_ttc_ = ros::Duration(min_ttc);
      
      return true;
    }
    
    bool BasicTrajectoryVerifier::verifyTrajectory(TrajectoryWrapper::Ptr traj)
    {
      int collision_ind = collisionCheckTrajectory(traj);
      
      ROS_INFO_STREAM_NAMED("trajectory_verifier", "[" << name_ << "] collision_ind=" << collision_ind);
      
      traj->collision_check_res = std::make_shared<BasicCollisionCheckingResult>(collision_ind);
      if(collision_ind >=0)
      {
        if(min_ttc_ < ros::Duration(0))
        {
          ROS_INFO_STREAM_NAMED("trajectory_verifier", "[" << name_ << "] Verification failed: no collisions permitted");
          return false;
        }
        auto collision_time = traj->getDurations()[std::min((int)traj->getDurations().size()-1,collision_ind)];
        if(collision_time < min_ttc_)
        {
          ROS_INFO_STREAM_NAMED("trajectory_verifier", "[" << name_ << "] Verification failed: collision_time (" << collision_time << ") < min_ttc (" << min_ttc_ << ")");
          return false;
        }
        else
        {
          ROS_INFO_STREAM_NAMED("trajectory_verifier", "[" << name_ << "] Verification passed: collision_time (" << collision_time << ") >= min_ttc (" << min_ttc_ << ")");
          return true;
        }
      }
      else
      {
        ROS_INFO_STREAM_NAMED("trajectory_verifier", "[" << name_ << "] Verification passed: no collision");
        return true;
      }
      
    }

} //end namespace trajectory_based_nav
