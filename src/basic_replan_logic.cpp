#include <trajectory_based_nav/default_implementations.h>


namespace trajectory_based_nav
{

    BasicReplanLogic::BasicReplanLogic(trajectory_based_nav::TrajectoryVerifier::Ptr verifier, std::string name):
      ReplanLogic(name)
    {
      verifier_ = verifier;
      //min_tte_ = ros::Duration(3);
      //max_replan_period_ = ros::Duration(3);  //TODO: use ros parameters
    }
    
    bool BasicReplanLogic::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      auto mnh = ros::NodeHandle(pnh, name_);
      
      double min_tte=3;
      mnh.getParam("min_tte", min_tte);
      mnh.setParam("min_tte", min_tte);
      
      min_tte_ = ros::Duration(min_tte);
      
      double max_replan_period=0.2;
      mnh.getParam("max_replan_period", max_replan_period);
      mnh.setParam("max_replan_period", max_replan_period);
      
      max_replan_period_ = ros::Duration(max_replan_period);  //TODO: use ros parameters
      
      return true;
    }
    
    bool BasicReplanLogic::shouldReplan(const nav_msgs::Odometry& odom, TrajectoryWrapper::Ptr traj)
    {
      auto last_planning_time = last_planning_time_;
      
//       if(!current_trajectory_)
//       {
//         ROS_INFO_STREAM("No current trajectory, definitely replan!");
//         return true;
//       }
      
      auto durations = traj->getDurations();
      
      bool replan = false;
      
      if(durations.size()==0)
      {
        ROS_INFO_STREAM("[Replanning] No existing trajectory, time to replan!");
        replan = true;
      }
      else
      if(!traj->to_goal && durations.size()>0 && durations.back() < min_tte_)
      {
        ROS_INFO_STREAM("[Replanning] Time to end of trajectory=" << durations.back() << ", less than permitted minimum");
        replan = true;
      }
      else  
      if(max_replan_period_ >= ros::Duration() && (odom.header.stamp - last_planning_time) > max_replan_period_)
      {
        ROS_INFO_STREAM("[Replanning] Time since last plan=" << (odom.header.stamp - last_planning_time) << ", time to replan!");
        replan = true;
      }
      else
      if(!verifier_->verifyTrajectory(traj))
      {
        ROS_INFO_STREAM("Verifier is not satisfied, replan");
        replan = true;
      }
      
      if(replan)
      {
        last_planning_time_ = odom.header.stamp;
      }
      else
      {
        ROS_DEBUG_STREAM("[Replanning] Not replanning!");
      }
      return replan;
    }

} //end namespace trajectory_based_nav
