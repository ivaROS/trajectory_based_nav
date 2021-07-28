#ifndef NAV_GENERAL_GLOBAL_GOAL_NAV_H
#define NAV_GENERAL_GLOBAL_GOAL_NAV_H

#include <trajectory_based_nav/global_goal_state.h>
#include <pips_egocylindrical/free_space_checker.h>
//#include <nav_quadrotor/new_impl.h>
#include <trajectory_based_nav/default_implementations.h>
#include <trajectory_based_nav/interfaces.h>
//#include <nav_quadrotor/global_potentials_interface.h>
#include <boost/thread/mutex.hpp>
#include <pips_egocylindrical/FreeSpaceCheckerService.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>

#include <pips/utils/param_utils.h>


namespace trajectory_based_nav
{

  inline
  double getYaw(const geometry_msgs::Quaternion& quaternion)
  {
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quaternion, quat);
    
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    return yaw;
  }
  
  
  
  /* Some global goal thoughts:
   * Should have some mechanism for ensuring that consistent goal is used; ex, the goals in different frames include some ID (or even just a pointer to the original goal message)
   * that allows them to be compared. For example, trajectory source can confirm that the global plan provided was for the same goal.
   * 
   * 
   */
  
  
  class GlobalGoalReplanLogic : public BasicReplanLogic
  {
  public:
    GlobalGoalReplanLogic(trajectory_based_nav::TrajectoryVerifier::Ptr verifier, std::shared_ptr<GlobalGoalState> ggs):
      BasicReplanLogic(verifier),
      ggs_(ggs)
    {
      
    }
    
    virtual bool shouldReplan(const nav_msgs::Odometry& odom, TrajectoryWrapper::Ptr traj)
    {
      if(ggs_->hasNewGoal())
      {
        ROS_INFO_STREAM("New goal received, replan!");
        return true;
      }
      else
      {
        return BasicReplanLogic::shouldReplan(odom, traj);
      }
    }
    
    
  protected:
    std::shared_ptr<GlobalGoalState> ggs_;
    
  };
  
  class GlobalGoalTerminationCriteria : public TerminationCriteria
  {
  protected:
    std::shared_ptr<GlobalGoalState> ggs_;
    
    double goal_dist_tolerance_;
    std::string name_ = "termination_criteria";
    
  public:
    
    GlobalGoalTerminationCriteria(std::shared_ptr<GlobalGoalState> ggs):
      ggs_(ggs)
    {}
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      
      //get goal tolerance parameter values
      goal_dist_tolerance_ = 0.1;
      
      return true;
    }
    
    virtual bool shouldTerminate(const nav_msgs::Odometry& odom) 
    {
      if(!ggs_->hasGoal())
      {
        return true;
      }
      
      //If close enough to goal
      {
        auto pos = odom.pose.pose.position;
        auto dpos = ggs_->getGoal().position;
        double dx = pos.x - dpos.x;
        double dy = pos.y - dpos.y;
        //double dz = pos.z - dpos.z;
        
        double dist = std::sqrt(dx*dx+dy*dy);//TODO: Maybe include dz?
        
        bool reached_goal = dist < goal_dist_tolerance_;
        
        ROS_INFO_STREAM_NAMED(name_+".goal_distance", "[" + name_ + "]: reached_goal=" << reached_goal << ", goal dist= " << dist << ", goal_pos=" << dpos << ", robot_pos=" << pos);
        
        if(reached_goal)
        {
          ggs_->reachedGoal();
        }
        return reached_goal;
      }
      
    }
    
    using Ptr=std::shared_ptr<GlobalGoalTerminationCriteria>;
    
  };
  
  //this really isn't different, just different parameter choice
  class GlobalTrajectoryVerifier : public trajectory_based_nav::BasicTrajectoryVerifier
  {
  public:
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      min_ttc_ = ros::Duration(5);
      return true;
    }
    using Ptr=std::shared_ptr<GlobalTrajectoryVerifier>;
    
  };
  
    

  
} //end namespace trajectory_based_nav
  
  
#endif //NAV_GENERAL_GLOBAL_GOAL_NAV_H
