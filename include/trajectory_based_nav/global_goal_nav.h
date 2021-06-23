#ifndef NAV_GENERAL_GLOBAL_GOAL_NAV_H
#define NAV_GENERAL_GLOBAL_GOAL_NAV_H

#include <pips_egocylindrical/free_space_checker.h>
//#include <nav_quadrotor/new_impl.h>
#include <trajectory_based_nav/default_implementations.h>
#include <trajectory_based_nav/interfaces.h>
//#include <nav_quadrotor/global_potentials_interface.h>
#include <boost/thread/mutex.hpp>
#include <pips_egocylindrical/FreeSpaceCheckerService.h>
#include <tf2_ros/message_filter.h>
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
  
  class GlobalGoalState
  {
  public:
    using Ptr = std::shared_ptr<GlobalGoalState>;
    
  protected:
    geometry_msgs::PoseStamped::ConstPtr goal_pose_;
    bool have_goal_=false, have_new_goal_=false;
    
    std::string planning_frame_id_;
    
    message_filters::Subscriber<geometry_msgs::PoseStamped> goal_sub_;
    
    using GoalTFFilter = tf2_ros::MessageFilter<geometry_msgs::PoseStamped>;
    std::shared_ptr<GoalTFFilter> goal_filter_;
    
    typedef boost::mutex::scoped_lock Lock;
    boost::mutex goal_mutex_;
    
    //geometry_msgs::TransformStamped goal_to_planning_transform_;
    
    tf2_utils::TransformManager tfm_;
    
    //TODO: store new goals but don't update current goal until 'update' run?
    virtual void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
    {
      geometry_msgs::PoseStamped transformed_goal;
      //geometry_msgs::TransformStamped goal_to_planning_transform;
      
      ROS_INFO_STREAM("Received goal!");
      
      bool goal_valid = false;
      try //This may not be necessary due to the TF MessageFilter
      {
        transformed_goal = tfm_.getBuffer()->transform(*goal, planning_frame_id_);  //This works, but if transform is needed again later, best to hold onto it
        //goal_to_planning_transform = tfm_.getBuffer()->lookupTransform(planning_frame_id_, goal->header.frame_id, goal->header.stamp);        
        goal_valid = true;
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN_NAMED("global_goal_state", "Unable to transform new goal into planning frame!: %s",ex.what());
        //If we know a new goal was sent but we were unable to transform it, should probably stop what we're doing
      }

      if(goal_valid)
      {
        Lock lock(goal_mutex_);
        have_goal_ = true;
        have_new_goal_ = true;
        //tf2_ros::doTransform(*goal, transformed_goal, goal_to_planning_transform);
        //goal_to_planning_transform_ = goal_to_planning_transform;
        
        goal_pose_ = boost::make_shared<geometry_msgs::PoseStamped>(transformed_goal);
        //TODO: allow callbacks to be registered
      }
      else
      {
        Lock lock(goal_mutex_);
        have_goal_ = false;
        have_new_goal_ = false;
        goal_pose_ = boost::make_shared<geometry_msgs::PoseStamped>(transformed_goal);
      }
    }
    
  public:
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      tfm_ = tfm;
      /* TODO: Need to ensure that consistent coordinate frames are being used. Either:
       * 1. Always plan in the odometry frame and set it automatically based on first message received.
       * 2. Transform poses to the specified planning frame whenever needed.
       */
      pips::utils::searchParam(pnh, "planning_frame_id", planning_frame_id_, "odom");
      
      goal_sub_.subscribe(nh, "/move_base_simple/goal", 1);
      goal_filter_ = std::make_shared<GoalTFFilter>(goal_sub_, *tfm.getBuffer(), planning_frame_id_, 2, nh);
      goal_filter_->registerCallback(boost::bind(&GlobalGoalState::GoalCallback, this, _1));
      
      return true;
    }
    
//     virtual bool update(const nav_msgs::Odomeetry& odom)
//     {
//       return true;
//     }
    
    virtual message_filters::SimpleFilter<geometry_msgs::PoseStamped>& getGoalSource()
    {
      return goal_sub_;
    }
    
    virtual bool hasGoal()
    {
      Lock lock(goal_mutex_);
      return have_goal_;
    }
    
    virtual bool hasNewGoal()
    {
      Lock lock(goal_mutex_);
      return have_goal_ && have_new_goal_;
    }
    
    virtual void setNewGoal(const geometry_msgs::PoseStamped::ConstPtr& goal)
    {
      
    }
    
    virtual geometry_msgs::Pose getGoal()
    {
      Lock lock(goal_mutex_);
      
      have_new_goal_ = false;
      auto pose = goal_pose_->pose;
      pose.position.z =1.2;
      return pose;
    }
    
    virtual geometry_msgs::PoseStamped getGoalStamped()
    {
      Lock lock(goal_mutex_);
      
      have_new_goal_ = false;
      return *goal_pose_;
    }
    
    //virtual geometry_msgs::TransformStamped get
    
    virtual void reachedGoal()
    {
      Lock lock(goal_mutex_);
      
      have_goal_ = false;
      have_new_goal_ = false;
    }
  };
  
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
        double dz = pos.z - dpos.z;
        
        double dist = std::sqrt(dx*dx+dy*dy);//TODO: Maybe include dz?
        
        bool reached_goal = dist < goal_dist_tolerance_;
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
