#include <trajectory_based_nav/global_goal_state.h>
#include <pips/utils/param_utils.h>
#include <tf2_utils/transform_manager.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace trajectory_based_nav
{


    //TODO: store new goals but don't update current goal until 'update' run?
    void GlobalGoalState::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
    {
      geometry_msgs::PoseStamped transformed_goal;
      //geometry_msgs::TransformStamped goal_to_planning_transform;
      
      ROS_INFO_STREAM("Received goal!");
      
      bool goal_valid = false;
      try //This may not be necessary due to the TF MessageFilter
      {
        transformed_goal = tfm_.getBuffer()->transform(*goal, fixed_frame_id_);  //This works, but if transform is needed again later, best to hold onto it
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
    
    bool GlobalGoalState::init(ros::NodeHandle nh, ros::NodeHandle pnh, tf2_utils::TransformManager tfm)
    {
      ros::NodeHandle ppnh(pnh, "global_goal_state");
      tfm_ = tfm;
      /* TODO: Need to ensure that consistent coordinate frames are being used. Either:
       * 1. Always plan in the odometry frame and set it automatically based on first message received.
       * 2. Transform poses to the specified planning frame whenever needed.
       */
      pips::utils::searchParam(ppnh, "planning_frame_id", planning_frame_id_, "world", 100);
      
      pips::utils::searchParam(ppnh, "fixed_frame_id", fixed_frame_id_, "world", 100);
      
      //goal_sub_.subscribe(nh, "/move_base_simple/goal", 1);
      
      //pass_through_filter_ = std::make_shared<message_filters::PassThrough>();
      
      pass_through_filter_.connectInput(goal_sub_);
      
      goal_filter_ = std::make_shared<GoalTFFilter>(pass_through_filter_, *tfm.getBuffer(), fixed_frame_id_, 2, nh);
      goal_filter_->registerCallback(boost::bind(&GlobalGoalState::GoalCallback, this, _1));
      
      return true;
    }
    
//     virtual bool update(const nav_msgs::Odomeetry& odom)
//     {
//       return true;
//     }
    
    message_filters::SimpleFilter<geometry_msgs::PoseStamped>& GlobalGoalState::getGoalSource()
    {
      return pass_through_filter_;  // goal_sub_;
    }
    
    message_filters::SimpleFilter<geometry_msgs::PoseStamped>& GlobalGoalState::getReadyGoalSource()
    {
      return *goal_filter_;
    }
    
    bool GlobalGoalState::hasGoal()
    {
      Lock lock(goal_mutex_);
      return have_goal_;
    }
    
    bool GlobalGoalState::hasNewGoal()
    {
      Lock lock(goal_mutex_);
      return have_goal_ && have_new_goal_;
    }
    
    void GlobalGoalState::setNewGoal(const geometry_msgs::PoseStamped::ConstPtr& goal)
    {
      ROS_INFO_STREAM_NAMED("global_goal_state.set_new_goal", "[GlobalGoalState] received a new goal [" << *goal << "]");
      pass_through_filter_.add(goal);
    }
    
    geometry_msgs::Pose GlobalGoalState::getGoal()
    {
      geometry_msgs::PoseStamped goal_stamped = getGoalStamped();
      return goal_stamped.pose;
    }
    
    bool GlobalGoalState::updateGoalTransform(const nav_msgs::Odometry& odom)
    {
      Lock lock(goal_mutex_);
      
      try
      {
        //planning_frame_goal_ = tfm_.getBuffer()->transform(*goal_pose_, planning_frame_id_);
        planning_frame_goal_ = tfm_.getBuffer()->transform(*goal_pose_, planning_frame_id_, ros::Time(0), fixed_frame_id_);
        ROS_DEBUG_STREAM_NAMED("global_goal_state.update_goal_transform", "[GlobalGoalState] updated planning frame goal [" << planning_frame_goal_ << "]");
        
        return true;
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN_NAMED("global_goal_state", "Unable to transform current goal into planning frame [%s]! (fixed_frame=%s): %s", planning_frame_id_.c_str(), fixed_frame_id_.c_str(), ex.what());
        //If we know a new goal was sent but we were unable to transform it, should probably stop what we're doing
        return false;
      }
    }
    
    geometry_msgs::PoseStamped GlobalGoalState::getGoalStamped()
    {
      Lock lock(goal_mutex_);
      
      have_new_goal_ = false;
      //return *goal_pose_;
      return planning_frame_goal_;
    }
    
    //virtual geometry_msgs::TransformStamped get
    
    void GlobalGoalState::reachedGoal()
    {
      ROS_INFO_NAMED("global_goal_state.reached_goal", "[global_goal_state] Reached goal!");
      
      Lock lock(goal_mutex_);
      
      have_goal_ = false;
      have_new_goal_ = false;
      
      //TODO: Replace with a more general system (possibly using message_filters?) to allow multiple callbacks, etc
      if(goal_reached_cb_)
      {
        goal_reached_cb_();
      }
    }
    
    void GlobalGoalState::setGoalReachedCallback(NoArgsCallback cb)
    {
      goal_reached_cb_ = cb;
    }
  
} //end namespace trajectory_based_nav
