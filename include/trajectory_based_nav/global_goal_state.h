#ifndef TRAJECTORY_BASED_NAV_GLOBAL_GOAL_STATE_H
#define TRAJECTORY_BASED_NAV_GLOBAL_GOAL_STATE_H

#include <tf2_utils/transform_manager.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

namespace trajectory_based_nav
{

  class GlobalGoalState
  {
  public:
    using Ptr = std::shared_ptr<GlobalGoalState>;
    using NoArgsCallback = boost::function<void() >;
    
  protected:
    geometry_msgs::PoseStamped::ConstPtr goal_pose_;
    geometry_msgs::PoseStamped planning_frame_goal_;
    bool have_goal_=false, have_new_goal_=false;
    
    std::string planning_frame_id_, fixed_frame_id_;
    
    message_filters::Subscriber<geometry_msgs::PoseStamped> goal_sub_;
    
    using GoalTFFilter = tf2_ros::MessageFilter<geometry_msgs::PoseStamped>;
    std::shared_ptr<GoalTFFilter> goal_filter_;
    
    using PassThroughFilter = message_filters::PassThrough<geometry_msgs::PoseStamped>;
    PassThroughFilter pass_through_filter_;
    
    
    typedef boost::mutex::scoped_lock Lock;
    boost::mutex goal_mutex_;
    
    NoArgsCallback goal_reached_cb_;
    
    //geometry_msgs::TransformStamped goal_to_planning_transform_;
    
    tf2_utils::TransformManager tfm_;
    
    //TODO: store new goals but don't update current goal until 'update' run?
    virtual void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
    
  public:
    virtual bool init(ros::NodeHandle nh, ros::NodeHandle pnh, tf2_utils::TransformManager tfm);
    
//     virtual bool update(const nav_msgs::Odomeetry& odom);

    virtual message_filters::SimpleFilter<geometry_msgs::PoseStamped>& getGoalSource();
    
    virtual message_filters::SimpleFilter<geometry_msgs::PoseStamped>& getReadyGoalSource();
    
    virtual bool hasGoal();
    
    virtual bool hasNewGoal();
    
    virtual void setNewGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);
    
    virtual geometry_msgs::Pose getGoal();
    
    virtual bool updateGoalTransform(const nav_msgs::Odometry& odom);
    
    virtual geometry_msgs::PoseStamped getGoalStamped();
    
    virtual void reachedGoal();
    
    virtual void setGoalReachedCallback(NoArgsCallback cb=0);
  };
  
} //end namespace trajectory_based_nav

#endif  //TRAJECTORY_BASED_NAV_GLOBAL_GOAL_STATE_H
