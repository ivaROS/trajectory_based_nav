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
  
  
  /*
  class GlobalGoalManager
  {
      using MoveBaseActionServer = actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>;
      
      
      
      bool init()
      {
          as_ = std::make_shared<MoveBaseActionServer>(nh, "move_base", boost::bind(&GlobalGoalManager::executeCb, this, _1), false);
          
          current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );
          
          ros::NodeHandle action_nh("move_base");
          action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
          
          //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
          //they won't get any useful information back about its status, but this is useful for tools
          //like nav_view and rviz
          ros::NodeHandle simple_nh("move_base_simple");
          goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&GlobalGoalManager::goalCB, this, _1));
          
      }
      
      //Modified from move_base.cpp
      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
          ROS_DEBUG_NAMED("global_goal_manager","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
          move_base_msgs::MoveBaseActionGoal action_goal;
          action_goal.header.stamp = ros::Time::now();
          action_goal.goal.target_pose = *goal;
          
          action_goal_pub_.publish(action_goal);
      }
      
      
      void executeCb(const move_base_msgs::MoveBaseGoal::ConstPtr& move_base_goal)
      {
          if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
              as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
              return;
          }
          
          geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);
          
          //we have a goal so start the planner
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();
          
          current_goal_pub_.publish(goal);
          std::vector<geometry_msgs::PoseStamped> global_plan;
          
          
          //we want to make sure that we reset the last time we had a valid plan and control
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
          
          ros::NodeHandle n;
          while(n.ok())
          {

              
              if(as_->isPreemptRequested()){
                  if(as_->isNewGoalAvailable())
                  {
                      //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
                      move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();
                      
                      if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
                          as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
                          return;
                      }
                      
                      goal = goalToGlobalFrame(new_goal.target_pose);
                      
                      //we'll make sure that we reset our state for the next execution cycle
                      recovery_index_ = 0;
                      state_ = PLANNING;
                      
                      //we have a new goal so make sure the planner is awake
                      lock.lock();
                      planner_goal_ = goal;
                      runPlanner_ = true;
                      planner_cond_.notify_one();
                      lock.unlock();
                      
                      //publish the goal point to the visualizer
                      ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
                      current_goal_pub_.publish(goal);
                      
                      //make sure to reset our timeouts and counters
                      last_valid_control_ = ros::Time::now();
                      last_valid_plan_ = ros::Time::now();
                      last_oscillation_reset_ = ros::Time::now();
                      planning_retries_ = 0;
                  }
                  else 
                  {
                      //if we've been preempted explicitly we need to shut things down
                      resetState();
                      
                      //notify the ActionServer that we've successfully preempted
                      ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
                      as_->setPreempted();
                      
                      //we'll actually return from execute after preempting
                      return;
                  }
              }
              
              
              
              //for timing that gives real time even in simulation
              ros::WallTime start = ros::WallTime::now();
              
              //the real work on pursuing a goal is done here
              bool done = executeCycle(goal, global_plan);
              
              //if we're done, then we'll return from execute
              if(done)
                  return;
              
              //check if execution of the goal has completed in some way
              
              ros::WallDuration t_diff = ros::WallTime::now() - start;
              ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());
              
              r.sleep();
              //make sure to sleep for the remainder of our cycle time
              if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
                  ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
          }
          
          //wake up the planner thread so that it can exit cleanly
          lock.lock();
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();
          
          //if the node is killed then we'll abort and return
          as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
          return;
      }
      
      
  };
  
  */
  
  
  
  class GlobalGoalState
  {
  public:
    using Ptr = std::shared_ptr<GlobalGoalState>;
    using NoArgsCallback = boost::function<void() >;
    
  protected:
    geometry_msgs::PoseStamped::ConstPtr goal_pose_;
    bool have_goal_=false, have_new_goal_=false;
    
    std::string planning_frame_id_;
    
    message_filters::Subscriber<geometry_msgs::PoseStamped> goal_sub_;
    
    using GoalTFFilter = tf2_ros::MessageFilter<geometry_msgs::PoseStamped>;
    std::shared_ptr<GoalTFFilter> goal_filter_;
    
    typedef boost::mutex::scoped_lock Lock;
    boost::mutex goal_mutex_;
    
    NoArgsCallback goal_reached_cb_;
    
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
      
      //TODO: Replace with a more general system (possibly using message_filters?) to allow multiple callbacks, etc
      if(goal_reached_cb_)
      {
        goal_reached_cb_();
      }
    }
    
    virtual void setGoalReachedCallback(NoArgsCallback cb=0)
    {
      goal_reached_cb_ = cb;
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
