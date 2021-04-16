#ifndef NAV_GENERAL_GLOBAL_GOAL_NAV_H
#define NAV_GENERAL_GLOBAL_GOAL_NAV_H

#include <pips_egocylindrical/free_space_checker.h>
#include <nav_quadrotor/new_impl.h>
#include <trajectory_based_nav/interfaces.h>
#include <nav_quadrotor/global_potentials_interface.h>
#include <boost/thread/mutex.hpp>
#include <pips_egocylindrical/FreeSpaceCheckerService.h>


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
    //TODO: Add mutex protection
  protected:
    geometry_msgs::PoseStamped::ConstPtr goal_pose_;
    bool have_goal_=false, have_new_goal_=false;
    ros::Subscriber goal_sub_;
    
    typedef boost::mutex::scoped_lock Lock;
    boost::mutex goal_mutex_;
    
    void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
    {
      //TODO: transform goal into planning frame
      ROS_INFO_STREAM("Received goal!");
      {
        Lock lock(goal_mutex_);
        have_goal_ = true;
        have_new_goal_ = true;
        goal_pose_ = goal;
      }
    }
    
  public:
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      goal_sub_ = nh.subscribe("/move_base_simple/goal",1, &GlobalGoalState::GoalCallback, this);
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
    
    virtual geometry_msgs::Pose getGoal()
    {
      Lock lock(goal_mutex_);
      
      have_new_goal_ = false;
      auto pose = goal_pose_->pose;
      pose.position.z =1.2;
      return pose;
    }
    
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
    }
    using Ptr=std::shared_ptr<GlobalTrajectoryVerifier>;
    
  };


  class GlobalGoalTrajectoryScoring : public TrajectoryScoring
  {
  protected:
    std::shared_ptr<nav_quadrotor::GlobalPotentialsInterface> gpi_;
    std::vector<std::shared_ptr<pips_egocylindrical::FreeSpaceChecker> > fscs_;
    double min_height_, max_height_, des_height_, height_weight_, max_height_cost_;
    double desired_freespace_, obstacle_scale_;
    ros::ServiceServer free_space_checking_service_;
    
  public:
    
    GlobalGoalTrajectoryScoring()
    {
      
    }
    
    
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm, std::vector<std::shared_ptr<pips_egocylindrical::FreeSpaceChecker> > fscs)
    {
      TrajectoryScoring::init(nh, pnh, tfm);
      
      fscs_ = fscs;
      std::string potentials_topic_ = "/raw_potential";
      
      gpi_ = std::make_shared<nav_quadrotor::GlobalPotentialsInterface>(nh);
      gpi_->init(potentials_topic_);
      

      
      auto my_pnh = ros::NodeHandle(pnh, "global_goal_trajectory_scoring");
      
      auto height_pnh = ros::NodeHandle(my_pnh, "height_cost");
      
      min_height_ = 0.5;
      max_height_ = 2.5;
      des_height_ = 1.25;
      height_weight_ = 1;
      max_height_cost_ = 1;
      
      height_pnh.getParam("min_height", min_height_);
      height_pnh.setParam("min_height", min_height_);
      
      height_pnh.getParam("max_height", max_height_);
      height_pnh.setParam("max_height", max_height_);
      
      height_pnh.getParam("des_height", des_height_);
      height_pnh.setParam("des_height", des_height_);
      
      height_pnh.getParam("weight", height_weight_);
      height_pnh.setParam("weight", height_weight_);
      
      height_pnh.getParam("max_cost", max_height_cost_);
      height_pnh.setParam("max_cost", max_height_cost_);
      
      auto obstacle_pnh = ros::NodeHandle(my_pnh, "obstacle_cost");
      
      //search_radius_ = 0.5;
      desired_freespace_ = 0.5;
      obstacle_scale_ = 2;
      
      obstacle_pnh.getParam("desired_freespace", desired_freespace_);
      obstacle_pnh.setParam("desired_freespace", desired_freespace_);
      
      obstacle_pnh.getParam("weight", obstacle_scale_);
      obstacle_pnh.setParam("weight", obstacle_scale_);
      
      free_space_checking_service_ = obstacle_pnh.advertiseService("free_space_checker", &GlobalGoalTrajectoryScoring::freeSpaceCheckingSrv, this);
      
    }
    
    virtual bool update(const nav_msgs::Odometry& odom)
    {
      //TODO: get actual transform between odom frame and frame of potentials, rather than assuming it is always identity
      tf::StampedTransform transform;
      transform.setIdentity();
      if(gpi_->updateTransform(transform))
      {
        ROS_INFO_ONCE("Potentials ready!");
        return true;
      }
      return false;
    }
    
    
  protected:
    
    
    bool freeSpaceCheckingSrv(pips_egocylindrical::FreeSpaceCheckerService::Request &req, pips_egocylindrical::FreeSpaceCheckerService::Response &res)
    {
      res.distance = getMinDistance(req.pose.pose);
      return true;
    }
    
    double getGoalDist(const geometry_msgs::Pose& pose)
    {

      float pot = gpi_->getPotential(pose);
      ROS_DEBUG_STREAM("Potential at [" << pose.position.x << "," << pose.position.y << "]: " << pot);
      if(pot<0 || pot >= 1e9) //NOTE: Previously tested == POT_HIGH, uncertain if this change is necessary
      {
        ROS_DEBUG_STREAM("Encountered invalid potential at [" << pose.position.x << "," << pose.position.y << "]: " << pot);
        return std::numeric_limits<float>::max();
      }
      return pot;
    }
    
    double getHeightCost(const geometry_msgs::Pose& pose)
    {
      double height = pose.position.z;
      double height_cost;
      if(height <= min_height_)
      {
        height_cost = 1e9;
        ROS_DEBUG_STREAM_NAMED("trajectory_scoring.height", "Too low!");
      }
      else if(height >= max_height_)
      {
        height_cost = 1e9;
        ROS_DEBUG_STREAM_NAMED("trajectory_scoring.height", "Too high!");
      }
      else
      {
        height_cost = std::abs(height - des_height_);
        height_cost = height_weight_ * height_cost;
        height_cost = std::min(max_height_cost_, height_cost);
      }
      return height_cost;
    }
    
    double getMinDistance(const geometry_msgs::Pose& pose)
    {
      float min_depth = std::numeric_limits<float>::max();
      for(const auto fsc : fscs_)
      {
        float free_space = fsc->getFreeSpaceRadius(pose);
        min_depth = std::min(min_depth, free_space);
      }
      return min_depth;
    }
    
    double getObstacleCost(const geometry_msgs::Pose& pose)
    {
      if(obstacle_scale_ > 0)
      {
        float min_depth = getMinDistance(pose);
        
        double cost;
        if(min_depth < desired_freespace_)
        {
          cost = desired_freespace_ - min_depth;
        }
              
        cost *= obstacle_scale_;
        
        ROS_DEBUG_STREAM_NAMED("obstacle_cost", "Min_depth=" << min_depth << ", desired_freespace=" << desired_freespace_ << ", cost=" << cost);
        
        return cost;
      }
      else
      {
        ROS_DEBUG_STREAM_NAMED("obstacle_cost", "obstacle_scale=" << obstacle_scale_);
        return 0;
      }
        
    }
    
    
  public:
    
    virtual bool scoreTrajectory(TrajectoryWrapper::Ptr traj)
    {
      auto pose_array = traj->getPoseArray();
            
      if(pose_array.poses.size() > 0)
      {
        float best_cost = std::numeric_limits<float>::max();
        unsigned int best_ind=0;
        float max_obstacle_cost = 0;
        
        for(unsigned int ind=0; ind < pose_array.poses.size(); ++ind)
        {
          const auto& pose = pose_array.poses[ind];
          float dist = getGoalDist(pose);
          float heightcost = getHeightCost(pose);
          float obstcost = getObstacleCost(pose);
          
          max_obstacle_cost = std::max(obstcost, max_obstacle_cost);
          
          float cost = dist + heightcost + max_obstacle_cost;
          
          if(cost >= std::numeric_limits<float>::max())
          {
            ROS_DEBUG_STREAM("Cost is too high, stop analyzing trajectory here (at index " << ind << ")");
            
            break;
          }
          
          if(cost < best_cost)
          {
            best_cost = cost;
            best_ind = ind;
          }
        }
        
        ROS_DEBUG_STREAM_NAMED("trajectory_scoring.scoring", "best_ind=" << best_ind);
        
        auto costs = std::make_shared<BasicTrajectoryCosts>();
        costs->single_cost = best_cost;
        
        traj->costs = costs;
        traj->trim(best_ind+1);
        
        return true;
      }
      else
      {
        ROS_WARN("Pose Array contains 0 poses!");
        return false;
      }
    }
    
    using Ptr=std::shared_ptr<GlobalGoalTrajectoryScoring>;
  };
  
  } //end namespace trajectory_based_nav
  
  
#endif //NAV_GENERAL_GLOBAL_GOAL_NAV_H
