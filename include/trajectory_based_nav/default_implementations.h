#ifndef NAV_GENERAL_DEFAULT_IMPLEMENTATIONS_H
#define NAV_GENERAL_DEFAULT_IMPLEMENTATIONS_H

#include<trajectory_based_nav/interfaces.h>

namespace trajectory_based_nav
{
  
  class BasicCollisionCheckingResult : public trajectory_based_nav::CollisionCheckingResult
  {
  protected:
    int colliding_ind=-1; //-1: 
    
  public:
    BasicCollisionCheckingResult(int ind): colliding_ind(ind)
    {}
    
    
  public:
    virtual int collision_ind()
    {
      return colliding_ind;
    }
    //     virtual bool isSafe(int ind)
    //     {
    //       if(colliding_ind < 0)
    //       {
    //         return true;
    //       }
    //     }
  };
  
  
  
/*  
  class BasicTrajectoryVerifier : public trajectory_based_nav::TrajectoryVerifier
  {
  protected:
    ros::Duration min_ttc_;
    std::string name_;
    
  public:
    
    BasicTrajectoryVerifier(std::string name="basic_trajectory_verifier") : name_(name) {}
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      //min_ttc_ = ros::Duration(3);
      
      auto mnh = ros::NodeHandle(pnh, name_);
      
      double min_ttc=3;
      mnh.getParam("min_ttc", min_ttc);
      mnh.setParam("min_ttc", min_ttc);
      
      min_ttc_ = ros::Duration(min_ttc);
    }
  
  */
  
  
  class BasicTrajectoryVerifier : public trajectory_based_nav::TrajectoryVerifier
  {
  protected:
    ros::Duration min_ttc_;
    std::string name_;
    
  public:
    
    BasicTrajectoryVerifier(std::string name="verifier"):
      name_(name)
      {}
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      auto ppnh = ros::NodeHandle(pnh, name_);
      
      double min_ttc=3;
      ppnh.getParam("min_ttc", min_ttc);
      ppnh.setParam("min_ttc", min_ttc);
      
      min_ttc_ = ros::Duration(min_ttc);
      
      return true;
    }
    
    virtual bool verifyTrajectory(TrajectoryWrapper::Ptr traj)
    {
      int collision_ind = collisionCheckTrajectory(traj);
      
      ROS_INFO_STREAM_NAMED("trajectory_verifier", "[" << name_ << "] collision_ind=" << collision_ind);
      
      traj->collision_check_res = std::make_shared<BasicCollisionCheckingResult>(collision_ind);
      if(collision_ind >=0)
      {
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
    
  };
  
  

  
  
  class BasicReplanLogic : public trajectory_based_nav::ReplanLogic
  {
    
  protected:
    trajectory_based_nav::TrajectoryVerifier::Ptr verifier_;
    ros::Duration min_tte_;
    
  public:
    using Ptr = std::shared_ptr<BasicReplanLogic>;
    
  public:
    BasicReplanLogic(trajectory_based_nav::TrajectoryVerifier::Ptr verifier, std::string name="basic_replan_logic"):
      ReplanLogic(name)
    {
      verifier_ = verifier;
      //min_tte_ = ros::Duration(3);
      //max_replan_period_ = ros::Duration(3);  //TODO: use ros parameters
    }
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
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
    
    virtual bool shouldReplan(const nav_msgs::Odometry& odom, TrajectoryWrapper::Ptr traj)
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
    
  };
  
  
  class WanderTerminationCriteria : public TerminationCriteria
  {
  public:
    virtual bool shouldTerminate(const nav_msgs::Odometry& odom) {return false;}
    
    using Ptr=std::shared_ptr<WanderTerminationCriteria>;
    
  };
  
  
//   class VectorTrajectoryCosts
//   {
//   protected:
//     std::vector<double> costs;
//     double cost;
//     
//   public:
//     virtual double costs()
//   };
  
  class BasicTrajectoryCosts : public TrajectoryCosts
  {
  public:
    double single_cost;
    
  public:
    virtual double cost() { return single_cost;}
    
    using Ptr=std::shared_ptr<BasicTrajectoryCosts>;
    
  };
  
  class TrajectoryScoringFunction
  {
  protected:
    std::string name_;
    
  public:
    virtual double scoreTrajectory(TrajectoryWrapper::Ptr traj)=0;
    
    std::string name() { return name_; }
  };
  
  class TrajectoryCostsContainer : BasicTrajectoryCosts
  {
  protected:
    std::vector<double> costs;
    
  public:
    void update()
    {
      BasicTrajectoryCosts::single_cost = std::accumulate(costs.begin(), costs.end(), 0);
    }
    
    void addCost(double cost, std::string name="") { costs.push_back(cost); }
    
    
    virtual double cost()
    {
      //TODO: Add optional printout here
      return BasicTrajectoryCosts::cost();
    }
  };
  
  class WanderTrajectoryScoring : public TrajectoryScoring
  {
  public:
    virtual bool scoreTrajectory(TrajectoryWrapper::Ptr traj)
    {
      auto costs = std::make_shared<BasicTrajectoryCosts>();
      
      auto poses = traj->getPoseArray().poses;
      auto sp = poses.front().position;
      auto ep = poses.back().position;
      
      auto dx = sp.x-ep.x;
      auto dy = sp.y-ep.y;
      auto dz = sp.z-ep.z;
      
      auto dist = std::sqrt(dx*dx+dy*dy+dz*dz);
      
      costs->single_cost = 1/dist;
      
      traj->costs = costs;
      
      return true;
    }
    
    using Ptr=std::shared_ptr<WanderTrajectoryScoring>;
  };
  
} //end namespace trajectory_based_nav

#endif //NAV_GENERAL_DEFAULT_IMPLEMENTATIONS_H
