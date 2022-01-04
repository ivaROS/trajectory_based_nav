#ifndef NAV_GENERAL_DEFAULT_IMPLEMENTATIONS_H
#define NAV_GENERAL_DEFAULT_IMPLEMENTATIONS_H

#include<trajectory_based_nav/interfaces.h>
#include <numeric> //for accumulate

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
  
  
  class BasicTrajectoryVerifier : public trajectory_based_nav::TrajectoryVerifier
  {
  protected:
    ros::Duration min_ttc_;
    std::string name_;
    
  public:
    
    BasicTrajectoryVerifier(std::string name="verifier"):
      name_(name)
      {}
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm);
    
    virtual bool verifyTrajectory(TrajectoryWrapper::Ptr traj);
    
  };
  
  
  class BasicReplanLogic : public trajectory_based_nav::ReplanLogic
  {
    
  protected:
    trajectory_based_nav::TrajectoryVerifier::Ptr verifier_;
    ros::Duration min_tte_;
    
  public:
    using Ptr = std::shared_ptr<BasicReplanLogic>;
    
  public:
    BasicReplanLogic(trajectory_based_nav::TrajectoryVerifier::Ptr verifier, std::string name="basic_replan_logic");
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm);
    
    virtual bool shouldReplan(const nav_msgs::Odometry& odom, TrajectoryWrapper::Ptr traj);
    
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
