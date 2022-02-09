#ifndef TRAJECTORY_BASED_NAV_INTERFACES_H
#define TRAJECTORY_BASED_NAV_INTERFACES_H

//#include <nav_quadrotor/rotors_control/extendable_lee_position_controller_node.h>
//#include <pips_egocylindrical/egocylindrical_image_cc_wrapper.h>
#include <pips_trajectory_testing/pips_cc_wrapper.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
//#include <tf2_utils/odom_to_tf.h>
//#include <nav_quadrotor/TrajectoryArray.h>
//#include <message_filters/cache.h>
//#include <message_filters/subscriber.h>
//#include <nav_quadrotor/global_potentials_interface.h>
//#include <nav_quadrotor/ros_interface_se2_z.h>
//#include <benchmarking_tools/benchmarking_tools.h>
//#include <pips_msgs/PathArray.h>
#include <ros/ros.h>
#include <memory>
#include <type_traits>

namespace trajectory_based_nav
{

  //struct Trajectory;  //For the most part, everything should be generic/reuseable, the only domain-specific part would be the actual trajectory format (conveniently, the type that my trajectory generation is also based on at a certain level)
  
  //TODO: try templating obstacle avoidance based on the types provided by local planner
  
  //TODO: Possibly remove most of the 'inits' from interfaces and make local planner responsible for initing them all; this is in part due to the possibility of some overlap and something being inited twice. Leaving it in interface for now, but will only call from local planner
  
  //using Trajectory = trajectory_msgs::MultiDOFJointTrajectory;

  struct TrajectoryCosts
  {
    virtual double cost() const=0;
    
    //Methods that need to be defined in global namespace
    virtual std::ostream& print(std::ostream& stream) const
    {
      stream << "Total Cost=[" << cost() << "]";
      return stream;
    }

    using Ptr=std::shared_ptr<TrajectoryCosts>;
  };
  
  //Methods that need to be defined in global namespace
  inline
  std::ostream& operator<< (std::ostream& stream, const TrajectoryCosts& costs)
  {
    return costs.print(stream);
  }
  
//   template <class T, typename std::enable_if<std::is_base_of<TrajectoryCosts, T>::value>::type* = nullptr>
//   std::ostream& operator<< (std::ostream& stream, const T& costs)
//   {
//     return costs.print(stream);
//   }
//   
//   template <class T, typename std::enable_if<std::is_base_of<TrajectoryCosts, T>::value>::type* = nullptr>
//   std::ostream& operator<< (std::ostream& stream, const T* costs)
//   {
//     return costs->print(stream);
//   }
  
  struct CollisionCheckingResult
  {
    //virtual bool isSafe(int start, int end)=0;
    virtual int collision_ind()=0;
    
    using Ptr=std::shared_ptr<CollisionCheckingResult>;
  };
  
  struct TrajectoryWrapper //holds trajectory as well as other useful info derived from it
  {
    TrajectoryCosts::Ptr costs;
    CollisionCheckingResult::Ptr collision_check_res;
    bool to_goal=false; //TODO: move this to derived 'goal-based' trajectory wrapper? 
    virtual double cost() {return getCosts()->cost();}
    virtual TrajectoryCosts::Ptr getCosts()
    {
      return costs;
    }
    
    virtual bool trim(unsigned int size)=0;
    
    virtual geometry_msgs::PoseArray getPoseArray() const =0;
    virtual std::vector<ros::Duration> getDurations() const =0;
    virtual std::vector<geometry_msgs::Twist> getTwists() const =0;
    
    virtual geometry_msgs::PoseArray getPoseArray() =0;
    virtual std::vector<ros::Duration> getDurations() =0;
    virtual std::vector<geometry_msgs::Twist> getTwists() =0;
    using Ptr=std::shared_ptr<TrajectoryWrapper>;
  };
  
  template<typename T>
  struct TypedTrajectoryWrapper : public TrajectoryWrapper
  {
    virtual typename T::Ptr getTrajectoryMsg() const =0;
    
    using Ptr=std::shared_ptr<TypedTrajectoryWrapper<T> >;
  };


  template<typename T>
  class TrajectorySource
  {
  public:
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm){ return true; }
    virtual bool update(const nav_msgs::Odometry& odom, const TypedTrajectoryWrapper<T>& current_trajectory)=0;
    virtual bool hasMore() const =0;
    virtual typename TypedTrajectoryWrapper<T>::Ptr getNext()=0;
    
    //virtual TypedTrajectoryWrapper<T>::Ptr getTrajectoryWrapper(const mav_msgs::EigenTrajectoryPointDeque& commands, const std::deque<ros::Duration>& command_waiting_times)=0; //This is encapsulation-breaking; there needs to be a specific level of abstraction at which the trajectory type/controller type is considered
    
    using Ptr=std::shared_ptr<TrajectorySource>;
  };
  
  
  /*
  class GeneratorTrajectorySource : public TrajectorySource
  {
    
  };
  */
  
  
  class TrajectoryController
  {
  public:
    virtual nav_msgs::Odometry::ConstPtr getCurrentOdom()=0;
    virtual void stop(bool force_stop=false)=0;
    virtual void enable() {}
    virtual void disable() {}
    
    using Ptr=std::shared_ptr<TrajectoryController>;
  };
  
  template<typename T>
  class TypedTrajectoryController : public TrajectoryController
  {
  public:
    using msg_type = T;
    virtual void setTrajectory(const typename T::ConstPtr& traj)=0;
    virtual typename TypedTrajectoryWrapper<T>::Ptr getCurrentTrajectory()=0;
    
    using Ptr=std::shared_ptr<TypedTrajectoryController<T> >;
  };
  
  class PlanningData;
  class ReplanLogic
  {
  protected:
    std::string name_;
    TrajectoryWrapper::Ptr current_trajectory_;
    
    ros::Time last_planning_time_;
    ros::Duration max_replan_period_;
    
  public:
    ReplanLogic(std::string name="replan_logic"): name_(name){}
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm){ return true; }
    
    virtual void setTrajectory(TrajectoryWrapper::Ptr traj) { current_trajectory_ = traj; }
    virtual bool shouldReplan(const nav_msgs::Odometry& odom, TrajectoryWrapper::Ptr traj)=0;
    virtual void setPlanningData(const PlanningData& data) {}
    
    using Ptr=std::shared_ptr<ReplanLogic>;
    
  };
  
  class TerminationCriteria
  {
  public:
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm){ return true; }
    virtual bool shouldTerminate(const nav_msgs::Odometry& odom)=0;
    
    using Ptr=std::shared_ptr<TerminationCriteria>;
    
  };

//   class TrajectoryMonitor
//   {
//   protected:
//     TrajectoryWrapper::Ptr current_trajectory_;
//     std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc_wrapper_;
//     
//   public:
//     virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)={};
//     
//     virtual int checkTrajectory(const nav_msgs::Odometry& odom, TrajectoryWrapper::Ptr traj);
//     
//     void setCC(std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc)
//     {
//       cc_wrapper_=cc;
//     }
//     
//     using Ptr=std::shared_ptr<TrajectoryMonitor>;
//   };
  
  class TrajectoryScoring 
  {
  public:
    virtual bool scoreTrajectory(TrajectoryWrapper::Ptr traj)=0;
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm){ return true; };
    
    virtual bool update(const nav_msgs::Odometry& odom) { return true; }
    using Ptr=std::shared_ptr<TrajectoryScoring>;
  };
  
  class TrajectoryVerifier
  {
  protected:
    std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc_wrapper_;
    
  public:
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm){ return true; };
    
    virtual bool update(const nav_msgs::Odometry& odom);
    virtual bool verifyTrajectory(TrajectoryWrapper::Ptr traj)=0;
    virtual int collisionCheckTrajectory(TrajectoryWrapper::Ptr traj);
    
    void setCC(std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc) { cc_wrapper_=cc; }
    
    using Ptr=std::shared_ptr<TrajectoryVerifier>;
  };
  
  
  using NoArgsCallback = boost::function<void() >;
  
  template<typename T>
  class NavImpl
  {
  public:
    std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc_wrapper_; //just temporary
    
  public:
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm){ return true; };
    
    virtual bool update(const nav_msgs::Odometry& odom)=0;
    
    virtual typename TypedTrajectoryController<T>::Ptr getTrajectoryController()=0;
    virtual typename TrajectorySource<T>::Ptr getTrajectorySource()=0;
    virtual ReplanLogic::Ptr getReplanLogic()=0;
    virtual TrajectoryScoring::Ptr getTrajectoryScoring()=0;
    virtual TerminationCriteria::Ptr getTerminationCriteria()=0;
    virtual TrajectoryVerifier::Ptr getTrajectoryMonitor()=0;
    virtual TrajectoryVerifier::Ptr getTrajectoryVerifier()=0;
    
    virtual void setPlanningCB(NoArgsCallback planning_cb)
    {
        planning_cb_ = planning_cb;
    }
    
    virtual void enablePlanning()=0;
    virtual void disablePlanning()=0;
    
    
    //virtual TrajectoryWrapper::Ptr getTrajectory(const nav_msgs::Odometry& odom)=0;
    
    using Ptr=std::shared_ptr<NavImpl>;
    using msg_type = T;
  protected:
      NoArgsCallback planning_cb_;
  };
  
  
  
//   class CollisionCheckingTrajectoryMonitor : public TrajectoryMonitor
//   {
//   protected:
//     std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc_wrapper_;
//     
//     void checkTrajectory(const nav_msgs::Odometry& odom);
//     
//   public:
//     using Ptr=std::shared_ptr<CollisionCheckingTrajectoryMonitor>;
//     
//   };
  
/* //EndPoint-based implementation 
  class EndPointSource
  {
  public:
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh)={};
    virtual bool update(const nav_msgs::Odometry& odom)=0;
    virtual bool hasMore()=0;
    virtual TrajectoryWrapper::Ptr getNext()=0;
    
    using Ptr=std::shared_ptr<EndPointSource>;
  };
  
  class EndPointTrajectorySource : public TrajectorySource
  {
    std::shared_ptr<EndPointSource> source_;
  public:
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh) {source_->init(nh, pnh);}
    virtual bool update(const nav_msgs::Odometry& odom) {source_->update(odom);}
    virtual bool hasMore()=0;
    virtual TrajectoryWrapper::Ptr getNext()=0;
    
    using Ptr=std::shared_ptr<EndPointTrajectorySource>;
  };*/
  
  
  /*
    
    class TrajectoryScoringFunction
    {
    public:
      virtual void init(ros::NodeHandle nh, ros::NodeHandle pnh) {}
      
      virtual void update() {}  //What might generally be needed here? current pose?
      
      virtual float score(TrajectoryWrapper)=0;
      
    protected:
      //getParam()
    };*/
    
} //end namespace trajectory_based_nav

#endif //TRAJECTORY_BASED_NAV_INTERFACES_H
