#ifndef TRAJECTORY_BASED_NAV_GLOBAL_PLANNING_INTERFACE_H
#define TRAJECTORY_BASED_NAV_GLOBAL_PLANNING_INTERFACE_H

#include <trajectory_based_nav/global_goal_nav.h>
#include <tf/transform_listener.h>
#include <tf2_utils/transform_manager.h>
#include <costmap_2d/costmap_2d_ros.h>
//#include <trajectory_based_nav/default_implementations.h>
//#include <trajectory_based_nav/interfaces.h>
#include <boost/thread/mutex.hpp>
#include <tf2_ros/message_filter.h>
//#include <pips/utils/param_utils.h>
//#include <condition_variable>


namespace trajectory_based_nav
{

  /* Usage:
   * 
   * GlobalGoalState needs some slight modifications to ensure that there is a consistent 'currentGoal' throughout 
   * a single planning cycle
   * There may be some use in having a 'newerGoalAvailable' function
   * Goal should be updated when 'update' called
   * Should possibly indicate goal is new for entire planning cycle?
   * Or just call global planning if goal is new?
   * Ideally, new goal should be transformed as necessary and sent to global planning ASAP
   * 
   * call it with a goal (in the planning frame)
   * getCurrentPlan() returns planning result structure for current goal
   * 
   */
  
  template <typename T>
  class GlobalPlannerInterface
  {
  public:
    using planner_result_t = T;
    
    //NOTE: Poses must be provided in the frame given by getPlanningFrameID()
    virtual T plan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal)=0;
    
    virtual std::string getPlanningFrameID()=0;
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm) {};
    
    virtual bool getRobotPose(geometry_msgs::PoseStamped& start)=0;
    
  };
  
  
  template <typename T>
  class CostmapBasedGlobalPlannerInterface : GlobalPlannerInterface<T>
  {
  protected:
    tf::TransformListener tf_;  //Newer versions of navigation packages make the switch to tf2_ros, but holding off on upgrading for now
    costmap_2d::Costmap2DROS costmap_;
    
  public:
    //using planner_result_t = T;
    
    CostmapBasedGlobalPlannerInterface():
      tf_(),
      costmap_("global_costmap", tf_)
      {}
      
    virtual std::string getPlanningFrameID()
    {
      return costmap_.getGlobalFrameID();
    }
    
    virtual bool getRobotPose(geometry_msgs::PoseStamped& start)
    {
      //I'd still rather use odometry msg, but this may be easier for now
      tf::Stamped<tf::Pose> global_pose;
      if(!costmap_.getRobotPose(global_pose)) {
        ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
        return false;
      }
      
      tf::poseStampedTFToMsg(global_pose, start);
    }
  };
  
  
  

  class GlobalPlanningInterface
  {
  public:
    using Ptr = std::shared_ptr<GlobalPlanningInterface>;
    
  protected:
    geometry_msgs::PoseStamped::ConstPtr goal_pose_;
    
    
    GlobalGoalState::Ptr ggs_;
    std::string name_;
    
    using GoalTFFilter = tf2_ros::MessageFilter<geometry_msgs::PoseStamped>;
    std::shared_ptr<GoalTFFilter> goal_filter_;
    
    //typedef boost::mutex::scoped_lock Lock;
    //boost::mutex goal_mutex_;
    
    //geometry_msgs::TransformStamped goal_to_planning_transform_;
    
    ros::NodeHandle nh_, pnh_;
    tf2_utils::TransformManager tfm_;
    
  public:
    GlobalPlanningInterface(const GlobalGoalState::Ptr& ggs, std::string name="global_planning_interface"):
      ggs_(ggs),
      name_(name)
    {      
      
    }
    
    
  public:
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      tfm_ = tfm;
      nh_ = nh;
      pnh_ = ros::NodeHandle(pnh, name_);
      
      goal_filter_ = std::make_shared<GoalTFFilter>(ggs_->getGoalSource(), *tfm_.getBuffer(), getPlanningFrameID(), 2, nh_);
      goal_filter_->registerCallback(boost::bind(&GlobalPlanningInterface::goalCB, this, _1));
      
      return true;
    }

    //NOTE: Goal may be in any coordinate frame
    //TODO: Ideally, don't wait for transform before clearing current plan/goal information
    //TODO: Low priority, but if planning in progress when new goal comes in, could start new thread to service it and kill of old thread
    virtual void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
    {
      ROS_INFO_STREAM("[GlobalPlanningInterface] received new goal: " << *goal);
      geometry_msgs::PoseStamped transformed_goal = tfm_.getBuffer()->transform(*goal, getPlanningFrameID());
      clearPlan();
      setNewGoal(transformed_goal);
      triggerPlanning();
    }
    
    //NOTE: Not sure if this is necessary or not
    virtual bool update(const nav_msgs::Odometry& odom)
    {
      return true;
    };
    
    virtual bool triggerPlanning()=0;
    
    virtual bool triggerPlanning(const ros::TimerEvent& event)
    {
      return triggerPlanning();
    }
    
    virtual std::string getPlanningFrameID()=0;
    
    virtual void clearPlan()=0;
    
    virtual void setNewGoal(const geometry_msgs::PoseStamped& goal)=0;
//     {
//       Lock lock(goal_mutex_);
//       goal_pose_ = boost::make_shared<geometry_msgs::PoseStamped>(goal);
//       
//     }
    
  };
  
  
  
  
  template<typename T>
  class TypedGlobalPlanningInterface : public GlobalPlanningInterface
  {
  public:
    using Ptr = std::shared_ptr<TypedGlobalPlanningInterface<T> >;
    
  protected:
    typename T::Ptr planner_;
    using planner_result_t = typename T::planner_result_t;
    
    typedef boost::mutex::scoped_lock Lock;
    using PlanningLock = boost::recursive_mutex::scoped_lock;
    
    boost::mutex current_plan_mutex_;
    boost::recursive_mutex planning_mutex_;
    boost::condition_variable_any planner_cond_;
    
    bool run_planner_;
    bool planning_in_progress_;
    
    nav_msgs::Odometry odom_;
    
    std::shared_ptr<planner_result_t> current_plan_;
    
    std::shared_ptr<boost::thread> planning_thread_;
    
  public:
    
    TypedGlobalPlanningInterface(const GlobalGoalState::Ptr& ggs, const typename T::Ptr& planner, std::string name):
      GlobalPlanningInterface(ggs, name),
      planner_(planner),
      run_planner_(false),
      planning_in_progress_(false)
    {
      planning_thread_ = std::make_shared<boost::thread>(boost::bind(&TypedGlobalPlanningInterface<T>::planningThread, this));
    }
    
    ~TypedGlobalPlanningInterface()
    {
      planning_thread_->interrupt();
      planning_thread_->join();
    }
    
    virtual std::string getPlanningFrameID() override
    {
      return planner_->getPlanningFrameID();
    }
    
    virtual void setNewGoal(const geometry_msgs::PoseStamped& goal) override
    {
      PlanningLock lock(planning_mutex_);
      goal_pose_ = boost::make_shared<geometry_msgs::PoseStamped>(goal);
    }
    
    virtual void clearPlan() override
    {
      Lock lock(current_plan_mutex_);
      
      current_plan_ = nullptr;
    }
    
    /*NOTE: On second thought, there's no good reason to block and wait for newer plan. Rather than delaying local planning for global planning, 
     * local planning should just fail, allowing it to try again shortly thereafter but with updated local information.
     */
    virtual typename planner_result_t::ConstPtr getCurrentPlan(bool block=false)
    {
//       if(block)
//       {
//         if(!current_plan_)
//         {
//           triggerPlanning();  //If no plan exists yet for current goal,
//         }
//         {
//           Lock lock(planning_mutex_); //Wait for current planning operation to finish
//         }
//       }
      //NOTE: I'm not sure if this lock is necessary
      {
        Lock lock(current_plan_mutex_);
        return current_plan_;
      }
    }
    
    //TODO: Since global planning occurs asynchronously with everything else, may be better to get current robot pose asynchronously as well
//     virtual bool update(const nav_msgs::Odometry& odom) override
//     {
//       PlanningLock lock(planning_mutex_);  //When planning_mutex_ released, current_plan_ already updated
//       
//       odom_ = odom;
//       
//       return (bool)current_plan_;
//     }
    
    virtual bool triggerPlanning() override
    {
      ROS_INFO("Triggering planning!");
      run_planner_ = true;
      planner_cond_.notify_one();
    }
    
    
    //Based heavily on MoveBase::planThread() https://github.com/ros-planning/navigation/blob/e2e9482695f11d8d30326480e3283967d94d83f5/move_base/src/move_base.cpp#L559
    void planningThread()
    {
      bool wait_for_wake = true;
      
      ROS_INFO("Planning thread started!");
      
      boost::unique_lock<boost::recursive_mutex> lock(planning_mutex_);
      while(nh_.ok()){
        //check if we should run the planner (the mutex is locked)
        while(wait_for_wake || !run_planner_){
          //if we should not be running the planner then suspend this thread
          ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
          planner_cond_.wait(lock);
          wait_for_wake = false;
        }
        ros::Time start_time = ros::Time::now();
        
        ROS_INFO("Planning...");
        
        //time to plan! get a copy of the goal and unlock the mutex
        auto goal = *goal_pose_;
        
        lock.unlock();
        ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");
        
        
        
        
        
        
        geometry_msgs::PoseStamped current_pose;
        if(planner_->getRobotPose(current_pose))
        {
          //current_pose.header = current_odom.header;
          //current_pose.pose = current_odom.pose.pose;
          
          planner_result_t result = planner_->plan(current_pose, goal);
          
          {
            Lock lock(current_plan_mutex_);
            current_plan_ = std::make_shared<planner_result_t>(result);
          }
          ROS_INFO("Updated 'current_plan_'");
          //run planner
          //bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);
        }
        
        lock.lock();
        wait_for_wake = true;
      }
      
    }
  };
  
  
  struct PlanningResult
  {
    public:
//       bool isValid() const=0;
//     
//       virtual geometry_msgs::PoseStamped goal() const=0;
//       
//       virtual geometry_msgs::PoseStamped start() const=0;
      
  };
  
  struct BasicPlanningResult : public PlanningResult
  {
  public:
    bool is_valid=false;
    geometry_msgs::PoseStamped goal, start;
    
//   public:
//     
//     BasicPlanningResult()
//     {}
//     
//     bool isValid() const { return is_valid; }
//     
//     virtual geometry_msgs::PoseStamped goal() const { return goal_; }
//     
//     virtual geometry_msgs::PoseStamped start() const { return start_; }
//     
    
    
  };
  
  struct PathPlanningResult : public BasicPlanningResult
  {
  public:
    std::vector<geometry_msgs::PoseStamped> path;
    
//   public:
//     virtual const std::vector<geometry_msgs::PoseStamped>& getPath() const
//     {
//       return path;
//     }
  };
  
  
  
} //end namespace trajectory_based_nav
  
  
#endif //TRAJECTORY_BASED_NAV_GLOBAL_PLANNING_INTERFACE_H
