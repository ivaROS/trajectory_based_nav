#ifndef TRAJECTORY_BASED_NAV_GENERAL_NAV_IMPL_BEHAVIOR_H
#define TRAJECTORY_BASED_NAV_GENERAL_NAV_IMPL_BEHAVIOR_H
  
#include <trajectory_based_nav/general_nav_impl.h>
#include <trajectory_based_nav/behavior_manager.h>

namespace trajectory_based_nav
{
    
    template<typename P, typename T=typename P::msg_type>
    class GeneralNavImplBehavior : public Behavior, public GeneralNavImpl<T>
    {
    public:
        GeneralNavImplBehavior(std::string name, ros::NodeHandle nh, ros::NodeHandle pnh, typename std::shared_ptr<P> planner):
            trajectory_based_nav::Behavior(name),
            GeneralNavImpl<T>(planner),
            planner_(planner),
            //is_planning_(false),
            should_plan_(false)
        {
            planner_->init(nh, pnh);
            GeneralNavImpl<T>::init(nh);
        }
        
        //Behavior methods:
        virtual bool start() override
        {
            {
                ScopedLock lock(planning_mutex_);
                should_plan_ = true;
            }
            planner_->enablePlanning();
            
            ROS_INFO_STREAM("[" << name_ << "] started!");
            
            return true;
        }
        
        //The planner must be in a 'stopped' state by the time this function returns
        virtual bool stop() override
        {
            {
                ScopedLock lock(planning_mutex_);   //Wait for planning to finish
                should_plan_ = false;   //Ensures planning does not proceed even if a callback has just started
            }
            planner_->disablePlanning();
            ROS_INFO_STREAM("[" << name_ << "] stopped!");
            return true;
        }
        
        //GeneralNavImpl methods:
        virtual void Plan() override
        {
            ScopedLock lock(planning_mutex_);
        
            if(!should_plan_)
            {
                ROS_INFO_STREAM("Aborted plan callback due to in-progress stop(). This should probably be a very rare occurence; if you see this message frequently, double check your code design");
                return;
            }
            GeneralNavImpl<T>::Plan();
        }
        
        
    public:
        using Ptr = std::shared_ptr<GeneralNavImplBehavior<P,T> >;
        
        std::shared_ptr<P> planner_;
        
    protected:
        
        
        
        using Mutex = boost::mutex;
        using UniqueLock = boost::unique_lock<Mutex>;
        using ScopedLock = Mutex::scoped_lock;
        
        Mutex planning_mutex_;
        
        bool should_plan_;
        
        boost::condition_variable_any planning_cond_;
        
        
    };
    
    
    template <typename Q, typename P=typename Q::element_type, typename T=typename P::msg_type>
    auto MakeGeneralNavImplBehavior(std::string name, ros::NodeHandle nh, ros::NodeHandle pnh, Q planner) -> decltype(std::make_shared<GeneralNavImplBehavior<P,T> >(name, nh, pnh, planner))
    {
        return std::make_shared<GeneralNavImplBehavior<P,T> >(name, nh, pnh, planner);
    }

}   //end namespace trajectory_based_nav

#endif //TRAJECTORY_BASED_NAV_GENERAL_NAV_IMPL_BEHAVIOR_H