#ifndef TRAJECTORY_BASED_NAV_BEHAVIOR_MANAGER_H
#define TRAJECTORY_BASED_NAV_BEHAVIOR_MANAGER_H

#include <boost/thread/mutex.hpp>
//#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <deque>

namespace trajectory_based_nav
{

class Behavior
{
public:
    Behavior(std::string name):
        is_active_(false),
        name_(name)
    {
        
    }
    
    virtual ~Behavior() = default;
    
    virtual bool activate();
    
    virtual bool deactivate();
    
    //TODO: make const
    virtual bool is_active();
    
    const std::string& name() const {return name_;}
    
protected:
    virtual bool start() {return true;}
    virtual bool stop() {return true;}
    
    //virtual void pause() {}
    //virtual void resume() {}
    
    
protected:
    using Mutex = boost::mutex;
    using Lock = Mutex::scoped_lock;
    
    Mutex current_status_mutex_;    //
    bool is_active_;
    
    std::string name_;
    
    
public:
    using Ptr = std::shared_ptr<Behavior>;
};

/*
// Allows safely activating/deactivating the behavior in a threadsafe manner
template <typename T>
class BehaviorInterface
{
public:
    BehaviorInterface(T&& behavior):
        behavior_(behavior),
        name_(behavior_.name() + "_interface"),
        is_active_(false),
        should_activate_(false),
        should_deactivate_(false)
    {
        monitor_thread_ = std::make_shared<boost::thread>(boost::bind(&BehaviorInterface<T>::monitorThread, this));
        
    }
    
    virtual void requestActivate()
    {
        ROS_WARN_NAMED_COND(should_activate_, name_ + ".request_activate", "'should_activate_' is already `true`; do you really  mean to call 'requestActivate' again?");
        ROS_WARN_NAMED_COND(should_deactivate_, name_ + ".request_activate", "'should_deactivate_' is still `true`; do you really  mean to call 'requestActivate' now?");
        
        ROS_INFO_STREAM_NAMED(name_ + ".request_activate", "Request activation of [" << behavior_.name << "]!");
        should_activate_ = true;
        request_cond_.notify_one();
    }
    
    virtual void requestDeactivate()
    {
        ROS_WARN_NAMED_COND(should_deactivate_, name_ + ".request_deactivate", "'should_deactivate_' is already `true`; do you really  mean to call 'requestDeactivate' again?");
        ROS_WARN_NAMED_COND(should_activate_, name_ + ".request_deactivate", "'should_activate_' is still `true`; do you really  mean to call 'requestDeactivate' now?");
        
        ROS_INFO_STREAM_NAMED(name_ + ".request_deactivate", "Request deactivation of [" << behavior_.name << "]!");
        should_deactivate_ = true;
        request_cond_.notify_one();
    }
    
    bool waitForActivation()
    {
        bool wait_for_wake = true;
        UniqueLock lock(current_status_mutex_);

        while(wait_for_wake || !is_active_)
        {
            //if we should not be running the planner then suspend this thread
            ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Waiting for activation...");
            //ROS_DEBUG_NAMED(name_,"Monitor thread is suspending");
            status_cond_.wait(lock);
            ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Status received signal...");
            wait_for_wake = false;
        }
        
        lock.unlock();
        
        ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Done waiting for activation!");
        return true;
    }
    
    bool waitForDeactivation()
    {
        bool wait_for_wake = true;
        UniqueLock lock(current_status_mutex_);
        
        while(wait_for_wake || is_active_)
        {
            //if we should not be running the planner then suspend this thread
            ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Waiting for deactivation...");
            //ROS_DEBUG_NAMED(name_,"Monitor thread is suspending");
            status_cond_.wait(lock);
            ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Status received signal...");
            wait_for_wake = false;
        }
        
        lock.unlock();
        
        ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Done waiting for activation!");
        return true;
    }
    
    
protected:
    
    virtual void monitorThread()
    {
        ros::NodeHandle nh;
        
        bool wait_for_wake = true;
        UniqueLock lock(request_status_mutex_);
        
        while(nh.ok())
        {
            //check if we should run the planner (the mutex is locked)
            while(wait_for_wake || (!should_activate_ && !should_deactivate_))
            {
                //if we should not be running the planner then suspend this thread
                ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Monitor thread suspending...");
                //ROS_DEBUG_NAMED(name_,"Monitor thread is suspending");
                request_cond_.wait(lock);
                ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Monitor thread woke up!");
                wait_for_wake = false;
            }
            ros::Time start_time = ros::Time::now();
            
            
            ROS_ASSERT_MSG(!(should_activate_ && should_deactivate_), "Should you really be trying to activate and deactivate at the same time like this?");

            if(should_activate_)
            {
                should_activate_ = false;
                lock.unlock();
                ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Preparing to activate behavior...");
                
                behavior_.activate();
                
                {
                    ScopedStatusLock status_lock(current_status_mutex_);
                    is_active_ = true;
                }
                status_cond_.notify_one();
            }
            else if(should_deactivate_)
            {
                should_deactivate_ = false;
                lock.unlock();
                ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Preparing to deactivate behavior...");
                
                behavior_.deactivate();
                {
                    ScopedStatusLock status_lock(current_status_mutex_);
                    is_active_ = false;
                }
                status_cond_.notify_one();
            }
            else
            {
                lock.unlock();
                ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] No action requested");
            }
            
            lock.lock();
            wait_for_wake = true;
        }
    }
    
protected:
    T behavior_;
    std::string name_;
    bool is_active_;
    
    using RequestMutex = boost::mutex;
    using UniqueLock = boost::unique_lock<RequestMutex>;
    
    using ScopedStatusLock = boost::scoped_lock<RequestMutex>;
    Mutex request_status_mutex_, current_status_mutex_;
    bool should_activate_, should_deactivate_;
    
    boost::condition_variable_any request_cond_, status_cond_;
    
    std::shared_ptr<boost::thread> monitor_thread_;
    
    
};

//Behaviors that can request that they be activated
class PreemptingBehavior : public Behavior
{
    
    
    
    
    
};*/

class TestBehavior : public Behavior
{
public:
    TestBehavior(std::string name):
        Behavior(name)
    {}
    
    virtual ~TestBehavior() = default;
    
    virtual bool start() override
    {
        ROS_INFO_STREAM("[" << name_ << "] started!");
        return true;
    }
    
    virtual bool stop() override
    {
        ROS_INFO_STREAM("[" << name_ << "] stopped!");
        return true;
    }
};


struct BehaviorRequest
{
    BehaviorRequest(){}
    
    BehaviorRequest(Behavior::Ptr behavior, bool should_be_active):
        behavior(behavior),
        should_be_active(should_be_active)
        {}
    
    Behavior::Ptr behavior;
    bool should_be_active;
};


class BehaviorManager
{
public:
    
    BehaviorManager(std::string name);
    
    ~BehaviorManager();
    
    bool addRequest(BehaviorRequest request);
    
    bool addRequests(const std::vector<BehaviorRequest>& requests);
    
protected:
    void requestsThread();
    
protected:
    
    using Mutex = boost::mutex;
    using UniqueLock = boost::unique_lock<Mutex>;
    using ScopedLock = Mutex::scoped_lock;
    
    Mutex request_list_mutex_;
    std::deque<BehaviorRequest> requests_;

    boost::condition_variable_any request_cond_;
    
    std::shared_ptr<boost::thread> requests_thread_;
    
    std::string name_;
    
};

}   //end namespace trajectory_based_nav

#endif //TRAJECTORY_BASED_NAV_BEHAVIOR_MANAGER_H
