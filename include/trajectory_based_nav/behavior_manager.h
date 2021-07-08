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

class BehaviorManager;

class Behavior
{
public:
    Behavior(std::string name):
        is_active_(false),
        name_(name)
    {
        
    }
    
    virtual bool activate()
    {
        bool redundant_call=false;
        {
            Lock lock(current_status_mutex_);
            
            if(!is_active_)
            {
                if(start())
                {
                    is_active_ = true;
                    return true;
                }
                else 
                {
                    return false;
                }
            }
            else
            {
                redundant_call = true;
            }
        }
        
        if(redundant_call)
        {
            ROS_WARN_STREAM("[" << name_ << "] was already active!");
            return true;
        }
    }
    
    virtual bool deactivate()
    {
        bool redundant_call=false;
        {
            Lock lock(current_status_mutex_);
            if(is_active_)
            {
                if(stop())
                {
                    is_active_ = false;
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                redundant_call = true;
            }
        }
        if(redundant_call)
        {
            ROS_WARN_STREAM("[" << name_ << "] was already inactive!");
            return true;
        }
    }
    
    virtual bool is_active()
    {
        Lock lock(current_status_mutex_);
        return is_active_;
    }
    
    const std::string& name() {return name_;}
    
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

/*
class ThreadSafeDeque
{
public:
    bool getNextRequest(BehaviorRequest& request)
    {
        Lock lock(request_mutex_);
        if(requests_.size() > 0)
        {
            request = requests_.front();
            requests_.pop_front();
            return true;
        }
        return false;
    }
    
    void addRequest(BehaviorRequest& request)
    {
        //TODO: finish?
    }
    
protected:
    std::deque<BehaviorRequest> requests_;
    
    using Mutex = boost::mutex;    
    using Lock = boost::scoped_lock<Mutex>;
    Mutex request_mutex_;
};*/


class BehaviorManager
{
public:
    
    BehaviorManager(std::string name):
        name_(name)
    {
        requests_thread_ = std::make_shared<boost::thread>(boost::bind(&BehaviorManager::requestsThread, this));
        
    }
    
    ~BehaviorManager()
    {
        requests_thread_->interrupt();
        requests_thread_->join();
    }
    
    bool addRequest(BehaviorRequest request)
    {
        std::vector<BehaviorRequest> requests {request};
        return addRequests(requests);
    }
    
    bool addRequests(const std::vector<BehaviorRequest>& requests)
    {
        ScopedLock lock(request_list_mutex_);
        requests_.insert(requests_.end(), requests.begin(), requests.end());
        request_cond_.notify_one();
        return true;
    }
    
protected:
    void requestsThread()
    {
        /* this thread is responsible for actually changing states?
         * If a behavior finishes/fails, it calls ?(pause?)/deactivate?
         * Once this is done, it sends a signal (notify_one/all?) that
         * results in this thread checking that it is paused/stopped/etc, and 
         * 
         * 
         * Or, a behavior receives a request to activate (either from itself or elsewhere),
         * which blocks on some mutex/condition.
         * and when 
         * 
         * 
         * Ok, so a behavior has its own thread to manage async communication with other modules,
         * and the thread blocks on a condition variable, and tests whether it should be active or not (should_activate).
         * 
         * a behavior's activate method sets the should_activate variable to true (mutex proteted)
         * 
         * when a behavior fails/finishes, it calls its deactivate method, which calls the suspend function (to stop whatever
         * actions it was doing) and then releases
         * 
         * 
         * 
         * Basically: does a behavior trigger/activate another behavior, or does the manager handle that?
         * For behaviors to be general/reusable, they can't be hardcoded with eachother in mind, so the transitions need
         * to be specified at higher/different location
         * 
         * 
         * when a behavior fails/finishes, it needs to clean up itself, update its activated/running/etc state, and send a signal finished.
         * 
         * elsewhere (in the manager's thread), that signal causes a check to be performed, which sees that the 'active' behavior is not longer active.
         * It then looks up which (if any) behavior to activate and calls that behaviors activate method.
         * 
         * A behavior can call its own deactivate method directly as long as it is from the same thread used for behavior action stuff (whether that is
         * a dedicated spinning thread or a callback thread), but other threads must call the public requestDeactivate/stop method.
         * The request method must not block unreasonably (eg. any mutexes required should only be held long enoough to update a status variable, not to perform actual work)
         * The behavior is responsible for calling deactivate at a suitable time, which may depend on other, longer held mutexes. One option is to use a timer to periodically 
         * check if the necessary conditions are satisfied to deactivate... though a condition variable might be better for this too? Ex: at end of each execution, sends signal to 
         * conditon variable which checks if the....
         *
         * This should probably be made into a provided feature by the behavior class, where the condition is specified
         * by a virtual function.
         * The main thing is ensuring that the condition will become true, that if the behavior isn't doing anything, it evalutes as true, etc.
         * 
         * 
         */
        
        
        ros::NodeHandle nh;
        
        bool wait_for_wake = true;
        UniqueLock lock(request_list_mutex_);
        
        while(nh.ok())
        {
            //Wait for there to be a request to process
            while(requests_.empty())
            {
                //if we should not be running the planner then suspend this thread
                ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Requests thread suspending, waiting for requests...");
                //ROS_DEBUG_NAMED(name_,"Monitor thread is suspending");
                request_cond_.wait(lock);
                ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Requests thread woke up!");
            }
            BehaviorRequest request = requests_.front();
            requests_.pop_front();
            
            lock.unlock();
            
            ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Received a request to process!");
            
            //Perform the action requested
            
            ros::Time start_time = ros::Time::now();
            
            if(request.should_be_active)
            {
                ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Preparing to activate behavior [" << request.behavior->name() << "]...");
                if(request.behavior->activate())
                {
                    ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Successfully activated behavior [" << request.behavior->name() << "]...");
                }
                else
                {
                    ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Error when attempting to activate behavior [" << request.behavior->name() << "]...");
                }
            }
            else
            {
                ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Preparing to deactivate behavior [" << request.behavior->name() << "]...");
                if(request.behavior->deactivate())
                {
                    ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Successfully deactivated behavior [" << request.behavior->name() << "]...");
                }
                else
                {
                    ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Error when attempting to deactivate behavior [" << request.behavior->name() << "]...");
                }
            }
            
            lock.lock();
        }
    }
    
protected:
    
    using Mutex = boost::mutex;
    using UniqueLock = boost::unique_lock<Mutex>;
    using ScopedLock = Mutex::scoped_lock;
    
    Mutex request_list_mutex_;
    std::deque<BehaviorRequest> requests_;
    
    //, current_status_mutex_;
    //bool should_activate_, should_deactivate_;
    
    boost::condition_variable_any request_cond_;
    
    std::shared_ptr<boost::thread> requests_thread_;
    
    std::string name_;
/*    
    std::vector<Behavior::Ptr> behaviors_;
    Behavior::Ptr active_behavior_;*/
    
    
};

}   //end namespace trajectory_based_nav

#endif //TRAJECTORY_BASED_NAV_BEHAVIOR_MANAGER_H
