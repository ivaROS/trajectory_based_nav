#include <trajectory_based_nav/behavior_manager.h>


namespace trajectory_based_nav
{

    BehaviorManager::BehaviorManager(std::string name):
        name_(name)
    {
        requests_thread_ = std::make_shared<boost::thread>(boost::bind(&BehaviorManager::requestsThread, this));
    }
    
    BehaviorManager::~BehaviorManager()
    {
        requests_thread_->interrupt();
        requests_thread_->join();
    }
    
    bool BehaviorManager::addRequest(BehaviorRequest request)
    {
        std::vector<BehaviorRequest> requests {request};
        return addRequests(requests);
    }
    
    bool BehaviorManager::addRequests(const std::vector<BehaviorRequest>& requests)
    {
        ScopedLock lock(request_list_mutex_);
        requests_.insert(requests_.end(), requests.begin(), requests.end());
        request_cond_.notify_one();
        return true;
    }
    
    void BehaviorManager::requestsThread()
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
            
            //ros::Time start_time = ros::Time::now();
            
            if(request.should_be_active)
            {
                ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Preparing to activate behavior [" << request.behavior->name() << "]...");
                if(request.behavior->activate())
                {
                    ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Successfully activated behavior [" << request.behavior->name() << "]!");
                }
                else
                {
                    ROS_ERROR_STREAM_NAMED(name_, "[" << name_ << "] Error when attempting to activate behavior [" << request.behavior->name() << "]!");
                }
            }
            else
            {
                ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Preparing to deactivate behavior [" << request.behavior->name() << "]...");
                if(request.behavior->deactivate())
                {
                    ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] Successfully deactivated behavior [" << request.behavior->name() << "]!");
                }
                else
                {
                    ROS_ERROR_STREAM_NAMED(name_, "[" << name_ << "] Error when attempting to deactivate behavior [" << request.behavior->name() << "]!");
                }
            }
            
            lock.lock();
        }
    }

}   //end namespace trajectory_based_nav
