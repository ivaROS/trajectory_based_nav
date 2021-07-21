#ifndef TRAJECTORY_BASED_NAV_RECOVERY_BEHAVIOR_H
#define TRAJECTORY_BASED_NAV_RECOVERY_BEHAVIOR_H

#include <trajectory_based_nav/interfaces.h>    //Only needs TrajectoryController interface
#include <ros/console.h>

namespace trajectory_based_nav 
{

    class RecoveryBehavior : public Behavior
    {
    public:
        using NoArgsCallback = boost::function<void() >;
        
    protected:
        NoArgsCallback cb_;
        
    public:
        RecoveryBehavior(std::string name):
            Behavior(name)
        {}
        
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
            
        virtual void setFinishedCB(NoArgsCallback cb)
        {
            cb_ = cb;
        }
        
        virtual void done()
        {
            ROS_INFO_STREAM_NAMED(name_, "[" << name_ << "] done()!");
            if(cb_)
            {
                cb_();
            }
        }
    };

    template <typename T>
    class TypedRecoveryBehavior : public RecoveryBehavior
    {
    public:
        using msg_type = T;
        
    protected:
        typename TrajectoryController<T>::Ptr controller_;
        
    public:
        TypedRecoveryBehavior(std::string name, typename TrajectoryController<T>::Ptr controller):
            RecoveryBehavior(name),
            controller_(controller)
        {
            if(!controller_)
            {
                ROS_ERROR_STREAM_NAMED(name_, "[" << name_ << "] was provided with a null controller!");
            }
        }
        
        virtual bool start() override
        {
            RecoveryBehavior::start();
            controller_->setTrajectory(getTrajectory());
            
            ROS_DEBUG_STREAM_NAMED(name_, "[" << name_ << "] sent a trajectory to the controller!");
            return true;
        }
        
        virtual bool stop() override
        {
            RecoveryBehavior::stop();
            controller_->stop();    //Ensures that robot doesn't continue to move if behavior needs to be terminated early for whatever reason
            
            return true;
        }
        
        virtual typename T::ConstPtr getTrajectory()=0;
        
    };

}   //end namespace trajectory_based_nav

#endif //TRAJECTORY_BASED_NAV_RECOVERY_BEHAVIOR_H
