#ifndef TRAJECTORY_BASED_NAV_GLOBAL_GOAL_MANAGER
#define TRAJECTORY_BASED_NAV_GLOBAL_GOAL_MANAGER

#include <trajectory_based_nav/global_goal_state.h>
#include <trajectory_based_nav/interfaces.h>  //Only need TrajectoryController

//ActionServer
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace trajectory_based_nav 
{

    class GlobalGoalManager
    {
        using MoveBaseActionServer = actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>;
        using NoArgsCallback = boost::function<void() >;
        
        
    public:
        
        GlobalGoalManager(GlobalGoalState::Ptr ggs, TrajectoryController::Ptr controller=nullptr);
        
        virtual bool init(ros::NodeHandle nh, ros::NodeHandle pnh);
        
        virtual void reachedGoal();
        
        virtual void addSuccessCallback(NoArgsCallback cb);
        
        virtual void addAbortCallback(NoArgsCallback cb);
        
    protected:
        
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
        
        void ActionGoalCB();
        
        void publishFeedback(const ros::TimerEvent& e);
        
        std::shared_ptr<MoveBaseActionServer> as_;
        ros::Publisher current_goal_pub_, action_goal_pub_;
        ros::Subscriber goal_sub_;
        
        GlobalGoalState::Ptr ggs_;
        TrajectoryController::Ptr controller_;
        ros::Timer feedback_timer_;
        
        NoArgsCallback success_cb_, abort_cb_;
        //boost::recursive_mutex action_mutex_;
    };

}   //end namespace trajectory_based_nav

#endif //TRAJECTORY_BASED_NAV_GLOBAL_GOAL_MANAGER
