#include <trajectory_based_nav/global_goal_manager.h>

namespace trajectory_based_nav 
{

        GlobalGoalManager::GlobalGoalManager(GlobalGoalState::Ptr ggs, TrajectoryController::Ptr controller):
            ggs_(ggs),
            controller_(controller),
            success_cb_(0),
            abort_cb_(0)
        {
            ggs_->setGoalReachedCallback(boost::bind(&GlobalGoalManager::reachedGoal, this));
        }
        
        //Modified from move_base.cpp
        bool GlobalGoalManager::init(ros::NodeHandle nh, ros::NodeHandle pnh)
        {
            as_ = std::make_shared<MoveBaseActionServer>(nh, "move_base", false); //, boost::bind(&GlobalGoalManager::executeCb, this, _1), false);
            as_->registerGoalCallback(boost::bind(&GlobalGoalManager::ActionGoalCB, this));
            
            current_goal_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );
            
            ros::NodeHandle action_nh("move_base");
            action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
            
            double feedback_publish_freq=5;
            bool found_publish_freq = pnh.getParam("feedback_publish_freq", feedback_publish_freq);
            if(controller_)
            {
              
                ROS_WARN_STREAM_COND_NAMED(!found_publish_freq, "global_goal_manager", "WARNING! No [feedback_publish_freq] parameter specified, using default [" << feedback_publish_freq << "]Hz]");
                
                if(feedback_publish_freq>0)
                {
                    feedback_timer_ = nh.createTimer(ros::Duration(1/feedback_publish_freq), &GlobalGoalManager::publishFeedback, this, false, false);
                }
                else
                {
                    ROS_WARN_STREAM_NAMED("global_goal_manager", "WARNING! [feedback_publish_freq] =" << feedback_publish_freq << ", disabling feedback publishing");
                }
            }
            else if(found_publish_freq)
            {
                ROS_WARN_STREAM_NAMED("global_goal_manager", "WARNING! Parameter [" << pnh.resolveName("feedback_publish_freq", true) << "] was set, but no controller was provided to GlobalGoalManager; disabling feedback publishing");
            }
            
            
            //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
            //they won't get any useful information back about its status, but this is useful for tools
            //like nav_view and rviz
            ros::NodeHandle simple_nh("move_base_simple");
            goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&GlobalGoalManager::goalCB, this, _1));
            
            as_->start(); //TODO: should this be moved before starting subscriber?
            return true;
        }
        
        void GlobalGoalManager::reachedGoal()
        {
            as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
            feedback_timer_.stop(); //Does nothing if timer never initialized
            
            if(success_cb_)
            {
                success_cb_();
            }
        }
        
        
        void GlobalGoalManager::addSuccessCallback(NoArgsCallback cb)
        {
            success_cb_ = cb;
        }
        
        void GlobalGoalManager::addAbortCallback(NoArgsCallback cb)
        {
            abort_cb_ = cb;
        }
        
        //Modified from move_base.cpp
        void GlobalGoalManager::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
            ROS_DEBUG_NAMED("global_goal_manager","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
            move_base_msgs::MoveBaseActionGoal action_goal;
            action_goal.header.stamp = ros::Time::now();
            action_goal.goal.target_pose = *goal;
            
            action_goal_pub_.publish(action_goal);
        }
        
        /* How the action server should be used:
        * 
        * Execute callback started with the goal action message
        * Callback should remain active as long as goal is active
        * If the goal is preempted, switch to the new one without shutting things down.
        * 
        * Need to publish feedback (current position) whenever things update (so whenever feedback controller sends command?)
        *    move_base_msgs::MoveBaseFeedback feedback;
        *    feedback.base_position = current_position;
        *    as_->publishFeedback(feedback);
        * 
        * move_base publishes feedback at control rate
        * 
        * 
        * 
        * If there's a problem, set as aborted:  as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        * 
        * If reach goal, set final result as success:           as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
        * 
        * 
        * 
        * 
        * 
        *  * Actually, it looks like the GoalHandle of action server might work well for my purposes, it has a getGoalID function (for associating matches),
        * and its status can be set to canceled/succeeded, etc. 
        * The key then is to 
        * 
        * 
        * Looks like goal callback might be the way to go, to allow immediate reaction to new goal, though maybe not quite as straightforward as when using execute callback
        * For starters, at least, use execute callback
        * 
        * 
        * Need to add some kind of monitor for failure conditions; not sure at what level to put that;
        * possibly create a mechanism for value (ex. return value from call) to be automatically registered w/ monitor, using some lambda to determine its behavior.
        * Baiscally, a general way of adding conditions and making decisions based on them
        * 
        * 
        * 
        * Should clear part of egocircle as part of recovery behavior
        * Possibly also local part of global costmap? is that done?
        * 
        * TODO: if nothing about global plan has changed, don't redo anything? Is there anything?
        * 
        * 
        * 
        * 
        */
        
        
        void GlobalGoalManager::ActionGoalCB()
        {
            ROS_DEBUG_STREAM_NAMED("global_goal_manager","ActionServer received a goal!");
            move_base_msgs::MoveBaseGoal::ConstPtr new_goal = as_->acceptNewGoal();
            auto new_goal_pose = boost::make_shared<geometry_msgs::PoseStamped>(new_goal->target_pose);
            ggs_->setNewGoal(new_goal_pose);
            current_goal_pub_.publish(new_goal_pose);
            feedback_timer_.start(); //Does nothing if timer never initialized
        }
        
        void GlobalGoalManager::publishFeedback(const ros::TimerEvent& e)
        {
            const auto odom = controller_->getCurrentOdom();
            //push the feedback out
            move_base_msgs::MoveBaseFeedback feedback;
            feedback.base_position.header = odom->header;
            feedback.base_position.pose = odom->pose.pose;
            as_->publishFeedback(feedback);
        }
        
        
        /*
        *    void executeCb(const move_base_msgs::MoveBaseGoal::ConstPtr& move_base_goal)
        *    {
        *        if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
        *            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
        *            return;
    }

    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

    //we have a goal so start the planner
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    current_goal_pub_.publish(goal);
    std::vector<geometry_msgs::PoseStamped> global_plan;


    //we want to make sure that we reset the last time we had a valid plan and control
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;

    ros::NodeHandle n;
    while(n.ok())
    {


    if(as_->isPreemptRequested()){
        if(as_->isNewGoalAvailable())
        {
        //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
        move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();
        
        if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
    }

    goal = goalToGlobalFrame(new_goal.target_pose);

    //we'll make sure that we reset our state for the next execution cycle
    recovery_index_ = 0;
    state_ = PLANNING;

    //we have a new goal so make sure the planner is awake
    lock.lock();
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //publish the goal point to the visualizer
    ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
    current_go  ros::Duration(5.0).sleep();
    m.addRequest(BehaviorRequest(b1, false));al_pub_.publish(goal);

    //make sure to reset our timeouts and counters
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;
    }
    else 
    {
    //if we've been preempted explicitly we need to shut things down
    resetState();

    //notify the ActionServer that we've successfully preempted
    ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
    as_->setPreempted();

    //we'll actually return from execute after preempting
    return;
    }
    }



    //for timing that gives real time even in simulation
    ros::WallTime start = ros::WallTime::now();

    //the real work on pursuing a goal is done here
    bool done = executeCycle(goal, global_plan);

    //if we're done, then we'll return from execute
    if(done)
        return;

    //check if execution of the goal has completed in some way

    ros::WallDuration t_diff = ros::WallTime::now() - start;
    ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

    r.sleep();
    //make sure to sleep for the remainder of our cycle time
    if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    //wake up the planner thread so that it can exit cleanly
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //if the node is killed then we'll abort and return
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
    }*/

}   //end namespace trajectory_based_nav
