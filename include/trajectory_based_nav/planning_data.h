#ifndef TRAJECTORY_BASED_NAV_PLANNING_DATA_H
#define TRAJECTORY_BASED_NAV_PLANNING_DATA_H

namespace trajectory_based_nav
{

    struct PlanningData
    {
//         enum class Outcome {ERROR, NO_REPLAN, NEW_TRAJECTORY, PREVIOUS_TRAJECTORY, NO_VALID_TRAJECTORY, TERMINATE};
//         Outcome outcome;
        ros::Time stamp;
        
        bool error=false;
        bool terminate=false;
        bool replan=false;
        
        enum class Trajectory {NEW, PREVIOUS, NONE};
        Trajectory trajectory;
        
    };


}   //end namespace trajectory_based_nav


#endif //TRAJECTORY_BASED_NAV_PLANNING_DATA_H
