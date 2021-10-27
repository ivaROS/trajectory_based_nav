#ifndef TRAJECTORY_BASED_NAV_TRAJECTORY_EVENT_H
#define TRAJECTORY_BASED_NAV_TRAJECTORY_EVENT_H

#include <nav_msgs/Odometry.h>

namespace trajectory_based_nav
{
    enum class EventResponse {NONE, SHOULD_REPLAN, SHOULD_NOT_REPLAN};
    
    class TrajectoryEvent
    {
    public:
        virtual EventResponse evaluate(const nav_msgs::Odometry& odom) const =0; 
        
    };
  
} //end namespace trajectory_based_nav

#endif //TRAJECTORY_BASED_NAV_TRAJECTORY_EVENT_H
