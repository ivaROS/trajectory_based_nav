#ifndef TRAJECTORY_BASED_NAV_GLOBAL_POTENTIALS_PLANNER_INTERFACE_H
#define TRAJECTORY_BASED_NAV_GLOBAL_POTENTIALS_PLANNER_INTERFACE_H

#include <trajectory_based_nav/global_planning_interface.h>
#include <global_planner_plus/PotentialGrid.h>
#include <global_planner_plus/global_potentials_interface.h>
//#include <trajectory_based_nav/default_implementations.h>
//#include <trajectory_based_nav/interfaces.h>
#include <boost/thread/mutex.hpp>
//#include <tf2_ros/message_filter.h>
//#include <pips/utils/param_utils.h>

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
  
  
  struct GlobalPotentialsPlanningResult : public trajectory_based_nav::PathPlanningResult
  {
  public:
    using Ptr = std::shared_ptr<GlobalPotentialsPlanningResult>;
    using ConstPtr = std::shared_ptr<const GlobalPotentialsPlanningResult>;
    global_planner_plus::PotentialGrid::ConstPtr potentials;
    
    //global_planner_plus::PotentialGrid::ConstPtr getPotentials() {return potentials;}
  };

  
  class GlobalPotentialsPlannerInterface : public trajectory_based_nav::CostmapBasedGlobalPlannerInterface<GlobalPotentialsPlanningResult>, private global_planner_plus::GlobalPlannerPlus
  {
    using trajectory_based_nav::CostmapBasedGlobalPlannerInterface<GlobalPotentialsPlanningResult>::costmap_;
    
  protected:
    
  public:
    using Ptr = std::shared_ptr<GlobalPotentialsPlannerInterface>;
    using planner_result_t = GlobalPotentialsPlanningResult;
    
    GlobalPotentialsPlannerInterface(std::string name):
      trajectory_based_nav::CostmapBasedGlobalPlannerInterface<GlobalPotentialsPlanningResult>(),
      global_planner_plus::GlobalPlannerPlus(name, costmap_.getCostmap(), costmap_.getGlobalFrameID())
      {}
      
    
    virtual planner_result_t plan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal)
    {
      planner_result_t result;
      result.goal = goal;
      result.start = start;
      result.is_valid = false;
      
      
      costmap_2d::Costmap2D::mutex_t::scoped_lock lock(*(costmap_.getCostmap()->getMutex()));
      
      
      if(costmap_.isCurrent())
      {
        if(global_planner_plus::GlobalPlannerPlus::makePlan(result.start, result.goal, result.path))
        {
          result.is_valid = true;
          result.potentials = potential_msg_;
        }
        else
        {
          ROS_WARN_STREAM("Global planning failed!");
        }
      }
      else
      {
        ROS_WARN_STREAM("Costmap is not current!");
      }
      
      
      
      
      return result;
    }

    
  };
  
  
  
  
  
} //end namespace trajectory_based_nav


#endif //TRAJECTORY_BASED_NAV_GLOBAL_POTENTIALS_PLANNER_INTERFACE_H

