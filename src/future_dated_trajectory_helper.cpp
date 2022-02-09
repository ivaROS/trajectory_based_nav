#include <trajectory_based_nav/future_dated_trajectory_helper.h>
#include <pips/utils/param_utils.h>

namespace trajectory_based_nav
{

    FutureDatedTrajectoryHelper::FutureDatedTrajectoryHelper()
    {
    }
    
    bool FutureDatedTrajectoryHelper::init(ros::NodeHandle nh, ros::NodeHandle pnh, tf2_utils::TransformManager tfm)
    {
      double delay_duration = 1; //in seconds
      //set from pnh
      
      pips::utils::get_param(pnh, "delay_duration", delay_duration);
      
      delay_duration_ = ros::Duration(delay_duration);
      
      return true;
    }
    
    bool FutureDatedTrajectoryHelper::update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory)
    {
      const auto& durations = current_trajectory.getDurations();
      const int durations_size = (int)durations.size();
      
      int traj_ind = -1;
      for(; traj_ind + 2 < durations_size; traj_ind++)
      {
        if(durations[traj_ind+1]>delay_duration_)
        {
          break;
        }
      }
      
      if(traj_ind<0)
      {
        future_odom_ = odom;
      }
      else
      {
        nav_msgs::Odometry future_odom;
        future_odom.pose.pose = current_trajectory.getPoseArray().poses[traj_ind];
        future_odom.twist.twist = current_trajectory.getTwists()[traj_ind];
        future_odom.header = odom.header;
        future_odom.header.stamp += durations[traj_ind+1];
        
        future_odom_ = future_odom;
      }
      
      future_poses_ = current_trajectory.getPoseArray();
      future_poses_.poses.resize(traj_ind+1);
      future_durations_ = current_trajectory.getDurations();
      future_durations_.resize(traj_ind+1);
      future_twists_ = current_trajectory.getTwists();
      future_twists_.resize(traj_ind+1);
      
//         traj_ind_ = traj_ind;
//         current_trajectory_ = &current_trajectory;
      
      return true;
    }

} //end namespace trajectory_based_nav

