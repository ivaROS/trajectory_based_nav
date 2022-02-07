#ifndef TRAJECTORY_BASED_NAV_FUTURE_DATED_TRAJECTORY_HELPER_H
#define TRAJECTORY_BASED_NAV_FUTURE_DATED_TRAJECTORY_HELPER_H

#include <trajectory_based_nav/interfaces.h>
#include <type_traits>

namespace trajectory_based_nav
{

  class FutureDatedTrajectoryHelper
  {
      ros::Duration delay_duration_;
      nav_msgs::Odometry future_odom_;
      //int traj_ind_;
      //const TrajectoryWrapper* current_trajectory_;
      geometry_msgs::PoseArray future_poses_;
      std::vector<ros::Duration> future_durations_;
      std::vector<geometry_msgs::Twist> future_twists_;
      
  public:
      FutureDatedTrajectoryHelper()
      {
      }
      
      virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
      {
        double delay_duration = 1; //in seconds
        //set from pnh
        
        delay_duration_ = ros::Duration(delay_duration);
        
        return true;
      }
      
      bool update(const nav_msgs::Odometry& odom, const TrajectoryWrapper& current_trajectory)
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
      
      nav_msgs::Odometry getFutureOdom() const
      {
        return future_odom_;
      }
      
      template<typename T>
      T getPrependedTrajectory(const T& traj)
      {
        auto temp_poses = future_poses_;
        auto temp_durations = future_durations_;
        auto temp_twists = future_twists_;
        
        const auto& poses = traj->getPoseArray().poses;
        const auto& durations = traj->getDurations();
        const auto& twists = traj->getTwists();
        
        temp_poses.poses.insert(std::end(temp_poses.poses), std::begin(poses), std::end(poses));
        temp_durations.insert(std::end(temp_durations), std::begin(durations), std::end(durations));
        temp_twists.insert(std::end(temp_twists), std::begin(twists), std::end(twists));
        //temp_poses.poses.insert(std::end(temp_poses.poses), std::begin(traj->getPoseArray().poses), std::end(traj->getPoseArray().poses));
        //temp_durations.insert(std::end(temp_durations), std::begin(traj->getDurations()), std::end(traj->getDurations()));
        //temp_twists.insert(std::end(temp_twists), std::begin(traj->getTwists()), std::end(traj->getTwists()));
        
        using M = typename T::element_type;// typename std::remove_pointer<T>::type;
        
        auto new_traj = std::make_shared<M>(temp_poses, temp_durations, temp_twists);
        return new_traj;
      }
        
  };

}

#endif //TRAJECTORY_BASED_NAV_FUTURE_DATED_TRAJECTORY_HELPER_H
