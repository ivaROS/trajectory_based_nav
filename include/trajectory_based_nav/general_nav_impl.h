#ifndef TRAJECTORY_BASED_NAV_GENERAL_NAV_IMPL_H
#define TRAJECTORY_BASED_NAV_GENERAL_NAV_IMPL_H
  
#include <trajectory_based_nav/interfaces.h>
#include <benchmarking_tools/benchmarking_tools.h>
#include <visualization_msgs/MarkerArray.h>

namespace trajectory_based_nav
{
  //NOTE: This function doesn't really need to know what kind of trajectories they are, may be possible to un-template it
  template<typename T>
  geometry_msgs::PoseArray::ConstPtr CombinedPoseArray(const std::vector<typename TypedTrajectoryWrapper<T>::Ptr>& trajectories)
  {
    geometry_msgs::PoseArray::Ptr pose_arrays = boost::make_shared<geometry_msgs::PoseArray>();
    
    for(const auto& traj : trajectories)
    {
      const auto& pose_array = traj->getPoseArray();
      pose_arrays->poses.insert(pose_arrays->poses.end(), pose_array.poses.begin(), pose_array.poses.end());
      pose_arrays->header = pose_array.header;
    }
    return pose_arrays;
  }
  
  
  
  template<typename T>
  class GeneralNavImpl
  {
    //ros::NodeHandle nh_, pnh_;
    //tf2_utils::TransformManager tfm_;
    //nav_msgs::OdometryConstPtr curr_odom_;
    typename NavImpl<T>::Ptr lp_;
    
    //ros::Publisher remaining_traj_pub_, commanded_traj_pub_;
    ros::Publisher evaluated_traj_pub_, original_traj_pub_, viz_pub_, selected_traj_pub_;
    
  public:
    GeneralNavImpl(typename NavImpl<T>::Ptr lp):
      lp_(lp)
    {
    }
    
    bool init(ros::NodeHandle nh)
    {
      original_traj_pub_ = nh.advertise<geometry_msgs::PoseArray>("original_trajectories", 1);
      evaluated_traj_pub_ = nh.advertise<geometry_msgs::PoseArray>("evaluated_trajectories", 1);
      selected_traj_pub_ = nh.advertise<geometry_msgs::PoseArray>("selected_trajectory", 1);
      viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vizuals", 1);
      lp_->cc_wrapper_->setCallback(boost::bind(&GeneralNavImpl<T>::Plan, this));
      return true;
    }

  
    //Based on https://en.cppreference.com/w/cpp/algorithm/sort
    struct {
      bool operator()(const typename TypedTrajectoryWrapper<T>::Ptr& a, const typename TypedTrajectoryWrapper<T>::Ptr& b) const
      {   
        return a->cost() < b->cost();
      }   
    } customLess;
    
    void Plan()
    {
      ROS_INFO_STREAM("Plan");
      
      auto t0 = ros::WallTime::now();
      
      auto odom_ptr = lp_->getTrajectoryController()->getCurrentOdom();
      
      if(!odom_ptr)
      {
        ROS_WARN_STREAM("No odom! Can't plan");
        return;
      }
      auto odom = *odom_ptr;
      
      if(!lp_->update(odom))
      {
        ROS_WARN_STREAM("Planner failed to update, stopping!");
        lp_->getTrajectoryController()->stop();
        
        return;
      }
      
      DURATION_INFO_STREAM("planning", 50);
      if(lp_->getTerminationCriteria()->shouldTerminate(odom))
      {
        ROS_INFO_STREAM("Terminating!");
        lp_->getTrajectoryController()->stop();
        return;
      }
      auto remaining_traj = lp_->getTrajectoryController()->getCurrentTrajectory();
      
      
//       //TODO: somehow make code work correctly (and efficiently) whether or not separate monitoring and verifying cc_wrappers are in use
//       {
//         auto cc_wrapper_ = lp_->cc_wrapper_;
//         std_msgs::Header header = cc_wrapper_->getCurrentHeader();
//         
//         if(!cc_wrapper_->isReady(odom.header))
//         {
//           ROS_WARN_STREAM("cc_wrapper is not ready!");
//           return;
//         }
//         cc_wrapper_->update();
//       }
      
      auto t01 = ros::WallTime::now();
      bool planned = false;
      if(lp_->getReplanLogic()->shouldReplan(odom, remaining_traj))
      {
        planned = true;
        ROS_INFO_STREAM("Replanning!");
        auto t1 = ros::WallTime::now();
        
        lp_->getTrajectorySource()->update(odom, *remaining_traj);
        
        
        auto candidate_trajs = std::vector<typename TypedTrajectoryWrapper<T>::Ptr>();
        while(lp_->getTrajectorySource()->hasMore())
        {
          candidate_trajs.push_back(lp_->getTrajectorySource()->getNext());
          //lp_->getTrajectoryScoring()->scoreTrajectory(candidate_trajs.back());
        }
        ROS_INFO_STREAM("Generated " << candidate_trajs.size() << " trajectories");
        
        if(original_traj_pub_.getNumSubscribers()>0)
        {
          geometry_msgs::PoseArray::ConstPtr pose_arrays = CombinedPoseArray<T>(candidate_trajs);
          original_traj_pub_.publish(pose_arrays);
        }
        
        auto t2 = ros::WallTime::now();
        
        
        for(auto& traj : candidate_trajs)
        {
          lp_->getTrajectoryScoring()->scoreTrajectory(traj);
        }
        
        auto t3 = ros::WallTime::now();
        
        
        if(evaluated_traj_pub_.getNumSubscribers()>0)
        {
          
          geometry_msgs::PoseArray::ConstPtr pose_arrays = CombinedPoseArray<T>(candidate_trajs);
          evaluated_traj_pub_.publish(pose_arrays);
        }
        
        std::sort(candidate_trajs.begin(), candidate_trajs.end(), customLess);
        
        if(remaining_traj->getPoseArray().poses.size()>0)
        {
          lp_->getTrajectoryScoring()->scoreTrajectory(remaining_traj);
          candidate_trajs.push_back(remaining_traj);
        }
        
        if(viz_pub_.getNumSubscribers()>0)
        {
          visualization_msgs::MarkerArray markers;
      
        
          visualization_msgs::Marker clearing_marker, label_marker, score_marker;
          clearing_marker.ns = "labels";
          clearing_marker.action = visualization_msgs::Marker::DELETEALL;
          markers.markers.push_back(clearing_marker);
          
          label_marker.ns = "labels";
          label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          
          label_marker.pose.orientation.w = 1;
          label_marker.pose.position.z = 0.1;
          
          label_marker.scale.x=0.1;
          label_marker.scale.y=0.1;
          label_marker.scale.z=0.05; //Only z matters, others filled in to prevent rviz warnings
          
          label_marker.color.a=1;
          label_marker.color.r=0;
          label_marker.color.g=1;
          label_marker.color.b=0;
          
          
          score_marker.ns = "scores";
          score_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          
          score_marker.pose.orientation.w = 1;
          score_marker.pose.position.z = 0.1;
          
          score_marker.scale.x=0.1;
          score_marker.scale.y=0.1;
          score_marker.scale.z=0.3; //Only z matters, others filled in to prevent rviz warnings
          
          score_marker.color.a=1;
          score_marker.color.r=1;
          score_marker.color.g=1;
          score_marker.color.b=1;
          
          label_marker.header = odom.header;
          score_marker.header = odom.header;
          
          int ind = 0;
          for(const auto& traj : candidate_trajs)
          {
            const auto& end_pos = traj->getPoseArray().poses.back().position;
            label_marker.pose.position.x = end_pos.x;
            label_marker.pose.position.y = end_pos.y;
            label_marker.pose.position.z = end_pos.z;
            label_marker.text = std::to_string(ind);
            markers.markers.push_back(label_marker);
            label_marker.id++;
            ind++;
          }
          
          viz_pub_.publish(markers);
        }
        
        
        /*
        if(remaining_traj->cost() < candidate_trajs.front()->cost())
        {
          ROS_INFO_STREAM("Best generated trajectory is not as good as the currently executed one");
          //selected_traj = remaining_traj;
          //Actually, do nothing!
          return;
        }
        else
          */
        auto t4 = ros::WallTime::now();
        
        int traj_count=0;
        {
          typename TypedTrajectoryWrapper<T>::Ptr selected_traj;
          for(auto traj : candidate_trajs)
          {
            traj_count++;
            if(lp_->getTrajectoryVerifier()->verifyTrajectory(traj))
            {
              selected_traj = traj;
              break;
            }
            
          }
          
          if(selected_traj)
          {
            if(selected_traj != remaining_traj)
            {
              typename T::ConstPtr trajectory_msg = selected_traj->getTrajectoryMsg();
              lp_->getTrajectoryController()->setTrajectory(trajectory_msg);
              
              selected_traj_pub_.publish(selected_traj->getPoseArray());
              ROS_INFO_STREAM("Selected new trajectory!");
            }
            else
            {
              ROS_INFO_STREAM("Continuing on previous trajectory.");
            }
          }
          else
          {
            ROS_WARN_STREAM("No valid trajectory found, stopping!");
            lp_->getTrajectoryController()->stop();
          }
        }
        
        auto t5 = ros::WallTime::now();
        
        ROS_INFO_STREAM("Total: " << (t5-t0).toSec()*1000 << "ms, Replan logic: " << (t1-t01).toSec()*1000 << "ms, Init2: " << (t01-t0).toSec()*1000 << "ms, Generation: " << (t2-t1).toSec() * 1000 << "ms, Scoring: " <<  (t3-t2).toSec() * 1000 << "ms, Other: " <<  (t4-t3).toSec() * 1000 << "ms, Collision checking: " <<   (t5-t4).toSec() * 1000 << "ms");
        
        ROS_DEBUG_STREAM_NAMED("timing", "STATISTICS: {\"total_planning_time\":" << (ros::WallTime::now()-t0).toNSec() / 1e3 << ",\"planned\":true, \"num_collision_checked\":" << traj_count << ",\"generation_time\":" << (t2-t1).toNSec() / 1000 << ",\"scoring_time\":" << (t3-t2).toNSec()/1000 << ",\"verification_time\":" << (t5-t4).toNSec()/1000 << ",\"num_trajectories\":" << candidate_trajs.size() << "}");
        
      }
      else
      {
        ROS_DEBUG_STREAM_NAMED("timing", "STATISTICS: {\"total_planning_time\":" << (ros::WallTime::now()-t0).toNSec() / 1e3 << ",\"planned\":false}");
      }
    }
    
  
  };
} //end namespace trajectory_based_nav

#endif //TRAJECTORY_BASED_NAV_GENERAL_NAV_IMPL_H
