#ifndef TRAJECTORY_BASED_NAV_GENERAL_NAV_IMPL_H
#define TRAJECTORY_BASED_NAV_GENERAL_NAV_IMPL_H
  
#include <trajectory_based_nav/interfaces.h>
#include <trajectory_based_nav/planning_data.h>

#include <benchmarking_tools/benchmarking_tools.h>
#include <visualization_msgs/MarkerArray.h>

namespace trajectory_based_nav
{
  //NOTE: This function doesn't really need to know what kind of trajectories they are, may be possible to un-template it
  template<typename T>
  geometry_msgs::PoseArray::Ptr CombinedPoseArray(const std::vector<typename TypedTrajectoryWrapper<T>::Ptr>& trajectories)
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
  public:
    using Ptr = std::shared_ptr<GeneralNavImpl<T> >;
    //ros::NodeHandle nh_, pnh_;
    //tf2_utils::TransformManager tfm_;
    //nav_msgs::OdometryConstPtr curr_odom_;
    typename NavImpl<T>::Ptr lp_;
    
    //ros::Publisher remaining_traj_pub_, commanded_traj_pub_;
    ros::Publisher evaluated_traj_pub_, original_traj_pub_, viz_pub_, selected_traj_pub_, individual_traj_pub_;
    
  public:
    GeneralNavImpl(typename NavImpl<T>::Ptr lp);
    
    virtual ~GeneralNavImpl() = default;
    
    bool init(ros::NodeHandle nh);
  
    //Based on https://en.cppreference.com/w/cpp/algorithm/sort
    struct {
      inline
      bool operator()(const typename TypedTrajectoryWrapper<T>::Ptr& a, const typename TypedTrajectoryWrapper<T>::Ptr& b) const
      {   
        return a->cost() < b->cost();
      }   
    } customLess;
    
    virtual PlanningData Plan();
  
  };
  
    template<typename T>
    GeneralNavImpl<T>::GeneralNavImpl(typename NavImpl<T>::Ptr lp):
        lp_(lp)
    {
        lp_->setPlanningCB(boost::bind(&GeneralNavImpl<T>::Plan, this));
    }
    
    template<typename T>
    bool GeneralNavImpl<T>::init(ros::NodeHandle nh)
    {
        original_traj_pub_ = nh.advertise<geometry_msgs::PoseArray>("original_trajectories", 1);
        evaluated_traj_pub_ = nh.advertise<geometry_msgs::PoseArray>("evaluated_trajectories", 1);
        individual_traj_pub_ = nh.advertise<geometry_msgs::PoseArray>("individual_trajectories", 50);
        selected_traj_pub_ = nh.advertise<geometry_msgs::PoseArray>("selected_trajectory", 1);
        viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vizuals", 1);
        //lp_->setPlanningCB(boost::bind(&GeneralNavImpl<T>::Plan, this));
        //lp_->cc_wrapper_->setCallback(boost::bind(&GeneralNavImpl<T>::Plan, this));
        return true;
    }
  
    template<typename T>
    PlanningData GeneralNavImpl<T>::Plan()
    {
        ROS_INFO_STREAM("Plan");
        PlanningData data;
        
        auto t0 = ros::WallTime::now();
        
        auto odom_ptr = lp_->getTrajectoryController()->getCurrentOdom();
        
        if(!odom_ptr)
        {
            ROS_WARN_STREAM("No odom! Can't plan");
            data.error = true;
            data.stamp = ros::Time::now();
            return data;
        }
        auto odom = *odom_ptr;
        
        data.stamp = odom.header.stamp;
        
        if(!lp_->update(odom))
        {
            ROS_WARN_STREAM("Planner failed to update, stopping!");
            lp_->getTrajectoryController()->stop();
            
            data.error = true;
            return data;
        }
        
        DURATION_INFO_STREAM("planning", 50);
        if(lp_->getTerminationCriteria()->shouldTerminate(odom))
        {
            ROS_INFO_STREAM("Terminating!");
            lp_->getTrajectoryController()->stop();
            data.terminate=true;
            return data;
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
        //bool planned = false;
        if(lp_->getReplanLogic()->shouldReplan(odom, remaining_traj))
        {
            data.replan = true;
            
            //planned = true;
            ROS_INFO_STREAM("Replanning!");
            auto t1 = ros::WallTime::now();
            
            //TODO: do something with the return value of trajectory source, eg: bool traj_source_result = 
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
                geometry_msgs::PoseArray::Ptr pose_arrays = CombinedPoseArray<T>(candidate_trajs);
                if(pose_arrays->header.frame_id.empty())
                {
                    pose_arrays->header = odom.header;
                }
                evaluated_traj_pub_.publish((geometry_msgs::PoseArray::ConstPtr)pose_arrays);
            }
            
            if(individual_traj_pub_.getNumSubscribers()>0)
            {
                for(const auto& traj : candidate_trajs)
                {
                    individual_traj_pub_.publish(traj->getPoseArray());
                }
            }
            
            if(remaining_traj->getPoseArray().poses.size()>0)
            {
                if(lp_->getTrajectoryScoring()->scoreTrajectory(remaining_traj))
                {
                    candidate_trajs.push_back(remaining_traj);
                    ROS_INFO_STREAM("Current trajectory cost=" << remaining_traj->cost());
                }
//                 else
//                 {
//                     ROS_ERROR("Unable to score current trajectory!");
//                 }
            }
            
            std::sort(candidate_trajs.begin(), candidate_trajs.end(), customLess);
            
            //TODO: Move this to a separate function
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
               *   if(remaining_traj->cost() < candidate_trajs.front()->cost())
               *   {
               *     ROS_INFO_STREAM("Best generated trajectory is not as good as the currently executed one");
               *     //selected_traj = remaining_traj;
               *     //Actually, do nothing!
               *     return;
          }
          else
              */
              auto t4 = ros::WallTime::now();
          
          int traj_count=0;
          {
              typename TypedTrajectoryWrapper<T>::Ptr selected_traj;
              for(auto traj : candidate_trajs)
              {
                  double traj_cost = traj->cost();
                  if(traj_cost == std::numeric_limits<double>::max())
                  {
                    ROS_INFO_STREAM_NAMED("general_nav_impl.selection", "Trajectory cost (" << traj_cost << ") is fatal.");
                    break;
                  }
                  else
                  {
                    ROS_DEBUG_STREAM_NAMED("general_nav_impl.selection", "Trajectory cost (" << traj_cost << ") is not fatal.");
                  }
                
                  traj_count++;
                  if(lp_->getTrajectoryVerifier()->verifyTrajectory(traj))
                  {
                      selected_traj = traj;
                      break;
                  }
                  
              }
              
              if(selected_traj)
              {
                  data.trajectory = selected_traj;
                  if(selected_traj != remaining_traj)
                  {
                      typename T::ConstPtr trajectory_msg = selected_traj->getTrajectoryMsg();
                      lp_->getTrajectoryController()->setTrajectory(trajectory_msg);
                      
                      selected_traj_pub_.publish(selected_traj->getPoseArray());
                      ROS_INFO_STREAM("Selected new trajectory! Trajectory cost=" << selected_traj->cost());
                      data.trajectory_type=PlanningData::TrajectoryType::NEW;
                  }
                  else
                  {
                      ROS_INFO_STREAM("Continuing on previous trajectory.");
                      data.trajectory_type=PlanningData::TrajectoryType::PREVIOUS;
                  }
              }
              else
              {
                  ROS_WARN_STREAM("No valid trajectory found, stopping!");
                  lp_->getTrajectoryController()->stop();
                  data.trajectory_type=PlanningData::TrajectoryType::NONE;
              }
          }
          
          lp_->getReplanLogic()->setPlanningData(data);
          
          auto t5 = ros::WallTime::now();
          
          ROS_INFO_STREAM_NAMED("timing","Total: " << (t5-t0).toSec()*1000 << "ms, Replan logic: " << (t1-t01).toSec()*1000 << "ms, Init2: " << (t01-t0).toSec()*1000 << "ms, Generation: " << (t2-t1).toSec() * 1000 << "ms, Scoring: " <<  (t3-t2).toSec() * 1000 << "ms, Other: " <<  (t4-t3).toSec() * 1000 << "ms, Collision checking: " <<   (t5-t4).toSec() * 1000 << "ms");
          
          ROS_DEBUG_STREAM_NAMED("timing.statistics", "STATISTICS: {\"total_planning_time\":" << (ros::WallTime::now()-t0).toNSec() / 1e3 << ",\"planned\":true, \"num_collision_checked\":" << traj_count << ",\"generation_time\":" << (t2-t1).toNSec() / 1000 << ",\"scoring_time\":" << (t3-t2).toNSec()/1000 << ",\"verification_time\":" << (t5-t4).toNSec()/1000 << ",\"num_trajectories\":" << candidate_trajs.size() << "}");
          
          }
          else
          {
              ROS_DEBUG_STREAM_NAMED("timing", "STATISTICS: {\"total_planning_time\":" << (ros::WallTime::now()-t0).toNSec() / 1e3 << ",\"planned\":false}");
          }
          
          return data;
      }
  
} //end namespace trajectory_based_nav

#endif //TRAJECTORY_BASED_NAV_GENERAL_NAV_IMPL_H
