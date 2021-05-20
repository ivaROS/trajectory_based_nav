#ifndef TRAJECTORY_BASED_NAV_GLOBAL_POTENTIALS_WRAPPER_H
#define TRAJECTORY_BASED_NAV_GLOBAL_POTENTIALS_WRAPPER_H

#include <global_planner_plus/global_potentials_interface.h>
//#include <tf2_ros/message_filter.h>
#include <tf/transform_datatypes.h>


namespace trajectory_based_nav
{

  class GlobalPotentialsWrapper
  {
  protected:
    //std::string planning_frame_id_;
        
    //using PotentialsTFFilter = tf2_ros::MessageFilter<geometry_msgs::PoseStamped>;
    //std::shared_ptr<PotentialsTFFilter> potentials_filter_;
        
    //geometry_msgs::TransformStamped potentials_frame_transform_;
    
    tf2_utils::TransformManager tfm_;
    std::shared_ptr<global_planner_plus::GlobalPotentialsInterface> interface_;
    
    
  public:
    GlobalPotentialsWrapper()
    {}
    
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm)
    {
      tfm_ = tfm;
      
      std::string potentials_topic =  "/raw_potential"; //TODO: replace with parameter
      
      interface_ = std::make_shared<global_planner_plus::GlobalPotentialsInterface>(nh);
      interface_->init(potentials_topic);
      
      return true;
    }
    
    
    ///NOTE: Disabling for now, but might be desirable to include tf_filter
//     virtual void init(std::string potentials_topic)
//     {
//       potentials_sub_.subscribe(nh_, potentials_topic, 1);
//       potentials_sub_.registerCallback(&GlobalPotentialsInterface::potentialsCB, this);
//       
//       potentials_filter_ = std::make_shared<GoalTFFilter>(potentials_sub_, *tfm.getBuffer(), planning_frame_id_, 2, nh);
//       potentials_filter_->registerCallback(boost::bind(&GlobalPotentialsWrapper::GoalCallback, this, _1));
//       
//       potentials_cache_.setCacheSize(1);
//       potentials_cache_.connectInput(potentials_filter_);
//       
//       //planner_.initialize("potentials_planner", &lcr_);
//       inited_=true;
//     }
    
    
    virtual bool update(const std_msgs::Header& planning_header)
    {
      std_msgs::Header potentials_header = interface_->getHeader();
      if(potentials_header.frame_id == "")
      {
        return false;
      }
      
      tf::StampedTransform transform;
      try
      {
        //for now, use most recent available transform. Later, if necessary, perform more precise transform
        auto transform_msg = tfm_.getBuffer()->lookupTransform(planning_header.frame_id, potentials_header.frame_id, ros::Time());
        tf::transformStampedMsgToTF(transform_msg, transform);
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN_NAMED("global_goal_state", "Unable to get transform for potentials: %s",ex.what());
        return false;
      }
      
      return interface_->updateTransform(transform);
    }
    
    virtual float getPotential(const geometry_msgs::Pose& pose) const
    {
      return interface_->getPotential(pose);
    }
    
    virtual bool getGradient(const geometry_msgs::Pose& pose, Eigen::Vector2d& grad) const
    {
      return interface_->getGradient(pose, grad);
    }
    
//     virtual std_msgs::Header getHeader() const
//     {
//       std_msgs::Header header;
//       global_planner_plus::PotentialGrid::ConstPtr msg = potentials_cache_.getElemAfterTime(ros::Time());
//       
//       if(msg)
//       {
//         header = msg->header;
//       }
//       else
//       {
//         ROS_WARN_NAMED("global_potentials_wrapper", "No potentials received, unable to return header!");
//       }
//       
//       return header;
//     }
    
  };
  
} //end namespace trajectory_based_nav

#endif //TRAJECTORY_BASED_NAV_GLOBAL_POTENTIALS_WRAPPER_H
