#ifndef TRAJECTORY_BASED_NAV_MULTI_LEVEL_CC_WRAPPER_H
#define TRAJECTORY_BASED_NAV_MULTI_LEVEL_CC_WRAPPER_H

#include <pips_trajectory_testing/pips_cc_wrapper.h>
#include <pips_egocylindrical/egocan_cc_wrapper.h>
#include <pips_egocylindrical/egocylindrical_image_cc_wrapper.h>
#include <boost/thread/mutex.hpp>

namespace trajectory_based_nav
{
  
  class MultiLevelCollisionChecker : public pips::collision_testing::TransformingCollisionChecker
  {
    std::vector<std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> > collision_checkers_;
    std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> conservative_cc_, liberal_cc_, detailed_cc_, egocan_cc1_, egocan_cc2_;
    size_t num_cons_checks_=0, num_lib_checks_=0, num_det_checks_=0, num_egocan_checks_=0;
    static constexpr const char* DEFAULT_NAME="multi_level_collision_checker";
    
  public:
    
    MultiLevelCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME) : 
      pips::collision_testing::TransformingCollisionChecker(nh, pnh, name)
    {
      
    }
    
    virtual void setCollisionCheckers(std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> cons_cc, std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> lib_cc, std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> det_cc, std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> egocan_cc1, std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> egocan_cc2)
    {
      conservative_cc_ = cons_cc;
      liberal_cc_ = lib_cc;
      detailed_cc_ = det_cc;
      egocan_cc1_ = egocan_cc1;
      egocan_cc2_ = egocan_cc2;
      
      collision_checkers_.push_back(cons_cc);
      collision_checkers_.push_back(lib_cc);
      collision_checkers_.push_back(det_cc);
      collision_checkers_.push_back(egocan_cc1);
      collision_checkers_.push_back(egocan_cc2);
    }
    
    //TODO: Move CCResult out of global namespace
    virtual CCResult testCollisionImpl(geometry_msgs::Pose pose, CCOptions cco)
    {
      auto& cons_cc = *conservative_cc_;
      auto& lib_cc = *liberal_cc_;
      auto& det_cc = *detailed_cc_;
      
      auto& egocan1 = *egocan_cc1_;
      auto& egocan2 = *egocan_cc2_;
      
      bool collided = false;
            
      bool egocan_res = egocan1.testCollision(pose, cco) || egocan2.testCollision(pose, cco);
      num_egocan_checks_++;
      
      if(egocan_res)
      {
        collided = true;
      }
      else
      {
        bool cons_res = cons_cc.testCollision(pose, cco);
        num_cons_checks_++;
        if(cons_res)
        {
          bool lib_res = lib_cc.testCollision(pose, cco);
          num_lib_checks_++;
          if(lib_res)
          {
            //ROS_ERROR_STREAM("Actually had a liberal failure!" << pose);
            collided = true;
          }
          else
          {
            collided = det_cc.testCollision(pose, cco);
            num_det_checks_++;
          }
        }
      }
      ROS_WARN_STREAM_THROTTLE(1, "[" << name_ << "] Performed " << num_egocan_checks_ << " egocan, " << num_cons_checks_ << " conservative, " << num_lib_checks_ << " liberal, and " << num_det_checks_ << " detailed collision checks");
      return collided;
    }
    
    virtual void setTransform(const geometry_msgs::TransformStamped& base_optical_transform)
    {
      //Pretty sure this can be left blank, though it does indicate that this class doesn't fit perfectly into the existing inheritance hierarchy
    }
    
    
  protected:
    //This may not be relevant here
    virtual void initImpl() 
    {
//       for(auto& cc : collision_checkers_)
//       {
//         cc->init();
//       }
    }
    
    
  public:
    using Ptr = std::shared_ptr<MultiLevelCollisionChecker>;
  };
  
  
  class MultiLevelCCWrapper : public pips_trajectory_testing::PipsCCWrapper
  {
  protected:
    std::vector<std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> > cc_wrappers_;
    std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> conservative_wrapper_, liberal_wrapper_, detailed_wrapper_, egocan_wrapper1_, egocan_wrapper2_;
    
    MultiLevelCollisionChecker::Ptr cc_;
    
    
    std::string name_;
    static constexpr const char* DEFAULT_NAME="multi_level_cc_wrapper";
    
    boost::mutex status_mutex_;
    int status_;
    
  public:
    
    MultiLevelCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name=DEFAULT_NAME) :
      pips_trajectory_testing::PipsCCWrapper(nh, pnh, name, tfm)
    {
      {
        auto cc_wrapper = std::make_shared<pips_egocylindrical::EgocylindricalRangeImageCCWrapper>(nh, pnh, tfm);
        
        auto cons_cc_wrapper = std::make_shared<pips_egocylindrical::EgocylindricalRangeImageCCWrapper>(nh, pnh, tfm, "conservative_inflated_egocylindrical_image_cc_wrapper");
        auto lib_cc_wrapper = std::make_shared<pips_egocylindrical::EgocylindricalRangeImageCCWrapper>(nh, pnh, tfm, "liberal_inflated_egocylindrical_image_cc_wrapper");
        auto egocan_cc_wrapper1 = std::make_shared<pips_egocylindrical::EgoCanCCWrapper>(nh, pnh, tfm, "inflated_egocan_cc_wrapper1");
        auto egocan_cc_wrapper2 = std::make_shared<pips_egocylindrical::EgoCanCCWrapper>(nh, pnh, tfm, "inflated_egocan_cc_wrapper2");
        
        cc_ = std::make_shared<MultiLevelCollisionChecker>(nh_, pnh_);
        
        setCCWrappers(cons_cc_wrapper, lib_cc_wrapper, cc_wrapper, egocan_cc_wrapper1, egocan_cc_wrapper2);
      }
    }
    
    virtual bool init()
    {      
      auto general_cb = [this](int id)
      {
        int mask = ~(1<<id);
        ROS_INFO_STREAM_NAMED("multi_level_cc_wrapper.callback", "MultiLevelCCWrapper: sub-wrapper [" << id << "] received a new message");
        
        boost::mutex::scoped_lock lock(status_mutex_);
                       
        status_ &= mask;
        ROS_INFO_STREAM_NAMED("multi_level_cc_wrapper.callback", "MultiLevelCCWrapper: mask=" << mask << ", status:" << status_);
        
        if(status_==0)
        {
          ROS_INFO_STREAM_NAMED("multi_level_cc_wrapper.callback", "MultiLevelCCWrapper: All wrapper callbacks have executed, now executing top level callback");
          //All cc_wrappers have been updated, execute callback
          doCallback();
          status_ = 1 << (cc_wrappers_.size() + 1) - 1;
        }
        else
        {
        } 
      };

      int i = 0;
      for(const auto cc_wrapper : cc_wrappers_)
      {
        cc_wrapper->init();
        
        auto cb = [&general_cb, i](){ general_cb(i);};
        cc_wrapper->setCallback(cb);
        i++;
      }
      
      return true;
    };

    virtual void setCCWrappers(std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cons_wrapper, std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> lib_wrapper, std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> det_wrapper, std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> egocan_wrapper1, std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> egocan_wrapper2)
    {
      conservative_wrapper_ = cons_wrapper;
      liberal_wrapper_ = lib_wrapper;
      detailed_wrapper_ = det_wrapper;
      egocan_wrapper1_ = egocan_wrapper1;
      egocan_wrapper2_ = egocan_wrapper2;
      
      cc_wrappers_.push_back(conservative_wrapper_);
      cc_wrappers_.push_back(liberal_wrapper_);
      cc_wrappers_.push_back(detailed_wrapper_);
      cc_wrappers_.push_back(egocan_wrapper1_);
      cc_wrappers_.push_back(egocan_wrapper2_);
      
      cc_->setCollisionCheckers(conservative_wrapper_->getCC(), liberal_wrapper_->getCC(), detailed_wrapper_->getCC(), egocan_wrapper1_->getCC(), egocan_wrapper2_->getCC());
    }
    
    virtual std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> getCC()
    {
      return cc_;
    }

    virtual std_msgs::Header getCurrentHeader()
    {
      return cc_wrappers_.front()->getCurrentHeader();
    }

    virtual void update()
    {
      for(const auto cc_wrapper : cc_wrappers_)
      {
        cc_wrapper->update();
      }
    }
    
//     virtual bool isReady()
//     {
//       for(const auto cc_wrapper : cc_wrappers_)
//       {
//         std_msgs::Header header = cc_wrapper->getCurrentHeader();
//         
//         if(!cc_wrapper->isReady(odom.header))
//         {
//           return false;
//         }
//         cc_wrapper->update();
//       }
//     }
    
    virtual bool isReady()
    {
      for(const auto cc_wrapper : cc_wrappers_)
      {
        if(!cc_wrapper->isReady())
        {
          return false;
        }
      }
      return true;
    }
    
    virtual bool isReady(const std_msgs::Header& header)
    {
      for(const auto cc_wrapper : cc_wrappers_)
      {
        if(!cc_wrapper->isReady(header))
        {
          return false;
        }
      }
      return true;
    }
    
//     virtual bool isReadyImpl()
//     {
//       for(const auto cc_wrapper : cc_wrappers_)
//       {
//         if(!cc_wrapper->isReadyImpl())
//         {
//           return false;
//         }
//       }
//       return true;
//     }
    
    
    

    using Ptr=std::shared_ptr<MultiLevelCCWrapper>;
  };

} //end namespace trajectory_based_nav
    
#endif  //TRAJECTORY_BASED_NAV_MULTI_LEVEL_CC_WRAPPER_H
