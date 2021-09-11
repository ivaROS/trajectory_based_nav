#ifndef TRAJECTORY_BASED_NAV_MULTI_LEVEL_CC_WRAPPER_H
#define TRAJECTORY_BASED_NAV_MULTI_LEVEL_CC_WRAPPER_H

#include <pips_trajectory_testing/pips_cc_wrapper.h>
//#include <pips_egocylindrical/egocan_cc_wrapper.h>
//#include <pips_egocylindrical/egocylindrical_image_cc_wrapper.h>
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
    
    MultiLevelCollisionChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME);
    
    virtual void setCollisionCheckers(std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> cons_cc, std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> lib_cc, std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> det_cc, std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> egocan_cc1, std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> egocan_cc2);
    
    //TODO: Move CCResult out of global namespace
    virtual CCResult testCollisionImpl(geometry_msgs::Pose pose, CCOptions cco);
    
    virtual void setTransform(const geometry_msgs::TransformStamped& base_optical_transform) {}      //Pretty sure this can be left blank, though it does indicate that this class doesn't fit perfectly into the existing inheritance hierarchy

    
  protected:
    //This may not be relevant here
    virtual void initImpl() {}
    
    
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
    
    MultiLevelCCWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name=DEFAULT_NAME);
    
    virtual bool init();
    
    virtual void setCCWrappers(std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cons_wrapper, std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> lib_wrapper, std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> det_wrapper, std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> egocan_wrapper1, std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> egocan_wrapper2);
    
    virtual std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> getCC()
    {
      return cc_;
    }

    virtual std_msgs::Header getCurrentHeader();

    virtual void update();
    
    virtual bool isReady();
    
    virtual bool isReady(const std_msgs::Header& header);

  protected:
    void processCB(int id);
    
  public:
    using Ptr=std::shared_ptr<MultiLevelCCWrapper>;
  };

} //end namespace trajectory_based_nav
    
#endif  //TRAJECTORY_BASED_NAV_MULTI_LEVEL_CC_WRAPPER_H
