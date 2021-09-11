#ifndef TRAJECTORY_BASED_NAV_JSON_WRITER_H
#define TRAJECTORY_BASED_NAV_JSON_WRITER_H

#include <sstream>
#include <iostream>
// #include <ros/duration.h>

class JSONWriter
{
    //std::string keyword_;
    std::stringstream buffer_;                
    bool first_entry_;

public:
    
    JSONWriter():
        first_entry_(true)
    {
        buffer_ << "STATISTICS: {";
    }
    
    template<typename T>
    JSONWriter(const std::string& name, const T& value):
        first_entry_(true)
    {
        buffer_ << "STATISTICS: {";
        addEntry(name, value);
    }
    
//     JSONWriter(std::string keyword="STATISTICS"):
//         //keyword_(keyword),
//         first_entry_(true)
//     {
//         buffer_ << keyword << ": {";
//     }
    
//     template<typename T>
//     void addEntry(const std::string& name, const ros::DurationBase<T>& duration)
//     {
//         addEntry(name, duration.toNSec());
//     }
    
    void addEntry(const std::string& name, const bool value)
    {
        addEntry(name, value ? "true" : "false");
    }
    
    template<typename T>
    void addEntry(const std::string& name, const T& value)
    {
        if(!first_entry_)
        {
            buffer_ << ",";
        }
        first_entry_ = false;
        
        buffer_ << "\"" << name << "\":" << value;
    }
    
    friend std::ostream& operator << (std::ostream& out, const JSONWriter& j)
    {
        out << j.buffer_.str() << "}";
        return out;
    }
    
    //ROS_DEBUG_STREAM_NAMED("timing.statistics", "STATISTICS: {\"total_planning_time\":" << (ros::WallTime::now()-t0).toNSec() / 1e3 << ",\"planned\":true, \"num_collision_checked\":" << traj_count << ",\"generation_time\":" << (t2-t1).toNSec() / 1000 << ",\"scoring_time\":" << (t3-t2).toNSec()/1000 << ",\"verification_time\":" << (t5-t4).toNSec()/1000 << ",\"num_trajectories\":" << candidate_trajs.size() << "}");
};

#endif //TRAJECTORY_BASED_NAV_JSON_WRITER_H
