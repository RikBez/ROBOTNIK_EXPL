#ifndef UNITY_PATH_JOY_CMD_VEL_H
#define UNITY_PATH_JOY_CMD_VEL_H

#include <ros/ros.h>

#include <boost/foreach.hpp>
#include "message_filters/subscriber.h"

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

namespace unity_path
{


class JoyCmdvel
{
public:
  JoyCmdvel(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
  ~JoyCmdvel();

private:
 
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    
    ros::Publisher cmd_vel_pub_;       

    float linear_vel;
    float angular_vel;
    
    ros::Subscriber yaw_rate_sub_;
    ros::Subscriber forward_vel_sub_;
    
    geometry_msgs::Twist cmd_vel_msg;
    std::string cmd_vel_topic_;
    
    void YawRateCallback (const std_msgs::Float64::ConstPtr& vel_msg);
    void ForwardVelCallback (const std_msgs::Float64::ConstPtr& vel_msg);
    void PublishCmdVel();
};

} // end namespace

#endif //UNITY_PATH_JOY_CMD_VEL_H




