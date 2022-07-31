#include "unity_path/joy_cmd_vel.h" 

using namespace unity_path;
using namespace std;


        
      
JoyCmdvel::JoyCmdvel(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
    
    
    nh_local_.param<string>("cmd_vel_topic",cmd_vel_topic_,"/robot/robotnik_base_control/cmd_vel");
    
    
        
   
    
    yaw_rate_sub_ = nh_.subscribe("/unity/yaw_rate", 100, &JoyCmdvel::YawRateCallback,this);
    forward_vel_sub_ = nh_.subscribe("/unity/forward_vel", 100, &JoyCmdvel::ForwardVelCallback,this);

 
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
    PublishCmdVel();
}



void JoyCmdvel::YawRateCallback (const std_msgs::Float64::ConstPtr& yaw_msg)
{

    angular_vel = yaw_msg->data;
     
}

void JoyCmdvel::ForwardVelCallback (const std_msgs::Float64::ConstPtr& vel_msg)
{

    linear_vel = vel_msg->data;

     
}


void JoyCmdvel::PublishCmdVel()
{
   ros::Rate r(20.0);
 
   cmd_vel_msg.linear.x = 0.0;
   cmd_vel_msg.angular.z = 0.0;

   
   while(ros::ok())
      {
        
        cmd_vel_msg.linear.x = linear_vel;
        cmd_vel_msg.angular.z = -angular_vel;

        cmd_vel_pub_.publish(cmd_vel_msg);     

  
      
        ros::spinOnce();
        r.sleep();
      }
     
                      
}

JoyCmdvel::~JoyCmdvel()
 {

 }
