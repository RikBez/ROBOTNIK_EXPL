#include "unity_path/joy_cmd_vel.h" 

using namespace unity_path;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_cmd_vel", ros::init_options::NoRosout);

   
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");

    try {
    ROS_INFO(" Initializing node");
    JoyCmdvel CD(nh, nh_local);
    ros::spin();
    }
    catch (const char* s) {
    ROS_FATAL_STREAM(" "  << s);
    }
    catch (...) {
    ROS_FATAL_STREAM(": Unexpected error");
    }

    return 0;    
    
}



