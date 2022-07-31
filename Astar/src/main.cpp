#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/opencv.hpp>
#include "Astar.h"
#include "OccMapTransform.h"

#include "std_msgs/Int32MultiArray.h"

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>


using namespace cv;
using namespace std;


//-------------------------------- Global variables ---------------------------------//
int NofTarget = 6; //se viene modificato, si deve modificare il dovuto numero di 
                    //path_i nelle dichiarazioni iniziali e nel main, aggiungendo 
                    //o togliendo iterazioni

// Subscriber
ros::Subscriber map_sub;
//ros::Subscriber odom2foot;
//ros::Subscriber startPoint_sub;
//ros::Subscriber targetPoint_sub;
// Publisher
ros::Publisher mask_pub;////////////////////////////////////////////////////////////////
ros::Publisher path_pub;
ros::Publisher path_pub_1;
ros::Publisher path_pub_2;
ros::Publisher path_pub_3;
ros::Publisher path_pub_4;
ros::Publisher path_pub_5;
//ros::Publisher path_pub_6;
//ros::Publisher vector_pub; 

// Object
nav_msgs::OccupancyGrid OccGridMask;

std_msgs::Int32MultiArray Target_vec;
//nav_msgs::Path path;/////////////////
nav_msgs::Path path;
// nav_msgs::Path path_1;
// nav_msgs::Path path_2;
// nav_msgs::Path path_3;
// nav_msgs::Path path_4;
//nav_msgs::Path path_5;
pathplanning::AstarConfig config;
pathplanning::Astar astar;
OccupancyGridParam OccGridParam;
Point startPoint; 
Point real_start;
Point targetPoint;

//transform listeneer object
tf2_ros::Buffer tfbuff;

// geometry_msgs::TransformStamped transformStamped;

//         try
//         {
//             transformStamped = tfBuffer.lookupTransform("robot_odom", "robot_base_footprint",ros::Time(0));
//         }
//         catch (tf2::TransformException &ex) 
//         {
//             ROS_WARN("%s",ex.what());
//             return;
//         }

// void publish_visual_marker(geometry_msgs::TransformStamped transformStamped)
//     {
//         tf::Quaternion q(
//             transformStamped.transform.rotation.x,
//             transformStamped.transform.rotation.y,
//             transformStamped.transform.rotation.z,
//             transformStamped.transform.rotation.w);
//         tf::Matrix3x3 m(q);
//         double roll, pitch, yaw;
//         m.getRPY(roll, pitch, yaw);
//     }

// Parameter
double InflateRadius;
bool map_flag;
bool startpoint_flag;
bool MULTIPATH_flag;
//bool targetpoint_flag;
bool start_flag;
int rate;
int a = 0;
int d = 0;  //define the direction of exploration
int q = 0;
int nof_layers = 60;
int semi_step = 1;
int last_q = 0;

int orig_nof = NofTarget;
float pos_x;
float pos_y;

//-------------------------------- Callback function ---------------------------------//
void MapCallback(const nav_msgs::OccupancyGrid& msg)
{
    // Get parameter
    OccGridParam.GetOccupancyGridParam(msg);

    // Get map
    int height = OccGridParam.height;
    int width = OccGridParam.width;
    //pos_x = OccGridParam.x;
    //pos_y = OccGridParam.y;
    //float pos_z = OccGridParam.origin.position.z;
    //OccGridParam.origin.orientation;
    int OccProb;
    int resolution = OccGridParam.resolution;
    //int x, y;
    //int o = 0;

    Mat Map(height, width, CV_8UC1);
    for(int i = 0; i < height; i++)
    {
        for(int j = 0; j < width; j++)
        {
            OccProb = msg.data[i * width + j];
            OccProb = (OccProb < 0) ? 100 : OccProb; // set Unknown to 0
            // The origin of the OccGrid is on the bottom left corner of the map
            Map.at<uchar>(height-i-1, j) = 255 - round(OccProb * 255.0 / 100.0);
        }
    }

    Point2d src_point = Point2d(0.5, 0);
    OccGridParam.Map2ImageTransform(src_point, startPoint);
    OccGridParam.Map2ImageTransform(src_point, targetPoint);
    OccGridParam.Map2ImageTransform(src_point, real_start);

    // Initial Astar
    Mat Mask;
    config.InflateRadius = round(InflateRadius / OccGridParam.resolution);
    astar.InitAstar(Map, Mask, config);

    // Publish Mask
    OccGridMask.header.stamp = ros::Time::now();
    OccGridMask.header.frame_id = "robot_base_footprint";
    OccGridMask.info = msg.info;
    OccGridMask.data.clear();
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = Mask.at<uchar>(height-i-1, j) * 255;
            OccGridMask.data.push_back(OccProb);
        }
    }

    // Set flag

    ROS_INFO("setting flag\n");
    map_flag = true;

    startpoint_flag = true;
    //if(map_flag && startpoint_flag) //&& targetpoint_flag
    //{
    start_flag = true;
    //}
}

//-------------------------------- Main function ---------------------------------//
int main(int argc, char * argv[])
{
    //  Initial node
    
    ros::init(argc, argv, "astar");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start astar node!\n");
    

    // Initial variables
    map_flag = false;
    startpoint_flag = false;
    //targetpoint_flag = false;
    start_flag = false;
    Point FAILp = Point(0, 0);

    // Parameter
    nh_priv.param<bool>("Euclidean", config.Euclidean, true);
    nh_priv.param<int>("OccupyThresh", config.OccupyThresh, -1);
    nh_priv.param<double>("InflateRadius", InflateRadius, -1);
    nh_priv.param<int>("rate", rate, 100);

    // Subscribe topics
    map_sub = nh.subscribe("map", 1, MapCallback);
    /////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////

    // Advertise topics
    mask_pub = nh.advertise<nav_msgs::OccupancyGrid>("mask", 1);/////////////////////////
    path_pub = nh.advertise<nav_msgs::Path>("nav_path", 1);
    path_pub_1 = nh.advertise<nav_msgs::Path>("nav_path_1", 1);
    path_pub_2 = nh.advertise<nav_msgs::Path>("nav_path_2", 1);
    path_pub_3 = nh.advertise<nav_msgs::Path>("nav_path_3", 1);
    path_pub_4 = nh.advertise<nav_msgs::Path>("nav_path_4", 1);
    path_pub_5 = nh.advertise<nav_msgs::Path>("nav_path_5", 1);
    //path_pub_6 = nh.advertise<nav_msgs::Path>("nav_path_6", 1);
    //vector_pub = nh.advertise<std_msgs::Int32MultiArray>("target_vector", 1);

    // Loop and wait for callback
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        if(start_flag)
        {   
            // pulire i topic dei path////////////////////////////////////////////////////////
            // path.poses.clear();
            // path_1.poses.clear();
            // path_2.poses.clear();
            // path_3.poses.clear();
            // path_4.poses.clear();
            // path_5.poses.clear();
            ///////////////////////////////////////
            double start_time = ros::Time::now().toSec();
            // Start planning path
            vector<Point> PathList;
            vector<Point> Tlist_def;
            vector<Point> Target_pub_list;

            MULTIPATH_flag = false;

            astar.Exploration(startPoint, targetPoint, FAILp, PathList, d, Tlist_def, nof_layers, semi_step, MULTIPATH_flag, NofTarget, q, Target_pub_list, last_q);

            if(!PathList.empty())
            {
                //ROS_INFO("%f x dell'origine", pos_x);
                //ROS_INFO("%f y dell'origine", pos_y);
                path.header.stamp = ros::Time::now();
                path.header.frame_id = "robot_base_footprint";
                path.poses.clear();
                
                for(int i = 0; i < PathList.size(); i++)
                {
                    Point2d dst_point;
                    OccGridParam.Image2MapTransform(PathList[i], dst_point);

                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header.stamp = ros::Time::now();
                    pose_stamped.header.frame_id = "robot_base_footprint";
                    pose_stamped.pose.position.x = dst_point.x;
                    pose_stamped.pose.position.y = dst_point.y;
                    pose_stamped.pose.position.z = 0;
                    path.poses.push_back(pose_stamped);
                }
                
                path_pub.publish(path);

            }
            else
            {
                ROS_ERROR("Can not find a valid path");
                path.poses.clear();
            }

            //ros::Duration(0.01).sleep(); // sleep for half a second
            
            int new_nof = Tlist_def.size();
            //ROS_INFO("%d il new_nof", new_nof);
            NofTarget = orig_nof;
            
///////////////////////////////////////////////////////////////////////////////////////
            
            MULTIPATH_flag = true;

            if(new_nof < NofTarget)
            {
                NofTarget = new_nof;
            }

            //int vec_to_pub[(2 * NofTarget) + 2] = {0};
            
            targetPoint = Target_pub_list.at(0);
            ////Target_pub_list.erase(Target_pub_list.begin());
            
            //vec_to_pub[0] = targetPoint.x; 
            //vec_to_pub[1] = targetPoint.y; 
            

            //ROS_INFO("numero di target %d", NofTarget);
            //for (q = 0; q < NofTarget; q++)
            //{
            q = 0;
            last_q = 0;

            /////////////////////////////////////////
            if (q == 0)
            {
                q = last_q;
            }

            last_q = q;
            /////////////////////////////////////////

            astar.Exploration(startPoint, targetPoint, FAILp, PathList, d, Tlist_def, nof_layers, semi_step, MULTIPATH_flag, NofTarget, q, Target_pub_list, last_q);
            nav_msgs::Path path_1;

            if ( q == 0 && q < NofTarget)
            {
                if(!PathList.empty())
                {
                    path_1.header.stamp = ros::Time::now();
                    path_1.header.frame_id = "robot_base_footprint";
                    //path_1.poses.clear();
                    for(int i = 0; i < PathList.size(); i++)
                    {
                        Point2d dst_point;
                        OccGridParam.Image2MapTransform(PathList[i], dst_point);

                        geometry_msgs::PoseStamped pose_stamped;
                        pose_stamped.header.stamp = ros::Time::now();
                        pose_stamped.header.frame_id = "robot_base_footprint";
                        pose_stamped.pose.position.x = dst_point.x;
                        pose_stamped.pose.position.y = dst_point.y;
                        pose_stamped.pose.position.z = 0;
                        path_1.poses.push_back(pose_stamped);
                    }

                    //path_pub.publish(path_1);
                    path_pub_1.publish(path_1);

                    //double end_time = ros::Time::now().toSec();

                    //ROS_INFO("Find a valid path successfully! SECOND STEP in %f", end_time - start_time);
                }
                else
                {
                    ROS_ERROR("Can not find a valid path");
                    path_1.poses.clear();
                    path_pub_1.publish(path_1);
                }
            }
            else
            {
                path_1.header.stamp = ros::Time::now();
                path_1.header.frame_id = "robot_base_footprint";
                //std::cout<<"pathlist empty"<<std::endl;
                path_1.poses.clear();
                path_pub_1.publish(path_1);
            }

            
            //ros::Duration(0.01).sleep(); // sleep for half a second

            if (q >= (NofTarget - 1))
            {
                q = 0;
            }
            else
            {
                targetPoint = Target_pub_list.at(0);
                //Target_pub_list.erase(Target_pub_list.begin());
                // if (vec_to_pub[q + 2] != 0)
                // {
                //     vec_to_pub[q + 2] = vec_to_pub[q + 2];
                // }
                // else
                // {
                //     vec_to_pub[q + 2] = targetPoint.x; 
                // }

                // if (vec_to_pub[q + 3] != 0)
                // {
                //     vec_to_pub[q + 3] = vec_to_pub[q + 3];
                // }
                // else
                // {
                //     vec_to_pub[q + 3] = targetPoint.y; 
                // }

                q++;
            }

            /////////////////////////////////////////
            if (q == 0)
            {
                q = last_q;
            }

            last_q = q;
            /////////////////////////////////////////

            astar.Exploration(startPoint, targetPoint, FAILp, PathList, d, Tlist_def, nof_layers, semi_step, MULTIPATH_flag, NofTarget, q, Target_pub_list, last_q);
            nav_msgs::Path path_2;

            if ( q == 1 && q < NofTarget)
            {
                if(!PathList.empty())
                {
                    path_2.header.stamp = ros::Time::now();
                    path_2.header.frame_id = "robot_base_footprint";

                    for(int i=0;i<PathList.size();i++)
                    {
                        Point2d dst_point;
                        OccGridParam.Image2MapTransform(PathList[i], dst_point);

                        geometry_msgs::PoseStamped pose_stamped;
                        pose_stamped.header.stamp = ros::Time::now();
                        pose_stamped.header.frame_id = "robot_base_footprint";
                        pose_stamped.pose.position.x = dst_point.x;
                        pose_stamped.pose.position.y = dst_point.y;
                        pose_stamped.pose.position.z = 0;
                        path_2.poses.push_back(pose_stamped);
                    }
                    
                    //path_pub.publish(path_2);
                    path_pub_2.publish(path_2);

                    //double end_time = ros::Time::now().toSec();

                    //ROS_INFO("Find a valid path successfully! SECOND STEP in %f s", end_time - start_time);
                }
                else
                {
                    ROS_ERROR("Can not find a valid path");
                    path_2.poses.clear();
                    path_pub_2.publish(path_2);
                }
                
                //ros::Duration(0.01).sleep(); // sleep for half a second
            }
            else
            {
                path_2.header.stamp = ros::Time::now();
                path_2.header.frame_id = "robot_base_footprint";
                //std::cout<<"pathlist empty"<<std::endl;
                path_2.poses.clear();
                path_pub_2.publish(path_2);
            }

            if (q >= (NofTarget - 1))
            {
                q = 0;
            }
            else
            {
                targetPoint = Target_pub_list.at(0);
                //Target_pub_list.erase(Target_pub_list.begin());
                // if (vec_to_pub[q + 3] != 0)
                // {
                //     vec_to_pub[q + 3] = vec_to_pub[q + 3];
                // }
                // else
                // {
                //     vec_to_pub[q + 3] = targetPoint.x; 
                // }

                // if (vec_to_pub[q + 4] != 0)
                // {
                //     vec_to_pub[q + 4] = vec_to_pub[q + 4];
                // }
                // else
                // {
                //     vec_to_pub[q + 4] = targetPoint.y; 
                // }

                q++;
            }

            /////////////////////////////////////////
            if (q == 0)
            {
                q = last_q;
            }

            last_q = q;
            /////////////////////////////////////////


            astar.Exploration(startPoint, targetPoint, FAILp, PathList, d, Tlist_def, nof_layers, semi_step, MULTIPATH_flag, NofTarget, q, Target_pub_list, last_q);
            nav_msgs::Path path_3;

            if ( q == 2 && q < NofTarget)
            {
                if(!PathList.empty())
                {
                    path_3.header.stamp = ros::Time::now();
                    path_3.header.frame_id = "robot_base_footprint";

                    for(int i=0;i<PathList.size();i++)
                    {
                        Point2d dst_point;
                        OccGridParam.Image2MapTransform(PathList[i], dst_point);

                        geometry_msgs::PoseStamped pose_stamped;
                        pose_stamped.header.stamp = ros::Time::now();
                        pose_stamped.header.frame_id = "robot_base_footprint";
                        pose_stamped.pose.position.x = dst_point.x;
                        pose_stamped.pose.position.y = dst_point.y;
                        pose_stamped.pose.position.z = 0;
                        path_3.poses.push_back(pose_stamped);
                    }

                    //path_pub.publish(path_3);
                    path_pub_3.publish(path_3);

                    //double end_time = ros::Time::now().toSec();

                    //ROS_INFO("Find a valid path successfully! SECOND STEP in %f s", end_time - start_time);
                }
                else
                {
                    ROS_ERROR("Can not find a valid path");
                    path_3.poses.clear();
                    path_pub_3.publish(path_3);
                }

                //ros::Duration(0.01).sleep(); // sleep for half a second
            }
            else
            {
                path_3.header.stamp = ros::Time::now();
                path_3.header.frame_id = "robot_base_footprint";
                //std::cout<<"pathlist empty"<<std::endl;
                path_3.poses.clear();
                path_pub_3.publish(path_3);
            }

            if (q >= (NofTarget - 1))
            {
                q = 0;
            }
            else
            {
                targetPoint = Target_pub_list.at(0);
                // //Target_pub_list.erase(Target_pub_list.begin());
                // if (vec_to_pub[q + 4] != 0)
                // {
                //     vec_to_pub[q + 4] = vec_to_pub[q + 4];
                // }
                // else
                // {
                //     vec_to_pub[q + 4] = targetPoint.x; 
                // }

                // if (vec_to_pub[q + 5] != 0)
                // {
                //     vec_to_pub[q + 5] = vec_to_pub[q + 5];
                // }
                // else
                // {
                //     vec_to_pub[q + 5] = targetPoint.y; 
                // }
                
                q++;
            }
            
            /////////////////////////////////////////
            if (q == 0)
            {
                q = last_q;
            }

            last_q = q;
            /////////////////////////////////////////

            astar.Exploration(startPoint, targetPoint, FAILp, PathList, d, Tlist_def, nof_layers, semi_step, MULTIPATH_flag, NofTarget, q, Target_pub_list, last_q); 
            nav_msgs::Path path_4;

            if ( q == 3 && q < NofTarget)
            {
                if(!PathList.empty())
                {
                    path_4.header.stamp = ros::Time::now();
                    path_4.header.frame_id = "robot_base_footprint";
                    
                    for(int i=0;i<PathList.size();i++)
                    {
                        Point2d dst_point;
                        OccGridParam.Image2MapTransform(PathList[i], dst_point);

                        geometry_msgs::PoseStamped pose_stamped;
                        pose_stamped.header.stamp = ros::Time::now();
                        pose_stamped.header.frame_id = "robot_base_footprint";
                        pose_stamped.pose.position.x = dst_point.x;
                        pose_stamped.pose.position.y = dst_point.y;
                        pose_stamped.pose.position.z = 0;
                        path_4.poses.push_back(pose_stamped);
                    }

                    path_pub.publish(path_4);
                    //path_pub_4.publish(path_4);

                    //double end_time = ros::Time::now().toSec();

                    //ROS_INFO("Find a valid path successfully! SECOND STEP in %f s", end_time - start_time);
                }
                else
                {
                    ROS_ERROR("Can not find a valid path");
                    path_4.poses.clear();
                    path_pub.publish(path_4);
                }

                //ros::Duration(0.01).sleep(); // sleep for half a second 
            }
            else
            {
                path_4.header.stamp = ros::Time::now();
                path_4.header.frame_id = "robot_base_footprint";
                //std::cout<<"pathlist empty"<<std::endl;
                path_4.poses.clear();
                path_pub_4.publish(path_4);
            }

            if (q >= (NofTarget - 1))
            {
                q = 0;
            }
            else
            {
                targetPoint = Target_pub_list.at(0);
                //Target_pub_list.erase(Target_pub_list.begin());
                // if (vec_to_pub[q + 5] != 0)
                // {
                //     vec_to_pub[q + 5] = vec_to_pub[q + 5];
                // }
                // else
                // {
                //     vec_to_pub[q + 5] = targetPoint.x; 
                // }

                // if (vec_to_pub[q + 6] != 0)
                // {
                //     vec_to_pub[q + 6] = vec_to_pub[q + 6];
                // }
                // else
                // {
                //     vec_to_pub[q + 6] = targetPoint.y; 
                // }

                q++;
            }

            /////////////////////////////////////////
            if (q == 0)
            {
                q = last_q;
            }

            last_q = q;
            /////////////////////////////////////////


            astar.Exploration(startPoint, targetPoint, FAILp, PathList, d, Tlist_def, nof_layers, semi_step, MULTIPATH_flag, NofTarget, q, Target_pub_list, last_q);
            nav_msgs::Path path_5;

            if ( q == 4 && q < NofTarget)
            {
                if(!PathList.empty())
                {
                    std::cout<<PathList.size()<<std::endl;

                    path_5.header.stamp = ros::Time::now();
                    path_5.header.frame_id = "robot_base_footprint";

                    for(int i=0;i<PathList.size();i++)
                    {
                        Point2d dst_point;
                        OccGridParam.Image2MapTransform(PathList[i], dst_point);

                        geometry_msgs::PoseStamped pose_stamped;
                        pose_stamped.header.stamp = ros::Time::now();
                        pose_stamped.header.frame_id = "robot_base_footprint";
                        pose_stamped.pose.position.x = dst_point.x;
                        pose_stamped.pose.position.y = dst_point.y;
                        pose_stamped.pose.position.z = 0;
                        path_5.poses.push_back(pose_stamped);
                    }

                    path_pub_5.publish(path_5);
                }
                else
                {
                    //ROS_ERROR("Can not find a valid path");
                    //std::cout<<"pathlist empty"<<std::endl;
                    path_5.poses.clear();
                    path_pub_5.publish(path_5);
                }

            }
            else
            {
                path_5.header.stamp = ros::Time::now();
                path_5.header.frame_id = "robot_base_footprint";
                //std::cout<<"pathlist empty"<<std::endl;
                path_5.poses.clear();
                path_pub_5.publish(path_5);
            }

            // if (q >= (NofTarget - 1))
            // {
            //     q = 0;
            // }
            // else
            // {
            //         //targetPoint = Target_pub_list.at(0);
            //         //Target_pub_list.erase(Target_pub_list.begin());
            //         // if (vec_to_pub[q + 6] != 0)
            //         // {
            //         //     vec_to_pub[q + 6] = vec_to_pub[q + 6];
            //         // }
            //         // else
            //         // {
            //         //     vec_to_pub[q + 6] = targetPoint.x; 
            //         // }

            //         // if (vec_to_pub[q + 7] != 0)
            //         // {
            //         //     vec_to_pub[q + 7] = vec_to_pub[q + 7];
            //         // }
            //         // else
            //         // {
            //         //     vec_to_pub[q + 7] = targetPoint.y; 
            //         // }
            //     q = 0;                
            // }

            // for (int k = 0; k < ((2 * NofTarget) + 2); k = k + 2 )
            // {
            //     ROS_INFO("%d e %d", vec_to_pub[k], vec_to_pub[k + 1]);
            //     Target_vec.data.push_back(vec_to_pub[k]);
            //     Target_vec.data.push_back(vec_to_pub[k + 1]);
            // }

            // if (NofTarget < 5)
            // {
            //     for (int w = 0; w < (5 - NofTarget); w++)
            //     {
            //         Target_vec.data.push_back(0);
            //         Target_vec.data.push_back(0);
            //     }
            // }
            // else
            // {
            //     ROS_INFO("trovati 6 target");   
            // }
            //}
            q = 0;
            // Set flag
            start_flag = false;
        }

        // path.poses.clear();
        // path_1.poses.clear();
        // path_2.poses.clear();
        // path_3.poses.clear();
        // path_4.poses.clear();
        // path_5.poses.clear();

        if(map_flag)
        {
            mask_pub.publish(OccGridMask);
        }

        // vector_pub.publish(Target_vec);
        // Target_vec.data.clear();

        //loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
