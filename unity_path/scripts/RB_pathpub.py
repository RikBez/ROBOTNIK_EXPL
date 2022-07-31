#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def robot_pose_callback(nav_msgs,pub_path):
	path_msg = Path()
	#point_number = len(nav_msgs.poses)
	print(len(nav_msgs.poses))
	point_number = 7

	for i in range(point_number):
		pose = PoseStamped()
        
		pose.header.stamp = rospy.Time.now()
		pose.header.frame_id = "robot_base_footprint"
		pose.pose.position.x = nav_msgs.poses[i].pose.position.x
		pose.pose.position.y = nav_msgs.poses[i].pose.position.y
		pose.pose.position.z = nav_msgs.poses[i].pose.position.z 
	
		path_msg.poses.append(pose)
	
	path_msg.header.stamp = rospy.Time.now()
	path_msg.header.frame_id = "robot_base_footprint"
	pub_path.publish(path_msg)

def publish_path():
	rospy.init_node('path_pub')	#definisco nome nodo
	pub_path = rospy.Publisher('/path_1', Path, queue_size=1)	

	rospy.Subscriber('/nav_path_5', Path, robot_pose_callback, pub_path)
	rospy.spin()
	
if __name__ == '__main__':
	try:
		publish_path()
	except rospy.ROSInterruptException:
		pass