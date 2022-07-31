#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

def callback(yaw_rate, forward_vel,pub_vel):
	cmd_vel_msg = Twist()
    
	cmd_vel_msg.linear.x = forward_vel.data
	cmd_vel_msg.linear.y = 0
	cmd_vel_msg.linear.z = 0
	cmd_vel_msg.angular.z = -yaw_rate.data
	cmd_vel_msg.angular.x = 0
	cmd_vel_msg.angular.y = 0
	pub_vel.publish(cmd_vel_msg)

def joy_vel_task():
	rospy.init_node('joy_cmd_vel')	#definisco nome nodo
	pub_vel = rospy.Publisher('/robot/robotnik_base_control/cmd_vel', Twist, queue_size=1)	#definisco pubblicazione

	yaw_rate_sub_ = message_filters.Subscriber('unity/yaw_rate', Float32)
	forward_vel_sub_ = message_filters.Subscriber('unity/forward_vel', Float32)

	ts = message_filters.ApproximateTimeSynchronizer([yaw_rate_sub_, forward_vel_sub_], 10, 0.1, allow_headerless=True)
	ts.registerCallback(callback,pub_vel)
	rospy.spin()
	
if __name__ == '__main__':
	try:
		joy_vel_task()
	except rospy.ROSInterruptException:
		pass
