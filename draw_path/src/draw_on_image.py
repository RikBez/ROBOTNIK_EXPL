#!/usr/bin/env python

# Ros libraries
import rospy

# Python libraries
import numpy as np
import cv2
import time

# Ros Messages
from sensor_msgs.msg import CameraInfo, Image
from rospy.numpy_msg import numpy_msg

# The CvBridge is an object that converts between OpenCV Images and ROS Image messages.
from cv_bridge import CvBridge, CvBridgeError


class PT2Cam():

    def __init__(self):

        ###### PUBLISHING ######
        # publish topic for image with point clouds projected        
        self.img_pub = rospy.Publisher('/camera_RB', Image, queue_size = 1)

        rospy.init_node("PC2img_node", anonymous=False)
        
        rospy.loginfo_once("Publishing to the topic %s", '\033[1m' + self.img_pub.name + '\033[0m')

        rate_node = rospy.Rate(50) # 10hz

        while not rospy.is_shutdown():

            # class that makes convertion between OpenCV Images and ROS Image messages
            self.bridge = CvBridge()

            ###### SUBSCRIBING ######
            # subscribing to point cloud numpy
            rospy.Subscriber("/robot/front_rgbd_camera/rgb/image_raw", Image, self.clbk_img)

            rospy.spin()

    def clbk_img(self, img_msg):
        
        ##### CAMERA DATA #####
        # convert ROS message to cv2 image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
      
        cv2.circle(cv_image, (50,50), 10, 255)
        
        # can't display from python, there is a bug with ros
        # cv2.imshow("Point Cloud projected", cv_image)
        # cv2.waitKey(3)

        try:
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    try:
        L2PT = PT2Cam()
    except rospy.ROSInterruptException:
        print("Shutting down")
        cv2.destroyAllWindows()
        pass
