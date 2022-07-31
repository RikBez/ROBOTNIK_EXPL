#!/usr/bin/env python
from sys import getsizeof
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TwistStamped, Pose, Point, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA, String, Int32MultiArray

class TrajectoryInteractiveMarkers:

    
    def __init__(self):
            self.count = 0 
            
            #self.point1 = fi_point
            #self.point2 = sec_point
            #Point(x, y)       
            #rospy.Subscriber("/arm_1/arm_controller/cartesian_velocity_command",TwistStamped, self.event_in_cb)
            rospy.Subscriber("/target_vector", Int32MultiArray, self.event_in_cb)
            rospy.sleep(0.15)

    def event_in_cb(self,msg):
        self.waypoints = msg
        
        self.ycost = 200 ### 184?
        self.yscale = 0.07 # the delta from the mapping

        self.xcost = 214 ### 214
        self.xscale = 0.07

        #per costruzione delle mappe, 186 circa 3 qui
        #divido per 62 gli elementi di self.a
        dim = getsizeof(msg.data)

        rospy.loginfo('dimension %d', dim)

        self.count = 1

        if dim > 56:
            self.old = self.a = [msg.data[0] - self.xcost, -(msg.data[1] - self.ycost), msg.data[2] - self.xcost, -(msg.data[3] - self.ycost), msg.data[4] - self.xcost, -(msg.data[5] - self.ycost), msg.data[6] - self.xcost, -(msg.data[7] - self.ycost), msg.data[8] - self.xcost, -(msg.data[9] - self.ycost), msg.data[10] - self.xcost, -(msg.data[11] - self.ycost)]
            #self.memory = 1
            for i in range(12):
                if (msg.data[i] == 0):
                        self.a[i] = 0
                i = i + 1
        else:
            self.a = self.old

        self.show_text_in_rviz()

    def show_text_in_rviz(self):
        k = 0

        self.marker = Marker()
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size = 12)
        #for i in (range(6)):    
        self.marker = Marker(
                type=Marker.LINE_STRIP,
                id=1,
                lifetime=rospy.Duration(1),
                pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.05, 0.05, 0.05),
                header=Header(frame_id='odom_bis'),
                color=ColorRGBA(1.0 * 2, 0.5 * 2, 1.0 *32, 21 * 2),
                points = [Point(0, 0, 0), Point((self.a[k]*self.xscale), self.a[k + 1]*self.yscale, 0)]
                #points = [Point(0, 0, 0), Point(3.5, -1.8, 0)]
                )
        #self.count += 1
        k = k + 2

        self.markert = Marker()
        #self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size = 2)
        #for i in (range(6)):    
        self.markert = Marker(
                type=Marker.LINE_STRIP,
                id=2,
                lifetime=rospy.Duration(1),
                pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.05, 0.05, 0.05),
                header=Header(frame_id='odom_bis'),
                color=ColorRGBA(9.0 * k * 3, 15.0 * k * 4, 1.0 * k * 5, 3 * k * 8), 
                points = [Point(0, 0, 0), Point((self.a[k]*self.xscale), self.a[k + 1]*self.yscale, 0)]
                )
        #self.count += 1
        k = k + 2

        self.markera = Marker()
        #self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size = 2)
        #for i in (range(6)):    
        self.markera = Marker(
                type=Marker.LINE_STRIP,
                id=6,
                lifetime=rospy.Duration(1),
                pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.05, 0.05, 0.05),
                header=Header(frame_id='odom_bis'),
                color=ColorRGBA(10.0 * k * 3, 13.0 * k * 4, 8.0 * k * 5, 4 * k * 8), 
                points = [Point(0, 0, 0), Point((self.a[k]*self.xscale), self.a[k + 1]*self.yscale, 0)]
                )
        #self.count += 1
        k = k + 2

        self.markerr = Marker()
        #self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size = 2)
        #for i in (range(6)):    
        self.markerr = Marker(
                type=Marker.LINE_STRIP,
                id=3,
                lifetime=rospy.Duration(1),
                pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.05, 0.05, 0.05),
                header=Header(frame_id='odom_bis'),
                color=ColorRGBA(2.0 * k * 3, 9.0 * k * 4, 1.0 * k * 5, 6 * k * 8), 
                points = [Point(0, 0, 0), Point((self.a[k]*self.xscale), self.a[k + 1]*self.yscale, 0)]
                )
        #self.count += 1
        k = k + 2

        self.markerg = Marker()
        #self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size = 2)
        #for i in (range(6)):    
        self.markerg = Marker(
                type=Marker.LINE_STRIP,
                id=4,
                lifetime=rospy.Duration(1),
                pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.05, 0.05, 0.05),
                header=Header(frame_id='odom_bis'),
                color=ColorRGBA(5.0 * k * 3, 3.0 * k * 4, 1.0 * k * 5, 8 * k * 8), 
                points = [Point(0, 0, 0), Point((self.a[k]*self.xscale), self.a[k + 1]*self.yscale, 0)]
                )
        #self.count += 1
        k = k + 2

        self.markere = Marker()
        #self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size = 2)
        #for i in (range(6)):    
        self.markere = Marker(
                type=Marker.LINE_STRIP,
                id=5,
                lifetime=rospy.Duration(1),
                pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.05, 0.05, 0.05),
                header=Header(frame_id='odom_bis'),
                color=ColorRGBA(1.0 * k * 3, 1.0 * k * 4, 1.0 * k * 5, 1 * k * 8), 
                points = [Point(0, 0, 0), Point((self.a[k]*self.xscale), self.a[k + 1]*self.yscale, 0)]
                )
        #self.count += 1
        #self.marker.id = self.count
        k = k + 2

        self.marker_publisher.publish(self.marker)
        #rospy.sleep(0.05)
        self.marker_publisher.publish(self.markert)
        #rospy.sleep(0.05)
        self.marker_publisher.publish(self.markera)
        #rospy.sleep(0.05)
        self.marker_publisher.publish(self.markerr)
        #rospy.sleep(0.05)
        self.marker_publisher.publish(self.markerg)
        #rospy.sleep(0.05)
        self.marker_publisher.publish(self.markere)

        # self.marker = Marker()
        # self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size = 2)

        # for x in range(6):
        #     self.marker = Marker(
        #                       type=Marker.LINE_STRIP,
        #                 id=x,
        #                 lifetime=rospy.Duration(3),
        #                 pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
        #                 scale=Vector3(0.05, 0.05, 0.05),
        #                 header=Header(frame_id='odom_bis'),
        #                 color=ColorRGBA(5.0 * k, 10.0 * k, 1.0 * k, 0.8), 
        #                 # marker line points
        #                 #fi_point = Point(),
        #                 #sec_point = Point(),
        #                 points = [Point( , , 1), Point( , , 1)]
        #                 # points = self.list
        #                 )
        #     self.count+=1
        #     self.marker.id = self.count
        #     self.marker_publisher.publish(self.marker)
        rospy.loginfo('msg published')

if __name__ == '__main__':
    rospy.init_node("trajectory_interactive_markers_node", anonymous=True)
    trajectory_interactive_markers = TrajectoryInteractiveMarkers()
    rospy.sleep(0.1)
    rospy.spin()
