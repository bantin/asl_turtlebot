#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Twist, PoseArray, Pose2D

class MarkerPublisher():
    def __init__(self):
        rospy.init_node('marker_publisher', anonymous=True)

        self.pub1= rospy.Publisher('/turtle_marker', Marker, queue_size=10)
        self.pub2 = rospy.Publisher('/goal_marker', Marker, queue_size=10)
        rospy.Subscriber('/cmd_pose', Pose2D, self.cmd_pose_callback)

        # goal state
        self.x_g = 0.0
        self.y_g = 0.0
        self.theta_g = 0.0

    def cmd_pose_callback(self, data):

        self.x_g = data.x
        self.y_g = data.y
        self.theta_g = data.theta

    def run(self):
        while not rospy.is_shutdown():
            circle = Marker()
            circle.header.frame_id = "base_footprint"
            circle.header.stamp = rospy.Time.now()
            circle.type = Marker.SPHERE
            circle.action = Marker.ADD
            circle.pose.position.x = 0
            circle.pose.position.y = 0
            circle.pose.position.z = 0
            circle.pose.orientation.x = 0;
            circle.pose.orientation.y = 0;
            circle.pose.orientation.z = 0;
            circle.pose.orientation.w = 1;
            circle.scale.x = 0.15
            circle.scale.y = 0.15
            circle.scale.z = 0.1
            circle.color.a = 1.0
            circle.color.r = 0.0
            circle.color.g = 1.0
            circle.color.b = 0.0


            cube = Marker()
            cube.header.frame_id = "map"
            cube.header.stamp = rospy.Time.now()
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = self.x_g
            cube.pose.position.y = self.y_g
            cube.pose.position.z = 0.0
            cube.pose.orientation.x = 0.0
            cube.pose.orientation.y = 0.0
            cube.pose.orientation.z = 0.0
            cube.pose.orientation.w = 1.0
            cube.scale.x = 0.15
            cube.scale.y = 0.15
            cube.scale.z = 0.1
            cube.color.a = 1.0
            cube.color.r = 1.0
            cube.color.g = 0.0
            cube.color.b = 0.0
            self.pub1.publish(circle)
            self.pub2.publish(cube)
            rospy.sleep(0.01)

if __name__ == '__main__':
    m = MarkerPublisher()
    print "running marker"
    m.run()
