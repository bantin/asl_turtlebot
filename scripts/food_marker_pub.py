#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Twist, PoseArray, Pose2D
from asl_turtlebot.msg import DetectedObject, DetectedObjectList

class FoodMarkerPublisher():
    def __init__(self):
        rospy.init_node('food_publisher', anonymous=True)

        self.pub= rospy.Publisher('/food_marker', MarkerArray, queue_size=10)
        rospy.Subscriber('/detector/objects', DetectedObjectList, self.food_callback)

        rospy.Subscriber('/detector/objects', DetectedObjectList, self.food_callback)
        self.object_list = []
        self.msg_list = []

    def food_callback(self, data):
        ######### YOUR CODE HERE ############
        # fill out cmd_pose_callback
        self.object_list = data.objects
        self.msg_list = data.ob_msgs

        ######### END OF YOUR CODE ##########

    def run(self):
    	markerArray = MarkerArray()
        while not rospy.is_shutdown():
        	for msg in self.msg_list:
	            circle = Marker()
	            circle.header.frame_id = "map"
	            circle.header.stamp = rospy.Time.now()
	            circle.type = Marker.SPHERE
	            circle.action = Marker.ADD
	            circle.pose.position.x = msg.distance 

	            circle.pose.position.y = self.y_g
	            circle.pose.position.z = 0.0
	            circle.pose.orientation.x = 0;
	            circle.pose.orientation.y = 0;
	            circle.pose.orientation.z = 0;
	            circle.pose.orientation.w = 1;
	            circle.scale.x = 0.15
	            circle.scale.y = 0.15
	            circle.scale.z = 0.1
	            circle.color.a = 1.0
	            circle.color.r = 1.0
	            circle.color.g = 0.0
	            circle.color.b = 0.0
	            circle.id = msg.name
            # marker = Marker(
            #     type=Marker.SPHERE,
            #     id=0,
            #     lifetime=rospy.Duration(20),
            #     pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
            #     header=Header(frame_id='base_footprint', stamp=rospy.Time(0)),
            #     color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
            #     scale=(5,5,5))
            self.pub.publish(markerArray)
   			rospy.sleep(0.01)

if __name__ == '__main__':
    m = FoodMarkerPublisher()
    print "running target marker"
    m.run()