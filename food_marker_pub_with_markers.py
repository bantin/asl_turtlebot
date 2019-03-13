#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D
from asl_turtlebot.msg import DetectedObject, DetectedObjectList
import tf
import numpy as np

# max distance to detect fun_stuff object
MAX_DETECT_DIST = 1


class FoodMarkerPublisher():
    def __init__(self):
        rospy.init_node('food_publisher', anonymous=True)
        
        rospy.Subscriber('/delivery_request', String, self.delivery_request_callback)
                                             
        self.food_goal_pub = rospy.Publisher('/food_pose', Pose2D, queue_size=10) 
        self.go_home_pub = rospy.Publisher('/home', String, queue_size = 10)
        self.marker_pub = rospy.Publisher('/seen_food_markers', MarkerArray, queue_size=10)

        # for viz
        #self.pub = rospy.Publisher('/food_marker', MarkerArray, queue_size=10)
        rospy.Subscriber('/detector/objects', DetectedObjectList, self.food_callback)

        rospy.Subscriber('/get_food', String, self.send_request_callback)

        self.trans_listener = tf.TransformListener()
        self.food_dict = {} 

        # Testing 
        self.fun_stuff = ['laptop','hot dog', 'banana', 'apple', 'orange', 'stop sign', 'bottle']

        #self.object_list = []
        self.msg_list = []

    def robot_pose(self):
        try: 
            origin_frame = "/map" 
            t, r = self.trans_listener.lookupTransform(origin_frame, "/base_footprint", rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(r)
            return t[0], t[1], euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("ROBOT POSE FOOD MARKER BAD!!!")
            print e
            pass

    def delivery_request_callback(self, msg):
        request = msg.data
        print(request)
        self.desired_foods = request.split(',')

        self.send_request_callback("First food")

    def send_request_callback(self, msg):
        if len(self.desired_foods) > 0:
            next_food = self.desired_foods.pop()
            # Do GrANT THING FoR OpTiMiZaTiOn
            
            #think about timing issues?
            if next_food in self.food_dict.keys():
                print("request in keys")
                pose_food = Pose2D()
                pose_food.x = self.food_dict[next_food][0]
                pose_food.y = self.food_dict[next_food][1]
                pose_food.theta = self.food_dict[next_food][2]
                print(pose_food)
                t = 5
                s = 1.
                for i in range(t):
                    self.food_goal_pub.publish(pose_food)
                    rospy.sleep(s)
            else:
                rospy.loginfo("Food item not found in explore phase.")

        else:
            print("Time to go home")
            self.go_home_pub.publish("Time to go home")
            
    
    def food_to_world_frame(self, msg):
        dist = np.max(msg.distance - 0.1, 0)
        robo_x, robo_y, robo_theta = self.robot_pose()
        print(self.robot_pose())
        food_x = robo_x + dist * np.cos(robo_theta)
        food_y = robo_y + dist * np.sin(robo_theta)
        return (food_x, food_y, robo_theta, msg.distance)
        

    def food_callback(self, data):
        ######### YOUR CODE HERE ############
        # make list of whitelisted (aka 'fun' objects) 
        inds = []
        for i,obj in enumerate(data.objects):
            if obj in self.fun_stuff:
                if obj in self.food_dict:
                    lastDist = self.food_dict[obj][-1] 
                    curDist = data.ob_msgs[i].distance
                    if curDist < lastDist:
                        self.food_dict[obj] = self.food_to_world_frame(data.ob_msgs[i])
                        print(obj, " updated in keys")
                else:
                    self.food_dict[obj] = self.food_to_world_frame(data.ob_msgs[i])
                    print(obj, " added to keys")

                #self.object_list.append(obj)
                #inds.append(i)

        #self.msg_list = data.ob_msgs[inds]

        ######### END OF YOUR CODE ##########

    def run(self):
        #x, y, theta = self.robot_pose()
        markerArray = MarkerArray()

        while not rospy.is_shutdown():
            for food in self.food_dict.keys():
    	        circle = Marker()
    	        circle.header.frame_id = "map"
    	        circle.header.stamp = rospy.Time.now()
    	        circle.type = Marker.SPHERE
    	        circle.action = Marker.ADD
    	        circle.pose.position.x = self.food_dict[food][0] 
    	        circle.pose.position.y = self.food_dict[food][1]
    	        circle.pose.position.z = 0.0
    	        circle.pose.orientation.x = 0;
    	        circle.pose.orientation.y = 0;
    	        circle.pose.orientation.z = 0;
    	        circle.pose.orientation.w = 1;
    	        circle.scale.x = 0.15
    	        circle.scale.y = 0.15
    	        circle.scale.z = 0.1
    	        circle.color.a = 1.0
    	        circle.color.r = 0.0
    	        circle.color.g = 0.0
    	        circle.color.b = 1.0
    	        circle.id = food
                #marker = Marker(
                #type=Marker.SPHERE,
                #id=0,
                #lifetime=rospy.Duration(20),
                #pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                #header=Header(frame_id='base_footprint', stamp=rospy.Time(0)),
                #color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                #scale=(5,5,5))
                markerArray.append(circle)

            self.marker_pub.publish(markerArray)

   	    rospy.sleep(0.01)

if __name__ == '__main__':
    m = FoodMarkerPublisher()
    rospy.spin()
    # print "running target marker"
    # m.run()
