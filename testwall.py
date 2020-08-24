#! /usr/bin/env python

import rospy
from math import sqrt
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point,Twist
from nav_msgs.msg import Odometry
from numpy import arange
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import message_filters
from math import atan2
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
class OdometryModifier:
    
    def __init__(self):
       
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        self.pub_o = rospy.Publisher('odom2', Odometry, queue_size=10)
        self.total_distance = 0.
        self.previous_x = 1
        self.previous_y = 1
        self.first_run = True
        self.test = 0
        self.subLaserScan = rospy.Subscriber('/scan', LaserScan,self.wallCtrl) #We subscribe to the laser's topic
        self.odomPub = rospy.Publisher('odom',Odometry, queue_size=10)
        self.angel_x = 0
        self.type_d = 0
        self.speed = 0.02
        self.pub  = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.PI = 3.1415926535897
        self.sub_g = rospy.Subscriber("odom", Odometry, self.get_rotation)
        self.pub_g = rospy.Publisher('odom2', Odometry, queue_size=10)
        self.sub_newOdom = rospy.Subscriber("odom", Odometry, self.newOdom)
        self.pub_newOdom = rospy.Publisher('odom2', Odometry, queue_size=10)
        self.kp=0.5
        self.yaw = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.target_rad = 0
        self.right_y = 0
        self.right_z = 0
        self.right_x = 0
        self.pub_cheak = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 10)

    def wallCtrl(self , msg):
        move = Twist()
        rate = rospy.Rate(5)
        dis = self.total_distance + 1.0
        print("wall right side distance :", self.right_y)

        if( self.right_y <= 1.3 and self.right_y >= 0.8):
            print "0------0", self.total_distance
            self.target_rad = self.angel_x * math.pi / 180
            move.angular.z = self.kp * (self.target_rad - self.yaw)
            move.linear.x = 0.1
            self.pub.publish(move)

            if msg.ranges[270]  == float('inf'): 
                print("1")
                robot_d = 0
                go = False
                print("Door--------------", self.right_x)
                
                if(self.right_x > 1.7 and dis < 3.0):
                    print("*Door 1-------")
                    move.linear.x = 0
                    move.angular.z = 0
                    self.angel_x = -90
                    self.type_d = 1
                    robot_d = self.total_distance + 2
                    go = True
                elif(self.right_x > 3.7 and dis < 5.0):
                    print("*Door 2-------")
                    move.linear.x = 0
                    move.angular.z = 0
                    self.angel_x = -180
                    self.type_d = 2
                    go = True
                    robot_d = self.total_distance + 2
                elif(self.right_x > 5.7 and dis < 7.0):
                    print("*Door 3-------")
                    move.linear.x = 0
                    move.angular.z = 0
                    self.angel_x = 90
                    self.type_d = 3
                    go = True
                    robot_d = self.total_distance + 2
                elif(self.right_x > 7.7):
                    print("*Door 4-------")
                    move.linear.x = 0
                    move.angular.z = 0
                    self.angel_x = 0
                    self.type_d = 4
                    go = True
                    robot_d = self.total_distance + 2
                self.pub.publish(move)

                if( go == True ):
                    while not rospy.is_shutdown():
                        self.target_rad = -90 * math.pi / 180
                        move.angular.z = self.kp * (self.target_rad - self.yaw)
                        move.linear.x = 0.1
                        self.pub.publish(move)
                        if( self.type_d == 1 ):
                            if( robot_d < self.total_distance):
                                print("1--",self.total_distance)
                                self.target_rad = self.angel_x * math.pi / 180
                                move.angular.z = self.kp * (self.target_rad-self.yaw)
                                move.linear.x = 0
                                self.pub.publish(move)
                                rate.sleep()
                        elif(self.type_d == 2 ):
                            if( robot_d < self.total_distance):
                                print("2--",self.total_distance)
                                self.target_rad = self.angel_x * math.pi / 180
                                move.angular.z = self.kp * (self.target_rad-self.yaw)
                                move.linear.x = 0
                                self.pub.publish(move)
                                rate.sleep()
                        elif(self.type_d == 3 ):
                            if( robot_d < self.total_distance):
                                print("3--",self.total_distance)
                                self.target_rad = self.angel_x * math.pi / 180
                                move.angular.z = self.kp * (self.target_rad-self.yaw)
                                move.linear.x = 0
                                self.pub.publish(move)
                                rate.sleep()
                        elif(self.type_d == 4 ):
                            if( robot_d < self.total_distance):
                                print("4--",self.total_distance)
                                self.target_rad = self.angel_x * math.pi / 180
                                move.angular.z = self.kp * (self.target_rad - self.yaw)
                                move.linear.x = 0
                                self.pub.publish(move)
                                rate.sleep()
        
        if( self.right_y > 1.3):
            print("2")
            if(self.right_x > 1 ): 
                self.target_rad = -180 * math.pi / 180
                move.angular.z = self.kp * (self.target_rad - self.yaw)
                move.linear.x = 0.1
                self.pub.publish(move)
                if msg.ranges[360] > 1.5:
                    move.linear.x = 0.5
                    move.angular.z = 0.0
                if msg.ranges[360] < 1.5:          
                    move.linear.x = 0.0
                    move.angular.z = 0.0
            else:
                
                self.target_rad = -90 * math.pi / 180
                move.angular.z = self.kp * (self.target_rad - self.yaw)
                move.linear.x = 0.1
                self.pub.publish(move)
                if msg.ranges[360] > 1.5:
                    move.linear.x = 0.5
                    move.angular.z = 0.0
                if msg.ranges[360] < 1.5:          
                    move.linear.x = 0.0
                    move.angular.z = 0.0
            self.pub.publish(move)

        if(self.right_y < 0.7):
            if(self.right_x > 1 ): 
                self.target_rad = -180 * math.pi / 180
                move.angular.z = self.kp * (self.target_rad - self.yaw)
                move.linear.x = 0.05
                self.pub.publish(move)
                print(len(msg.ranges))
                if msg.ranges[360] > 1.0:
                    move.linear.x = 0.05
                    move.angular.z = 0.0
                if msg.ranges[360] < 1.0:          
                    move.linear.x = 0.0
                    move.angular.z = 0.0
            else: 
                self.target_rad = 90 * math.pi / 180
                move.angular.z = self.kp * (self.target_rad - self.yaw)
                move.linear.x = 0.01
                self.pub.publish(move)
                print(len(msg.ranges))
                if msg.ranges[360] > 1.0:
                    move.linear.x = 0.01
                    move.angular.z = 0.0
                if msg.ranges[360] < 1.0:          
                    move.linear.x = 0.0
                    move.angular.z = 0.0
            self.pub.publish(move)

    def callback(self, data):
        self.right_x = data.pose.pose.position.x
        self.right_y = data.pose.pose.position.y
        self.right_z = data.pose.pose.position.z
        if(self.first_run):
            self.previous_x = data.pose.pose.position.x
            self.previous_y = data.pose.pose.position.y
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        d_increment = sqrt((x - self.previous_x)**2 + (y - self.previous_y)**2)
        self.total_distance = self.total_distance + d_increment
        self.pub_o.publish(data)
        self.previous_x = data.pose.pose.position.x
        self.previous_y = data.pose.pose.position.y
        self.first_run = False

    def get_rotation (self, msg):
        roll = pitch = 0.0
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion (orientation_list)

    def newOdom(self, msg):
        
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        roll = pitch = 0.0
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])  
    

rospy.init_node('rotw5_node', anonymous=True)
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 10)
odom = OdometryModifier()
     
rospy.spin()

