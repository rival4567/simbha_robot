#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class GoForward():
    def __init__(self):
        rospy.init_node('forward', anonymous=False)
        rospy.loginfo("To stop robot CTRL + C")
        # rospy.on_shutdown(self.shutdown)
        rospy.Subscriber('scan',LaserScan,self.range)
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.destination)
        rospy.Subscriber('odom', Odometry, self.robot_parameters)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # r = rospy.Rate(10);
        self.move_cmd = Twist()
        self.center = 0
        self.right =0
        self.left = 0
        self.right_index =0
        self.left_index=0
        self.EulerAngles=[0,0,0]
        self.final_x = 2
        self.final_y = 1
        self.CurrentX = 0
        self.CurrentY = 0
        self.heading =False
        self.new_goal =True
        # while not rospy.is_shutdown():
	    # publish the velocity
            # if(self.range_v[0] >):

            # self.navigation()
            # self.forward()
            # self.cmd_vel.publish(self.move_cmd)
	    # wait for 0.1 seconds (10 HZ) and publish again
            # r.sleep()
    def robot_parameters(self, robot):
        self.CurrentX = robot.pose.pose.position.x;
        self.CurrentY = robot.pose.pose.position.y;

        # self.inclination_angle = math.atan2(self.final_y - self.CurrentY, self.final_x - self.CurrentX);

        quatx = robot.pose.pose.orientation.x;
        quaty = robot.pose.pose.orientation.y;
        quatz = robot.pose.pose.orientation.z;
        quatw = robot.pose.pose.orientation.w;

        x2 = pow(quatx, 2);
        y2 = pow(quaty, 2);
        z2 = pow(quatz, 2);
        w2 = pow(quatw, 2);

        self.EulerAngles[0] = (math.atan2(2.0 * (quaty * quatz + quatx * quatw),(-x2 - y2 + z2 + w2)) * (180.0/math.pi));
        self.EulerAngles[1] = (math.asin(-2.0 * (quatx * quatz - quaty * quatw)) * (180.0/math.pi));
        self.EulerAngles[2] = (math.atan2(2.0 * (quatx * quaty + quatz * quatw),(x2 - y2 - z2 + w2)) * (180.0/math.pi));

        # print self.EulerAngles[2]

    def destination(self,point):
        self.final_x = point.pose.position.x
        self.final_y = point.pose.position.y
        self.heading =False
        self.new_goal =True
        # while(self.heading ==False):
        #     slope,line_angle,distance = self.path_calculation(self.final_x, self.CurrentX, self.final_y, self.CurrentY)
        #     self.turn_toward_destination(line_angle, self.EulerAngles[2])
        #     self.heading = True
    #     # print self.final_x , self.final_y
    def distance(self, x2, x1, y2, y1):
        distance = math.sqrt( ( (x2 - x1) ** 2) + ( (y2 - y1) ** 2) )
        return distance
    def path_calculation(self, x2, x1, y2, y1):
        slope = (x2-x1)/(y2-y1)
        distance = math.sqrt( ( (x2 - x1) ** 2) + ( (y2 - y1) ** 2) )
        inclination_angle = math.atan2(y2 - y1, x2 - x1);
        return slope, inclination_angle,distance

    def navigation(self):
        print "navigation"
        obstacle =0
        right_take =0
        left_take=0
        self.heading =False
        rospy.sleep(.2)
        startx = self.CurrentX
        starty = self.CurrentY
        while(True) & (self.new_goal==True):
            # print "again inside"
            while(self.heading==False):
                slope,line_angle,distance = self.path_calculation(self.final_x, self.CurrentX, self.final_y, self.CurrentY)
                self.turn_toward_destination(line_angle, self.EulerAngles[2])
                # print distance
            if(self.center >= .6) &(obstacle==0):
                # print"here i am"
                # distance = self.distance(self.final_x, self.CurrentX, self.final_y, self.CurrentY)
                # if(distace > pre_distance):
                self.follow_slop_line(startx,starty)
            elif(obstacle==0):
                if(self.right >self.left):
                    self.right_turn()
                    print "right_turn"
                    self.move_cmd.linear.x=0.0
                    self.move_cmd.linear.y=0.0
                    self.move_cmd.linear.z=0.0
                    self.move_cmd.angular.x= 0.0
                    self.move_cmd.angular.y= 0.0
                    self.move_cmd.angular.z= 0.0
                    self.cmd_vel.publish(self.move_cmd)
                    # rospy.sleep(.2)
                    right_take = 1
                else:
                    print"left_turn"
                    self.left_turn_90()
                    self.move_cmd.linear.x=0.0
                    self.move_cmd.linear.y=0.0
                    self.move_cmd.linear.z=0.0
                    self.move_cmd.angular.x= 0.0
                    self.move_cmd.angular.y= 0.0
                    self.move_cmd.angular.z= 0.0
                    self.cmd_vel.publish(self.move_cmd)
                    # rospy.sleep(.2)
                    left_take = 1
                self.stop()
                obstacle =1
            if (obstacle==1) & (right_take==1):
                self.right_wall_follow()
                break
            if (obstacle==1) & (left_take==1):
                self.left_wall_follow()
                break   
        self.stop()
        print "goal achieved"

            # self.heading = True
                # self.cmd_vel.publish(self.move_cmd)
    def degree_to_rad(self,deg):
        return 2 * math.pi * (deg/180)
    def forward(self):

        self.move_cmd.linear.x = 0.2
        self.move_cmd.angular.z = 0.0
        self.cmd_vel.publish(self.move_cmd)
    def right_wall_follow(self):
        while(True):
            if(self.left < .7):
                self.right_move()
            if(self.left >.7):
                self.left_move()
            if(self.left > 10.0 ):
                self.aviod_obstacle(0)
                rospy.sleep(5)
                self.stop()
                self.heading=False
                self.obstacle = 0
                print "stoped"
                break
            else:
                self.forward()

    def follow_slop_line(self,startx,starty):
        while(True):
            slope,line_angle,distance = self.path_calculation(self.final_x, startx, self.final_y, starty)
            # offset = self.final_y - slope * self.final_x
            # print slope, offset, startx, starty
            line =(self.CurrentX * slope)
            # print line, self.CurrentY
            if(self.CurrentY <= (self.CurrentX * slope) + 1) & ( self.CurrentY >= (self.CurrentX * slope)- 1) :
                if line_angle - self.degree_to_rad(self.EulerAngles[2] <0.01):
                    self.forward()
                    # print "moving forward`"
                    if(self.center <.6):
                        distance = self.distance(self.final_x, self.CurrentX, self.final_y, self.CurrentY)
                        print distance
                        break
                else:
                    self.turn_toward_destination(line_angle, self.EulerAngles[2])
                    distance = self.distance(self.final_x, self.CurrentX, self.final_y, self.CurrentY)
                    print distance
                    break
            else:
                self.heading=False
                self.turn_toward_destination(line_angle, self.EulerAngles[2])
                self.forward()
                rospy.sleep(.1)
                print "set heading"
                distance = self.distance(self.final_x, self.CurrentX, self.final_y, self.CurrentY)
                # print distance
                if (distance < 3):
                    self.new_goal =False
                    break
                    pre_distance =distance
                if(self.center <.6):
                    print self.center
                    break



    def left_wall_follow(self):
        while(True):
            if(self.right > 0.7):
                self.right_move()
            if (self.right < 0.7):
                self.left_move()
            if (self.right > 10.0):
                self.aviod_obstacle(0)
                rospy.sleep(5)
                self.heading=False
                self.obstacle = 0
                self.stop()
                print "stoped"
                break
            else:
                self.forward()

    def stop(self):
        self.move_cmd.linear.x=0.0
        self.move_cmd.linear.y=0.0
        self.move_cmd.linear.z=0.0
        self.move_cmd.angular.x= 0.0
        self.move_cmd.angular.y= 0.0
        self.move_cmd.angular.z= 0.0
        self.cmd_vel.publish(self.move_cmd)
    def aviod_obstacle(self,velocity):
        self.move_cmd.linear.x=0.2
        self.move_cmd.linear.y=0.0
        self.move_cmd.linear.z=0.0
        self.move_cmd.angular.x= 0.0
        self.move_cmd.angular.y= 0.0
        self.move_cmd.angular.z= velocity
        self.cmd_vel.publish(self.move_cmd)

    def right_turn(self):
        while(self.center <= 10.0):
            self.move_cmd.linear.x=0.0
            self.move_cmd.linear.y=-0.0
            self.move_cmd.linear.z=0.0
            self.move_cmd.angular.x= 0.0
            self.move_cmd.angular.y= 0.0
            self.move_cmd.angular.z= -0.2
            self.cmd_vel.publish(self.move_cmd)
            # print "right_turning"

    def left_turn(self):
        while(self.center <= 10.0):
            self.move_cmd.linear.x=0.0
            self.move_cmd.linear.y=0.0
            self.move_cmd.linear.z=0.0
            self.move_cmd.angular.x= 0.0
            self.move_cmd.angular.y= 0.0
            self.move_cmd.angular.z= 0.2
            self.cmd_vel.publish(self.move_cmd)
            # print "left_turning"
    def left_turn_90(self):
   
        self.move_cmd.linear.x=0.0
        self.move_cmd.linear.y=0.0
        self.move_cmd.linear.z=0.0
        self.move_cmd.angular.x= 0.0
        self.move_cmd.angular.y= 0.0
        self.move_cmd.angular.z= 0.4
        self.cmd_vel.publish(self.move_cmd)
        rospy.sleep(2.5)
    def right_move(self):
        self.move_cmd.linear.x= 0.1
        self.move_cmd.angular.z =-0.3
        self.cmd_vel.publish(self.move_cmd)
        # while not(self.center == 30.0):
        #     break
    def left_move(self):
         self.move_cmd.linear.x=0.1
         self.move_cmd.angular.z= 0.3
         self.cmd_vel.publish(self.move_cmd)
         # while not(self.center == 30.0):
         #     break
    def turn_toward_destination(self, line_angle, robot_angle ):
        r_angle = self.degree_to_rad(robot_angle)
        turn_angle = line_angle - r_angle
        # print turn_angle
        self.move_cmd.linear.x=0.0
        self.move_cmd.linear.y=0.0
        self.move_cmd.linear.z=0.0
        self.move_cmd.angular.x= 0.0
        self.move_cmd.angular.y= 0.0
        self.move_cmd.angular.z= turn_angle

        self.cmd_vel.publish(self.move_cmd)

        if(abs(turn_angle) <.001):
            self.move_cmd.linear.x=0.0
            self.move_cmd.linear.y=0.0
            self.move_cmd.linear.z=0.0
            self.move_cmd.angular.x= 0.0
            self.move_cmd.angular.y= 0.0
            self.move_cmd.angular.z= 0.0

            self.cmd_vel.publish(self.move_cmd)
            self.heading =True


    def range(self,range_value):
        range_v = range_value.ranges
        self.center = range_v[350]
        self.right = range_v[0]
        self.left = range_v[699]
        self.center_to_left=range_v[350:]
        self.center_to_right = range_v[350: :-1]
        # print self.center_to_left
        # self.left_index = self.center_to_left.index(30.0)
        # self.right_index = self.center_to_right.index(30.0)
        # print "left_index = " + str(self.left_index), "right index = " + str(self.right_index) 
        # print type (self.range_v)
        # print self.range_v[0], self.range_v[349], self.range_v[699]
    # def shutdown(self):
    #     rospy.loginfo("Stop robot")
    #     self.cmd_vel.publish(Twist())
    #     rospy.sleep(1)
        
if __name__ == '__main__':
    # try:
    
    test = GoForward()
    while(True):
        test.navigation()
    # except:
    #     rospy.loginfo("GoForward node terminated.")

