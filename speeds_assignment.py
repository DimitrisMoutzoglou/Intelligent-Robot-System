#!/usr/bin/env python

import rospy
import math
import time

from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation

# Class for assigning the robot speeds
class RobotController:

    # Constructor
    def __init__(self):

      # Debugging purposes
      self.print_velocities = rospy.get_param('print_velocities')

      # Where and when should you use this?
      self.stop_robot = False

      # Create the needed objects
      self.sonar_aggregation = SonarDataAggregator()
      self.laser_aggregation = LaserDataAggregator()
      self.navigation  = Navigation()

      self.linear_velocity  = 0
      self.angular_velocity = 0
      self.emergency=0  # variable used in case the robot is very close to obstacle in wandering mode
      # Check if the robot moves with target or just wanders
      self.move_with_target = rospy.get_param("calculate_target")

      # The timer produces events for sending the speeds every 110 ms
      rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)
      self.velocity_publisher = rospy.Publisher(\
              rospy.get_param('speeds_pub_topic'), Twist,\
              queue_size = 10)

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):

      # Produce speeds
      self.produceSpeeds()

      # Create the commands message
      twist = Twist()
      twist.linear.x = self.linear_velocity
      twist.linear.y = 0
      twist.linear.z = 0
      twist.angular.x = 0
      twist.angular.y = 0
      twist.angular.z = self.angular_velocity

      # Send the command
      self.velocity_publisher.publish(twist)

      # Print the speeds for debuggind purposes
      if self.print_velocities == True:
        print "[L,R] = [" + str(twist.linear.x) + " , " + \
            str(twist.angular.z) + "]"

    # Produces speeds from the laser
    def produceSpeedsLaser(self):
      scan = self.laser_aggregation.laser_scan

      linear  = 0
      angular = 0
      max_mean=0    #the maximum mean of lidar values of all segments
      max_index=0      #the segment with the maximum mean of lidar values
      light_width=30   #the lidar values are divided in 30 segments

      # in case of emergency, the robot is set in reverse motion
      if self.emergency!=0:
        linear=-0.3
        angular=self.emergency*0.15
        if self.emergency>0:
          self.emergency=self.emergency-1
        else:
 	  self.emergency=self.emergency+1
        return[linear,angular]


      #the lidar values are divided in segments and for each segment a mean is calculated .
      #the index of the segment with the maximum mean determines the direction of the robot

      k=0.3/len(scan)
      for i in range(0,22):
        sum=0
      	for j in range(1,light_width):
          sum=sum+scan[i*light_width+j]
       	if sum/light_width>=max_mean:
          max_mean=sum/light_width
          max_index=i*light_width+15

      angular=k*(max_index-len(scan)/2)  #the angular velocity is proportional to the value of the angle the robot has to turn
      linear=(2*(0.3*scan[333]))/10     #the linear velocity is proportional of the distance between the robot and the obstacles directly ahead of it
                                        #10 is by experience the maximum value the scan can read

      if linear>0.3:    #restrict the values of speeds under 0.3
      	linear=0.3
      for i in range(0,100):
        if scan[233+i]<0.3:
          linear=-0.3
          angular=0.3
          self.emergency=2
        elif scan[334+i]<0.3:
          linear=-linear
	  angular=-0.3
          self.emergency=-2

      ############################### NOTE QUESTION ############################
      # Check what laser_scan contains and create linear and angular speeds
      # for obstacle avoidance

      ##########################################################################
      return [linear, angular]

    # Combines the speeds into one output using a motor schema approach
    def produceSpeeds(self):

      # Produce target if not existent
      if self.move_with_target == True and \
              self.navigation.target_exists == False:

        # Create the commands message
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        # Send the command
        self.velocity_publisher.publish(twist)
        self.navigation.selectTarget()

      # Get the submodule's speeds
      [l_laser, a_laser] = self.produceSpeedsLaser()

      # You must fill these
      self.linear_velocity  = l_laser
      self.angular_velocity = a_laser

      if self.move_with_target == True:
        [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()

        self.linear_velocity=l_goal
        self.angular_velocity=a_goal
        ############################### NOTE QUESTION ############################
        # You must combine the two sets of speeds. You can use motor schema,
        # subsumption of whatever suits your better.
        u_obs=0
        w_obs=0
        scan=self.laser_aggregation.laser_scan

        #calculation of u_obs and w_obs according to formulas
        #the range of the lidar values used for u_obs correspond to 60 frontal degrees
        #and the range of the values used for w_obs correspond to 60 degrees sideways of the robot
        for i in range(260,410):
          theta=(30*(333-i))/72
          u_obs+=(math.cos(math.radians(theta)))/pow(10*scan[i],2)
        u_obs=-u_obs

        for i in range(0,149):
          theta=60+i*0.4047
          w_obs+=(math.sin(math.radians(theta)))/pow(10*scan[i+407],2)  #left scan  w_obs
        for i in range(0,149):
          theta=-120+i*0.4047             # 0.4047 is the proportion between cells and degrees
          w_obs+=(math.sin(math.radians(theta)))/pow(10*scan[i],2)  #right scan  w_obs
        w_obs=-w_obs

        linear=0.1*u_obs      #multiply speeds with scaling factor derived by trials
        angular=0.045*w_obs

        #determination of the effect the obstacles speeds have on the final speeds
        if linear>0.07 :
          linear=0.07
        if linear<-0.07:
          linear=-0.07
        if angular>0.3:
          angular=0.3
        if angular<-0.3:
          angular=-0.3

        if abs(l_goal+linear)<0.03:
          self.linear_velocity=0.03
          self.angular_velocity+=angular
        else:
          self.linear_velocity+=linear
          self.angular_velocity+=angular
        if u_obs < -7:
          self.linear_velocity=-0.3
          self.angular_velocity=0

        if abs(self.linear_velocity)>0.3:
          self.linear_velocity=0.3*(self.linear_velocity/abs(self.linear_velocity))
        if abs(self.angular_velocity)>0.3:
          self.angular_velocity=0.3*(self.angular_velocity/abs(self.angular_velocity))

        ##########################################################################
      else:
        ############################### NOTE QUESTION ############################
        # Implement obstacle avoidance here using the laser speeds.
        # Hint: Subtract them from something constant
        pass
        ##########################################################################

    # Assistive functions
    def stopRobot(self):
      self.stop_robot = True

    def resumeRobot(self):
      self.stop_robot = False
