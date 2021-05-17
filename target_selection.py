#!/usr/bin/env python

import rospy
import random
import math
import time
import numpy as np
from timeit import default_timer as timer
from utilities import RvizHandler
from utilities import OgmOperations
from utilities import Print
from brushfires import Brushfires
from topology import Topology
import scipy
from sensor_msgs.msg import Range
from sonar_data_aggregator import SonarDataAggregator
from path_planning import PathPlanning


# Class for selecting the next best target
class TargetSelection:

    # Constructor
    def __init__(self, selection_method):
        self.goals_position = []
        self.goals_value = []
        self.omega = 0.0
        self.radius = 0
        self.method = selection_method
        self.previous_target =[]
        self.brush = Brushfires()
        self.topo = Topology()
        self.path_planning = PathPlanning()
        self.previous_target.append(50)
        self.previous_target.append(50)
        self.node2_index_x=0
        self.node2_index_y=0
        self.sonar=SonarDataAggregator()
        self.timeout_happened=0

    def selectTarget(self, init_ogm, coverage, robot_pose, origin, \
        resolution, force_random = False):

        target = [-1, -1]


        ######################### NOTE: QUESTION  ##############################
        # Implement a smart way to select the next target. You have the
        # following tools: ogm_limits, Brushfire field, OGM skeleton,
        # topological nodes.

        # Find only the useful boundaries of OGM. Only there calculations
        # have meaning
        ogm_limits = OgmOperations.findUsefulBoundaries(init_ogm, origin, resolution)
        #print(ogm_limits)
        # Blur the OGM to erase discontinuities due to laser rays
        ogm = OgmOperations.blurUnoccupiedOgm(init_ogm, ogm_limits)

        # Calculate Brushfire field
        tinit = time.time()
        brush = self.brush.obstaclesBrushfireCffi(ogm, ogm_limits)
        Print.art_print("Brush time: " + str(time.time() - tinit), Print.ORANGE)

        # Calculate skeletonization
        tinit = time.time()
        skeleton = self.topo.skeletonizationCffi(ogm, \
                   origin, resolution, ogm_limits)
        Print.art_print("Skeletonization time: " + str(time.time() - tinit), Print.ORANGE)

        # Find topological graph
        tinit = time.time()
        nodes = self.topo.topologicalNodes(ogm, skeleton, coverage, origin, \
                resolution, brush, ogm_limits)
        Print.art_print("Topo nodes time: " + str(time.time() - tinit), Print.ORANGE)

        # Visualization of topological nodes
        vis_nodes = []
        for n in nodes:
            vis_nodes.append([
                n[0] * resolution + origin['x'],
                n[1] * resolution + origin['y']
            ])
        RvizHandler.printMarker(\
            vis_nodes,\
            1, # Type: Arrow
            0, # Action: Add
            "map", # Frame
            "art_topological_nodes", # Namespace
            [0.3, 0.4, 0.7, 0.5], # Color RGBA
            0.1 # Scale
        )

        # Random point

        #if statement that makes the target selection random in case the remaining nodes are two
        if len(nodes)<=2:
            target = self.selectRandomTarget(ogm, coverage, brush, ogm_limits)
            return target

        pose_global_x=int(robot_pose['x_px']-origin['x']/resolution)
        pose_global_y=int(robot_pose['y_px']-origin['y']/resolution)

        #the folowing variables receive the values read by the sonar sensor
        sonar_left=self.sonar.sonar_left_range
        sonar_right=self.sonar.sonar_right_range
        sonar_front=self.sonar.sonar_front_range

        #a total sum is calculated for the front,right and left sonar sensors
        sonar_sum=sonar_left+sonar_right+sonar_front
        numrows=ogm.shape[1]
        numcols=ogm.shape[0]

        #if statement used in case the robot is in a tight spot and determines the target in the direction there is maximum space
        if sonar_sum<1.2:
            target=self.sonar_avoidance(pose_global_x,pose_global_y,ogm,numcols,numrows)
            return target

        #in case of a time out or a failure in path planning
        if self.method == 'random' or force_random == True:
          target=[self.node2_index_x,self.node2_index_y] #sets the current target as the previously calculated second best-scored target
          if self.timeout_happened==1:
               target=self.sonar_avoidance(pose_global_x,pose_global_y,ogm,numcols,numrows)
               self.timeout_happened=0
               return target
          self.timeout_happened=1
          return target
        ########################################################################


        sum_min=10000000
        dist_min=10000000
        node_index_x=0 #x value of the node with the lowest total cost
        node_index_y=0 #y value of the node with the lowest total cost
        topo_cost=[] #list of topological costs for each node
        dist_cost=[] #list of distance costs for each node
        cov_cost=[] #list of coverage costs for each node

        #using the brushfire array in order to calculate topology cost for each node
        for n in nodes:
          sum_temp=0
          index=n[1]+1
          while brush[n[0],index]!=0:
            sum_temp+=1
            index+=1
            if index==numrows-1:  #numrows
              break
          index=n[1]-1
          while brush[n[0],index]!=0 :
            sum_temp+=1
            index-=1
            if index==0:
              break
          index=n[0]+1
          while brush[index,n[1]]!=0:
            sum_temp+=1
            index+=1
            if index==numcols-1:  #numcols
              break
          index=n[0]-1
          while brush[index,n[1]]!=0:
            sum_temp+=1
            index-=1
            if index==0:
              break

          topo_cost.append(sum_temp/4)

        #using the coverage array in order to calculate coverage cost for each node
        numrows=len(coverage)
        numcols=len(coverage[0])
        for n in nodes:
          total_sum=0
          sum_temp=0
          index=n[1]+1
          if index>=numrows or n[0]>=numcols:
            total_sum=5000
            cov_cost.append(total_sum)
            continue
          while coverage[n[0],index]!=100:
            sum_temp+=1
            index+=1
            if index>=numcols-1 : #numrows-1:
              break
          total_sum+=sum_temp*coverage[n[0],index]/100
          index=n[1]-1
          sum_temp=0
          while coverage[n[0],index]!=100 :
            sum_temp+=1
            index-=1
            if index==0:
              break
          total_sum+=sum_temp*coverage[n[0],index]/100
          index=n[0]+1
          sum_temp=0
          if index>=numcols or n[1]>=numrows:
            total_sum=5000
            cov_cost.append(total_sum)
            continue

          while coverage[index,n[1]]!=100:
            sum_temp+=1
            index+=1
            if index>=numrows-1:   #numcols-1
              break
          total_sum+=sum_temp*coverage[index,n[1]]/100
          index=n[0]-1
          sum_temp=0
          while coverage[index,n[1]]!=100:
            sum_temp+=1
            index-=1
            if index==0:
              break
          total_sum+=sum_temp*coverage[index,n[1]]/100
          if total_sum==0:
              total_sum=5000
          cov_cost.append(total_sum)

          pose_global_x=int(robot_pose['x_px']-origin['x']/resolution)
          pose_global_y=int(robot_pose['y_px']-origin['y']/resolution)


          #eucledean distance between the robot pose and each node
          dist=math.sqrt(math.pow(pose_global_x-n[0],2)+math.pow(pose_global_y-n[1],2))
          dist_cost.append(dist)
        maxi=0
        for i in range(0,len(cov_cost)):
            if cov_cost[i]!=5000 and cov_cost[i]>maxi:
                maxi=cov_cost[i]
        for i in range(0,len(cov_cost)):
            if cov_cost[i]==5000:
                cov_cost[i]=maxi*1.2

        #lists to store the normalized costs for each node
        topo_cost_norm=[]
        dist_cost_norm=[]
        cov_cost_norm=[]
        final_cost=[]
        min_final_cost=1000000

        for i in range(0,len(dist_cost)):
          topo_cost_norm.append((np.float(topo_cost[i]-min(topo_cost))/np.float(max(topo_cost)-min(topo_cost))))
          dist_cost_norm.append((dist_cost[i]-min(dist_cost))/(max(dist_cost)-min(dist_cost)))
          if np.float(max(cov_cost)-min(cov_cost))!=0:
            cov_cost_norm.append((np.float(cov_cost[i]-min(cov_cost))/np.float(max(cov_cost)-min(cov_cost))))
          if max(cov_cost)!=min(cov_cost):
            final_cost.append(4*topo_cost_norm[i]+2*dist_cost_norm[i]+4*cov_cost_norm[i])   #optimal factor values in order to determine the best node to approach
          else:
            final_cost.append(6*topo_cost_norm[i]+4*dist_cost_norm[i]) # exception if statement for the case coverage array cannot be used yet

          if(final_cost[i]<min_final_cost):
            min_final_cost=final_cost[i]
            self.node2_index_x=node_index_x #storing the second best node to approach in case of path planning failure or time out
            self.node2_index_y=node_index_y #
            node_index_x=nodes[i][0]
            node_index_y=nodes[i][1]



        target=[node_index_x,node_index_y]
        #in case current target is the same with the previous , sonar avoidance function is used to modify the robot pose
        if target==self.previous_target :
          target=self.sonar_avoidance(pose_global_x,pose_global_y,ogm,numcols,numrows)
        self.previous_target=target

        return target

    def selectRandomTarget(self, ogm, coverage, brushogm, ogm_limits):
      # The next target in pixels
        tinit = time.time()
        next_target = [0, 0]
        found = False
        while not found:
          x_rand = random.randint(0, ogm.shape[0] - 1)
          y_rand = random.randint(0, ogm.shape[1] - 1)
          if ogm[x_rand][y_rand] < 50 and coverage[x_rand][y_rand] < 50 and \
              brushogm[x_rand][y_rand] > 5:
            next_target = [x_rand, y_rand]
            found = True
        Print.art_print("Select random target time: " + str(time.time() - tinit), \
            Print.ORANGE)
        return next_target

    #sonar_avoidance is a function that calculates the direction the robot should head in order to escape a tight spot
    def sonar_avoidance (self,pose_global_x,pose_global_y,ogm,numcols,numrows):

            tally=[0,0,0,0] #a sum is calculated for north,east,south and west direction

            index=pose_global_y+1
            while ogm[pose_global_x,index]!=100:
                tally[0]=tally[0]+1
                index+=1
                if index==numrows-1:  #numrows
                  break
            index=pose_global_y-1
            while ogm[pose_global_x,index]!=100 :
                tally[1]=tally[1]+1
                index-=1
                if index==0:
                  break
            index=pose_global_x+1
            while ogm[index,pose_global_y]!=100:
                tally[2]=tally[2]+1
                index+=1
                if index==numcols-1:  #numcols
                  break
            index=pose_global_x-1
            while ogm[index,pose_global_y]!=100:
                tally[3]=tally[3]+1
                index-=1
                if index==0:
                  break
            index=tally.index(max(tally))
            if index==0:
                target=[pose_global_x,pose_global_y+(int)(tally[0]/2)]
                return target
            elif index==1:
                target=[pose_global_x,pose_global_y-(int)(tally[1]/2)]
                return target
            elif index==2:
                target=[pose_global_x+(int)(tally[2]/2),pose_global_y]
                return target
            elif index==3:
                target=[pose_global_x-(int)(tally[3]/2),pose_global_y]
                return target
