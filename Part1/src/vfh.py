#!/usr/bin/python3

#import rospy library for ROS activities

import rospy

# Import the Odometry message
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
import matplotlib.pyplot as plt
from copy import deepcopy

from final_proj.msg import mammad

from final_proj.srv import vfhs, vfhsResponse, vfhsRequest

#import mathematics libraries 
import math
import numpy as np
import tf


class vfh:

    def __init__(self):

        
        self.h_unit = 6
        self.treshhold = 2

        self.a = 1
        self.b = 1 / 1.7
        
        self.current_x = 0
        self.current_y = 0

        self.laser_buffer = []
        self.laser_buffer_0 = []
        
        self.ID = 0

        self.valleys = []

        self.target_position = [7,13]

        self.result = 17
        
        self.map_h = np.zeros(int(180/self.h_unit))
        self.map_h_smooth = np.zeros(int(180/self.h_unit))
        
        rospy.init_node("ObstacleDetector" , anonymous=False)

        self.pub = rospy.Publisher("/target",mammad , queue_size=10)
        
        self.run()

    
    def laser_scan_corrector(self):

        a = []
        # rospy.logerr(f'{self.laser_buffer_0[0][0]}')  

        for i in range(90):

            b = []

            b.append(self.laser_buffer_0[269+i][0])
            b.append(i)

            a.append(b)
        for i in range(90):
            b = []
            b.append(self.laser_buffer_0[i][0])
            b.append(i+90) 

            a.append(b)   

        self.laser_buffer = deepcopy(a) 

        # rospy.logerr(f'{self.laser_buffer}')  
        for i in range(len(self.laser_buffer)):

            self.laser_buffer[i][1] = 209 - i

        # self.laser_buffer = self.laser_buffer[::-1]

        # rospy.logerr(f'{self.laser_buffer_0}')       

    def callback_laser_scan(self):

        laser_data = rospy.wait_for_message("/scan", LaserScan)

        for i in range(len(laser_data.ranges)):
            
            a = [laser_data.ranges[i], i * laser_data.angle_increment]

            self.laser_buffer_0.append(a)
        
        # rospy.logerr(f'{self.laser_buffer_0[0][0]}')

    def histogram_maker(self) :

        for i in range(len(self.laser_buffer)):

            if self.laser_buffer[i][0] != float('inf') and self.laser_buffer[i][0] <= 2:

                m = self.a - self.b * self.laser_buffer[i][0]

                self.map_h[i//self.h_unit] += m

        self.h_smooth_filler()

    def h_smooth_filler(self):
        
        # for i in range(2,len(self.map_h)-2):

        #     self.map_h_smooth[i] = (self.map_h[i - 2] + self.map_h[i + 2] + 2*self.map_h[i - 1] + 2*self.map_h[i + 1] + 3*self.map_h[i])/5
        # self.map_h_smooth[0] = self.map_h[0]
        # self.map_h_smooth[1] = self.map_h[1]
        # self.map_h_smooth[len(self.map_h)-2] = self.map_h[len(self.map_h)-2]
        # self.map_h_smooth[len(self.map_h)-1] = self.map_h[len(self.map_h)-1]

        self.map_h_smooth = deepcopy(self.map_h)
        
    def visualization(self):

        fig, ax = plt.subplots(subplot_kw=dict(projection='polar'))

        degrees = [math.pi/180 * i for i in range(0, 180, 5)]
        counts = self.map_h_smooth.tolist()
        
        ax.bar(degrees, counts)

        ax.set_ylabel('m')
        ax.set_title('degree')
        ax.legend(title='h')

        plt.show()

    def self_position(self):

        msg = rospy.wait_for_message("/odom" , Odometry)

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def valley_finder(self) :

        potential_targets = []

        treshhold = (min(self.map_h_smooth) + max(self.map_h_smooth))/2

        for i in range(len(self.map_h_smooth)) :

            if self.map_h_smooth[i] <= self.treshhold :

                potential_targets.append(i)
                # rospy.logerr(f'{self.map_h_smooth[i]}')
            
        valleys = [] 
        i = 0

        while i < len(potential_targets):

            buffer = []

            # rospy.logerr(f'an')

            buffer.append(potential_targets[i])

            while True:

                # rospy.logerr(f'{i}')

                if i + 1 >= len(potential_targets):

                    i += 1
                    break

                elif potential_targets[i+1] - potential_targets[i] == 1:

                    i += 1

                    buffer.append(potential_targets[i])

                    if i == len(potential_targets)-1:
                        valleys.append(buffer)
                        break

                else:
                    i += 1
                    valleys.append(buffer)
                    break

        self.valleys = valleys 

    def target_heading(self):

        theta = math.atan2(7 - self.current_y , 13 - self.current_x)

        # rospy.logerr(f'{theta}')

        if theta < 0 :
            theta += 2*math.pi

        # rospy.logerr(f'{theta}')
        theta = math.degrees(theta)

        theta += 90

        if theta > 360 :
            theta -= 360

        # rospy.logerr(f'{theta}')

        k = theta // self.h_unit

        return k
        
    def selected_valley(self):

        valley_mid = []
        valley_mid_dis = []
        

        valley = deepcopy(self.valleys)
        rospy.logerr(f'{self.valleys}')
        f = len(valley)
        # rospy.logerr(f'{f}')

        aboo = []

        for i in valley:

            if len(valley) <= 1:
                break
            if len(i) > 5 :
                aboo.append(i)
            
        if len(aboo) != 0 :
            valley = deepcopy(aboo)
        

        for i in range(len(valley)):

            valley_mid .append( valley[i][len(valley[i])//2])

        for i in range(len(valley_mid)):

            valley_mid_dis.append(abs(valley_mid[i]-self.target_heading()))

        first_option = valley_mid_dis.index(min(valley_mid_dis))

        valley_depth = []

        for i in range(len(valley)):
            b = []
            for j in range(len(valley[i])):
                b.append(self.map_h_smooth[valley[i][j]])

            valley_depth.append(b)

        minimum_depths = []

        for i in range(len(valley_depth)):
            minimum_depths.append(min(valley_depth[i]))

        second_option = self.valleys.index(valley[minimum_depths.index(min(minimum_depths))])

        return second_option

        tyu = []

        for i in valley:
            tyu.append(len(i))
        
        third_option = self.valleys.index(valley[tyu.index(max(tyu))])

        # return third_option

        # if second_option < first_option :
        #     return second_option
        # else:
        #     return first_option
    
    def final_taget(self):

        # rospy.logerr(f'{self.laser_buffer}')
        # rospy.logerr(f'{self.map_h_smooth}')
        
        # rospy.logerr(f'{self.target_heading()}')
        rospy.logerr(f'{self.selected_valley()}')

        a = self.valleys[self.selected_valley()]
        rospy.logerr(f'{a[len(a)//2]}')
        return a[len(a)//2]
        # rospy.logerr(f'{a[a.index(min(a))]}')
        # rospy.logerr(f'{self.map_h_smooth.tolist().index(min(self.map_h_smooth[a[0]:a[-1]]))}')
        # return self.map_h_smooth.tolist().index(min(self.map_h_smooth[a[0]:a[-1]]))
        # return a[a.index(min(a))]

    def run(self):
        # res = vfhResponse()

        while True:

            self.callback_laser_scan()
            
            if len(self.laser_buffer_0) != 0 :

                self.laser_scan_corrector()

                self.histogram_maker()

                self.h_smooth_filler()

                self.valley_finder()

                self.result = self.final_taget()

                # self.visualization()
                self.map_h_smooth = np.zeros(int(180/self.h_unit))
                self.map_h = np.zeros(int(180/self.h_unit))
                self.laser_buffer_0 = []

                self.pub.publish(self.result)

                # break

if __name__ == '__main__':
    
    heay = vfh()

    # rospy.spin()
    