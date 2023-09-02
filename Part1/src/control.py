#!/usr/bin/python3

import rospy, tf, math
from final_proj.srv import vfhs, vfhsResponse, vfhsRequest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import time
from final_proj.msg import mammad

from math import radians

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("control_node" , anonymous=True)
        
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        self.scan_sub = rospy.Subscriber("/target", mammad, self.get_k)
        
        
        # getting specified parameters
        self.linear_speed = .2
        self.angular_speed = .2 

        self.k_p = .1
        self.us = 14
        self.k_d = 10
        self.k_i = 0

        self.dt = 0.005
        rate = 1/self.dt
        self.r = rospy.Rate(rate)

        self.eror = 0

        self.k = 0

        
    def get_k(self, msg):

        self.k = msg.amount
        

    def control_effort(self):
        
        p = self.k_p * (self.k -self.us)
        d = self.k_d * ((self.k -self.us) - self.eror)
        self.eror = self.k -self.us
        return p #+ d
        
    def run(self):
        
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.linear_speed

        # print(f"x = {self.next_x}, y = {self.next_y}")

        while not rospy.is_shutdown():

            move_cmd.angular.z = self.control_effort()

            # rospy.logerr(f'{self.get_k()}')
            lin_s = self.linear_speed

            # if self.get_k() == 17:
            #     lin_s = self.linear_speed
            # else:
            #     lin_s = self.linear_speed/abs(self.get_k()-17)

            move_cmd.linear.x = lin_s

            self.cmd_publisher.publish(move_cmd)   

            # print(f"head = {self.get_heading()} target = {self.target_angel()} next = {self.next_x}  {self.next_y}, current = {self.curent_x} {self.curent_y}")
            
            # rospy.loginfo(f"error : {self.errs[-1]} speed : {move_cmd.linear.x} theta : {move_cmd.angular.z}")

            self.r.sleep()



if __name__ == "__main__":

    time.sleep(3)

    controller = Controller()
    
    controller.run()
    # while True:
    #     print(controller.get_next_destination().next_x)