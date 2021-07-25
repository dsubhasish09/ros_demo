#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 25 19:15:21 2020

@author: kuttus09
"""


import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray


pi=np.pi
cos=np.cos
sin=np.sin

class Sine_Traj_Gen(object):
    """
    Generates sinusoidal joint space trajectory and publishes it to /theta_desired
    """
    
    def __init__(self):
        """
        Initializes the Sine_Traj_Gen Object.

        Returns
        -------
        None.

        """
        
        self.rate=rospy.Rate(200)#rate at which trajectory should be published. Currently set to 200Hz
        self.trajd=np.zeros((18,1))#stores the desired sinusoidal joint space trajectory
        self.traj=Float64MultiArray()#for sending trajectory through the topic \theta_desired
        self.trajectory_pub=rospy.Publisher('/theta_desired',Float64MultiArray,queue_size=1) #for publisher for the topic /theta_desired

        
    def send_trajectory(self):
        omega=0.25*2*pi#angular frequency
        t0=rospy.get_time()#initial time
        theta_max=(pi/2)*np.ones((6,1))
        while not rospy.is_shutdown():#while ROS node has not been shut down
            #send desired trajectory through the topic /theta_desired
            t_=rospy.get_time()#current time
            t=t_-t0#for computing trajectory
            f=0.5*cos(omega*t-pi)+0.5
            df=-0.5*omega*sin(omega*t-pi)
            ddf=-0.5*omega**2*cos(omega*t-pi)
            #compute desired trajectory so as to oscillate sinusoidaly between all angles 0 to all angles pi/2
            self.trajd[0:6]=f*theta_max
            self.trajd[6:12]=df*theta_max
            self.trajd[12:18]=ddf*theta_max
            self.traj.data=self.trajd[:]
            try:#kept within try-except so as to exit gracefully
                self.trajectory_pub.publish(self.traj)
                self.rate.sleep()
            except:
                break
    
        
if __name__ == '__main__':
    rospy.init_node('trajectory_generator')#start node
    trajgen=Sine_Traj_Gen()#start sine trajectory generation
    trajgen.send_trajectory()#start publishing
        
        
         
        
        