#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 25 19:15:21 2020

@author: dsubhasish

The class defined here can be used as a template for any trajectory generator
in the joint space. In this specific case, a sinusoidal trajectory is generated,
with the desired joint space position, velocity and acceleration sent to the
joint space controller by means of the topic /theta_desired.
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
    
    This generator produces a sinusoidal trajectory between the joint space position
    with all angles set to 0 and all angles set to 90 degrees. A suitable angular 
    frequency of oscillation is chosen so as to remain within the control
    bandwidth

    Attributes
    ----------
    rate : rospy.Rate(200)
        To set rate of publishing messages as 200 Hz.
    trajd : np.ndarray of shape (18,1)
        Desired trajectory
    traj : Float64MultiArray
        The computed trajectory will be transferred to this message type for publishing
    trajectory_pub : rospy.Publisher
        Publisher of desired joint trajectory topic (/theta_desired).
    """
    
    def __init__(self):
        
        self.rate=rospy.Rate(200)#rate at which trajectory should be published. Currently set to 200Hz
        self.trajd=np.zeros((18,1))#stores the desired sinusoidal joint space trajectory
        self.traj=Float64MultiArray()#for sending trajectory through the topic \theta_desired
        self.trajectory_pub=rospy.Publisher('/theta_desired',Float64MultiArray,queue_size=1) #for publisher for the topic /theta_desired

        
    def send_trajectory(self):
        """
        Runs the desired trajectory computation and publishing loop. At every
        iteration, takes in the current simulation time, computes the desired
        joint space position, velocity and acceleration, and then publishes it
        over to the topic /theta_desired.

        Returns
        -------
        None.

        """
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
        
        
         
        
        