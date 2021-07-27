#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 29 20:31:07 2020

@author: kuttus09
"""


import rospy#for working with ROS
#message types
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np#numpy
from control_func import *#control functions, kinematic and mass-inertia parameters of the Robot


class Joint_Controller(object):
    """Implementation of the Joint Space Inverse Dynamics Controller
    
    This controller object receives current joint state messages from 
    /manipulator/joint_states topic and the desired joint trajectory from the
    /theta_desired topic. It then computes the command torque by means of 
    Inverse dynamics and sends the computed torque to Gazebo through the
    /manipulator/group_effort_controller/command topic.

    Attributes
    ----------
    traj : np.ndarray of shape (18,1)
        The desired joint space trajectory (position, velocity and acceleration).
    theta : np.ndarray of shape (6,1)
        Current joint angle vector.
    dtheta : np.ndarray of shape (6,1)
        Current joint velocity vector
    traj_sub : rospy.Subscriber
        Subscriber of desired joint trajectory topic (/theta_desired).
    joint_state_sub : rospy.Subscriber
        Subscriber of current joint state topic (/manipulator/joint_states).
    torque_pub : rospy.Publisher
        Publisher of computed command torque to the /manipulator/group_effort_controller/command topic.
    Kp : float
        Kp gain.
    Kd : float
        Kd gain.
    msg : Float64MultiArray
        Torque message to be sent through /manipulator/group_effort_controller/command.
    rate : int
        Frequency at which to publish command torque. Reciprocal of control sampling interval

    """
    def __init__(self,Kp=100,Kd=20):
        """
        Initializes the joint state controller and Starts the torque computation loop.

        Parameters
        ----------
        Kp : float
            Kd gain.. The default is 100.
        Kd : float
            Kd gain. The default is 20.
        """
        
        #self.traj is a 6x1 numpy.ndarray used for storing desired joint angle, joint velocity and joint acceleration
        self.traj=np.zeros((18,1))
        self.traj[0:6]=np.array([[0,pi/3,-pi/2,0,pi/3,0]]).T#initial configuration
        #for storing current joint angles and joint velocities
        self.theta=np.zeros((6,1))
        self.dtheta=np.zeros((6,1))
        self.traj_sub=rospy.Subscriber('/theta_desired', Float64MultiArray, self.update_desired_state,queue_size=1) #subscribing to /theta_desired
        self.joint_state_sub=rospy.Subscriber('/manipulator/joint_states', JointState, self.update_state,queue_size=1) #subscribing to/hhumanoid/joint_states
        self.torque_pub=rospy.Publisher('/manipulator/group_effort_controller/command',Float64MultiArray,queue_size=1,latch=True) #for publishing command joint torque


        #controller gains
        self.Kp=Kp
        self.Kd=Kd

        self.msg=Float64MultiArray()#for sending command joint torque
        self.rate=rospy.Rate(200)#rate at which to compute torques
        self.send_torque()#start computing and sending torques

    def update_desired_state(self,msg):
        """
        Call back function for /theta_desired subscriber
        Sets self.traj to the desired trajectory received through /theta_desired

        Parameters
        ----------
        msg : TYPE Float64MultiArray
            DESCRIPTION. Array of desired joint angle, velocity and acceleration

        Returns
        -------
        None.

        """
        
        self.traj=np.array(msg.data).reshape((18,1))

    def update_state(self,msg):
        """
        Call back function for /hhumanoid/joint_states subscriber
        Sets self.theta and self.thetad to the current joint angle and joint velocity received through /hhumanoid/joint_states

        Parameters
        ----------
        msg : TYPE JointState
            DESCRIPTION. current joint_state of the half humanoid.

        Returns
        -------
        None.

        """
        
        theta=msg.position
        dtheta=msg.velocity

        self.theta[0:6,0]=theta[0:6]

        self.dtheta[0:6,0]=dtheta[0:6]


    def send_torque(self):
        """
        Computes the command joint torque and publishes to /hhumanoid/group_effort_controller/command
        loops till node is shut down

        Returns
        -------
        None.

        """
        
        while not rospy.is_shutdown():
            #take desired trajectory 
            traj=self.traj
            thetad=traj[0:6]
            dthetad=traj[6:12]
            ddthetad=traj[12:18]
            #take current trajectory
            theta=self.theta
            dtheta=self.dtheta
            phis=get_phi(alpha,a,theta0,theta,d,a0,alpha0)[0]#rigid body transformation matrices
            D=compute_D(SMs,phis,H)# D matrix
            V,A,g=forward_sweep(theta0,theta,dtheta,phis,H)#forward sweep
            CG=reverse_sweep(phis,SMs,m,V,A,g,H,COMs)#reverse sweep
            Tc=D @ (ddthetad+self.Kd*(dthetad-dtheta)+self.Kp*(thetad-theta)) + CG #command torque
            self.msg.data=(Tc.squeeze()) #message to be sent
            try:#this should be in a try-except block so that node exits gracefully
                self.torque_pub.publish(self.msg)
                self.rate.sleep()
            except:
                break
            


if __name__=="__main__":
    rospy.init_node('torque_commander')#starts node
    controller=Joint_Controller(100,20)#start controller
