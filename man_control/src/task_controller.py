#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun  1 21:39:56 2021

@author: dsubhasish

The class defined here can be used as a template for writing code for more 
complex task space controllers. Currently no constraints have been applied. 
This will be changed in future versions.
"""
import numpy as np
from numba import jit
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from control_func import *
import pickle
import time
import matplotlib.pyplot as plt


@jit(nopython=True,cache=True)
def compute_torque(theta,dtheta,Xd,dXd,ddXd,Kd,Kp):
    """
    Carries out one iteration of command torque computation.
    
    Takes current joint space position and velocity, uses it to get the rigid 
    body transformation matrices and calculate the Analytical Jacobian and its 
    time derivative. The Jacobian matrix is then regularized and its pseudo-
    inverse is taken.The required task to be performed is then computed, and 
    using the pseudo inverse of the Jacobian, the required joint space acceleration
    is obtained. The command torque is finally obtained by means of inverse 
    dynamics.

    Returns
    -------
    None.

    """
    #get current joint state and joint velocity
    
    #get rigid body transformation matrices and cumulative rigid body transformation matrices
    phis,phis_l,_=get_phi(alpha,a,theta0,theta,d,a0,alpha0)
    cphis=get_cummulative_phi(phis)
    #rigid body transformation from base frame to end effector frame
    ef=cphis[0] @ phi6_ef
    #get current task space coordinate
    task_spatial_pos=np.concatenate((R2Euler(ef[0:3,0:3]),pos_from_phi(ef)),axis=0)
    euler=task_spatial_pos[0:3]#get current euler angle
    Ta_inv=geometric2analytic_jacobian(euler)#matrix to be premultiplied with geometric jacobian
    J,J_,R06=geometric_jacobian(cphis)#get geometric jacobian
    Ja_=Ta_inv @ J#analytical jacobian

    Ja=Ja_.copy()#to separate from regularized analytical jacobian

    task_spatial_vel=Ja @ dtheta#current task space velocity
    Jd=dJ_dt(J_,R06,phis,phis_l,cphis,theta,dtheta,6)#derivative of geometric jacobian
    deuler=task_spatial_vel[0:3]#euler angle rate
    dte=dTe(euler,deuler)
    Jad=Ta_inv @ (Jd-dte @ Ja)#derivative of analytical jacobian
    
    Ja_,Ja_inv=regularize(Ja,100)#regularize
    err=state_difference(Xd,task_spatial_pos)#error
    # print(100*np.linalg.norm(err)/np.linalg.norm(self.task_spatial_pos))
    derr=dXd-task_spatial_vel#derivative error
    #command torque computation
    Bt=ddXd+Kd * derr +Kp * err-Jad @ dtheta
    qdd=Ja_inv @ Bt+(np.eye(6)-(Ja_inv @ Ja_)) @ (100-theta+20-dtheta)
    D=compute_D(SMs, phis,H)
    V,A,g=forward_sweep(theta0,theta,dtheta,phis,H)
    CG= reverse_sweep(phis,SMs,m,V,A,g,H,COMs)
    Tc= D @ qdd +CG
    return Tc
    
class Task_Controller(object):
    """A bare bones implementation of a task space controller
    
    This controller object receives current joint state messages from 
    /manipulator/joint_states topic and the desired task space trajectory from 
    the /task_desired topic. It then computes the command torque and sends it
    to Gazebo through the /manipulator/group_effort_controller/command topic.

    Attributes
    ----------
    joint_state : JointState
        Current joint state received from Gazebo.
    theta : np.ndarray of shape (6,1)
        Current joint angle vector.
    dtheta : np.ndarray of shape (6,1)
        Current joint velocity vector
    traj_sub : rospy.Subscriber
        Subscriber of desired joint trajectory topic (/task_desired).
    joint_state_sub : rospy.Subscriber
        Subscriber of current joint state topic (/manipulator/joint_states).
    torque_pub : rospy.Publisher
        Publisher of computed command torque to the /manipulator/group_effort_controller/command topic.
    Kp : float
        Kp gain.
    Kd : float
        Kd gain.
    f : float
        Torque publishing frequency
    Xd : np.ndarray of shape (6,1)
        Desired task space position
    dXd : np.ndarray of shape (6,1)
        Desired task space velocity
    ddXd : np.ndarray of shape (6,1)
        Desired task space acceleration
    Tc : np.ndarray of shape (6,1)
        Command torque
    msg : Float64MultiArray
        Torque message to be published
    """
    
    def __init__(self,Kp=100,Kd=20,f=200):
        """
        Parameters
        ----------
        Kp : TYPE, float
             The default is 100.
        Kd : TYPE, float
             The default is 20.
        f : TYPE, float
             The default is 200.Frequency at which to publish torque
        """
        
        self.Kp=Kp
        self.Kd=Kd
        self.joint_state=JointState()#stores joint state message
        #setting initial values
        self.joint_state.position=6*[0]
        self.joint_state.velocity=6*[0]
        #joint state subscriber
        self.joint_state_sub=rospy.Subscriber('/manipulator/joint_states', JointState, self.update_state,queue_size=1) #subscriber for joint state
        self.theta=np.zeros((6,1))#current joint angle
        self.dtheta=np.zeros((6,1))#current joint velocity
        self.f=f#torque publishing frequency
        
        #initial desired task space coordinate, task space velocity and task space acceleration
        theta=np.array([[0,pi/3,-pi/2,0,pi/3,0]]).T
        self.Xd=joint2task(theta)
        self.dXd=np.zeros((6,1))
        self.ddXd=np.zeros((6,1))

        self.Tc=np.zeros((6,1))#computed joint torque
        #torque publisher
        self.torque_pub=rospy.Publisher('/manipulator/group_effort_controller/command',Float64MultiArray,queue_size=1,latch=True)#command torque publisher
        self.msg=Float64MultiArray()#torque message to be published
        
        #subscriber for desired task space trajectory
        self.traj_sub=rospy.Subscriber('/task_desired', Float64MultiArray, self.update_desired_task,queue_size=1) #subscribing to /theta_desired
        

    def update_state(self,msg):
        """
        Call back function for joint state subscriber. Updates self.theta and self.thetad whenever a new joint state message is received

        Parameters
        ----------
        msg : JointState
             Message received on /hhumanoid/joint_states

        Returns
        -------
        None.

        """
        self.joint_state.position=msg.position
        self.joint_state.velocity=msg.velocity
        self.joint_state.header=msg.header
        
    def update_desired_task(self,msg):
        """
        Call back function for desired task space trajectory topic. Updates corresponding values 
        for command torque computation

        Parameters
        ----------
        msg : Float64MultiArray
             message within the desired task space trajectory topic

        Returns
        -------
        None.

        """
        traj=msg.data
        self.Xd[0:6,0]=traj[0:6]
        self.dXd[0:6,0]=traj[6:12]
        self.ddXd[0:6,0]=traj[12:18]

    def control_loop(self):
        """
        Carries out one iteration of torque computation, followed by publishing
        the torque to /manipulator/group_effort_controller/command.

        Returns
        -------
        None.

        """
        r=rospy.Rate(self.f)

        while not rospy.is_shutdown():
            self.theta[0:6,0],self.dtheta[0:6,0]=self.joint_state.position[0:6],self.joint_state.velocity[0:6]
            # self.compute_torque()#compute torque
            self.Tc[:,:]=compute_torque(self.theta,self.dtheta,self.Xd,self.dXd,self.ddXd,self.Kd,self.Kp)

            self.msg.data=self.Tc[:]#message to be published
            try:
                self.torque_pub.publish(self.msg)#publish
                r.sleep()
            except:
                break
    
    def get_time(self):
        """
        helper function to get current time from the joint state

        Returns
        -------
        t : float
             current time

        """
        header=self.joint_state.header
        t=header.stamp.secs+header.stamp.nsecs*10**-9
        return t
 
if __name__ == '__main__':
    rospy.init_node('torque_commander')#start node
    task=Task_Controller()#initialize controller
    task.control_loop()#start torque computation loop
