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
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from control_func import *
import pickle
import time
import matplotlib.pyplot as plt

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
    phis : np.ndarray of shape (6,6,6)
        array of Rigid body transformation matrices. phis[0] to phis[5] are the rigid body transformation matrices
        phi_i-1_i for i=1 to 6. 
    phis_l : int
        Frequency at which to publish command torque. Reciprocal of control sampling interval
    cphis : numpy.ndarray of shape (6,6,6)
        array of Cummulative Rigid body transformation matrices. cphis[0] to cphis[5] are the cummulative rigid body transformation matrices
        phi_i-1_6 for i=1 to 6. 
    task_spatial_pos : np.ndarray of shape (6,1)
        Current task space position
    task_spatial_vel : np.ndarray of shape (6,1)
        Current task space velocity
    Ja_ : np.ndarray of shape (6,6)
        Regularized analytic jacobian
    Ja_inv : np.ndarray of shape (6,6)
        Jacobian pseudo inverse
    Ja : np.ndarray of shape (6,6)
        Unregularized analytic jacobian
    Jad : np.ndarray of shape (6,6)
        Time derivative of analytic jacobian
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
    D : np.ndarray of shape (6,6)
        Joint Space Inertia Matrix
    CG : np.ndarray of shape (6,1)
        Vector of centrifugal, coriolis and gravity forces
    Bt : np.ndarray of shape (6,1)
        Task to be performed
    qdd : np.ndarray of shape (6,1)
        Commanded joint space acceleration
    V : list
        list of spatial velocities
    A : list
        list of spatial accelerations
    g : list
        list of spatial gravity
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
        self.phis=np.zeros((6,6,6))#rigid body transformation matrix
        self.phis_l=np.zeros((6,6,6))#link rigid body transformation matrix
        self.cphis=np.zeros((6,6,6))#cummulative rigid body transformation matrix
        self.task_spatial_pos=np.zeros((6,1))#current task space coordinate
        self.task_spatial_vel=np.zeros((6,1))#current task space velocity
        #for storing jacobians and jacobian derivatives
        self.Ja_=np.zeros((6,6))
        self.Ja_inv=np.zeros((6,6))
        self.Ja=np.zeros((6,6))
        self.Jad=np.zeros((6,6))
        
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
        
        #matrices used for command torque computation
        self.D=np.zeros((6,6))
        self.CG=np.zeros((6,1))
        self.Bt=np.zeros((6,1))
        self.qdd=np.zeros((6,1))
        self.V=6*[np.zeros((6,1))]
        self.A=6*[np.zeros((6,1))]
        self.g=6*[np.zeros((6,1))]
        
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

    def compute_torque(self):
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
        self.theta[0:6,0],self.dtheta[0:6,0]=self.joint_state.position[0:6],self.joint_state.velocity[0:6]
        #get rigid body transformation matrices and cumulative rigid body transformation matrices
        self.phis[:,:,:],self.phis_l[:,:,:],_=get_phi(alpha,a,theta0,self.theta,d,a0,alpha0)
        self.cphis[:,:,:]=get_cummulative_phi(self.phis)
        #rigid body transformation from base frame to end effector frame
        ef=self.cphis[0,:,:] @ phi6_ef
        #get current task space coordinate
        self.task_spatial_pos[0:3]=np.array(euler_from_matrix(ef[0:3,0:3],'rzyz')).reshape((3,1))
        self.task_spatial_pos[3:6]=pos_from_phi(ef)
        euler=self.task_spatial_pos[0:3]#get current euler angle
        Ta_inv=geometric2analytic_jacobian(euler)#matrix to be premultiplied with geometric jacobian
        J,J_,R06=geometric_jacobian(self.cphis)#get geometric jacobian
        self.Ja_[:,:]=np.matmul(Ta_inv,J)#analytical jacobian

        self.Ja[:,:]=self.Ja_[:,:]#to separate from regularized analytical jacobian

        self.task_spatial_vel[:,:]=np.matmul(self.Ja,self.dtheta)#current task space velocity
        Jd=dJ_dt(J_,R06,self.phis,self.phis_l,self.cphis,self.theta,self.dtheta,6)#derivative of geometric jacobian
        deuler=self.task_spatial_vel[0:3]#euler angle rate
        dte=dTe(euler,deuler)
        self.Jad[:,:]=Ta_inv @ (Jd-dte @ self.Ja)#derivative of analytical jacobian
        
        self.Ja_[:,:],self.Ja_inv[:,:]=self.regularize(self.Ja,100)#regularize
        err=self.state_difference(self.Xd,self.task_spatial_pos)#error
        # print(100*np.linalg.norm(err)/np.linalg.norm(self.task_spatial_pos))
        derr=self.dXd-self.task_spatial_vel#derivative error
        #command torque computation
        self.Bt[:,:]=self.ddXd+self.Kd * derr +self.Kp * err-np.matmul(self.Jad,self.dtheta)
        self.qdd[:,:]=self.Ja_inv @ self.Bt+(np.eye(6)-(self.Ja_inv @ self.Ja_)) @ (100-self.theta+20-self.dtheta)
        self.D[:,:]=compute_D(SMs, self.phis,H)
        self.V[:],self.A[:],self.g[:]=forward_sweep(theta0,self.theta,self.dtheta,self.phis,H)
        self.CG[:,:]= reverse_sweep(self.phis,SMs,m,self.V,self.A,self.g,H,COMs)
        self.Tc[:,:]= self.D @ self.qdd +self.CG

    def regularize(self,A,cond):
        """
        regularizes input matrix A using a modified truncated SVD regularization using
        cond as the threshold condition number

        Parameters
        ----------
        A : numpy.ndarray
             input matrix
        cond : float
             threshold condition number

        Returns
        -------
        A_ : numpy.ndarray
             regularized matrix
        A_pinv: numpy.ndarray
            pseudo inverse of regularized matrix

        """
        U,S,Vt=np.linalg.svd(A)

        S=S[S[0]/S<cond]

        S_=np.diag(S)
        n=len(S)
        A_=np.matmul(U[:,0:n],np.matmul(S_,Vt[0:n,:]))#regularized matrix

        S1=1/S
        S1_=np.diag(S1)
        A_pinv=np.matmul(Vt[0:n,:].T,np.matmul(S1_,U[:,0:n].T))
        return A_,A_pinv


    def state_difference(self,Xg,Xs):
        """
        The function which gives the difference between task space coordinates
        Xg and Xs as per the Euler Angle based scheme.

        Parameters
        ----------
        Xg : numpy.ndarray
             goal task space coordinate/desired task space coordinate
        Xs : numpy.ndarray
             start task space coordinate/current task space coordinate

        Returns
        -------
        Xd : numpy.ndarray
             difference between task space coordinates Xg and Xs as per the control scheme.

        """
        Xd=Xg-Xs
        Xde=Xd[0:3]
        #converting angles greater than pi to equivalent negative angle
        #done so that the difference indicates the shortest possible path
        idx=np.abs(Xde)>pi
        Xde[idx]=-np.sign(Xde[idx])*(2*pi-np.abs(Xde[idx]))
        Xd[0:3,0]=Xde[0:3,0]
        return Xd

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
            self.compute_torque()#compute torque
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
