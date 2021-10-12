#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jun 12 00:19:46 2021

@author: dsubhasish

This is a basic task space trajectory generator which can be used as a template to
create more complex trajectory generators.
"""


import numpy as np
from control_func import joint2task,pi,sin,cos,Rx,Ry,Rz
import rospy
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf.transformations import *
from geometry_msgs.msg import PoseStamped

class Task_Traj_Gen(object):
    """
    This class implements a minimal desired task space trajectory generator.
    
    The trajectory generated resembles an infinity symbol. This is achieved by 
    dividing the time period into five segments (straight line, circular arc, 
    straight line, circular arc and straight line), and then computing the 
    trajectory accordingly as per the current time segment. This is then repeated
    by resetting the time period.
    
    Attributes
    ----------
    traj : Float64MultiArray
        The computed trajectory will be transferred to this message type for publishing
    trajd : np.ndarray of shape (18,1)
        Desired trajectory
    trajd : np.ndarray of shape (18,1)
        Starting point for the trajectory
    traj_pub : rospy.Publisher
        Publisher of desired task trajectory topic (/task_desired).
    visualize : bool
        Whether or not additional topics should be published for visualization in RViz
    point_marker : Marker
        RViz visualization marker representing the desired tool tip point
    point_pub : rospy.Publisher
        Publisher for sending desired tool tip point to RViz over the topic /desired_tool_tip
    point_marker1 : Marker
        RViz visualization marker representing the desired tool tip direction
    point_pub1 : rospy.Publisher
        Publisher for sending desired tool tip direction to RViz over the topic /desired_tool_direction
    pose_msg : PoseStamped()
        Message for sending desired tool tip pose. 
    pose_pub : rospy.Publisher
        Publisher for sending desired tool tip pose to RViz over the topic /desired_tool_pose
    """
    def __init__(self,visualize=True):
        """
        

        Parameters
        ----------
        visualize : TYPE, optional boolean
            DESCRIPTION. The default is True. Is additional visualization required in RViz

        Returns
        -------
        None.

        """
        self.traj=Float64MultiArray()#desired trajectory message
        #initial desired task trajectory
        theta=np.array([[0,pi/3,-pi/2,0,pi/3,0]]).T
        self.trajd=np.zeros((18,1))
        self.trajd[0:6]=joint2task(theta)
        self.traj0=self.trajd[0:6].copy()
        self.traj_pub=rospy.Publisher('/task_desired',Float64MultiArray,queue_size=1,latch=True) #desired trajectory publisher
        self.visualize=visualize
        
        if self.visualize: #if visualization is needed
            #tool_tip point visualizer
            self.point_marker=Marker()
            self.point_marker.header.frame_id = "world"
            self.point_marker.type = self.point_marker.SPHERE_LIST
            self.point_marker.action = self.point_marker.ADD
            # marker scale
            self.point_marker.scale.x = 0.02
            self.point_marker.scale.y = 0.02
            self.point_marker.scale.z = 0.02
            # marker color
            self.point_marker.color.a = 1.0
            self.point_marker.color.r = 1.0
            self.point_marker.color.g = 0.0
            self.point_marker.color.b = 0.5
            # marker orientaiton
            self.point_marker.pose.orientation.x = 0.0
            self.point_marker.pose.orientation.y = 0.0
            self.point_marker.pose.orientation.z = 0.0
            self.point_marker.pose.orientation.w = 1.0
            # marker position
            self.point_marker.pose.position.x = 0.0
            self.point_marker.pose.position.y = 0.0
            self.point_marker.pose.position.z = 0.0
            self.point_marker.points=[]
            self.point_pub=rospy.Publisher('desired_tool_tip', Marker, queue_size=1, latch=True)
            
            #tool_tip direction as an arrow
            self.point_marker1=Marker()
            self.point_marker1.header.frame_id = "world"
            self.point_marker1.type = self.point_marker.ARROW
            self.point_marker1.action = self.point_marker.ADD
            # marker scale
            self.point_marker1.scale.x = 0.01
            self.point_marker1.scale.y = 0.01
            self.point_marker1.scale.z = 0.01
            # marker color
            self.point_marker1.color.a = 1.0
            self.point_marker1.color.r = 0.0
            self.point_marker1.color.g = 0.0
            self.point_marker1.color.b = 1.0
            # marker orientaiton
            self.point_marker1.pose.orientation.x = 0.0
            self.point_marker1.pose.orientation.y = 0.0
            self.point_marker1.pose.orientation.z = 0.0
            self.point_marker1.pose.orientation.w = 1.0
            # marker position
            self.point_marker1.pose.position.x = 0.0
            self.point_marker1.pose.position.y = 0.0
            self.point_marker1.pose.position.z = 0.0
            self.point_marker1.points=[]
            self.point_pub1=rospy.Publisher('desired_tool_direction', Marker, queue_size=1, latch=True)
            
            #tool_tip pose
            self.pose_msg=PoseStamped()
            self.pose_pub=rospy.Publisher('desired_tool_pose', PoseStamped, queue_size=1, latch=True)
            
    def publish_traj(self):
        """
        Starts the trajectory computation and publishing loop. Splits the time period into five segments
        and computes the desired trajectory accordingly. The magnitude of velocity and orientation
        is kept constant over the entire desired trajectory. The computed trajectory is then published to
        /task_desired topic.

        Returns
        -------
        None.

        """        
        r=rospy.Rate(200)#rate at which to publish
        t0=rospy.get_time()
        rospy.sleep(0.005)
        t0=rospy.get_time()
        if self.visualize:
            #update visualization markers
            R_euler=Rz(self.traj0[0,0]) @ Ry(self.traj0[1,0]) @ Rz(self.traj0[2,0])
            T_euler=np.eye(4)
            T_euler[0:3,0:3]=R_euler
            quat=quaternion_from_matrix(T_euler)
            
            self.pose_msg.header.frame_id="world"
            self.pose_msg.header.stamp=rospy.Time.now()
            
            self.pose_msg.pose.orientation.x=quat[0]
            self.pose_msg.pose.orientation.y=quat[1]
            self.pose_msg.pose.orientation.z=quat[2]
            self.pose_msg.pose.orientation.w=quat[3]
            
            pt1=Point()
            pt2=Point()
        while not rospy.is_shutdown():
            t_=rospy.get_time()
            t=t_-t0
            #split trajectory to five segments depending on the current time
            #and then use conditionals to compute the trajectory during 
            #these segments
            if t<1:#straight line
                p=t*(R(pi/4) @ np.array([[-0.3,0]]).T)
                dp=R(pi/4) @ np.array([[-0.3,0]]).T*1
                ddp=np.zeros((2,1))
            elif t>=1 and t<1+3*pi/2:#circular arc
                theta0=2*pi-pi/4
                theta=theta0-(t-1)
                p=0.3*np.array([[cos(theta),sin(theta)]]).T+np.array([[-np.sqrt(2)*0.3,0]]).T
                dp=0.3*np.array([[-sin(theta),cos(theta)]]).T*-1
                ddp=0.3*np.array([[-cos(theta),-sin(theta)]]).T*-1*-1
            elif t>=1+3*pi/2 and t<3+3*pi/2:#straight line
                p=np.array([[-0.3/np.sqrt(2),0.3/np.sqrt(2)]]).T+R(-pi/4) @ np.array([[0.3,0]]).T*(t-1-3*pi/2);
                dp=R(-pi/4) @ np.array([[0.3,0]]).T*1
                ddp=np.zeros((2,1))
            elif t>=3+3*pi/2 and t<3+3*pi:#circular arc
                theta0=pi+pi/4
                theta=theta0+(t-3-3*pi/2)
                p=0.3*np.array([[cos(theta),sin(theta)]]).T+np.array([[np.sqrt(2)*0.3,0]]).T
                dp=0.3*np.array([[-sin(theta),cos(theta)]]).T*1
                ddp=0.3*np.array([[-cos(theta),-sin(theta)]]).T*1*1
            elif t>=3+3*pi and t<4+3*pi:#straight line
                p=np.array([[0.3/np.sqrt(2),0.3/np.sqrt(2)]]).T+R(pi/4) @ np.array([[-0.3,0]]).T*(t-3-3*pi);
                dp=R(pi/4) @ np.array([[-0.3,0]]).T*1
                ddp=np.zeros((2,1))
            else:#reset time period
                t0=rospy.get_time()
            #need to be scaled by 0.9 because initial coordinates are out of reach    
            p=0.9*p
            dp=0.9*dp
            ddp=0.9*ddp
            #update desired trajectory message
            self.trajd[4:6,0]=self.traj0[4:6,0]+p[0:2,0]
            self.trajd[10:12,0]=dp[0:2,0]
            self.trajd[16:18,0]=ddp[0:2,0]
            self.traj.data[:]=self.trajd[:]
            if self.visualize:
                #update visualization markers
                pt1.x, pt1.y, pt1.z=self.trajd[3:6,0]
                
                pt2.x, pt2.y, pt2.z=(self.trajd[3:6]+R_euler @ np.array([[0,0,0.2]]).T).squeeze()
                self.point_marker.points=[pt1]
                self.point_marker1.points=[pt1,pt2]
                self.pose_msg.header.stamp=rospy.Time.now()
                self.pose_msg.pose.position.x=self.trajd[3,0]
                self.pose_msg.pose.position.y=self.trajd[4,0]
                self.pose_msg.pose.position.z=self.trajd[5,0]
            try:
                self.traj_pub.publish(self.traj)#oublish desired trajectory
                if self.visualize:
                    #publish visualization markers
                    self.point_pub.publish(self.point_marker)
                    self.point_pub1.publish(self.point_marker1)
                    self.pose_pub.publish(self.pose_msg)
                r.sleep()
            except:
                    break

def R(theta):
    """
    two dimensional rotation matrix

    Parameters
    ----------
    theta : TYPE
        DESCRIPTION.

    Returns
    -------
    TYPE
        DESCRIPTION.

    """
    return np.array([[cos(theta),-sin(theta)],
                      [sin(theta),cos(theta)]])

if __name__=="__main__":
    rospy.init_node('task_trajectory_generator')
    traj_gen=Task_Traj_Gen()
    traj_gen.publish_traj()