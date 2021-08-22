#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Aug 22 15:23:29 2021

@author: dsubhasish

This script is intended to dynamically change the physics properties of the 
gazebo simulation. Can be used to change the physics engine or simulation speed
or simulation step size and so on. This uses one of the services provided by
gazebo namely-/gazebo/set_physics_properties
"""
import rospy
from gazebo_msgs.srv import SetPhysicsProperties
from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ODEPhysics
import sys

class Gazebo_Physics:
    def __init__(self):
        self.time_step=0.001
        self.max_update_rate=1000
        self.gravity=Vector3()
        self.gravity.x=0
        self.gravity.y=0
        self.gravity.z=-9.8
        
    def set_time_step(self,time_step):
        self.time_step=time_step
    
    def set_max_update_rate(self,max_update_rate):
        self.max_update_rate=max_update_ratw
        
    def change_gravity(self,x,y,z):
        self.gravity.x=x
        self.gravity.y=y
        self.gravity.z=z
        
    

