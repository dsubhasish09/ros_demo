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
        rospy.wait_for_service('/gazebo/set_physics_properties')
        self.set_physics=rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        
    def set_time_step(self,time_step):
        self.time_step=time_step
    
    def set_max_update_rate(self,max_update_rate):
        self.max_update_rate=max_update_rate
        
    def change_gravity(self,x,y,z):
        self.gravity.x=x
        self.gravity.y=y
        self.gravity.z=z
        
    def set_physics_ODE(self, auto_disable:bool=False,precon_iters:int=0,iters:int=100,w:float=1.3,rms_error_tol:float=0.0,surface_layer:float=0.001,max_correcting_vel:float=100,cfm:float=0.0,erp:float=0.5,max_contacts:int=100):
        """
        Sets the physics engine to ODE and configures its parameters.

        Parameters
        ----------
        auto_disable : bool, optional
            DESCRIPTION. The default is False.
        precon_iters : int, optional
            DESCRIPTION. The default is 0.
        iters : int, optional
            DESCRIPTION. The default is 100.
        w : float, optional
            DESCRIPTION. The default is 1.3.
        rms_error_tol : float, optional
            DESCRIPTION. The default is 0.0.
        surface_layer : float, optional
            DESCRIPTION. The default is 0.001.
        max_correcting_vel : float, optional
            DESCRIPTION. The default is 100.
        cfm : float, optional
            DESCRIPTION. The default is 0.0.
        erp : float, optional
            DESCRIPTION. The default is 0.5.
        max_contacts : int, optional
            DESCRIPTION. The default is 100.

        Returns
        -------
        None.

        """
        self.physics=ODEPhysics()
        self.physics.auto_disable_bodies=auto_disable
        self.physics.sor_pgs_precon_iters=precon_iters
        self.physics.sor_pgs_iters=iters
        self.physics.sor_pgs_w=w
        self.physics.sor_pgs_rms_error_tol= rms_error_tol
        self.physics.contact_surface_layer= surface_layer
        self.physics.contact_max_correcting_vel= max_correcting_vel
        self.physics.cfm= cfm
        self.physics.erp= erp
        self.physics.max_contacts= max_contacts
    
    def change_physics(self):
        self.set_physics(self.time_step,self.max_update_rate,self.gravity,self.physics)
    
if __name__=="__main__":
    max_update_rate=int(sys.argv[1])
    time_step=float(sys.argv[2])
    g_phys=Gazebo_Physics()
    g_phys.set_max_update_rate(max_update_rate)
    g_phys.set_time_step(time_step)
    g_phys.set_physics_ODE()
    g_phys.change_physics()
