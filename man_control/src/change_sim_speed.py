#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun  8 21:50:09 2021

@author: dsubhasish
"""
import rospy
from gazebo_msgs.srv import SetPhysicsProperties
from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ODEPhysics
import sys

if __name__=="__main__":
    rospy.wait_for_service('/gazebo/set_physics_properties')
    set_phys = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
    time_step=0.001
    mult=float(sys.argv[1])
    max_update_rate=mult*1000.0
    gravity=Vector3()
    gravity.x=0.0
    gravity.y=0.0
    gravity.z=-9.8
    ode_config=ODEPhysics()
    ode_config.auto_disable_bodies=False
    ode_config.sor_pgs_precon_iters= 0
    ode_config.sor_pgs_iters= 50
    ode_config.sor_pgs_w= 1.3
    ode_config.sor_pgs_rms_error_tol= 0.0
    ode_config.contact_surface_layer= 0.001
    ode_config.contact_max_correcting_vel= 100.0
    ode_config.cfm: 0.0
    ode_config.erp: 0.2
    ode_config.max_contacts: 20
    set_phys(time_step,max_update_rate,gravity,ode_config)