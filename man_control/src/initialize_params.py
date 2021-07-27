#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 24 18:40:24 2020

@author: dsubhasish

This script automates the creation of xacro files, config files and launch files
required for starting the simulation in Gazebo and loading the necessary control
interfaces for retrieving joint state information from Gazebo and sending command
torque messages to Gazebo. This is achieved by the following three steps-

1) combines the urdf extracted from CAD within the man_urdf packahe with additional 
gazebo and transmission tags. Defines the control plugin. Resulting xacro file is 
man_control/urdf/man_control.xacro

2) defines the controller to be used and also the joint state publisher from gazebo.
This is written within man_control/config/control.yaml

3) writes a launch file for loading the controller and control interfaces and also
starts a robot state publisher along with some additional static transform publshers 
for visualization in RViz
"""

#Defining Link Names
print("Initializing paramaters...")
links=["L0","L1","L2","L3","L4","L5","L6"]
#Defining Joint Names
joints=["J1","J2","J3","J4","J5","J6"]
print("Done!!!")

#Combining URDFs and adding gazebo tags
print("Writing combined XACRO file...")
folder1="../urdf"
file1=open(folder1+"/man_control.xacro","w+")
file1.write('<?xml version="1.0"?>\n')
file1.write('<robot name="man_control" xmlns:xacro="http://www.ros.org/wiki/xacro">\n')
file1.write('<xacro:include filename="$(find man_control)/urdf/template.xacro" />\n')
file1.write('<xacro:controllib/>\n')
file1.write('<xacro:include filename="$(find man_urdf)/urdf/man_urdf.urdf" />\n')
#Adding gazebo link tags
for link in links:
    file1.write('<xacro:gazebo_link link="'+link+'"/>\n')
#Adding joint transmission
for joint in joints:
    file1.write('<xacro:gazebo_transmission joint="'+joint+'" gear_ratio="100"/>\n')
file1.write('</robot>')
file1.close()
print("Done!!!")

#Creating Config File
print("Writing config file...")
folder2="../config"
file2=open(folder2+"/control.yaml","w+")
file2.write("manipulator:\n")
file2.write(" joint_state_controller:\n")
file2.write("  type: joint_state_controller/JointStateController\n")
file2.write("  publish_rate: 200\n")
controllers="joint_state_controller"


#joint group effort controller
controller="group_effort_controller"
controllers=controllers+"\n"+controller
file2.write(" "+controller+":\n")
file2.write("  type: effort_controllers/JointGroupEffortController\n")
file2.write("  joints:\n")
for joint in joints:
    file2.write("    - "+joint+"\n")

file2.close()

print("Done!!!")

#Creating launch files
print("Writing launch files...")
folder3="../launch"
file3=open(folder3+"/control.launch","w+")
file3.write("<launch>")
file3.write('<rosparam file="$(find man_control)/config/control.yaml" command="load"/>"\n')
file3.write('<node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen" ns="/manipulator" args="'+controllers+'"/>\n')
file3.write('<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen">\n')
file3.write('<remap from="/joint_states" to="/manipulator/joint_states" />\n')
file3.write('<param name="publish_frequency" value="200" type="double"/>\n')
file3.write('</node>\n')
file3.write('<node pkg="tf2_ros" type="static_transform_publisher" name="tool_tip_broadcaster" args="0 0 0.25722535 0 0 0 1 /L6 /tool_tip"/>\n')
file3.write("</launch>")
file3.close()
print("Done!!!")
print("Initializing successful!!!")
