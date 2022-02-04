# ros_demo
These are demo packages for testing new ideas in ROS. The entire project tree is as follows-
<pre>
ros_demo
├── README.md
├── man_control
│   ├── CMakeLists.txt
│   ├── README.md
│   ├── bagfiles
│   │   └── recording.bag
│   ├── config
│   │   └── control.yaml
│   ├── launch
│   │   ├── README.md
│   │   ├── control.launch
│   │   ├── display.launch
│   │   ├── display_bag.launch
│   │   ├── gazebo.launch
│   │   ├── launch_all.launch
│   │   └── load_desc.launch
│   ├── package.xml
│   ├── rviz
│   │   └── urdf.rviz
│   ├── src
│   │   ├── README.md
│   │   ├── gazebo_physics_params.py
│   │   ├── control_func.py
│   │   ├── initialize_params.py
│   │   ├── joint_controller.py
│   │   ├── joint_trajectory_generator.py
│   │   ├── task_controller.py
│   │   └── task_trajectory_generator.py
│   └── urdf
│       ├── man_control.xacro
│       └── template.xacro
└── man_urdf
    ├── CMakeLists.txt
    ├── README.md
    ├── launch
    │   ├── display.launch
    │   └── gazebo.launch
    ├── meshes
    │   ├── L0.STL
    │   ├── L1.STL
    │   ├── L2.STL
    │   ├── L3.STL
    │   ├── L4.STL
    │   ├── L5.STL
    │   └── L6.STL
    ├── package.xml
    ├── textures
    ├── urdf
    │   ├── man_urdf.csv
    │   └── man_urdf.urdf
    └── urdf.rviz
</pre>

As seen from the above tree, currently two packages have been made-
## man_urdf
This is the ROS package having the URDF of a puma type 6 DOF serial chain manipulator. Some launch files have been added for basic visualization.
## man_control
This ROS package contains python files which implement planners, trajectory generators and controllers for the manipulator mentioned above and also launch files to start the simulation on Gazebo or ROS as required

The simulation can be launched via-

```
roslaunch man_control launch_all.launch
```
This starts the simulation in Gazebo with the Joint Space Inverse Dynamics Controller active. In order to shutdown the Joint Space controller and start the Task Space Inverse Dynamics Controller use-
```
rosrun man_control task_controller.py
```
To start a dummy task space trajectory generator use-
```
rosrun man_control task_trajectory_generator.py
```
For visualizing in RViz use-
```
roslaunch man_control display.launch
```