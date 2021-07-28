# ros_demo
These are demo packages for testing new ideas in ROS. The structure of the packages are as follows-
'''
.
|-- README.md
|-- man_control
|   |-- CMakeLists.txt
|   |-- bagfiles
|   |   `-- recording.bag
|   |-- config
|   |   `-- control.yaml
|   |-- include
|   |   `-- man_control
|   |-- launch
|   |   |-- control.launch
|   |   |-- display.launch
|   |   |-- display_bag.launch
|   |   |-- gazebo.launch
|   |   |-- launch_all.launch
|   |   `-- load_desc.launch
|   |-- package.xml
|   |-- rviz
|   |   `-- urdf.rviz
|   |-- src
|   |   |-- change_sim_speed.py
|   |   |-- control_func.py
|   |   |-- docs
|   |   |   `-- src
|   |   |       |-- change_sim_speed.html
|   |   |       |-- control_func.html
|   |   |       |-- index.html
|   |   |       |-- initialize_params.html
|   |   |       |-- joint_controller.html
|   |   |       |-- joint_trajectory_generator.html
|   |   |       |-- task_controller.html
|   |   |       `-- task_trajectory_generator.html
|   |   |-- initialize_params.py
|   |   |-- joint_controller.py
|   |   |-- joint_trajectory_generator.py
|   |   |-- task_controller.py
|   |   `-- task_trajectory_generator.py
|   `-- urdf
|       |-- man_controk.urdf
|       |-- man_control.xacro
|       |-- template.xacro
|       `-- urdf.rviz
`-- man_urdf
    |-- CMakeLists.txt
    |-- README.md
    |-- config
    |   `-- joint_names_man_urdf.yaml
    |-- export.log
    |-- launch
    |   |-- display.launch
    |   `-- gazebo.launch
    |-- meshes
    |   |-- L0.STL
    |   |-- L1.STL
    |   |-- L2.STL
    |   |-- L3.STL
    |   |-- L4.STL
    |   |-- L5.STL
    |   `-- L6.STL
    |-- package.xml
    |-- textures
    |-- urdf
    |   |-- man_urdf.csv
    |   `-- man_urdf.urdf
    `-- urdf.rviz
    '''
