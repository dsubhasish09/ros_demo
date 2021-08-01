# Control Algorithm Python files
The src subfolder of the man_control package has the python files which deal with the control algorithms and the simulation. The summary of what each of these python files are used for is given below-
## initialize_params.py
This script automates the creation of xacro files, config files and launch files required for starting the simulation in Gazebo and loading the necessary control interfaces for retrieving joint state information from Gazebo and sending command torque messages to Gazebo.
## control_func.py
This file has common functions which will be used across multiple files, classes and files. Also loads up the mass-inertia and kinematic parameters from the URDF. Should be imported individually in every python file.
## change_sim_speed.py
This script uses one of the services offered by Gazebo, namely /gazebo/set_physics_properties to slow down the rate of simulation update within Gazebo. The idea is to slow down the simulation sufficiently so that the command torque computation can be completed within the simulated control sampling interval.
## joint_controller.py
This file has a basic implementation of the joint space inverse dynamics controller. This can be used as a template for creating more complex controllers.
## joint_trajectory_generator.py
The class defined here can be used as a template for any trajectory generator in the joint space. In this specific case, a sinusoidal trajectory is generated, with the desired joint space position, velocity and acceleration sent to the joint space controller by means of the topic /theta_desired.
## task_controller.py
The class defined here can be used as a template for writing code for more complex task space controllers. Currently no constraints have been applied. This will be changed in future versions.
## task_trajectory_generator.py
This is a basic task space trajectory generator which can be used as a template to create more complex trajectory generators.

The detailed documentation for each of these files can be found at https://dsubhasish09.github.io/man_control/src/index.html.