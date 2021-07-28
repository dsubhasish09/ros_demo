# Launch files
The different launch files which are used for starting the simulations are-
## gazebo.launch
This launches gazebo and spawns the robot model in it. Arguments can be passed to this launch file so as to turn of its GUI, so as to reduce CPU load or to see visualization in RViz
## control.launch
This loads the required control interfaces required for receiving joint state messages from Gazebo and sending torque messages to Gazebo. Also starts the robot state publisher along with any additional static transform publisher which are required for visualization in RViz. This launch file should be run only after running gazebo.launch.
## launch_all.launch
This launch file launches the previous two launch files and also starts a node <b>sim_throttle</b> which slows down the simulation, and the <b>torque_commander</b> node which has the joint space controller, which initially holds the manipulator in a singularity free configuration. Recording of the simulation is also started.
## display.launch
Once launch_all.launch is started, display.launch can be launched to start visualization in RViz. RViz interface has options for more advanced visualization as compared to Gazebo.
## display_bag.launch
This launch file is used to playback a previously recorded simulation, such that the simulation time duration coincides with physical time duration, that is, real time playback.
## load_desc.launch
This launch file simply loads the URDF to the ros parameter server. This can be used when debugging any piece of code, which does not require a full simulation to be started.