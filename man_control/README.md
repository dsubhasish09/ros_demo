# man_control
This package houses the files corresponding to the control algorithms. The structure of this package is shown in the following tree-
<pre>
man_control
├── CMakeLists.txt
├── README.md
├── bagfiles
│   └── recording.bag
├── config
│   └── control.yaml
├── launch
│   ├── README.md
│   ├── control.launch
│   ├── display.launch
│   ├── display_bag.launch
│   ├── gazebo.launch
│   ├── launch_all.launch
│   └── load_desc.launch
├── package.xml
├── rviz
│   └── urdf.rviz
├── src
│   ├── README.md
│   ├── change_sim_speed.py
│   ├── control_func.py
│   ├── docs
│   │   └── src
│   │       ├── change_sim_speed.html
│   │       ├── control_func.html
│   │       ├── index.html
│   │       ├── initialize_params.html
│   │       ├── joint_controller.html
│   │       ├── joint_trajectory_generator.html
│   │       ├── task_controller.html
│   │       └── task_trajectory_generator.html
│   ├── initialize_params.py
│   ├── joint_controller.py
│   ├── joint_trajectory_generator.py
│   ├── task_controller.py
│   └── task_trajectory_generator.py
└── urdf
    ├── man_control.xacro
    └── template.xacro
</pre>
In the above tree, <b>package.xml</b> and <b>CMakeLists.txt</b> are used to define the dependencies and any message or service or action type which are defined within the package. The different subfolders in this package are explained below-
## urdf
This folder houses the xacro file, which was obtained by running initialize_params.py in the src folder. It also contains <b>template.xacro</b> which defines the xacro macros which are used.
## config
Contains the <b>control.yaml</b> which has the configuration for the controller to be loaded. 
## launch
Contains launch files for starting different types of simulation in Gazebo or/and RViz as required.
## src
This folder houses all of the control python files and its documentation.
## rviz
Contains <b>urdf.rviz</b> which is the rviz configuration file for loading the visualization in RViz.
## bagfiles
Contains any bag file which has the recording of any simulation session. This can be used to playback the simulated animation such that simulation time duration coincides with physical time duration.