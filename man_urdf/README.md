# man_urdf
This package has the URDF for the puma type 6 DOF serial link manipulator. The purpose of having a dedicated urdf only package is to aid in easy sharing of URDF between different development teams. The structure of this package is shown in the following tree-
<pre>
man_urdf
├── CMakeLists.txt
├── README.md
├── config
│   └── joint_names_man_urdf.yaml
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
In the above tree, <b>package.xml</b> and <b>CMakeLists.txt</b> are used to define the dependencies and any message or service or action type which are defined within the package. <b>urdf.rviz</b> is the rviz configuration file for loading a basic kinematic visualization with just the Robot Model. The different subfolders in this package are explained below-
## urdf
Contains the URDF file.
## meshes 
Contains the stl files corresponding to the visual and collision mesh for the different links of the manipulator.
## launch 
Contains launch files to start simulation.