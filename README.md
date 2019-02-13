# URDF to AMFB Converter
Convert URDF to AMBF Format

#### Author:
Adnan Munawar

Email: amunawar@wpi.edu

#### Decription:
1. This is a python converter for converting URDF files to compatible AMBF file format. 
AMBF stands for (Asynchoronous Multi-Body Framework). AMBF is real-time dynamics engine
based on Bullet and CHAI-3D with ROS Support on Linux. The source code is located at:
"https://github.com/WPI-AIM/ambf"

2. AMBF Config files are akin to URDF or SDF but are written in YAML rather than XML. AMBF also supports
soft bodies as well as multiple unconnected, semi-connected and fully connected dynamic bodies in simulation.

3. AMBF files allow multiple parents as well as cyclical interconnection which is not possible with URDF and SDF.

4. Joints are considered independent objects, similar to the bodies in the environment. Joints can easily be ignored, 
added and modified in the AMBF Yaml config files.

5. Because of the underlying philosophy of treating joints as independent objects, the AMBF yaml config files can seperate out joints from the bodies in differnet files. E.g. one config file can contain information about the bodies only and another config file can contain information about the joints. In addition to this features, the joints can be added at run-time for any dynamic ridig body in simulation.


#### Usage:
1. `python urdf_2_ambf <urdf_file> <save_to>`

Example: `python urdf_2_ambf /home/user/ros_ws/my_robot.urdf /home/user/my_robot.yaml`

2. `python urdf_2_ambf <urdf_file>`
	1. _Then specify output file at prompt_
	2. Or _Press 'x' to print to console_
