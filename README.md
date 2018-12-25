# URDF to AMFB Converter
Convert URDF to AF MultiBody Format

#### Author:
Adnan Munawar

Email: amunawar@wpi.edu

#### Decription:
1. This is a python converter for converting URDF files to compatible AFMB file format. 
AFMB stands for (Asynchoronous Framework Multi-Body). AFMB is real-time dynamics engine
based on Bullet and CHAI-3D with ROS Support on Linux. The source code is located at:
"https://github.com/WPI-AIM/chai3d"

2. AFMB Config files are akin to URDF or SDF but are written in YAML rather than XML. AFMB also supports
soft bodies as well as multiple unconnected, semi-connected and fully connected dynamic bodies in simulation.

3. AFMB files allow multiple parents as well as cyclical interconnection which is not possible with URDF and SDF.

4. Joints are considered independent objects, similar to the bodies in the environment. Joints can easily be ignored, 
added and modified in the AFMB Yaml config files.

5. Because of the underlying philosophy of treating joints as independent objects, the AFMB yaml config files can seperate out joints from the bodies in differnet files. E.g. one config file can contain information about the bodies only and another config file can contain information about the joints. In addition to this features, the joints can be added at run-time for any dynamic ridig body in simulation.


#### Usage:
1. __python urdf_2_afmb <urdf_file> <save_to>__
2. __python urdf_2_afmb <urdf_file>__
	1. _Then specify output file at prompt_
	2. Or _Press 'x' to print to console_
