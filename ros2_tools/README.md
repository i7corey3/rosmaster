# ros2_tools

This repository expands on the ros2 package creations commands by auto generating majority of the ros2 related code including:
- Creating a config folder with an auto generated parameters file
- Creating a launch folder with an auto generated launch file ready to use
- Creates a generic node file linked with the ros2 command
- Works with both python and c++ packages
- Available for ros2 on Linux and Windows (windows not fully tested)
- Creating a generic main folder for storing URDF files, maps, worlds and global launch scripts

## How To Use

Clone this repo inside your ros2_ws

> createPackage.sh if using ros2 on Linux, createPackage.bat for Windows


To create an example package in python run:

```
./createPackage.sh python <package_name> <node_name>
```

To create an example package in c++ run:

```
./createPackage.sh c++ <package_naem> <node_name>
```
To create a main package (URDF, etc) run:
```
./createPackage.sh main <package_name> 
```
To create custom message for topiccs and services run:
```
./createPackage.sh messages <package_name> 
```
To create a systemctl folder run:
```
./createPackage.sh systemctl
```
> systemctl folder handles allocating and managing custom systemctl scripts used for linux only.
### Notes

Currently these tools have been tested with ROS2 Humble. Experience may differ for other ROS Distros


Currently the windows createPackage.bat file only works with ros2 galactic 

If you have another ros2 distro change the following line to the correct path inside the createPackage.bat file

```
call C:\opt\ros\galactic\x64\setup.bat
```
