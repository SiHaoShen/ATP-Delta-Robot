## How to Start the Project

The Project is developed on

- Ubuntu 18.04
- ROS melodic

```python
#cd to the src directory, and make the python script executable
chmod u+x IK_Solver.py

#Redirect to ROS Workspace and start building process
catkin build
source ~/"ROS Workspace"/devel/setup.bash

#Launch the ROS Project
roslaunch delta_robot delta.launch
```

![Output sample](https://git.rwth-aachen.de/si.hao.shen/atp-delta-robot/-/raw/master/gif/Delta_Robot.gif)

## Project Roadmap

- Conduct concepts of ROS from tutorial, examples
- Implement the Ragnar Project, get familiar with ROS
- Investigate the Delta Robot Kinematic
- Design URDF of Delta Robot
- Move the Delta Robot in simulation rviz and observe its performance

## Direct and Inverse kinematics

Inverse kinematics calculation (upper angles):
For details see this [pdf](https://git.rwth-aachen.de/si.hao.shen/atp-delta-robot/-/tree/master/doku/Kinematic_Calculation_IK_FK_DeltaRobot.pdf)

Forward kinematics calculation (lower angles):
For details see this [pdf](https://git.rwth-aachen.de/si.hao.shen/atp-delta-robot/-/tree/master/doku/Delta_robot_Inverse_direct_and_intermediate_Jacobi.pdf)


## Robot Operating System ROS

- A opensource framework for robotic applications
- A variety of tools and libraries are available
- Provides standard operating system services
  (device drivers, package management, hardware abstraction...)
- As modular as possible: gives robustness and versatility by developing complex robotic applications
- Three level of abstraction:
  - File system: Meta packages, packages, messages, services, repositories
  - Computation Graph: nodes, ROS master, server, messages, topics, services
  - Community: resources, distributions, ROS wiki, repositories

## Design of URDF IRB360-3/1130 robot

More details about URDF and joints: http://wiki.ros.org/urdf/XML/joint#Elements

- base: base link of the ABB IRB 360-3/1130
- end_effector: end effector link
- Designed with 3 arms and only 1 arm is connected to end effector
- 3 upper arms: real geometry with length of 0.35m
- 6 lower arms: real geomatry with length of 0.8m

## How to make the robot move

- The angles of joints states are calculated using inverse kinematics equations
- The angles degree and position of end effector are printed out
- Then sent to the URDF model via sensor_msgs/JointState message using joint_state publisher
- The motion of robot can be observed in Rviz

## Limitation of URDF

- Closed kinematic chains cannot be simulated
- No universal joints in URDF, so we used coupled revolute joints
- Floating joints has 6DOF and can be used in URDF, but it cannot be controlled by joint_state_publisher, so we connected the end_effector with a dummy part

## To Be Done

- Create Moveit! Package for the Robot using the URDF file, add collision properties
- Connect ABB Robot using ABB Controller
