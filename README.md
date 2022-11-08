# ARNOLD FSM and AI-based Navigation on ROS framework
# Arnold introduction, hardware components, start phase
ARNOLD is mobile robot used to sanitize shared environment and objects (keyboards, tables, monitor etc) in an autonomous way. We worked hard trying to improve the intelligence, the object recognition, and the autonomy of this mobile platform. This report follows the footsteps that we have made to keep trace of this work evolution in time.

ARNOLD is built by integrating some different hardware components and it’s formed by two parts: the mobile platform (initial sanitizing with UV lamp) and the arm manipulator (final sanitation phase with UV lamp in the arm).

On the end-effector of the manipulator is mounted an RGB-D “Intel RealSense D455” camera used to map the surrounding environment, in order to make some trajectory planning and obstacle avoidance. The robot is also equipped with a LIDAR 3D sensor “Velodyne”(360° of range), used for the localization of the mobile platform. 
<p align="center">
  <img src="https://user-images.githubusercontent.com/117543360/200363990-82f51a1e-0449-400d-aebd-bc9c52621566.jpg" width="300"/>
  <img src="https://user-images.githubusercontent.com/117543360/200364007-d5eb268a-2ae2-4145-b543-20ad58b14784.jpg" width="300"/> 
</p>
In order to implement the position control of the platform high-performarce drivers are implemented, called “Odrive”. To ensure the correct operation of the Odrive with the motors used, an accurate phase of calibration is carried out for the PID velocity controller by using Ziegler-Nichols’s method. 
The central brain of ARNOLD is a mini-PC (“NUC”) Intel i9 CPU and 32GB RAM used for the computation. Moreover, we need also a local network implemented by a Gigabit access point to control and upload executable files in a wireless manner.
The power is supplied by the platform batteries (24V), the software control is implemented through the micro-USB port on the board connected to the on-board computer, the encoders are connected to the corresponding input pins (5 pins are required for each encoder : signals A, B, C + 5 volts for power supply and GND). The phases of the motors A, B, C are connected to the corresponding connectors, and finally, a brake resistor is also connected.
<p align="center">
<img src="https://user-images.githubusercontent.com/117543360/200368544-760891ee-8fe9-406e-bfd3-7808bf9b2478.jpg") width="400"/>
</p>
After turning on the robot (battery on, aubo robot arm, inverter turn on for the NUC) we connect to the Intel NUC using ssh protocol.

## Slam Architecture and Move Base node
In ROS it is possible to plan a path based on occupancy grid, e.g. the one obtained from slam_gmapping. Path planner is “move_base” node from move_base package.
The move_base package provides an implementation of an action (see the actionlib package) that, given a goal in the world, will attempt to reach it with a mobile base. The move_base node links together a global and local planner to accomplish its global navigation task.
Running the move_base node on a robot that is properly configured (please see navigation stack documentation for more details) results in a robot that will attempt to achieve a goal pose with its base to within a user-specified tolerance. In the absence of dynamic obstacles, the move_base node will eventually get within this tolerance of its goal or signal failure to the user. The move_base node may optionally perform recovery behaviors when the robot perceives itself as stuck. By default, the “move_base” node will take the following actions to attempt to clear out space:
- First, obstacles outside of a user-specified region will be cleared from the robot's map. Next, if possible, the robot will perform an in-place rotation to clear out space. If this too fails, the robot will more aggressively clear its map, removing all obstacles outside of the rectangular region in which it can rotate in place. This will be followed by another in-place rotation. If all this fails, the robot will consider its goal infeasible and notify the user that it has aborted. These recovery behaviors can be configured using the “recovery_behaviors” parameter and disabled using the “recovery_behavior_enabled” parameter.
This is a prototype of move base node launch file:

```
  <node pkg="move_base" type="move_base" name="move_base" output="screen">
    <param name="controller_frequency" value="10.0"/>
    <rosparam file="$(find tutorial_pkg)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find tutorial_pkg)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find tutorial_pkg)/config/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find tutorial_pkg)/config/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find tutorial_pkg)/config/trajectory_planner.yaml" command="load" />
  </node>
```
For move_base node some parameters for cost map and trajectory planner need to be defined, they are stored in .yaml files (package>config).
We have four files:
- Common parameters for Cost Map
- Parameters for local Cost Map
- Parameters for global Cost Map
- Parameters for trajectory planner

These parameters are used by trajectory planner: maximum/minimum linear velocity, angular velocity, accelerations, holonomic constraints, goal tolerance (xy, yaw).
Finally, having all the elements visualized, set goal for robot, from Toolbar click button 2D nav goal, then click a place in Visualization window, that will be destination for your robot.

## SMACH Node
SMACH is a task-level architecture for rapidly creating complex robot behavior. It is used to create a finite state machine, where all possible states and state transitions can be described explicitly.  At its core, SMACH is a powerful and scalable ROS-independent Python library to build hierarchical state machines: it could be used in any Python project.
"State" can mean different things in different contexts. In SMACH, a "state" is a local state of execution, or equivalently a "state" corresponds to the system performing some tasks. This is different from formal state machines, where each state describes not what the system is doing but rather describes a given configuration of the system.
Before using the SMACH architecture, the SLAM of the mobile base was organized through some “launch” files. 
The idea of using SMACH is crucial to obtain an optimized finite state machine in ROS, with perfectly organized states and transitions. In order to make the robot operations as automatic as possible, we also deleted the delays used in the launch files (they were used to make sequential the various nodes, but in a “dummy” and custom way).
So, we decided to implement three states for the mobile base:
- Initializing
- Check
- Kill and Intersection

During the Initializing state we launch two launch files(“base_node.launch”, “save_maps.launch”). In the first launch file the following nodes are launched: 
-	The virtual environment(Rviz), the odrive controller and the odometry publisher
-	The different nodelet packages:  manager, transform, laserscan
-	The “amcl” and “move_base” nodes
-	The realsense camera’s drivers
-	Pointcloud assembler for the environment, UVC and lamp
-	Exploration

In the same Smach state we launch “save_maps.launch”, where is present:
- Pointcloud saver
- UVC dosage saver
- Lamp dosage saver

Then the FSM passes to the next state “Check”. It is used to check when the “exploration phase” ends. Here we have improved the autonomy of ARNOLD, removing the custom delays used previously.

## Smach Manipulator
The smach state machine for the manipulator consists of three states: (this code is present in the Intel NUC of the robot, is not present in this repository)
- Initial Mapping
- Save
- Clean

In the first state, the launch file "arm_mapping.launch" is ran, containing:

- The node for the initial mapping(python script), with this script the arm reaches some points in order to make a good mapping of the environment using the RealSense at the end effector
- The arm point cloud assembler
- Octomap assembler

After this we have a unique pointcloud map and an octomap, obtained as combinations of the single scene captured by the camera. After the transition occurs, the next state is executed, in which is launched the file “saving_pointcloud.launch” and, after some seconds, the pointcloud savers are killed. 
The last state regards the cleaning phase. Originally the arm made a trajectory planning with segmentation of two perpendicular planes, supposed to be: one the desk and the other the monitor.
## AI-Based Navigation
In order to make Arnold a fully autonomous robot, we had to make use of vision sensors combined with state-of-the-art artificial intelligence algorithms for object recognition. Vision sensors have become mandatory for autonomous behavior in robots, since they provide a way for the robot to perceive the surroundings and take actions based on the current state of the environment. The vision sensor used for this work is the Realsense D455 RGBD camera.
Our goal is to make the robot intelligent by recognizing a workstation in the environment using state-of-the-art real-time object recognition algorithms, such as YOLOv3 and then approach our target. 
### Yolo Implementation
Yolo has been implemented in our robot by installing Darknet. 
It’s possible to change which YOLO algorithm you use by changing in the darknet_ros.launch file the following line: 
```
<arg name="network_param_file" default="$(find darknet_ros)/config/yolov2-tiny.yaml"/>
```
And you can change the “yolov2-tiny.yaml” param into different YOLO versions, for example if you want to use YOLOv3, change the line into “$(find darknet_ros)/config/yolov3.yaml”. Keep in mind that some of these Deep Neural Networks might be much more computationally intensive, for example YOLOv3 is a 70-layer Artificial Neural Network while yolov2-tiny is a 12 layer network, allowing for much faster inference at the cost of less accuracy. You also have to check which classes the network has been trained on, for example yolov3 does not recognize tables but yolov2-tiny does. 
An example of the working principle is shown below:
<p align="center">
<img src="https://user-images.githubusercontent.com/117543360/200402380-fa7b8b7f-7446-4126-8eca-77e7deb64824.png" />
</p>
We iterate through the list of classes, and if the class is a “diningtable” we start to approach the object (we assume that a table corresponds to an employee workstation that needs to be sanitized). 
<p align="center">
  <img src="https://user-images.githubusercontent.com/117543360/200404607-04a46044-2d77-4eb6-b84b-4a2dc69e4329.jpg" width="300"/>
  <img src="https://user-images.githubusercontent.com/117543360/200404616-b184c3a3-0b67-44e6-9ca7-545e044e4e99.jpg" width="290"/> 
</p>
The target distance is given by taking a square matrix of elements from the Depth map image around the center of the bounding box, and then an average value of those distances is given to the movebase actionlib. 
