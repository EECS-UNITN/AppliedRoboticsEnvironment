# Project of Apply robotics laboratory 
In this course the student will be introduced to a complete methodology for designing modern robot applications. The target applications will be those for which system intelligence plays a prominent role, first and foremost planning, task optimisation and perception. The student activities will revolve around a motivational application proposed in the form of a simple robotic competition, in which the robot has to complete a number of tasks in minimum time avoiding obstacles and dangerous spots. Contrary to a standard robotic competition, the student will have to discuss the theoretical underpinning of the method she/he chose to adopt for the competition. 

## Project objectives
- development and implementation of sensing algorithms to recognise objects and obstacles in the competition field
- development and implementation of motion planning algorithms to move the robot from an initial to a final position in minimum time avoiding obstacles
- development and implementation of intelligent planning algorithms to decide the optimal game strategy
- testing of the solution in simulation and on the field

## Prerequisites/Dependencies
* Linux 16.04
* Gazebo >= 7.0  
* ROS Kinetic  
* make >= 4.1
* gcc/g++ >= 5.4

*** work in progress Docker container ***

## Run the project
* Create and init the catkin workspace
```bash
mkdir -p /home/workspace/catkin_ws/src
cd /home/workspace/catkin_ws/src
catkin_init_workspace
```
* Clone the repository
```bash
cd /home/workspace/catkin_ws/src
git clone https://github.com/ValerioMa/AppliedRobotics.git
```
* Build the catkin package
```bash
cd /home/workspace/catkin_ws
catkin_make
```
* Open 2 terminal and run the simulation
On a first terminal run the physic engine (gazebo)
```bash
cd /home/workspace/catkin_ws
source devel/setup.bash
roslaunch sim_common sim.launch
```
Open the second terminal run the project pipeline:
```bash
cd /home/workspace/catkin_ws
source devel/setup.bash
roslaunch project_interface node_pipeline.launch
```

*** work in progress STUDENT API ***
In this state the simulator uses the default implementation of the project_interface library and reises various exception. 
To have the simulator running the student have to download another repository and implement the function. A working implementation for the default function can be downloaded from .... . 
Using the configuration file in 9_prokect_interface/config/default_implementation.config the user can select if the system should load the default or the custom defined functions.