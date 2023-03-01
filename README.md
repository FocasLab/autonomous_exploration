# Autonomous Exploration using Symbolic Motion Planner (SMP)

Autonomous_Exploration is framework for exploration planning and mapping. It is based on Symbolic controller based approach, to tackle navigation and exploration while providing safety guarantees. It can be used to perform Autonomous Navigation (SMP) or Exploration.

# Table of Contents
**Credits**
* Video

**Setup**
* [Installation](#Installation)
* [Example](#Example)

# Video
For a short overview of the system check out our video on youtube.

[![AutoExpl Demo](https://user-images.githubusercontent.com/56308805/222194781-07fbe5c3-a2ff-4e5d-a48b-aa81f4b6a83b.jpg)](
https://www.youtube.com/watch?v=DTSIa1ccz6o)

# Installation
Installation instructions for Linux.

**Prerequisites**
1. If not already done so, install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full is recommended).
   

**Installation**
1. Move to your catkin workspace:
    ```shell script
    cd ~/catkin_ws/src
    ```

2. Download repo using a SSH key or via HTTPS:
    ```shell
    git clone https://github.com/FocasLab/autonomous_exploration
    ```

3. Install system dependencies:
    ```shell script
    sudo apt-get install python-wstool python-catkin-tools
    ```

4. Download and install turtlebot3 packages from 
    ```
    https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation
    ```
    or other Robot Simulation such as AgileX Limo, Clearpath Jackal, etc.

5. Compile and source:
    ```shell 
    catkin build autoexpl
    source ../devel/setup.bash
    ```

**_NOTE:_**  This Package can be configured to be used with other robots (simulation or physical robot) with minimal changes.
Here we use turtlebot3 to showcase a simple example.

# Example

After successful installation now we will launch a minimal example in on Turtlebot3 in a Maze environment.

#### Launch the environment
```bash
roslaunch autoexpl_simulations autoexpl_world.launch world:=docker
```

#### Launch the slam 
```bash
roslaunch autoexpl_simulations autoexpl_slam.launch slam_methods:=karto
```

#### Launch the autonomous exploration nodes
```bash
roslaunch autoexpl_ros autonomous_exploration.launch
```

**_NOTE:_**  As discussed in the paper, this approach is computationally expensive, we recommend using a high spec'd system or run small experiments on small arena.  

Recommended Specs ( equivalent or higher )
* Processor - Intel i7 
* RAM - 16GB 
* Graphics Card - 4GB
* OS - Ubuntu 20.04 with ROS Noetic
