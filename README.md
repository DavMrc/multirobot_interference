# Multirobot interference package v1.0.0

## What is this package for
This package provides:  
1. multirobot navigation system for Turtlebot3 robots in a topological environment
2. planner that dispatches patrolling goals to all the robots
3. patrolling strategy, being the minimization of the idleness of each point of interest defined in the topological map
4. multirobot patrolling simulation, based on the above requirements
5. analysis of the **performance** of the simulation, based on the trade-off between idleness and intra-robot interference
  
Most Python files are commented, so the purpose of the scripts (and thus the respecting nodes) **won't** be covered here _unless strictly necessary_.  

## Requirements
* ROS Melodic version
* Gazebo as virtualizer, thus you'll need to have it installed. If you don't, refer to this [ROS Wiki guide](http://wiki.ros.org/gazebo_ros_pkgs).
* [Turtlebot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview) as virtual robot. As of version 1.0.0, no other robots are supported. Gladly accepting pull requests.
* [RViz](http://wiki.ros.org/rviz) as debugging and visualising tool. It should come with every ROS distro.
* **Python 2.7** and external modules such as:
    * numpy
    * matplotlib
    * cv2
    * webcolors
    * networkx
    * termcolor

    they all can be installed using `pip`

## Usage
1. [Create a new environment](https://github.com/DavMrc/multirobot_interference/wiki/Environments) for your simulations (or use one of the three that has been provided)
2. [Create a topological map](https://github.com/DavMrc/multirobot_interference/wiki/Topological-map) for the environment
3. [Configure the simulation parameters](https://github.com/DavMrc/multirobot_interference/wiki/Simulation-parameters) to suit your needs.
4. Run the simulation using one of the provided [launch files](https://github.com/DavMrc/multirobot_interference/wiki/Launch-files). For example, using the `office `environment, execute `roslaunch multirobot_interference start.launch environment:=office`
5. If you've chosen to save the simulation's performance data, [run the analysis tool](https://github.com/DavMrc/multirobot_interference/wiki/Performance-analysis).
