# Turtlebot3 carrot planner demo
Demonstrates the carrot planner running on a turtlebot in simulation.

Simulator:
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
Carrot planner navigation:
```bash
roslaunch turtlebot3_carrot_planner navigation_amcl.launch
```
(Or alternatively) the original navigation stack:
```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```
Visualization:
```bash
roslaunch turtlebot3_carrot_planner rviz.launch
```
