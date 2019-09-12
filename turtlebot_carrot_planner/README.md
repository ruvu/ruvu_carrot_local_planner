# Turtlebot carrot planner demo
Demonstrates the carrot planner running on a turtlebot in simulation.

## Usage
Execute the following commands in separate terminals:
```bash
roslaunch turtlebot_gazebo turtlebot_world.launch
roslaunch turtlebot_carrot_planner navigation_amcl.launch 
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

Teleop:
```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```
