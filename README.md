# UR-Lunabotics
Repository holding all UR Lunabotics 2021-2022 software
Developed by the UR Robotics Club

## Contributors:

- Benned Hedegaard

## Algorithms Implemented:

- Velocity-based motion model for forward simulation of a differential drive robot.
- A* search for planning paths on a discrete grid.
- Pure pursuit for path-following control.
- Occupancy grid mapping using laserscan sensor data.
 

## Demonstrations:

*Simulator* - Run `roslaunch simulator simulator-demo.launch` to see the simulator handle a constant motion command. The robot should drive in a circle.

*Planner* - Run `roslaunch planner planner_demo.launch` to see the planner operate on a continuous stream of queries from (0,0) to points on a 3m-radius circle.

*Joystick Control* - Run `roslaunch simulator joystick-demo.launch` to control a simulated robot using a joystick controller connected via USB to the machine running ROS.
