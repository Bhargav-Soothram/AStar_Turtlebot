# Simuation of A* algorithm using Turtlebot3
Following are the instructions required to run the script:

* Download the package and place it in a catkin workspace in the src folder.

* Build the project using `catkin build tbot_astar` in a terminal, whilst in the workspace.

* Now, use the command `roslaunch tbot_astar turtlebot3_prjct3.launch` to open a gazebo world with the map provided.

* In another terminal tab, enter the following commands:

    >> `cd ~/catkin_ws/src/tbot_astar/src` 

    >> `chmod +x Navigator.py`

    >> `python3 Navigator.py`

The action begins!

## Important Notes

* It is expected that ROS and turtlebot packages are already installed in the system and this code works perfectly only on ROS Noetic

* Dependencies: numpy, queue, argparse, rospy, geometry_msgs, time
