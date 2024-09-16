# skratch_ROS2
Skratch bot is a replacement for youBot-brsu used by b-it-bots@Work "mas_industrial_robotics". It is designed and developed using the robot parts of old and retired robots available in b-it-bots lab at Hochschule Bonn-Rhein-Sieg in collaboration with RoBonn-Systems.

This repositiory is for skratch ROS2 development

To open rviz2 and visualize the base, open the terminal and execute below commands

Create a new workspace

    mkdir robonn_ws

    cd robonn_ws && mkdir src

Build the workspace

    cd .. && colcon build

Clone the required repositories for skratch ROS2 and description 

    cd src

    git clone https://github.com/RoBonn-Systems/skratch_ros2.git

    git clone https://github.com/RoBonn-Systems/skratch_description.git

Build the repository packages

    cd .. && colcon build

Source the workspace

    source install/setup.bash

To run the basic rviz2 simulation execute below launch file

    ros2 launch skratch_ros2 robot.launch.py
