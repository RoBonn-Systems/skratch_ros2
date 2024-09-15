# skratch
Skratch bot is a replacement for youBot-brsu used by b-it-bots@Work "mas_industrial_robotics". It is designed and developed using the robot parts of old and retired robots available in b-it-bots lab at Hochschule Bonn-Rhein-Sieg in collaboration with RoBonn-Systems.

To open RVIZ and visualize the base, open the terminal and go to the workspace directory and execute below commands

`mkdir robonn_ws`

`cd robonn_ws && mkdir src`

`cd .. && colcon build`

`cd src`

`git clone https://github.com/RoBonn-Systems/skratch.git`

`git clone https://github.com/RoBonn-Systems/skratch_description.git`

`cd skratch && git checkout ros2_humble`

`cd ../.. && colcon build`

`ros2 launch skratch robot.launch.py`