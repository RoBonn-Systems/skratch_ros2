# skratch_ROS2

Skratch bot is a replacement for youBot-brsu used by b-it-bots@Work ["mas_industrial_robotics"](https://b-it-bots.readthedocs.io/projects/mas-industrial-robotics/en/latest/index.html). It is designed and developed using the robot parts of old and retired robots available in [b-it-bots](https://www.h-brs.de/en/a2s/b-it-bots) lab at [Hochschule Bonn-Rhein-Sieg](https://www.h-brs.de/en) in collaboration with RoBonn-Systems.

This repositiory is for skratch ROS2 development

## Fresh Install

1. Install the following system level packages:

```console
sudo apt-get install -y git ansible
mkdir -p ~/skratch_ws/src && cd ~/skratch_ws/src
git clone https://github.com/RoBonn-Systems/skratch_ros2 && cd skratch_ros2
```

Note 📌: do not change the workspace name, as the internal tools will always search for the previously defined workspace.

2. The following command will configure everything for you, just enter your user password when it says *BECOME password*, and sit back and relax:

```console
ansible-playbook local.yml --ask-become --tags developer
```

3. If everything runs smoothly, you only have to compile the workspace:

```console
cd ~/skratch_ws && colcon build --symlink-install && source install/local_setup.bash
```

At this point everything is ready ✅.

## Custom Setup

1. Follow just step (1) from [Fresh Install](#fresh-install).

The automation script allows using tags to execute only specific tasks, the available tags are the following:
- `developer`: setup everything to run the stack in the developer's computer.
- `robot`: setup everything to run the stack in the robot's computer.
- `tooling`: install some system level tools (e.g. tmux).
- `vcs`: pull all external repositories.
- `rosdep`: install all the packages dependencies.
- `env`: set environment variables, aliases and more at the system level.
- `ros`: install ros for you.

you can execute a single tag or a combination of multiple tags with the following command:

```console
ansible-playbook local.yml --ask-become --tags chosen_tag1 chosen_tag2 chosen_tag3
```

## Usage

To run the basic rviz2 simulation execute below launch file

```console
ros2 launch skratch_bringup robot.launch.py
```
