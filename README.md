[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/MsvVKf7Q)
# INFO 5356-030: Introduction to Human-Robot Interaction

# Lab 4: Social Navigation

In this lab, you will learn the basics of robot navigation using the Navigation Stack in Robot Operating System 2 (ROS2). There are two localization methods we can use to determine where the robot is on the map, including SLAM or localization. SLAM is useful for generating a new map or navigating in unknown or dynamic environments. It updates the map as it detects and changes, but cannot see areas of the environment that it has not discovered yet. Localization uses an existing map along with live odometry and laser scan data to figure out the position of the robot on the given map. It does not update the map if any changes have been made to the environment, but we can still avoid new obstacles when navigating. Because the map doesn't change, we can get more repeatable navigation results. In this lab, you will use localization to navigate on a map generated with SLAM. The <b>learning outcomes</b> are:
* Generate a map
* Launching localization
* Point-to-Point Navigation Using Nav2 Goals
* Write a social_navigation_node 

<b>Submit a written report with the Lab 4 Checkpoints deliverables using the ‘Assignment Guideline’ template on Canvas under ‘Course Logistics’.</b>

# Assignment Deadline
* Due: October 24, 2025 at 11:00PM 
* Assignment Type: Group ONLY (Up to 5 members)
* Submit code via GitHub
* Total Points: 23 points

# Relevant ROS 2 tutorials
* [Turtlebot4 Features](https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html)
* [Driving your TurtleBot 4](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/driving.html)
* [Generate a map](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html)
* [Navigation](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html)

# Assignment Outline
1. Connect to RedRover WiFi
2. Access the Turtlebot4 robot from the Virtual Machine 
3. Install Turtlebot Desktop on the Robot
4. Generate a map
5. Launching Localization
6. Point-to-Point Navigation Using Nav2 Goals
7. Create a ROS Workspace
8. Create a social_navigation_node.py ROS Package
9. Write a social_navigation_node.py ROS Node
10. Customize package.xml
11. Customize the setup.py file
12. Test the Social Navigation Node
13. Create a Team GitHub Repository

################################################

# Reading Prerequisites
* Chapter 8: ‘How people perceive robots’
* Chapter 6: ‘Nonverbal Interaction’
* Chapter 7: ‘Verbal Interaction’

# 1. Connect to RedRover WiFi

# 2. Access the Turtlebot4 robot from the Virtual Machine 

<b>2.1</b> Open the Microsoft Desktop application and log in using your Cornell email address and password.

<b>2.2</b> The robot’s onboard computer is a Raspberry Pi with ROS 2 Humble distribution preinstalled on it. You can access the Raspberry Pi from the Virtual Machine (VM) using the Secure Shell (SSH) protocol. SSH into a Turtlebot4 using the IP address of the robot (IP addresses shown below). The default command for accessing the robot using SSH is ‘ssh -X ubuntu@<IP_ADDRESS>’. For example, SSH into Turtlebot 1 with the following command (see Table 1 for robot IP addressed):

For example, SSH into Turtlebot 1 with the following command:
```
ssh ubuntu@10.52.16.30
```
All robots have the same Raspberry Pi password: turtlebot4

| Turtlebot # | IP address | Turtlebot # | IP address | Turtlebot # | IP address |
|---|---|---|---|---|---|
| 1 | 10.52.16.30 | 4 | 10.52.16.25 | 7 | 10.52.16.20 |
| 2 | 10.52.16.29 | 5 | 10.52.16.24 | 8 | 10.52.16.27 |
| 3 | 10.52.16.26 | 6 | 10.52.16.23 | 9 | 10.52.16.28 |

# 3. Install Turtlebot Desktop on the Robot

The turtlebot4_desktop metapackage contains packages used for visualizing and interfacing with the TurtleBot 4 from a PC.

To install the metapackage through apt:
```
sudo apt update 
sudo apt install ros-humble-turtlebot4-desktop
```
Restart the robot.
```
sudo reboot
```

# 4. Generate a map

<b>4.1</b> Set up an experimental testbed for the Turtlebot in Room Tata 422. Construct a fixed area to build a map using cardboard and tape in the room. Make sure the area isn’t too big so that other students have enough space to build a testbed and not too small such that the robot doesn’t have enough space to move around. 

Remember to always source your ROS 2 environment before trying to launch a node.
```
source /opt/ros/humble/setup.bash
```
Map an area by driving the TurtleBot 4 around and using SLAM. Start by making sure that the area you will be mapping is clear of unwanted obstacles. Ideally, you don't want people or animals moving around the area while creating the map.

<b>4.2</b> First, install turtlebot4_navigation:
```
sudo apt install ros-humble-turtlebot4-navigation
```
<b>4.3</b> Then launch the SLAM node. It is recommended to run synchronous SLAM on a remote PC to get a higher resolution map.
```
ros2 launch turtlebot4_navigation slam.launch.py
```
<b>4.4</b> RViz is a 3D visualizer for the Robot Operating System (ROS) framework. Note that RVIZ will not launch unless you use the -X parameter to SSH into the robot. Learn more about RVIZ here.

Open a new terminal, source it, and launch RVIZ2 to visualize the lidar data (be patient, it takes some time).
```
ros2 launch turtlebot4_viz view_robot.launch.py
```

<b>4.5</b> Open a new terminal, SSH into the robot, and use keyboard teleoperation to move the robot.
```
sudo apt update 
sudo apt install ros-humble-teleop-twist-keyboard
```

<b>4.6</b> Once installed, open a new terminal and run the node by calling:
```
source /opt/ros/humble/setup.bash 
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Teleoperate the robot to generate a map. Move the robot around, rotate it to collect observations of obstacles, and observe the map as the robot moves to fully cover the room. Make sure there are no people in the environment; otherwise, the map will contain errors while navigating the environment.

<b>4.7</b> Once you are happy with your map, you can save it with the following command:
```
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'map_name'"
```

<b>Checkpoint 1 (5 points):</b> Build a map, save the map file as .yaml and .pgm files in the Tata 422 room. Submit the map files in your code repository on GitHub.

# 5. Launching Localization

<b> 5.1</b>  Open a terminal and launch [localization](https://github.com/turtlebot/turtlebot4/blob/humble/turtlebot4_navigation/launch/localization.launch.py):
```
ros2 launch turtlebot4_navigation localization.launch.py map:=office.yaml
```

Replace office.yaml with the path to your own map.

<b> 5.2</b> Open a new terminal and launch [nav2](https://github.com/turtlebot/turtlebot4/blob/humble/turtlebot4_navigation/launch/nav2.launch.py):
```
ros2 launch turtlebot4_navigation nav2.launch.py
```

# 6. Point-to-Point Navigation Using Nav2 Goals

<b>6.1</b> In a new terminal launch Rviz so that you can view the map and interact with navigation:
```
ros2 launch turtlebot4_viz view_robot.launch.py
```

At the top of the Rviz window is the toolbar. You will notice that there are three navigation tools available to you.

<b>6.2 2D Pose Estimate:</b> The 2D Pose Estimate tool is used in localization to set the approximate initial pose of the robot on the map. This is required for the Nav2 stack to know where to start localizing from. Click on the tool, and then click and drag the arrow on the map to approximate the position and orientation of the robot.

<b>6.3 Publish Point:</b> The Publish Point tool allows you to click on a point on the map, and have the coordinates of that point published to the /clicked_point topic. Open a new terminal and call:
```
ros2 topic echo /clicked_point
```

Then, select the Publish Point tool and click on a point on the map. You should see the coordinates published in your terminal.

<b>6.4 Nav2 Goal:</b> The Nav2 Goal tool allows you to set a goal pose for the robot. The Nav2 stack will then plan a path to the goal pose and attempt to drive the robot there. Make sure to set the initial pose of the robot before you set a goal pose.

<b>Checkpoint 2 (3 points):</b> Take a screenshot of Rviz after the robot moves from a starting and goal location, showing obstacles and boundaries in the environment.

<b>Deliverable:</b> Submit a screenshot of Rviz in a PDF (see ‘Assignment Guidelines’ on Canvas).

# 7. Create a ROS Workspace

A workspace is a directory containing ROS 2 packages. Before using ROS 2, it’s necessary to source your ROS 2 installation workspace in the terminal you plan to work in. 

<b>7.1</b> Create a new directory called, ‘ros2_ws’ and a ‘src’ folder in the ros2_ws folder. Then, confirm that the directory was created using the command:
```
mkdir ros2_ws
mkdir ros2_ws/src
cd ros2_ws/src
```
This prints the current working directory in the terminal window.

# 8. Create a social_navigation_node.py ROS Package

A package is an organizational unit for your ROS 2 code. If you want to be able to install your code or share it with others, then you’ll need it organized in a package. With packages, you can release your ROS2 work and allow others to build and use it easily. 

Package creation in ROS 2 uses ament as its build system and colcon as its build tool. You can create a package using either CMake or Python, which are officially supported, though other build types do exist. We will focus on creating a Python package. 

<b>8.1</b> To create a package, source ROS2 installation:
```
source /opt/ros/humble/setup.bash
```
<b>8.2</b> The general command for creating a ‘lab4’ package and ‘social_navigation_node’ node is: 
```
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name <my_node> <my_package>
```
<b>8.3</b> A package must contain the contents listed below. 
* package.xml: file containing meta information about the package 
* resource/<package_name>: marker file for the package 
* setup.cfg: is required when a package has executables, so ros2 run can find them 
* setup.py: containing instructions for how to install the package 
* <package_name>: a directory with the same name as your package, used by ROS 2 tools to find your package, contains __init__.py

<b>8.4</b> Creating packages in a workspace is especially valuable because you can build many packages at once by running colcon build in the workspace root. Otherwise, you would have to build each package individually. Return to the root of your workspace and build the package:
```
cd ~/ros2_ws
colcon build
```

# 9. Write a social_navigation_node.py ROS Node

<b>9.1</b> Navigate into ros2_ws/src/social_navigation_node_pkg/social_navigation_node_pkg. Recall that this directory is a Python package with the same name as the ROS 2 package it’s nested in. 
```
cd ~/ros2_ws/src/lab4/lab4/
ls
```
<b>9.2</b> Copy the code from social_navigation_node.py file provided to you in the social_navigation_node.py file in your repository. Your goal is to become familiar with the code and set waypoints for the robot to navigate in an environment while avoiding obstacles. Review the code comments and links to documentation to familiarize yourself on the node execution. The social_navigation_node.py file contains one ROS publisher to the '/initialpose' topic with the ‘PoseWithCovarianceStamped’ ROS message. 

PoseWithCovarianceStamped is a ROS 2 message type that represents a robot's estimated position and orientation in 3D space, along with an uncertainty estimate, all tagged with a reference coordinate frame and timestamp. It is composed of a std_msgs/Header and a geometry_msgs/PoseWithCovariance message, which includes both the position and orientation (pose) and a 6x6 covariance matrix to describe the uncertainty. This message is commonly used for providing initial poses or estimated states, such as those from a localization system.

To generate a map and set waypoints for the robot, follow this online [tutorial1, tutorial2].

<b>Checkpoint 3 (3 points):</b> Set three navigation waypoints to the robot, from a starting location, to a midpoint location, to a final destination.

# 10. Customize package.xml

You may have noticed in the return message after creating your package that the fields description and license contain TODO notes. That’s because the package description and license declaration are not automatically set, but are required if you ever want to release your package. Complete the following fields: maintainer email, maintainer name, and test_depend, which are the dependencies of the package. The screenshot below shows some dependencies that you can potentially include in your package.xml depending on your implementation of the assignment.

<b>Checkpoint 4 (3 points):</b> You may have noticed in the return message after creating your package that the fields description and license contain TODO notes. That’s because the package description and license declaration are not automatically set, but are required if you ever want to release your package. The maintainer field may also need to be filled in.

<b>Deliverables:</b> Submit the ROS package with the package.xml file with the following fields filled in:  package name, description, and dependencies. Push the package.xml file to GitHub.

# 11. Customize the setup.py file

The setup.py file contains the same description, maintainer, and license fields as package.xml, so you need to set those as well. They need to match exactly in both files. The version and name (package_name) also need to match exactly, and should be automatically populated in both files. Open setup.py with your preferred text editor.
```
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>ament_python</buildtool_depend>
<exec_depend>rclpy</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>nav2_msgs</exec_depend>
<exec_depend>builtin_interfaces</exec_depend>
```

<b>11.1</b> Edit the setup name, maintainer,, and maintainer_email consistent with the last step. The entry_points{console_scripts} fields will be automatically populated from creating the package with a package and node name. The console_scripts define ROS 2 nodes in the package
name - setup name is the package name ‘lab4’
social_navigation_node - executable node name
social_navigation_node - node .py. filename

Under setup[‘data_files’], specifiy the launch file description
```
('share/' + package_name + '/launch', ['launch/bringup_and_waypoints.launch.py'])
```
Don’t forget to save the file and close it using ‘Esc’, ‘wq!’, and press ‘Enter’.

<b>11.2</b> Make a launch directory in the package (in the same directory as ‘package.xml’ file). Copy the http://bringup_and_waypoints.launch.py launch file into the launch folder.

<b>Checkpoint 5 (5 points):</b> Open setup.py with your preferred text editor. The setup.py file contains the same description, maintainer, maintainer_email and license fields as package.xml, so you need to set those as well. They need to match exactly in both files. Additionally, make sure the entry_points[‘console_script’] is set to the correct package and node name. The version and name (package_name) also need to match exactly, and should be automatically populated in both files. Open setup.py with your preferred text editor.

<b>Deliverables:</b> Submit the package setup.py file with the following fields filled in: description, maintainer, maintainer_email, entry points, and launch description. Push the setup.py file to GitHub.

<b>11.3</b> Open a new terminal. Change directory to the /ros_ws/ workspace, install dependencies, install and build packages, source the installation, and run the node.
```
cd ~/ros2_ws
rosdep install --from-path src -yi 
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install setup.bash
ros2 run lab4 social_navigation_node
```
You will see that the robot executes the social_navigation_node.py node file.

# 12. Configure ROS Launch File

<b>12.1</b> Change directories to /ros2_ws/src/, create a folder called ‘launch,’ and change directory to the launch directory.
```
cd ~/ros2_ws/src/lab4
mkdir launch
cd launch
```
<b>12.2</b> Move the bringup_and_waypoints.py file from Canvas to the ~/ros2_ws/src/lab4/launch directory. From the same terminal, open the bringup_and_waypoints.py file.
```
gedit bringup_and_waypoints.py
```
The final launch file, bringup_and_waypoints.py, is shown below in steps 12.3-12.6. 

<b>12.3</b> The file starts with definitions of launch libraries required to run lab4 executables, set ROS parameters, load pre-built maps, and launch multiple launch files.
```
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
```

<b>12.4</b> The launch description defines a path to the map (‘map_path’), start position, mid position, final position, and initial robot pose (‘publish_inital_pose’). Next, get_package_share_directory is a function used to programmatically obtain the absolute path to the "share" directory of a specified ROS 2 package. Lastly, IncludeLaunchDescription takes a path to another launch file (typically a Python launch file, but can also be XML or YAML) as an argument to insert all the actions defined within the included launch file into the current launch description. This means any nodes, services, parameters, or other actions declared in the included file will be launched as part of the main launch process.

```
def generate_launch_description():
   map_path = LaunchConfiguration('map')
   start = LaunchConfiguration('start')
   mid = LaunchConfiguration('mid')
   final = LaunchConfiguration('final')
   publish_initial_pose = LaunchConfiguration('publish_initial_pose')

   tb4_nav_share = get_package_share_directory('turtlebot4_navigation')
   nav_bringup = IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
           os.path.join(tb4_nav_share, 'launch', 'nav_bringup.launch.py')),
       launch_arguments={
           'slam': 'off',
           'localization': 'true',
           'map': map_path
       }.items()
   )
```

<b>12.5</b> This section of the a action in the launch description for the social_navigation_node node in the lab4 package with the initial start, mid, and final ROS parameters:
```
waypoints_node = Node(
    package='lab4',
    executable='social_navigation_node',
    name='social_navigation_node',
    output='screen',
    parameters=[{
        'frame_id': 'map',
        'publish_initial_pose': publish_initial_pose,
        'start': [0.0, 0.0, 0.0],  # will be overridden by CLI if provided
        'mid':   [1.0, 0.0, 0.0],
        'final': [2.0, 0.0, 0.0],
    }]
)
```

<b>12.6</b> This section of the launch file defines the location of map .yaml file, initial pose of the robot, and default start, midway point, and final point of navigation. Change the map location from ‘/home/ubuntu/Tata429_map.yaml’ to the location and name of the map on your robot filesystem. 
```
return LaunchDescription([
    DeclareLaunchArgument('map',  default_value='/home/ubuntu/Tata429_map.yaml'),
    DeclareLaunchArgument('publish_initial_pose', default_value='true'),
    DeclareLaunchArgument('start', default_value='[0.0, 0.0, 0.0]'),
    DeclareLaunchArgument('mid',   default_value='[1.0, 0.0, 0.0]'),
    DeclareLaunchArgument('final', default_value='[2.0, 0.0, 0.0]'),
    nav_bringup,
    waypoints_node,
])
```

Enter the name of the map generated by your team in Tata 422.

# 13. Test the Social Navigation Node

<b>13.1</b> Repeat step 4.1. Build a map with 2-3 obstacles in the environment so that the robot must avoid obstacles as it navigates the environment. 

Test the robot using the launch file. Open a terminal and run the following command:
```
ros2 run lab4 social_navigation_node --ros-args \ 
  -p publish_initial_pose:=true \ 
  -p start:="[5.708824, -2.778678, -171.7763]" \ 
  -p mid:="[-1.021936, -3.751442, -122.3231]" \ 
  -p final:="[-3.081923, -7.007109, -122.3231]"
```

<b>Checkpoint 6 (3 points):</b> Record a video demonstrating the robot navigating through three waypoints from a starting point, to a second waypoint, and finally, to a destination. The robot must successfully demonstrate obstacle avoidance in the video to avoid colliding into 2-3 obstacles.
Deliverable: Submit a link to the video on Google Drive in a PDF (see ‘Assignment Guidelines’ on Canvas).

<b>Checkpoint 7 (1 point):</b> Take a photo of the environment setup to test the robot. 
Deliverable: Submit a photo to the environment in a PDF (see ‘Assignment Guidelines’ on Canvas).

# 14. Create a Team GitHub Repository

* [Generate a new SSH key for the VM and add it to the ssh-agent](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent). The SSH key enables you to authenticate your GitHub account locally on the VM in order to pull, push, and merge code to your GitHub repository. Remember to select the Linux OS in GitHub Documents guide (see the SSH instructions in the Troubleshooting Guide on Canvas under ‘Course Logistics’).
* Create a GitHub repository. We recommend creating a team to share code with your team.
* At “Manage Access”, click “Create team”. Add a team name and add members. Then add this team to your repository at the “Manage Access” page.
* Clone the team repository on the VM by opening a terminal window and using the following command: ``git clone <SSH URL>``
* Commit changes to the repository using this command: ``git commit -m <add a message to describe code changes>``
* Push code changes to the repository by running this command in the terminal ``git push <add command>``

<b>14.1</b> Join the Lab 4 assignment using the GitHub Classroom.

<b>14.2</b> Clone the Lab 2 template code to the ~/ros2_ws/src directory
```
git clone <lab4_team_github_url>
cd ~/<lab4_team_folder_name>/lab4
```

# Further Issues and questions ❓

If you have issues or questions, don't hesitate to contact the teaching team:

* Angelique Taylor, Instructor, amt298@cornell.edu
* Yuanchen (Sophie) Bai, Teaching Assistant, yb299@cornell.edu
* Giuliano Pioldi, Grader, gp433@cornell.edu
* Aimalohi Ohirheme Alakhume, Grader, aoa36@cornell.edu
