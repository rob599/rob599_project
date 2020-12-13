# rob599_project

## Install

### Install dependencies
Need to install the rviz_visual_tools for the cone marker. Futher information [here](https://github.com/PickNikRobotics/rviz_visual_tools/blob/melodic-devel).
```
sudo apt-get update
sudo apt-get install ros-melodic-rviz-visual-tools
```

The octomap dependencies need to be installed.
```
sudo apt-get install ros-melodic-octomap
sudo apt-get install ros-melodic-octomap-server
sudo apt-get install ros-melodic-octomap-mapping
```

You also need to pip3 install:
* rospkg
* scipy
* sympy
* planar
* trimesh

### Build
Add the package to your src file in your workspace.

```
git clone https://github.com/rob599/rob599_project.git
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
Navigate to ~/catkin_ws/src/rob599_project
```
mv block ~/.gazebo/models
```
This adds the necessary gazebo models so that they can be imported


### Get started
Run launch files to get things started.

```
roslaunch fetch_project_moveit_config fetch_world.launch or roslaunch fetch_project_moveit_config fetch_world_collision.launch
roslaunch fetch_project_moveit_config disinfectant_project.launch
roslaunch fetch_project_moveit_config run_nodes.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
Click on the publish point feature and then click on one of the cubes in the octomap. This should populate an interactive marker at the location of the cube.
