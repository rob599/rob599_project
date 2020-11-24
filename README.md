# rob599_project

## Install

### Install dependencies
Need to install the rviz_visual_tools for the cone marker. Futher information [here](https://github.com/PickNikRobotics/rviz_visual_tools/blob/melodic-devel). 
```
sudo apt-get update
sudo apt-get install ros-melodic-rviz-visual-tools
```

You also need to pip install:
* rospkg
* scipy
* sympy
* planar

### Build
Add the package to your src file in your workspace.

```
git clone https://github.com/rob599/rob599_project.git
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

### Get started
Run launch files to get things started.

```
roslaunch fetch_project_moveit_config fetch_world.launch
roslaunch fetch_project_moveit_config rviz_setup.launch
roslaunch fetch_project_moveit_config run_nodes.launch
```
Click on the publish point feature and then click on one of the cubes in the octomap. This should populate an interactive marker at the location of the cube. 

