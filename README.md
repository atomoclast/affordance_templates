##**Quick Start:**##

We are quickly approaching the first "release" of this software, but in the meantime, here is a quick guide to using it with the Robonaut2 simulator.

###**Dependencies:**###
```
#!bash
$ sudo apt-get install ros-indigo-simulators ros-indigo-moveit* ros-indigo-control* ros-indigo-ros-control* ros-indigo-gazebo-ros-control*
```
NOTE: if you receive a 'broken packages' error, try uninstalling libsdformat2

1. checkout the following repositories into your catkin workspace to run R2 upperbody and switch to the specified branches.

    * [r2 simulator](https://bitbucket.org/nasa_ros_pkg/nasa_r2_simulator/src/c32521004a4a8f135c4298500d6ded3ce20e0070/?at=indigo-devel) - *traclabs-devel*
    * [r2 common](https://bitbucket.org/nasa_ros_pkg/nasa_r2_common/src/41b52f1747bdb0b484fb1c3788716c950d8e5d0e/?at=traclabs-devel) - *traclabs-tracik* 
    * current [affordance template](https://bitbucket.org/traclabs/affordance_templates/src/88fcd803b2f4d26e86a9bf3e40d43a0db8744104/?at=cpp-devel) development - *cpp-devel*
    * current [robot interaction tools](https://bitbucket.org/traclabs/robot_interaction_tools/src/faaaa732baf71a8340dfd6a24288824a7ae05cb4/?at=cpp-devel) development - *testing*

1. catkin_make at your workspace level
1. source each terminal - you'll need three

```
#!bash
    1. roslaunch r2_gazebo r2_gazebo.launch
    2. roslaunch affordance_template_server r2_upperbody_new_exec.launch
    3. rviz

```
1. add necessary dispalys in RViz
    * RobotModel (robot model) - Robot Description: robot_description
    * RobotModel (path visualization) - TF Prefix: ats
    * interactive marker - Update Topic: /affordance_template_interactive_marker_server/update
2. add affordance template panel by going to Panels->Add New Panel->RVizAffordanceTemplatePanel


##**Software Overview:**##
![AT Architecture.png](https://bitbucket.org/repo/r5rydq/images/1896767692-AT%20Architecture.png)