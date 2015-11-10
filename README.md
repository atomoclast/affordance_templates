Please see the the Affordance Template Project wikie here: [https://github.com/swhart115/affordance_templates/wiki](https://github.com/swhart115/affordance_templates/wiki)

JSON parsing using [yaml-cpp](https://github.com/jbeder/yaml-cpp)

##**Quick Start:**##

1. checkout the following repositories into your catkin workspace to run R2 upperbody and switch to the specified branches.

    * [r2 simulator](https://bitbucket.org/nasa_ros_pkg/nasa_r2_simulator/src/c32521004a4a8f135c4298500d6ded3ce20e0070/?at=indigo-devel) - *indigo-devel*
    * [r2 common](https://bitbucket.org/nasa_ros_pkg/nasa_r2_common/src/41b52f1747bdb0b484fb1c3788716c950d8e5d0e/?at=traclabs-devel) - *traclabs-devel* 
    * current [affordance template](https://bitbucket.org/traclabs/affordance_templates/src/88fcd803b2f4d26e86a9bf3e40d43a0db8744104/?at=cpp-devel) development - *cpp-devel*
    * current [robot interaction tools](https://bitbucket.org/traclabs/robot_interaction_tools/src/faaaa732baf71a8340dfd6a24288824a7ae05cb4/?at=cpp-devel) development - *cpp-devel*

1. catkin_make at your workspace level
1. source each terminal - you'll need five

```
#!bash
    1. roslaunch r2_gazebo r2c_upperbody.launch gui:=false robodyn:=true ros_control:=true
    2. roslaunch r2_action_server r2_upperbody.launch 
    3. roslaunch r2_upperbody_moveit_config move_group.launch
    4. rosrun affordance_template_server affordance_template_server_node
    5. rviz

```
1. add necessary topics in RViz
    * robot model - Robot Description: robot_description
    * interactive marker - Update Topic: /affordance_template_interactive_marker_server/update
2. add affordance template panel by going to Panels->Add New Panel->RVizAffordanceTemplatePanel