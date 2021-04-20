# Robot Path Planing using ROS

## Introduction
The goal is to simulate path planing of a four-wheeled robot in a 3D constructed model of desert keeping in view the robot should not ascend the elevation of more than 30 degrees.  
This repository lists desribes the approaches used to solve this problem and challenges being faced. So, the husky a well-known robot by clearpath robotics is used to simulate this whole scenerio. First the 3d desert model is loaded into Gazebo simulation environment and then husky is spawned on it. These were many challenges of simulating husky on the provided 3D model which are listed below in detail.  
All of this is developed and tested with Ubuntu18, ROS Melodic and Gazebo9.

## Loading 3D Model in Gazebo
A 3D constructed model of the desert is provided to spawn the four-wheeled robot. Initially, the model was in .obj format which gazebo does not support and has to be converted into supported format. The model (.obj) is loaded into blended for the conversion, it was not in a correct pose and texture which are corrected. After the correction, it is saved as .dae (Collada) format which suppored by Gazebo. Figure -- shows the model in blender before correction (left) and after correction (right). Figure -- shows the loaded model in Gazebo (saved as desert.world plased in models/worlds directory).


## Installing and Testing Husky
To install and simulate husky, run the follwing commands one by one.
```
sudo apt updata
sudo apt upgrade
sudo apt-get install ros-melodic-husky-simulator
sudo apt-get install ros-melodic-husky-navigation ros-melodic-husky-gazebo ros-melodic-husky-viz
```
Set the environment variable
```
export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro
```
In three separate terminals, run the foloowing commands in the order listed below.
- Luanch the Gazebo simulation environment
```
roslaunch husky_gazebo husky_playpen.launch
```
- Launch Rviz for visulaization
```
roslaunch husky_viz view_robot.launch
```

- Launch the husky naviagtion node to navigation it
```
roslaunch husky_navigation move_base_mapless_demo.launch
```

Note: sometimes, husky does not run properly in ROS Melodic, if this happens then husky packages can be built from source by following the steps [here](https://answers.ros.org/question/256756/how-to-install-husky-simulator-in-kinetic/), but just clone the first reposity as it contains all the pacages of second and third repositry as well.  
A glips of how this works is shown below. To watch high resolution vodes follow the link [this](https://youtu.be/2wyo-RTvQVg) and [this](https://youtu.be/XJE7356JK8o).


## Challenges
Getting back to the original goal that is to navigate a four-wheeled robot on the provided desert environment. There are many challanges were faced while doing this. The first challenge was that sometimes the desert environment does not load properly. It might be because of a bug in Gazebo9. The second challange was faced when husky was spawned on the desert environment. Husky works fine with other worlds but it stops publishing the transforms when both are loaded at the same time. Figure -- shows the spawned husky on desert world. It can be seen from figure -- that husky is giving error and not publishig anythong which hindered it's navigation. 










