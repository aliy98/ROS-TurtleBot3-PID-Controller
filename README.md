# TurtleBot3 PID Controller
This ROS package is a simple practice for controlling the TurtleBot3 to move in an elliptic path

## How to Use the Package
The first step is to make a workspace directory 
> mkdir -p catkin_ws/src 
> 
> cd catkin_ws/src
>
> source /opt/ros/noetic/setup.bash
>
> catkin_init_workspace
>
then clone the repository into the src directory
> clone https://github.com/aliy98/ROS_TurtleBot3_PI_Controller.git
>
now make the ros package using caktkin_make command
> cd ..
>
> catkin_make
>
the package would be up and and running using the command below
> source devel/setup.bash
>

## Simulation
open another terminal to choose the turtlebot type
> echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc 
>
now the simulation process can be done in gazebo 
> roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
>
you can also use rviz to gain a better understanding of the path planning and robot footprint so open another terminal and run this command
> roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
>
in the first terminal run the source files for visualising the robot footprint 
> pip install simple_pid
>
> rosrun pi_controller footprint.py
>
open another terminal to run controller.py which is for the whole control and path planning process.
but before running the package make sure that you have simple pid module installed on your os
> source catkin_ws/devel/setup.bash
>
> rosrun pi_contoller controller.py
>
![Screenshot from 2021-09-28 21-13-20](https://user-images.githubusercontent.com/65722399/135280561-db4b4861-8d30-4e3b-b6ef-322679088f09.png)
the package also plots the path planning error during the simulation time
![Figure_1](https://user-images.githubusercontent.com/65722399/135280985-a6255364-3e9c-47aa-b043-029b2a601756.png)
![Figure_2](https://user-images.githubusercontent.com/65722399/135281076-6ee6ceb1-81e9-45e4-8683-74c565299ee7.png)

