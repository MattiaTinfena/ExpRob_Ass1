# Experimental robotics laboratory Assignment 1

Made by:
- [Elisa Martinenghi mat: 6504193](https://github.com/mrtelisa)
- [Mattia Tinfena mat: 7852527](https://github.com/MattiaTinfena)
- [Roberto Bertelli mat: 7289118](https://github.com/obesk)

This is the submission for the first assignment of the Experimental robotics laboratory.

As you can see from the videos the robot is spawned inside of a circle of aruco markers. The robot first does a complete turn to collect all the trackers and visits all of them in order of ascending id value.

This simulation has been realized with ROS2 Jazzy, Gazebo for the simulation enviroment and the auruco_opencv library for the simulation environment


### 2 Wheels driving

![2 wheels](./gifs/2wheel.gif)

### 4 wheels driving
![4 wheels](gifs/4wheel.gif)


## Environment setup

Be sure to use ROS2 Jazzy to ensure compatibility

1. clone the package in a ros src folder
2. clone the aruco_opencv package in the same ros folder
    ```bash
    git clone https://github.com/fictionlab/ros_aruco_opencv.git
3. be sure that aruco is using the correct topics and uses the correct dictionary (ARUCO_ORIGINAL)
4. copy the aruco box generated on the `gazebo_models` folder
    ```bash
     cp aruco_box/ ~/gazebo_models/ -r
    ```
4. build the ros packages with colcon
    ```bash
    colcon build
    ```
## Run the launchfile for the simulation

Be sure to have sourced the local_setup:

```bash
source ./install/bin/local_setup
```
You can now launch the simulation with the provided launch file

```bash
ros2 launch erl1 erl1_simulation.launch.py
```

## Optional parts

#### Skid steering

To test the bot with 4 wheels in skid steering switch to the skid-steering branch

```bash
git switch skid-steering
```

rebuild with colcon and relaunch the file

### ROSBots (currently not tested)

Warning: we were not able to test on the lab for now, so this feature needs 
to be still fully implemente

To test the code with the rosbot switch to the Lab branch 

```bash
git switch skid-steering
```
rebuild with colcon and relaunch the file

