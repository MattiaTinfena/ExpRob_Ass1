# Experimental robotics laboratory Assignment 1

This is the submission for the first assignment of the Experimental robotics laboratory.

As you can see from the videos the robot is spawned inside of a circle of aruco markers. The robot first does a complete turn to collect all the trackers and visits all of them in order of ascending id value.

This simulation has been realized with ROS2 Jazzy, Gazebo for the simulation enviroment and the auruco_opencv library for the simulation environment

## Environment setup

Be sure to use ROS2 Jazzy to ensure compatibility

1. clone the package in a ros src folder
2. clone the aruco_opencv package in the same ros folder
    ```bash
    git clone https://github.com/fictionlab/ros_aruco_opencv.git
    ```
3. create the python environment in the ros folder:
    ```bash
    python3 -m venv venv
    ```
3. tell colcon to not build the venv folder
    ```bash
    touch PUT_FILE_HERE
    ```
4. source the venv
    ```bash
    source venv/bin/activate
    ```
5. install python packages
    ```bash
    pip install -r requirements.txt
    ```
6. build the ros packages with colcon
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

To test the code with the rosbot switch to the Lab branch 

```bash
git switch skid-steering
```
rebuild with colcon and relaunch the file

