# Experimental Laboratory Classes

- This repo contains the package `erl1`

- When it comes to the simulation with the MOGI robot, it is necessary to have also the package you find at the following link: `https://github.com/CarmineD8/erl1_sensors`

- The folder `meshes` il cloned from `https://github.com/CarmineD8/meshes`

- When gps is integrated, to see the robot moving you can run the command
`python3 gps_follower.py` in the folder `erl1_sensors`, obtained by cloning the repo previously linked

- When `lidar` is added, you need to subscribe to add to the bridge of the erl1_sensors pkg this two topics:
`"/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",`
`"/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",`