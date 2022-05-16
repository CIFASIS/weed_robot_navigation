# weed_robot_navigation

Navigation of the weeding robot.

The script [bin/weed_robot.sh](bin/weed_robot.sh) is used to run all the necessary components for the simulation of the weeding robot and the execution of the navigation. It receives the following parameters:
* -w, --world WORLD_NAME: name of the SDF file describing the Gazebo simulation scenario located at *weed_robot_gazebo/worlds*.
* -m, --map MAP_NAME: name of the PGM image with the map of the environment located at *weed_robot_navigation/maps*.
* -p, --pose POSE: position and orientation where the robot will be placed at the beginning, e.g. *"-x -4.16 -y 1.0 -Y 1.5708"*.
* -g, --gui: should run the Gazebo visual interface.
* -v, --visualization: should run the RViz visualization.
* -s, --sensor: should run point cloud generation for the local mapping, otherwise the static map is used.
* -n, --navigation: should run the coverage navigation.
* -t, --tracks TRACK_SEQUENCE: order in which the swaths should be visited, e.g. *"[2,4,1,3]"*.
* -l, --last POSITION: **y** component of the last waypoint generated.
* -o, --omega: should generate Omega-turns, otherwise T-turns are used.
* -b, --bottom: should robot starts from the bottom of the field, otherwise it starts from the top.

Examples:

```
$ ./bin/weed_robot.sh -w obstacles -m obstacles -p "-x 0.0 -y 0.0 -Y 0.0" -g -v
```

```
$ ./bin/weed_robot.sh -w field -m field -p "-x -14.56 -y 1.0 -Y 1.5708" -g -v -s -n -t "[3, 4]" -l "-10.0"
```
```
$ ./bin/weed_robot.sh -w field_plants -m field -p "-x -14.56 -y 1.0 -Y 1.5708" -g -v -s -n -t "[3, 4]" -l "-10.0"
```

![Screenshot](img/navigation.png)

The folder [test](test) contains python scripts used for the experiments related to turns and paths.

The global path planner [global_planner](http://wiki.ros.org/global_planner) and the local trajectory planner [teb_local_planner](http://wiki.ros.org/teb_local_planner) are used.
The configuration files are located in the folder [config](config).

The folder [maps](maps) contains PGM images representing the navigation scenarios with their obstacles for use as static global map.

The information from the topic *point_cloud_full/sensor* is used to generate the local map.
The node implemented in [src/point_cloud_obstacle.cpp](src/point_cloud_obstacle.cpp) publishes a point cloud in the surroundings of the robot (emulating the data that would be obtained from its stereo camera).
In the generated point cloud, the points corresponding to the two crop rows under the robot are filtered out.

The node implemented in [src/waypoint.cpp](src/waypoint.cpp) generates the *waypoints* for the coverage path through the entire field.
The node in [src/goal.cpp](src/goal.cpp) is in charge of taking these *waypoints* and publishing them one at a time as successive goals for the robot navigation.
