# Autonomous Mapping and Navigation
In one package I have kept both the autonomous mapping and navigation part. Explore lite isn't supported on ROS2 Jazzy and neither in python there is frontier exploration module.Slam toolbox, frontier search and obstacle avoider are used for autonomous mapping. Obstacle avoider isnt working properly, it needs tuning.For navigation, RRT algorithm and pure pursuit used. 
## Autonomous mapping of unkown area
## Installation
```bash
cd ~/rosbot_ws/src/rosbot_ros
git clone https://github.com/Maisha-afk/autonomous_mapping.git
cd ~/rosbot_ws
colcon build
```
## Commands for autonomous mapping
Source each terminal with rosbot_ws. Start the simulation in terminal 1:
```bash
cd ~/rosbot_ws
source install/setup.bash
ros2 launch rosbot_gazebo simulation.launch.py robot_model:=rosbot
```
Launch mapping node, Terminal 2:
```bash
cd ~/rosbot_ws
source install/setup.bash
ros2 launch rosbot_mapping slam.launch.py
```
Start the obstacle avoider node, Terminal 3:
```bash
cd ~/rosbot_ws
source install/setup.bash
ros2 run rosbot_mapping obstacle_avoider
```
Start the exploration launch file, Terminal 4:
```bash
cd ~/rosbot_ws
source install/setup.bash
ros2 launch rosbot_mapping explore.launch.py
```
This is the part where the robot will move autonomously and map the unkown areas and return to same location
## Saving the map file
Terminal 5:
```bash
cd ~/rosbot_ws
source install/setup.bash
ros2 service call /map_saver_server/save_map nav2_msgs/srv/SaveMap "{
  map_url: '$HOME/rosbot_ws/src/rosbot_ros/rosbot_mapping/maps/my_map',
  image_format: 'pgm',
  map_mode: 'trinary',
  free_thresh: 0.196,
  occupied_thresh: 0.65
}"
```
# Autonomous Navigation
For navigation, mapping procedure will end.
Start the simulation, terminal 1:
```bash
cd ~/rosbot_ws
source install/setup.bash
ros2 launch rosbot_gazebo simulation.launch.py robot_model:=rosbot
```
Launch the navigation, terminal 2:
```bash
cd ~/rosbot_ws
source install/setup.bash
ros2 launch rosbot_mapping navigation.launch.py
```

In the rviz, select path, rrt_path, costmap. Using 2d goal pose navigate to the desired position.






























