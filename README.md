# quad_autonomous_PID
This project demonstrates a quadcopter using information from a Kinect RGB-D simulated sensor to produce an Octomap occupancy grid. The code takes the RGB video stream and calculates a centroid to avoid obstacles. The source code used for the PID control algorithm and obstacle avoidance is provided. The drone infrastructure base code was provided by the Technische Universität Darmstadt robotics team, only change added was the Octomap functionality. As the calculated centroid changes, the two PIDs (yaw and altitude) compensate for the change accordingly and provide the velocity commands to the quadcopter. The forward velocity is always a constant and can be changed by the user.

# Instructions
## Hector Quadcopter 
First, install the Technische Universität Darmstadt ROS Packages:
`https://github.com/tu-darmstadt-ros-pkg`
The packages that will be used are:
```
hector_gazebo
hector_localization
hector_models
hector_quadrotor
hector_slam
```
Once the packages are installed, compile them using `catkin_make` 
If you recieve dependency problems, correct them as listed by the compiler.

## Add Octomap Functionality
First, you must install the Octomap library:
```
sudo apt-get install ros-kinetic-octomap ros-kinetic-octomap-mapping
rosdep install octomap_mapping
rosmake octomap_mapping
```
Next, navigate to your `src` directory. Then navigate to the following `hector_quadrotor/hector_quadrotor_demo/`. Add the following to the bottom of your CMakeLists.txt
```
## The following adds octomap capabilities
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
```
Then, in the same directory, add the following to the package.xml 
```
<!-- The following adds octomap capabilities-->
<build_depend>octomap</build_depend>
<run_depend>octomap</run_depend>
```
recompile using catkin_make in your root directory.

## Configuring the Octomap Functionality
Next, we have to configure the Octomap to work on our system. To edit the configuration file:
```
rosed octomap_server octomap_tracking_server.launch
```
If you are unable to save changes because of read only issues, then alter the file directly using `sudo`:
```
sudo vim /opt/ros/kinetic/share/octomap_server/launch/octomap_tracking_server.launch
```
Once the file is open, add the following changes from:
```
<param name="frame_id" type="string" value="map" />
...
<!--remap from="cloud_in" to="/rgbdslam/batch_clouds" /-->
```
to:
```
<param name="frame_id" type="string" value="world" />
...
<!--remap from="cloud_in" to="/camera/depth/points" /-->
```
## Launch the program
Finally, to launch the program first run the hector quadrotor demo world:
```
roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch
```
Next, run the Octomap server:
```
roslaunch octomap_server octomap_tracking_server.launch 
```
Lastly run our PID control system:
```
./follow_pid_drone.py
rosservice call /enable_motors "enable: true"
```
