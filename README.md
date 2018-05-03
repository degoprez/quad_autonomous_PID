**WiFi Detecting Drone for First Responders**

The goal of this project is to make a drone autonomously localizes survivors using wireless frequencies, and mapping. This is important to both civilians and rescue operators, as it should boost the efficiency and efficacy of rescue missions while reducing first responder injuries and fatalities.

In order to do that. This code has been written to make the drone autonomously moving and avoid obstacles. 

**Instructions**


**Demonstration**

This video demonstrates a quadcopter using information from a Kinect RGB-D simulated sensor to produce an Octomap occupancy grid. The code takes the RGB video stream and calculates a centroid to avoid obstacles. The source code used for the PID control algorithm and obstacle avoidance is provided. The drone infrastructure base code was provided by the Technische Universität Darmstadt robotics team, only change added was the Octomap functionality.

As the calculated centroid changes, the two PIDs (yaw and altitude) compensate for the change accordingly and provide the velocity commands to the quadcopter. The forward velocity is always a constant.

The demonstration in the following Link: https://www.youtube.com/embed/EU2-wwvtgI0


**Future Work**

We will work on testing a real drone with this code.
