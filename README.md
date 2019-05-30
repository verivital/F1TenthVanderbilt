# The FLASH Van 
The team name stands for **F**u**l**ly **A**utonomous but **S**tuff **H**appens and we are from **Van**derbilt.
Team members are:
* [Patrick Musau](https://www.linkedin.com/in/musaup/) (Project Leader)
* [Diego Manzanas Lopez](https://www.linkedin.com/in/diego-manzanas-3b4841106/)
* [Nathaniel (Nate) Hamilton](https://www.linkedin.com/in/nathaniel-hamilton-b01942112/)
* [Feiyang Cai](https://www.linkedin.com/in/feiyang-cai-8b845a124/)
* [Tim Darrah](https://www.linkedin.com/in/timothydarrah/)
* [Shreyas Ramakrishna](https://www.linkedin.com/in/shreyasramakrishna/)
* [Ayana Wild](https://www.linkedin.com/in/ayana-wild/)

![Team Picture](./images/team_picture.jpeg "Team Picture")

# Vehicle Configuration
Our car is modeled after the V2 using a Hokuyo UST-10LX lidar and ZED Camera for sensing and localization, with a NVIDIA TX2 for processing. For information about our build configuration, kindly refer to the following [pdf](https://github.com/verivital/F1TenthVanderbilt/blob/master/BuildV2.pdf).

# Repository Organization
**racecar-ws**: Code deployed on the physical car. 

**f110-fall2018-skeletons**: F1Tenth Simulation Code

**hector_slam**: ROS hector_slam package. Used to learn a map of the environment and simultaneosuly esimate the platforms 2D pose at laser scanner frame rate.

**particle_filter**: A fast particle filter localization algorithm developped by Corey Walsh et al.

**range_libc**: Raycasting library utilized the particle filter package

**a_stars_pure_pursuit**: ROS package for a pure pursuit motion planner developped by sidsingh@seas.upenn.edu



# Algorithms
A good summary of the algorithms needed to implement an autonomous racecar using localization and planning can be found on pages 46-73 of the [BuildV2 manual](https://github.com/verivital/F1TenthVanderbilt/blob/master/BuildV2.pdf).

We implemented two racing strategies in simulation that I will briefly summarize below.  
1. The first strategy made use of the [teb local planner](http://wiki.ros.org/teb_local_planner) ROS pacakge for path planning, Adaptive Montecarlo Localization [(AMCL)](http://wiki.ros.org/amcl) for probabilistic localization, and [gmapping](http://wiki.ros.org/gmapping) for laser-based SLAM (Simultaneous Localization and Mapping). The challenge here was generating goal points that resulted in smooth driving. Additionally one thing we will try in the future is to use the MIT particle filter for localization instead of amcl.
2. The second strategy used the a_stars_pure_pursuit pacakge for path planning, MIT particle filter for localiztion, and [gmapping](http://wiki.ros.org/gmapping) for laser-based SLAM (Simultaneous Localization and Mapping). This was by far our most successful strategy in terms of speed and smoothness of driving. However unfortunately we were not able to translate these results onto the physical car in time for the competition. To run the simulation follow the instructions below.

# F1Tenth Simulation
The simulation packages in this repository contain code to run a car autonomously on a race track/circuit. The simulator was  originally developed by the [mLAB: Real-Time and Embedded Systems Lab](https://github.com/mlab-upenn/f110-fall2018-skeletons) at the University of Pennsylvania and we have customized it for our own experiments. The simulator was built using [ROS](http://wiki.ros.org/) and [Gazebo](http://gazebosim.org/tutorials). There are several race tracks available for testing and they are contained in racecar_gazebo/worlds directory. However, the majority of our testing utilized the following .world files:
  1. track_barca.world
  2. track_porto.world
  3. track_levine.world

##### You will need to install the following ROS packages to get the simulator to work

Assuming you have ROS and Gazebo installed run: 

```bash
$ sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control ros-kinetic-ackermann-msgs ros-kinetic-joy
```
 
To launch the simulation run the following roslaunch command:
```roslaunch race f1tenth.launch```

This will start Gazebo, [RViz](http://wiki.ros.org/rviz) and ROS. At a high level, Gazebo is a robust physics engine, with convenient programmatic and graphical interfaces for robot simulation. RViz is a 3D visualizer for sensor data, robot models, environment maps, which is useful for developing and debugging code. Once you run the above command, a window will pop up showing the racecar in a virtual track equipped with a camera, lidar, an imu, and odometry estimation obtained from the [VESC](https://www.electric-skateboard.builders/t/new-vesc-user-read-this-complete-walktrough-of-the-vesc/2980). 


![Simulation Image](./images/simulator.png "Simulation Image")

#### Import ROS Nodes
- [/keyboard_node](https://github.com/verivital/F1TenthVanderbilt/blob/master/f110-fall2018-skeletons/simulator/f1_10_sim/race/scripts/keyboard.py): allows you to drive the car using the keys w-a-s-d
- [/sim_connector_node](https://github.com/verivital/F1TenthVanderbilt/blob/master/f110-fall2018-skeletons/simulator/f1_10_sim/race/scripts/sim_connector.py): translates keyboard commands into control inputs for the car. These commands are then published on the topic ```/vesc/ackermann_cmd_mux/input/teleop```
- [/message_to_tf](https://github.com/verivital/F1TenthVanderbilt/blob/master/f110-fall2018-skeletons/simulator/f1_10_sim/race/scripts/message_to_tf.py): creates an odometric frame for the car so that the transformation tree in ROS is completer
- [/map_server](http://wiki.ros.org/map_server#map_saver): loads and saves occupancy grids obtained from SLAM algorithms

#### Important ROS Topics
- ROS messages from the camera are broadcasted on the ```/camera/zed/rgb/*``` topics, 
- Lidar data is broadcasted on the ```/scan``` topic.
- Odometry data is broadcasted on the ```/vesc/odom``` topic
- IMU (Inertial Measurement Unit) data is broadcasted on the ```/imu``` topic
-  The topic ```/vesc/ackermann_cmd_mux/input/teleop```  is of type ackermann_msgs/AckermannDriveStamped. Publishing an AckermannDrive Stamped message on this topic will casue the car to move

#### Changing the track
To change the track utilized in the simulation change value parameter at the top of [f1_tenth.launch](https://github.com/verivital/F1TenthVanderbilt/blob/master/f110-fall2018-skeletons/simulator/f1_10_sim/race/launch/f1_tenth.launch) ```<arg name="world_name" value="track_porto"/>``` to one of the names listed in the racecar_gazebo/worlds directory as mentioned above.

#### Strategy 1: Hallway simulation using teb planner, amcl, map built from gmapping:
```roslaunch wall_following move_base.launch```

Once you have run the above command. Navigate to rviz and set to navigation goals by clicking on the toolbar at the top of the screen in order to make the car move. 
![move_base](./images/teb.png "move_base")

#### Strategy 2: Pure Pursuit and Particle Filter Localization
Run the simulation:
```roslaunch race f1tenth.launch```

This script simply launches the car in the racetrack, enables keyboard teleoperation, and launches the relevant sensors

Run the particle filter:
```roslaunch particle_filter localize.launch```

**Note**: It takes the particle filter a couple of seconds to receive an initial position from Gazebo before the node begins publishing. 

Run pure pursuit
```roslaunch a_stars_pure_pursuit pure_pursuit_sim.launch  ```




