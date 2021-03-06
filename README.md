# Gazebo plugin and plugin modifications for the BRASS project

This repository contains the source for the BRASS Gazebo plugins needed for evaluation:

1. A modification of the OpenNI (Kinect) simulation plugin that allows the sensor to be turned off and also to stop if from producing depth images
2. A battery simulator that simulates energy charge and discharge
3. A headlamp actuator that can be used to light in front of a robot. The simulator tracks the model that the light is attached to and allows it to be
turned on and off.

## Building

The plugins are built using catkin_make. The repository must be checked out into catkin_ws/src (or equivalent) and then catkin_make will 
build them. Errors should inform you which packages are required, but `libignition-math2-dev` is known to be required.

## Installation

The command `catkin_make install` will install these in the catkin_make/install, but we need to write a script to install the built libraries into the
evaluation environment.

# BRASS Battery Simulator Plugin

## ROS interface
### Output
```
/energy_monitor/energy_level : Float64

  The current energy level, in mwh.

/energy_monitor/voltage : Int32

  The current simulated voltage, determined from the energy level according to
  the empirical data Ivan gathered. The relevant data is in v_data.cc.

/mobile_base/commands/motor_power : kobuki_msgs::MotorPower

  If the power runs out, the OFF message is sent here to turn the robot off.

```

### Input
```
Service: /energy_monitor/set_charging : Bool

  True if the robot is in charging mode, False otherwise (initially: True)

Service: /energy_monitor/set_voltage : Int32

  Set the voltage to a value between [104, 166]. This turns into an energy 
  level (mwh) according to the table in `v_data.cc` which is generated by 
  running `calculate_percent_of_v.py`.

/energy_monitor/set_nuc_utilization : Float64

  Set the NUC utilization percentage to a value between 0.0 and 100.0.
```

We also subscribe to `/sensor/kinect/onoff : String` to monitor the Kinect's 
state, and `/gazebo/get_model_state : Twist` to monitor the simulated odometry.

# BRASS OpenNI Plugin

In addition to producing camera and depth image output, we extend it with the following:

```
Service: /mobile_base/kinect/mode: Int8

  0 Turns the kinect off
  1 Turns the kinect on
  2 Turns the kinect depth image off (only rgb image is provided)
 ```
 
 # BRASS Headlamp plugin
 
 This plugin is attached to a model, and advertises a service based on the name of this model.
 
 ```
 Service: /mobile_base/headlamp: Bool
 
   Pass true to turn the headlamp on, false to turn it off
```

# BRASS Laser Scanner

This plugin is attached to the model, and advertises a service to turn the laser on or off.

```
Service: /mobile_base/lidar/mode: Bool

  Pass true to turn the laser scanner on, false to turn it off
```
 
