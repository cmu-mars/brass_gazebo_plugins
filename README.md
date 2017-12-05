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
