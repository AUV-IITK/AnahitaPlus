#  Overview of the Software Stack (Anahita)

### This document contains a basic overview of the entire software stack that exists in this repository. 

Updated: 24th December, 2019

## Control Layer

This layer is basically concerned with the low-level controller of the vehicle. Previously we used a simple PID controller each of the DOF of Anahita. This was implemented in a decoupled manner, i.e. we ran the controllers independently for each of the axes, and according to what we intend to do, we pass the desired position, and just apply the PID formula. The parameters for PID are tuned during the time of testing the vehicle in the pool.

The current repository contains an implementation of the Cascaded PID Controller, which basically consists of two PID controllers on top of each other, one for position control and one for velocity control. This controller worked well, but since our localisation was heavily dependent on the readings from the DVL, the position controller failed to perform well in depths of water less than 1 metre. This controller was implemented w.r.t positions and velocities in all directions simultaneously (Euclidean distances) in contrast to the previous method.

We intend to study some new techniques in Controls, and implement them in due course of time. They are:

- **Thruster Allocation Matrix**: This is implemented in the code, but I doubt whether it works correctly. Need to check and fix this.
- **LQR controller**: Used by some teams coming to Robosub, performs well according to them
- **Reinforcement Learning based Controller**: This would be very advanced, we'll need to setup a simulation environment with APIs according to the RL literature.


## Navigation Layer

This layer was mostly written considering the DVL in mind. The Doppler Velocity Log provides us with the velocity information of the water profile near the vehicle. Using that along with the IMU, we have created a basic structure for odometry for Anahita. Odometry message tell us about the state of the vehicle in the pool, and is therefore necessary for navigation.

The mapping package is untested. We wrote down the code for an implementation of the FastSLAM paper, which is basically a landmark-based SLAM. SLAM stands for Simultaneous Localisation and Mapping, which is the process by which a robot creates a map for the obstacles and landmarks around itself, and further localises itself in the map, by estimating a position with the concerned error in the position. 

- **Vehicle Dynamics Model**: We are currently looking at methods concerning Vehicle Dynamic Model based localisation. This would incorporate the dynamics of our vehicle while estimating its position.
- **Visual SLAM**: This is something we are going to implement surely over the next semester. In this, features on images such as corners are located and the change in their positions are processed to calculate your change in state. ORB SLAM is a well-known implementation of this technique.
- **Disparity Map Generation**: With the procurement of new iDS cameras, we hope to setup a stereo mapping system, which would allow us to get depth maps, and find how far the objects are from the vehicle.

## Master Layer

This layer basically works as the highest most-layer authority for autonomy. Previously, we have used a complex set of `if-else` statements combined with timeouts to create this system. This is the basis of intelligent behaviour for the vehicle, and has scope for the most drastic change in all of the stack.

- **SMACH**: This is a general-purpose library for creating heirarichal state machines, where each behaviour is a state, and we have to code the transitions between them.
- **Behaviour Trees**: Bascially this is a system of trees, where we would setup a non-binary tree-like structure for performing tasks.
- **End-To-End Deep Reinforcement Learning for Task Completion**

## Hardware Layer

Pretty simple to explain, but one of the toughest challenges lie here. Multiple hardware driver packages are present here, such as: *IMU, Cameras, DVL, Arduino*. 

- **Hydrophones**: Currently we use a Virtual Machine for running the LabView software on our Ubuntu system. This is very unreliable, and has not been tested for longer durations. We need to reverse-engineer drivers made available for other Linux-distros and configure them to obtain data from the NI DAQ board directly to our system. Pure brainstorming and hacking stuff here.

- **iDS Cameras**: Would be pretty easy to implement depending upon the support for the cameras available online.

## Vision Layer

Currently has a pre-processing timeline implemented (Fusion Framework).
OpenCV nodes for most tasks have been written following a object-oriented structure for changing the current tasks. Along with this, a machine-learning based approach is also implemented. We have trained YOLO with the data collected from the swimming pool and tested it on the Jetson TX2.

- **Tracking and Filtering**: Would implement tracking for all the tasks and better filtering of random values during object detection
- **CNNs and Style Transfer for Data Generation**: Basically work on the computer vision part of the vision team.

## Debug Layer

Mostly used in the competition and testing time.

- **GUI**: We have developed a basic version of the GUI, which works on the local network using `ros-bridge`. We need to extend this to have more functionality.
- **Color Calibration**: Using static values for thresholding makes it suspectible to changing brightness in the pool, and color variations. We need to setup a very abstracted and simplified method for doing this.

