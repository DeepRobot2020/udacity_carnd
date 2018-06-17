# CarND-Path-Planning-Project Write Up
Self-Driving Car Engineer Nanodegree Program

# Introduction
In this project, we are supposed to design a path planner for the self-driving car or ego vehicle running inside the simulator. The path planner should read the sensor data from the simulator and compute a trajectory for the ego vehicle so that it can drive as close as to the max speed, minimize the jerk and also be able to switch lanes if current lane is not optimal

# Summary of the solution

To solve this problem, I created a few objects to help me design the path planner 
- EgoVehicle: this object reprents the self-driving vehicle running in the simulator
- TrafficVehicle: is to represent other vehicles running in the simulator 
- LaneState: is to represent the state of each lane including its average speed, free_space relative to the ego vehicle and how many vehicles (not including the ego-vehicle) are running on this lane. 
- Traffic: this is a parent object which will manager all the TrafficVehicle and LaneState information.
There are three LaneState objects inside Traffic, to represent left, middle and right lanes. Every time when the ego vehicle receives a sensor fusion json event, it will update traffic object. We will re-calculate the closest free space on this lane, and also the average speed on it.

## Lane change policy

Before switching to a new lane, we will compute the cost of switching to each lane and will only try to switch if the new lane will have lower cost compared to staying in current lane. The cost is made of 
the average speed of that lane, the frenet d value difference between current lane and the target lane and the length of free space. This is quite intuitive as we always to switch to a faster lane so that we can get to the destination faster, and we want to switch to a lane with more free space, so it will be saver and we always prefer staying in current lane. 

## Finite State Machine

In current implementation, there is only two states are used. KeepLane and ChangeLaneLeft/Right.
In the KeepLane state, we will compute the cost of staying current lane versus switching to a new lane and if switching to a new lane if more optimal, the next state will be ChangeLaneLeft/Right.

In the ChangeLaneLeft/Right state, we will first check whether current ego vehicle speed is faster than the average speed of the target lane, if not we will try to speed up first. We will also re-compute the optimal lane in this state and will fall back to KeepLane state if current target lane is no longer optimal.


# Reflection



