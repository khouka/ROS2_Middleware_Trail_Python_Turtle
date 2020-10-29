# ROS 2 Actions Survey

## What this is:
This is an extension to the ROS 2 Trial. There are three forms of communication in ROS: topics, services, and actions. In the previous trial you mastered the topics and services. Here, you will learn how to use the actions. Actions are comparable to services. Likes services, Action clients send a request to an action server in order to achieve some goal and will get a result. Unlike services, while the action is being performed an action server sends progress feedback to the client. 

## ROS 2 Actions Resources:
- rclpy : http://docs.ros2.org/crystal/api/rclpy/
- ROS 2 Actions: https://index.ros.org/doc/ros2/Tutorials/Actions/Creating-an-Action/ 
- ROS 2 Action Service: https://index.ros.org/doc/ros2/Tutorials/Actions/Writing-an-Action-Server-Python/ 
- ROS 2 Action Client: https://index.ros.org/doc/ros2/Tutorials/Actions/Writing-an-Action-Client-Python/ 
- ROS 2 Full Tutorial:  https://index.ros.org/doc/ros2/Tutorials/

## Overview: 
### Setup:
- Packages 
- Action Types

### Action Service:
- Developing an Action Server node

### Action Client: 
- Developing an Action Client node

### Task:
- Practice ROS 2 Action’s communication. 

## Setup: 

### Packages:
In the previous trial, you learned how to create a workspace and the two different types of packages. You won’t be needing to create a workspace in this trial, as you will use the ~/python_turtle_trial/ROS2  workspace. Navigate to the ROS2 workspace and create 2 new packages. Create a pure python package named turtle_action, here is where you will be developing and storing your code files. Next, create a Cmake package named interfaces2, in which you will develop your action files to be later imported in our code files.
Checkpoint 1: You should have 2 new folders inside your python_turtle_trial/ROS2/src, the turtle_action python package, and the interfaces2 Cmake package. Please direct to Robotics_Middleware_Trial_Python_Turtle/readme.md for the question post.

