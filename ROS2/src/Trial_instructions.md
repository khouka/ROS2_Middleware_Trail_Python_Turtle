# ROS 2 Survey 

The Robot Operating System is a set of software libraries for building robot applications. From drivers to state-of-the-art algorithms, ROS 2 has what you need for your next robotics project, and it’s all open source. The primary system of ROS 2 consists of the correlation between publishers and Subscribers through nodes. The goal of this trail is to integrate the examples given and the sub-concepts of ROS 2 to create different python turtle nodes. The final task will require you to use the webcam examples provided and create a client/server function where you move the turtle using the webcam.  

## ROS 2 Resources:

- rclpy (ROS Client Library for Python) : http://docs.ros2.org/crystal/api/rclpy/
- ROS 2 Workspace: https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/
- ROS 2 Package: https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/ 
- ROS 2 Messages and Services: https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/
- ROS 2 Client and Service: https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Py-Service-And-Client/ 
- ROS 2 Publisher/Subscriber Example: https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber/ 
- ROS 2 Full Tutorial: https://index.ros.org/doc/ros2/Tutorials/

## Overview: 

### Setup: 
  - Workspace
  - Package
  - Message Types
  - Service Types 
  - Build Package
  
### Pub/Sub Nodes:
  - ROS 2 Publisher 
  - ROS 2 Subscriber 

### Client/Service Nodes: 
  - Developing a Server node
  - Developing a Client node

## Setup:

### Workspace:
A workspace is a directory containing ROS 2 packages. Before using ROS 2, it’s necessary to source your ROS 2 installation workspace in every new terminal by `$ source/opt/ros/eloquent/setup.bash`. This makes ROS 2’s packages available for you to use in that specific terminal. It is best to create a new directory for every new workspace. You won’t be needing to create a workspace in this trial, as you are already in the `~/ROS2_Middleware_Trail_Python_Turtle/ROS2`workspace. 

### Package:
Packages work quite differently in ROS 2 then ROS. You can see a package as a container for your ROS 2 code.  If you want to be able to install your code or share it with others, then you’ll need to organize it in a package. A single workspace can contain as many packages as you want. It is best to have a `src` folder within your workspace, and to create your packages in there. This keeps the workspace neat and easy to navigate through. In this directory, you will find a completed webcam python-type package. Navigate to it with, `~/ROS2_Middleware_Trail_Python_Turtle/ROS2/src/webcam`. For this example, we will need 2 more packages. You’ll create 2 packages of different types, one for the python turtle, and the other for the interfaces(msg and srv):
 - Source your new terminal: 
   `$ source/opt/ros/eloquent/setup.bash`

 - Navigate to  `~/ROS2_Middleware_Trail_Python_Turtle/ROS2/src`:
   `$ cd ~/ROS2_Middleware_Trail_Python_Turtle/ROS2/src`

 - Create a pure python package named python_turtle, here is where you will be developing and storing your code files. 
   `$ ros2 pkg create --build-type ament_python python_turtle`

 - Next we create a Cmake package named `interfaces`, in which we will develop our msg and srv files to be later imported in our code files. There currently isn’t a way to generate a `.msg` or `.srv` file in a pure Python package. 
   `$ ros2 pkg create --build-type ament_cmake interfaces`

You should now have 2 new folders inside your `ROS2_Middleware_Trail_Python_Turtle/ROS2/src`, the `python_turtle/` package, and the `interfaces/` package. By doing that you have created two packages, now let’s customize them!  







