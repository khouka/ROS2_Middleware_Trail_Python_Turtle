# ROS 2 with Python Turtle 

This is a trail on ROS 2, the Robot Operating System, through the Python Turtle module. You will find completed scripts inside the `ROS2/src/Examples/`directory. Use them to flow through the trial's tasks. You will also find a `webcam/` directory, which will be explained later on. There will be further instructions on how to progress throughout this script and the two other to come inside the `ROS2/src/` directory. For now proceed by reading this script, if you encounter any issues throughout the trail feel free to ask questions through (https://answers.ros.org/questions/ask/). 

## Prerequisites: 
* Ubuntu 18.04: https://releases.ubuntu.com/18.04/ (The Desktop Server is Recommended)
* Git (`sudo apt-get install git`)
* ROS 2 Eloquent Elusor: (Installing ROS 2 via Debian Packages) 
* Python 3 is recommended 
* Install OpenCV (`pip3 install opencv-python`)
* Install imutils: (`pip3 install imutils`)

Before moving forward, clone this repository:
```
$ cd
$ git clone https://github.com/khouka/ROS2_Middleware_Trail_Python_Turtle.git
```

## Pre-trial Python Scripts:
Inside the  `ROS2/src/Example/` folder, you will find four python scripts. Simply run them by `$ python3 <script name>`. Please run them one by one and go over the code explanations provided below. They will come in handy later on while doing the tasks. 

- `Goal_seeker.py`: A python turtle script that drives the turtle towards a predestined goal, whenever the turtle reaches the goal, the goal displaces to a random point, and the user gets notified. There are other features inside. Upon running the file the output of this file will look like the following:


  <p align="left">
  <img src="figures/1.png" alt="" width="40%">
  </p>



