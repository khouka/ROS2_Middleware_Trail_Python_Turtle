# ROS 2 with Python Turtle 

This is a trail on ROS 2, the Robot Operating System, through the Python Turtle module. You will find completed scripts inside the `ROS2/src/Examples/`directory. Use them to flow through the trial's tasks. You will also find a `webcam/` directory, which will be explained later on. There will be further instructions on how to progress throughout this script and the two other to come inside the `ROS2/src/` directory. For now proceed by reading this script, if you encounter any issues throughout the trail feel free to ask questions through (https://answers.ros.org/questions/ask/). 

## Prerequisites: 
* Ubuntu 18.04: https://releases.ubuntu.com/18.04/ (The Desktop Server is Recommended)
* Git (`sudo apt-get install git`)
* ROS 2 Eloquent Elusor: (https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/) 
* Python 3 is recommended 
* Install OpenCV (`pip3 install opencv-python`)
* Install imutils: (`pip3 install imutils`)

Before moving forward, clone this repository:
```
$ cd
$ git clone https://github.com/khouka/ROS2_Middleware_Trail_Python_Turtle.git
```

## Pre-trial Python Scripts:
Inside the `ROS2/src/Example/`folder, you will find four python scripts. Simply run them by `$ python3 <script name>`. Please run them one by one and go over the code explanations provided below. They will come in handy later on while doing the tasks. 

- `Goal_seeker.py`: A python turtle script that drives the turtle towards a predestined goal, whenever the turtle reaches the goal, the goal displaces to a random point, and the user gets notified. There are other features inside. Upon running the file the output of this file will look like the following:
  <p align="left">
  <img src="figures/1.png" alt="" width="40%">
  </p>
  
- `keyboard.py`: A simple python script which detects key presses, precisely the arrow keys and prints messages based on the direction of the key pressed. 

- `red_detection.py`: A python script that uses OpenCV module. In this example, the program will read the image red.jpg inside the Examples/images as an OpenCV object. It will then display the original image and a mash of it, which only displays its red parts. 
  <p align="left">
  <img src="figures/1.png" alt="" width="40%">
  </p>

- `Turtle_Arena.py`: This is a python turtle script that creates a maze and sets the turtle to a specific location inside the maze. 
  <p align="left">
  <img src="figures/1.png" alt="" width="40%">
  </p>
  
Read the “The Code Explained” to have a more detailed and informative description of the examples. The code will be broken down and explained for each of the examples.

## The Code Explained

### `Goal_seeker.py`
  ```
  import turtle
  from tkinter import PhotoImage
  from turtle import Turtle, Screen, Shape
  import math
  import random
  ```
  We will use the turtle module in this example so we must first import it. We also import some of its classes. We import   the module random since the method randint will be used. Lastly, tkinter to upload an image, and math library.  
  ```
  myscreen = turtle.Screen()
  myscreen.title("Turtle_Driver")
  myscreen.bgcolor("silver")
  myscreen.setup(1100,1000)
  ```
  
  ```
  class Driver:
    def __init__(self):
     myscreen.tracer(0)
     self.xcor = 0
     self.ycor = 0     
     while True:
       myscreen.ontimer(self.Activity(), 300)
  ```
  
  ```
  def move_goal(self):          
       Goal.goto(self.xcor,self.ycor) 
          
  def robo_path_set(self):  
       Robot.setheading(Robot.towards(Goal))        
       Robot.fd(20)
  ```
  
  ```
  def Output(self):
       r_y = int(Robot.ycor()) 
       r_x = int(Robot.xcor())
       g_y = int(Goal.ycor()) 
       g_x = int(Goal.xcor()) 
       print ("Turtle:", int(r_x), int(r_y))
       print ("Goal:", int(g_x), int(g_y))
       self.dist = math.sqrt((g_x - r_x) **2 + (g_y - r_y) **2)
       print ("Distance left:", self.dist)
  ```
  
  ```
  def Update_goal(self):        
       if self.dist <= 10:   
         print("checkpoint successfully reached")     
         self.xcor = random.randint(-400,350)
         self.ycor = random.randint(-400,350)
         Goal.goto(self.xcor, self.ycor)
  ```
  
  ```
  def Activity(self):
       myscreen.update() 
       self.robo_path_set()
       self.move_goal()
       self.Output() 
       self.Update_goal() 
  ```
  
  ```
  class Robot(turtle.Turtle):
    def __init__(self):
      turtle.Turtle.__init__(self)
      self.shape("turtle")
      self.shapesize(2)
      self.color("red")
      self.penup()
      self.goto(-350,-400)  
  Robot = Robot()
  ```
  
  ```
  class Goal(turtle.Turtle):
    def __init__(self):
     turtle.Turtle.__init__(self)
     goal = PhotoImage(file="images/goal.png").zoom(1,1)          
     myscreen.addshape("goal", Shape("image", goal))
     self.shape("goal")
     self.penup()
     self.speed(0)
  Goal = Goal()
  ```
  
  ```
  if __name__ == '__main__':
    Driver()
  ```
### `keyboard.py`:
  
  ```
  import termios
  import fcntl
  import sys, os
  ```
  
  ```
  fd = sys.stdin.fileno()
  oldterm = termios.tcgetattr(fd)
  newattr = termios.tcgetattr(fd)
  newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
  termios.tcsetattr(fd, termios.TCSANOW, newattr)
  oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
  fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)
  ```
  
  ```
  try:
   while True:
     try:
       c = sys.stdin.read()
       if  c == "\x1b[A":
        print("driving forward")  
       elif  c == "\x1b[B":
        print("driving backward")                   
       elif  c == "\x1b[C":
        print("turning right") 
       elif  c == "\x1b[D":
        print("turning left")  
       elif c == "q":
        break
  ```
  
  ```
  except IOError: 
    pass
 except TypeError: 
    pass
  ```
  
  ```
  
finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
  ```




