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
  <img src="https://media.giphy.com/media/SScsKuefizu5mZlJvt/giphy.gif" alt="" width="50%">
  </p>
  
- `keyboard.py`: A simple python script which detects key presses, precisely the arrow keys and prints messages based on the direction of the key pressed. 

- `red_detection.py`: A python script that uses OpenCV module. In this example, the program will read the image red.jpg inside the Examples/images as an OpenCV object. It will then display the original image and a mash of it, which only displays its red parts. 
  <p align="left">
  <img src="https://media.giphy.com/media/fZd6ugRxgHMmsitBCr/giphy.gif" alt="" width="50%">
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
  We then use the function turtle.Screen() to create a window. We initialize the display and set some features including the background color, window’s title, and the size of the screen.
  ```
  class Driver:
    def __init__(self):
     myscreen.tracer(0)
     self.xcor = 0
     self.ycor = 0     
     while True:
       myscreen.ontimer(self.Activity(), 300)
  ```
  In Python, classes are declared by the keyword class, followed by the class name. After declaring the class name, we must then define a constructor method. No matter what the class name is, the constructor’s name  is always __init__. We set the initial x and y coordinates of the goal, and call the mainloop, inside we Install a timer that calls the subclass Activity every 300 milliseconds.
  ```
  def move_goal(self):          
       Goal.goto(self.xcor,self.ycor)      
  def robo_path_set(self):  
       Robot.setheading(Robot.towards(Goal))        
       Robot.fd(20)
  ```
  Here we have 2 subclasses, which define the movement of the goal and the robot turtle. Using the function setheading() and towards(), we are able to make the turtle face towards the goal at all times. 
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
  In this subclass, we first define the variables r_y, r_x, g_y, g_x as the coordinates of the robot turtle and the goal. We then find the remaining distance using the distance formula. Lastly, we let the user know of the Turtle pos, the goal’s pose, and the distance left with the print function. 
  ```
  def Update_goal(self):        
       if self.dist <= 10:   
         print("checkpoint successfully reached")     
         self.xcor = random.randint(-400,350)
         self.ycor = random.randint(-400,350)
         Goal.goto(self.xcor, self.ycor)
  ```
  In this subclass, we define what happens when the goal is reached. We use the random.randint and goto functions to displace the goal to a new random x and y coordinates. 
  ```
  def Activity(self):
       myscreen.update() 
       self.robo_path_set()
       self.move_goal()
       self.Output() 
       self.Update_goal() 
  ```
  Here we simply call all the previous subclasses we created to be executed in our loop. 
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
  We define a new class as our Robot class, this class will represent our robot turtle. We then use some turtle functions to adjust the features of the turtle and lastly we call the class to be executed. 
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
  Same as with the previous Robot class, except class Goal is dedicated for the goal, and we import its shape from `Examples/images/goal.png`.
  ```
  if __name__ == '__main__':
    Driver()
  ```
  And finally we call the class Driver to be carried out. 
  
### `keyboard.py`:
  
  ```
  import termios
  import fcntl
  import sys, os
  ```
  We import the needed modules for Linux. 
  ```
  fd = sys.stdin.fileno()
  oldterm = termios.tcgetattr(fd)
  newattr = termios.tcgetattr(fd)
  newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
  termios.tcsetattr(fd, termios.TCSANOW, newattr)
  oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
  fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)
  ```
  Here we set and initialize the system so that it can listen to onkey presses. We also save the terminal settings and set the new terminal setting unbuffered. 
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
  Next we have a while loop, the program will detect the defined keyboard characters if pressed. Stdin stands for standard input which is a stream from which the program reads its input data. It is used to get input from the keyboard. The arrow keys here start with \x1b. Lastly, if the  letter q is pressed, it breaks out of the loop and exits. 
  
  ```
  except IOError: 
    pass
  except TypeError: 
    pass
  ```
  In python, using read() when no input is available will result in IOError. The two except above will prevent empty buffer error. 
  ```
  
  finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
  ```
  Last but not least, we finally exit, and the system is then set back to normal. 

### `red_detection.py`:

  ```
  import cv2
  import numpy as np
  import imutils
  import time
  ```
  First we import the necessary libraries. cv2 for the OpenCV, numpy to be used for the arrays, imutils which aids in  finding contours, and lastly the time module.  
  ```
  while True:
   img = cv2.imread("images/red.jpg") # Defining our image
  ```
  Next we create a while loop, in which we will first define our image by instructing the program to read the image red.jpg inside the `Examples/images` as an OpenCV object. 
  ```
  hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #Convert the image in the BGR color space to the HSV color space.
  low_red = np.array([160, 155, 84]) # Define the upper bound of the red color in hsv format 
  high_red = np.array([180, 255, 255]) # Define the lower bound of the red color in hsv format
  ```
  In this example we will be using the HSV color space, so we must convert the BGR default to the HSV first. We do that using the method cv2.cvtColor(), then we define the lower and upper bounds of the red color in hsv format. 
  ```
  red_mask = cv2.inRange(hsv, low_red, high_red) # Returns a mask, specifies which pixels fall into the upper and lower  bound range.
  red_img = cv2.bitwise_and(img, img, mask = red_mask) # Applying the mask to the image
  ```
  Here, a mask is created which filters the image and  specifies which pixels fall into the upper and lower bound range. We then apply the mask to the image using the function `cv2.bitwise_and()`. 
  ```
  cv2.imshow("Original Image", img) # Name of the window, the image to be shown  
   cv2.imshow("Red Detection", red_img) # Display another window for the mask 
   key = cv2.waitKey(1) #Set the delay to 1 ms
   if key == 27: # Press esc key to exit 
     break 
  ```
  We call the method `imshow()` twice. The first call is to display the original image, and the second is to display only the filtered result.   
  ```
  cnts_red = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # Finding the Contours for the red mask 
  cnts_red = imutils.grab_contours(cnts_red)
  for r in cnts_red:
    if cv2.contourArea(r) > 1000: # If detected execute the following
      print('Red was detected')
      time.sleep(1.5) # Suspends execution for 1.5 seconds 
  ```
  We use the method `cv2.findContours` which takes 3 parameters: the image, cv2.RETR_TREE which retrieves the entire   hierarchy of contours in the image, and  `cv2.CHAIN_APPROX_SIMPLE)`. We lastly use the `imutils.grab_contours`. Imutils helps us in sorting the contours and detecting the edges.  If the contour is detected then let the user know, Lastly we suspend execution of one and a half second. 
  
### `Turtle_Arena.py`:

  ```
  import turtle
  from tkinter import PhotoImage
  ```
  First, we import the required modules.
  ```
  class TurtleMaze:
    def __init__(self):
     self.myscreen =  turtle.Screen()
     self.myscreen.title("Turtle road")
     self.myscreen.bgcolor("silver")
     self.myscreen.setup(1200,700)
     self.Turtlebot()
     self.Pen()     
     self.maze()
     self.setup_maze(self.blocks[1])
  ```
  We create a class called TurtleMaze, then inside the init, we initialize the display and set some features including the background color, window’s title, and the size of the screen. We then call the  four subclasses to be executed. 
  ```
  def Pen(self):
    sQuare = PhotoImage(file="images/block.gif").zoom(1,1)          
    self.myscreen.addshape("sQuare", turtle.Shape("image", sQuare))
    self.pen = turtle.Turtle()
    self.pen.shape("sQuare")
    self.pen.penup()
    self.pen.speed(0)
  ```
  Next we created a subclass called Pen. We use the method tkinter.PhotoImage, to read image block.gif inside the   `Examples/images`. Next we create a turtle and assign it the name pen. We give it the shape of the block image we read earlier. This is the turtle that will draw the blocks of the maze. 
  ```
  def Turtlebot(self):
    self.turtlebot = turtle.Turtle()
    self.turtlebot.shape("turtle")
    self.turtlebot.color("black")
    self.turtlebot.penup()
    self.turtlebot.speed(0)
    self.turtlebot.setheading(180)
  ```
  We also create another subclass Turtlebot, we create another turtle, this one will be used as a player's object. 
  ```
  def maze(self):
       self.blocks = [""]
       maze_blocks = [
  "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX",
  "X              X                           T X",
  "X  XXXXXXXXXX  XXXXXXXXXXXXX  XXXXXXX  XXXX  X",
  "X           X                 X        X     X",
  "X  XXXXXXXXXX  XXXXXXXXXXXXX  XXXXXXXXXXXXX  X",
  "X  X     X  X           X  X                 X",
  "X  X  X  X  X  X   XXX  X  X  XXXXXXXXXXXXX  X",
  "X  X  X  X  X  X  X     X  X  X  X        X  X",
  "X  X  XXXX  X  XXXXXXXXXX  X  X  XXXX  X  X  X",
  "X  X     X  X              X           X  X  X",
  "X  XXXX  X  XXXXXXXXXXXXXXXX  XXXXXXXXXXXXX  X",
  "X     X  X                    X              X",
  "XXXX  X  XXXXXXXXXXXXXXXXXXXXXX  XXXXXXXXXX  X",
  "X  X  X                    X     X        X  X",
  "X  X  XXXX  XXXXXXXXXXXXX  X  XXXX  X  X  X  X",
  "X  X  X     X     X     X  X  X     X  X  X  X",
  "X  X  X     X     X     X  X  X     X     X  X",
  "X  X  X  XXXXXXX  XXXX  X  X  X  XXXXXXXXXX  X",
  "X                       X  X  X              X",
  "XXXX  X  X  XXXXXXXXXX  X  X  X  XXXXXXXXXXXXX",
  "XXXXXXXXXXXXXXXXXXXXXXXXX  XXXXXXXXXXXXXXXXXXX",
  ]
     self.blocks.append(maze_blocks)
     self.walls = []
  ```
  Here we define the maze, we set the outline of the maze using the character ‘X’. After doing so, we append the maze_blocks.
  ```
  def setup_maze(self, block):
    for y in range(len(block)):
      for x in range(len(block[y])):
        character = block [y] [x]
        screen_x = -550 + (x * 24)
        screen_y = 250 - (y * 24)
  ```
  Next we define the motion the pen turtle will be taking when drawing the maze we outlined just earlier. 
  ```
  if character == "T":
    self.turtlebot.goto(screen_x, screen_y)
  if character == "X":
    self.pen.goto(screen_x, screen_y)
    self.pen.stamp() 
    self.walls.append((screen_x, screen_y))
  ```
  At last, we execute and display the maze. 
  ```
  TurtleMaze()
  turtle.mainloop()
  ```
  We call the class TurtleMaze to be carried out. And we end it off with the mainloop() function, this  must be the last statement in a turtle graphics program. 
  
## What to do next:


  

