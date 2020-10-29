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
In the previous trial, you learned how to create a workspace and the two different types of packages. You won’t be needing to create a workspace in this trial, as you will use the `~/ROS2_Middleware_Trail_Python_Turtle/ROS2` workspace. Navigate to the ROS2 workspace and create 2 new packages. Create a pure python package named `turtle_action`, here is where you will be developing and storing your code files. Next, create a Cmake package named `interfaces2`, in which you will develop your action files to be later imported in our code files.

  - `Checkpoint 1`: You should have 2 new folders inside your python_turtle_trial/ROS2/src, the `turtle_action` python package, and the `interfaces2` Cmake package. Please direct to ROS2_Middleware_Trail_Python_Turtle/README.md for any questions or issues you encounter.
  
### Action Types:
Actions are specified using a form of the ROS Message IDL. Unlike the services which contain 2 sections, action types contain three sections, each of which is a message specification. For this task, you will be creating a custom `.action` file in the `interfaces2` package , and then utilizing it in the `turtle_action` package: 

 - Navigate to  `~/ROS2_Middleware_Trail_Python_Turtle/ROS2/src/interfaces2`:
   ```
   $ cd ~/ROS2_Middleware_Trail_Python_Turtle/ROS2/src/interfaces2
   ```
   
 - It’s a good practice to keep an `.action` file in its own directory within a package. 
   ```
   $ mkdir action
   ```
   
 - Inside the `interfaces2/action` directory you just created, make a new file called `SquareSpiral.action`. Add the following lines of code: 
   ```
   int64 loop_length 
   ---
   int64 total
   ---
   int64 current_spiral
   ```
 - As you can see, the action you just created is made up of three message definitions separated by `---`. The first is referred to as a `goal`, a request message, that is sent from an action client to an action server initiating a new goal. The second, a `result` message, is sent from an action server to an action client when a goal is done. Lastly, `Feedback` messages are periodically sent from an action server to an action client with updates about a goal. 

Next you must convert the action interface you just created into language-specific code,  so that it can be used later, to do so; modify the `package.xml` as well as `CMakeLists.txt`: 
 - Open up `CMakeLists.txt`, and search for line below:
   ```
   # find_package(<dependency> REQUIRED)
   ```
 - Add the following lines underneath it: 
   ```
   find_package(rosidl_default_generators REQUIRED)
             
   rosidl_generate_interfaces(${PROJECT_NAME}            
     "action/SquareSpiral.action"            
   )
   ```
 - Next open your `package.xml`, and search for lines below:
   ```
   <buildtool_depend>ament_cmake</buildtool_depend>
   ```
 - Add the following lines underneath it: 
   ```
   <build_depend>rosidl_default_generators</build_depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   <depend>action_msgs</depend>
   <member_of_group>rosidl_interface_packages</member_of_group>
   ```
By finishing the above, the action type should be built together when building the package. You can include the action type similar to how you would include service types.
```
from interfaces2.action import SquareSpiral
```

Unlike Services and Messages, Actions needs more code and functions to function, we will go over how to develop an Action Client and Server in the next subtopic. But first, build your package using colcon build. 

  - `Checkpoint 2`: The Build should be successful without any errors. Please direct to `ROS2_Middleware_Trail_Python_Turtle/README.md` for any questions or issues you encounter.

## Action Client/Service Nodes: 

### Developing an Action Server node:
Navigate to `~/ROS2_Middleware_Trail_Python_Turtle/ROS2/src/turtle_action/src`. Inside create a new file called `Spiral_server.py`. After doing so, copy the lines of code below into it. 
```
import <ROS 2 python client library>
from rclpy.node import Node
from interfaces2.action import <action type> 
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
```

First we will import the needed libraries. The first two lines are needed for every ROS 2 python node. Next we import our custom action type. The ActionServer and GoalResponse are two necessary built-in libraries for any action server. Lastly, we import the built-in ReentrantCallbackGroup which allows callbacks to be executed in parallel without restrictions. 
```
import turtle 
from turtle import *
import time
```

Since this trial is through the python turtle library we must also import it.
```
def screen_setup():
     <screen name> = turtle.Screen()
     <screen name>.title("<window Title>")
     <screen name>.setup(600,600)
     <screen name>.bgcolor("<black")

screen_setup()
```

Here we set the parameters for our Turtle Window, we first define and call it, next we set a title, the length and width of the screen, and the background color. Finally, we call our function. 
```
class <Turtle class name>(turtle.Turtle):
    def __init__(self):
       turtle.Turtle.__init__(self)
       self.shape("classic")
       self.pensize(3)
       self.color("white")

Robot = <turtle class name>()  
```
Here we define our Turtle as a class. We set the turtle’s shape as classic, and the color as white, and the pen size to 3.  Lastly, we define our class object and call it. 
```
class Turtle_Action_Server(Node):
  def __init__(self):
    super().__init__('<random node name>')
```
We first create a class which inherits from the Node class.  Next we call super().__init__  which, in turn calls the Node class’s constructor and gives it your node name. This is the server part of the code. 
```
    self.action_server = ActionServer(self,<action type>,'<action name>', 
    execute_callback= self.execute_callback,
    callback_group= ReentrantCallbackGroup(),
    goal_callback= self.goal_callback,)
```
Here we are creating our action server. The basic action server requires the first four arguments. Self, the ros node to add the action client to. The action type, the action name, and a callback function for executing accepted goals. Keep in mind, this callback must return a result message. The next two lines are the callback group we imported earlier to allow multiple callbacks to run parallel and the goal callback which will return the client whether the goal was accepted or not. 
```
   def goal_callback(self, goal_request):
       self.get_logger().info('The goal request was received')
       return GoalResponse.ACCEPT
```
Next we will define the goal callback we called earlier, this function will accept or reject a client request to begin an action. But first, we notify the user that the goal was received.   
```
   def execute_callback(self, goal_handle):
       self.get_logger().info('Initializing goal execution....')
       feedback_msg = <action type>.Feedback()
       feedback_msg.<action type feedback> = 0
```
This is the main part of the action server. Here is where the goal will be executed, the feedback will be proceeded and published and the final result will be determined to be published back to the client. 
```
       for i in range(1, goal_handle.request.<action type goal>):
           Robot.right(90)
           Robot.forward(4*i)
```
We use the for in range statement it goes from 1 to the desired integer goal requested by the action client. For every integer the turtle will rotate 90 degrees and move forward in an increasing pattern thus creating a Square Spiral. 
```
           if i % 4 == 0:
              feedback_msg.<action type feedback> = i
              self.get_logger().info('Publishing feedback:{0}'.format(feedback_msg.<action type feedback>))
              goal_handle.publish_feedback(feedback_msg)
```
In this part we define what the progessive feedback will be. We set it so that every 4 loops the program runs, the server will publish a feedback to the client, notice how we use the get_logger rather than the print and the goal_handle rather than the publish used in publisher subscriber communication
```
           if goal_handle.is_cancel_requested:
              goal_handle.canceled()
              self.get_logger().info('Goal was canceled')
              return <action type>.Result()
```
This part of code is dealing with the possibility that the goal might be canceled. If the goal is canceled for some reason while the program is executing the request, the program will simply stop the execution and publish whatever result was reached when the goal was canceled. 

```
       goal_handle.succeed()
       result = <action type>.Result()
       result.total = int(feedback_msg.<action type feedback> /4)
       self.get_logger().info('Returning result: {0}'.format(result.total))
       return result
```
Lastly, we have the final part, the result. If the goal was successfully reached, and the execution was done thoroughly. We will publish a result back to the client, similar to how we would in a basic server and client nodes. We define the result as the total number of loops divided by 4 which results into how many spirals were done. It is very important that we end our execution with a return result. 
```
def main():
    rclpy.init()
    <node name> = Turtle_Action_Server()
    try:
       rclpy.spin(<node name>)
    except KeyboardInterrupt:
       pass 

    rclpy.shutdown()
if __name__ == '__main__':
    main()
```
Finally, we have the main. The exact same as any other main we ran across in this trail, just give it a random node name of your choice, and we are calling the Node class Turtle_Action_Server(). By filling up the `<>` sections above, you should have a complete Action server turtle.

  - `Checkpoint 3`: Run the code with `$ python3 Spiral_server.py`, it should be running with no error messages, and there should be a black screen turtle window with the turtle in static in the middle. Please direct to `ROS2_Middleware_Trail_Python_Turtle/README.md` for any questions or issues you encounter. 

### Developing an Action Client node:  
Next you will create an action Client for the `Spiral_server.py` you created just earlier. In the same directory. Create a new file called `Spiral_client.py`. Copy the lines of code below inside. 
```
import <ROS 2 python client library>
from rclpy.node import Node
from rclpy.action import ActionClient
from interfaces2.action import <action type>
```
We import the needed libraries. Our custom action type. The ActionClient is the necessary built-in library for any action client.
```
from action_msgs.msg import GoalStatus
import time
```
Here we import the built-in GoalStatus library from the action_msgs.msg package which is very useful when developing action nodes . This msg type will describe the goal’s current state machine status. 
```
class Turtle_Action_Client(Node):
  def __init__(self):
    super().__init__('turtle_action_client')  
    self.turtle_action = ActionClient(self,<action type>,'<action name>')
```
This part is identical to the action server, except we are calling an ActionClient, we won’t need to instantiate any callbacks ot functions as they will be called later through one another’s functions.
```
    def send_goal(self, loop_length):
        self.get_logger().info('Waiting for action server...')
        self.turtle_action.wait_for_server()
        goal_action = <action type>.Goal()
        goal_action.<action type goal> = loop_length
```
We define the primary part of our action client, the send_goal. We start off by using the wait_for_server method. This is used so that when we run the action client file, if no server is present, the client will wait until a server is found. Next we define our action goal. 
```
        self.goal_future =  self.turtle_action.send_goal_async(
        goal_action, 
        feedback_callback= self.feedback_callback)

        self.goal_future.add_done_callback(self.goal_callback)
```
Here we instruct the client to send the action goal asynchronously, we then call the next function to be executed after the goal has been sent, the goal_callback. 
```
    def goal_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('Goal was rejected')
            return
        self.get_logger().info('Goal was accepted, will proceed...')
        
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)
```
In the goal_callback function, is where the client will be notified whether the request goal has been successfully received and accepted. If the goal was rejected we notify the user and the process stops there, otherwise if it was accepted, we move on through the process, and call the next function to be executed, the feedback_callback, which will deal with the progressive feedback that the server will be publishing throughout the execution.  
```
    def feedback_callback(self, feedback_action):
        feedback = feedback_action.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.<action type feedback>))
```
Inside the feedback callback we simply define our feedback, and then print the feedback in the shell using the get_logger method. 
```
    def result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print('Goal succeeded! Total number of spirals:', result.<action type result>)
        else:
            print('Goal could not be reached')
        # Shutdown after receiving a result
        rclpy.shutdown()
```
Finally, the last part of the Action Client, the result callback. Here is will the final result, published by the server, be received back. We first define our result. Next we use the built-in action msg we imported earlier, the msg file consists of goal states. We will be using the STATUS_SUCCEEDED part. This conveys that the goal has been achieved successfully by the action server. If so, print the result, else notify the user the goal couldn't be reached. Lastly, shutdown after receiving a result. 
```
def main():
    rclpy.init()
    <node name> = Turtle_Action_Client()
    
    <node name>.send_goal(61)
    rclpy.spin(<node name>)

if __name__ == '__main__':
    main()
```
Finally, the main function is defined. Here we simply initialize ROS2 communications, next you will define a node name for the Turtle_Action_Client() Node. Since we didn’t call the send_goal function earlier inside the node, you must call it here. We left an argument to be filled in that function, the number of loops the client is requesting. Finally, make the node spin using rclpy.spin(). We don’t need to shutdown the node here as it will be done when the result is received. By filling up the `<>` sections above, you should have a complete Action Client turtle.

  - `Checkpoint 4`: In a shell, run the `Spiral_server.py` code, then in another terminal run the Action client:  `$ python3 Client_turtle.py` , it should bring up a window similar to the one shown below, with the correct feedback and result after the requested number of loops are executed. Please direct to `ROS2_Middleware_Trail_Python_Turtle/README.md` for any questions or issues you encounter.  


 


