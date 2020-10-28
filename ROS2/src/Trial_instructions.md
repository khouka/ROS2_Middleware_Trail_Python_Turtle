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
   ```
   $ source/opt/ros/eloquent/setup.bash
   ```
   
 - Navigate to  `~/ROS2_Middleware_Trail_Python_Turtle/ROS2/src`:
   ```
   $ cd ~/ROS2_Middleware_Trail_Python_Turtle/ROS2/src
   ```
   
 - Create a pure python package named python_turtle, here is where you will be developing and storing your code files. 
   ```
   $ ros2 pkg create --build-type ament_python python_turtle
   ```
   
 - Next we create a Cmake package named `interfaces`, in which we will develop our msg and srv files to be later imported in our code files. There currently isn’t a way to generate a `.msg` or `.srv` file in a pure Python package. 
   ```
   $ ros2 pkg create --build-type ament_cmake interfaces
   ```

You should now have 2 new folders inside your `ROS2_Middleware_Trail_Python_Turtle/ROS2/src`, the `python_turtle/` package, and the `interfaces/` package. By doing that you have created two packages, now let’s customize them!  

### Message Types:
`msg` files are simple text files that describe the fields of a ROS 2 message. They are used to generate source code for messages in different languages. In this trail you will be using them in pub/sub nodes. For this task, you will be creating a custom `.msg` file in the `interfaces` package , and then utilizing it in the `python_turtle` package:

 - Navigate to  `~/ROS2_Middleware_Trail_Python_Turtle/ROS2/src/interfaces`:
   ```
   $ cd ~/ROS2_Middleware_Trail_Python_Turtle/ROS2/src/interfaces
   ```

 - It is a good practice to keep a `.msg` file in its own directory within a package. Create the directory:
   ```
   $ mkdir msg
   ```

 - Inside the `interface/msg/` directory you just created, make a new file called `Setcolor.msg` with one line of code declaring its data structure:
   ```
   string color
   ```
   msg files are composed of two parts: `fields` and `constants`.This is your custom message that will transfer a single string called `color`. 

To convert the interface you just created into language-specific code (here python) so that they can be used later, you must first modify the `package.xml` as well as `CMakeLists.txt`. 

  - First, you must open up `CMakeLists.txt`, and search for lines below:
    ```
    # uncomment the following section in order to fill in
    # further dependencies manually.
    # find_package(<dependency> REQUIRED)
    ```
    
    Add the following lines underneath it:    
    ```
    find_package(rosidl_default_generators REQUIRED)    
    
    rosidl_generate_interfaces(${PROJECT_NAME}            
      "msg/Setcolor.msg"            
    )
    ```
    
  - Next open your package.xml, and search for lines below:
    ```
    <buildtool_depend>ament_cmake</buildtool_depend>
    ```
    
    Add the following lines underneath it: 
    ```
    <build_depend>rosidl_default_generators</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```

By finishing the above, the message type should be built together when building the package. 

- You can include the message type similar to how you would include it in ROS: 
  ```
  from interfaces.msg import Setcolor
  ```
  
- And to create an object of that message type:
  ```
  msg = Setcolor()
  msg.color = choice
  ```
  
### Service Types: 
`srv` files describe a service. They are composed of two parts: a `request` and a `response`. The request and response are message declarations. In this trail you will use them in the Client/Service nodes. For this task, you will create a custom `.srv` file in the interfaces package , and then utilize it in the `python_turtle` package:
  - Navigate to  `~/ROS2_Middleware_Trail_Python_Turtle/ROS2/src/interfaces`:
    ```
    $ cd ~/ROS2_Middleware_Trail_Python_Turtle/ROS2/src/interfaces
    ```
    
  - It is a good practice to keep a `.srv` file in its own directory within a package. Create the directory:
    ```
    $ mkdir srv
    ```
    
  - Inside the `interface/srv` directory you just created, make a new file called `Distance.srv` with with the following request and response structure:
    ```
    float64 x1 
    float64 y1 
    float64 x2 
    float64 y2
    ---
    float64 dist
    ```
    
    A service file consists of a request and a response of msg type that are separated by `---`. Any   two .msg files linked with a `—` is a service description.This is your custom service that requests 4 floats named `x1`, `y1`, `x2` and `y2`, and responds with a float called `dist`.

To convert the interface you just created into language-specific code (here python) so that they can be used later, you must first modify the same `package.xml` and `CMakeLists.txt` you modified for the msg type. 
  - Open up the  `CMakeLists.txt`, and search for lines below: 
    ```
    find_package(rosidl_default_generators REQUIRED)    
    
    rosidl_generate_interfaces(${PROJECT_NAME}            
     "msg/Setcolor.msg"            
    )
    ```
    
    Edit it by adding the srv file.
    ```
    find_package(rosidl_default_generators REQUIRED) 
    
    rosidl_generate_interfaces(${PROJECT_NAME}            
     "msg/Setcolor.msg"
     "srv/Distance.srv"            
    )
    ```
    You already did the necessary modifications for the `package.xml` in the msg task earlier. 

You can include the srv type similar to how we included the msg type. 
 ```
 from interfaces.srv import Distance 
 ```
 
We initialize the server by: 
 ```
 self.srv = self.create_service(Distance, 'var04', self.Find_Distance)
 ```
 
The function is then provided underneath in the same class. 
 ```
 def Find_Distance(self, request, response):
    response.dist = <The formula> 
    return response
 ```
 
Congrats! you have successfully customized your msg and srv files inside your interfaces package. Scroll to `Build Package` to compile your package. 

### Build Package:
Now that you have fully customized and organized your interfaces package, the next step is to build the package. In the root of your workspace (`~/ROS2_Middleware_Trail_Python_Turtle/ROS2`), run the following commands:
 ```
 $ colcon build --packages-select interfaces
 ```
Now the interfaces will be discoverable by other ROS 2 packages. Lastly run the following command from within your workspace (`~/ROS2_Middleware_Trail_Python_Turtle/ROS2`) to source it:
 ```
 $ . install/setup.bash
 ```
You have now fully completed the msg and srv tasks. Whenever you open a new terminal make sure you source your ROS2 installation, `$ source/opt/ros/eloquent/setup.bash` and then inside your workspace run `$ . install/setup.bash`.

* `Checkpoint 1`: The Build should be successful without any errors. Please direct to `ROS2_Middleware_Trail_Python_Turtle/README.md` for any questions or issues you encounter.

## Publisher/Subscriber nodes: 

### ROS 2 Publisher:
Under `~/ROS2_Middleware_Trail_Python_Turtle/ROS2/src/webcam/src/` there is a python script called `webcam_pub.py`. This script is done through using the OOP format. 
  ```
  import rclpy
  from rclpy.node import Node
  ```
  When writing a ROS 2 Node, you must always import rclpy, the ROS Client Library for Python. Next, from rclpy.node, we import the Node module so that its Node class can be used. 
  
  ```
  import cv2
  import numpy as np
  ```
  We then import the OpenCV module and the numpy for the arrays. 
  
  ```
  from sensor_msgs.msg import Image
  from cv_bridge import CvBridge
  ```
  Lastly, we import the Image msg from the sensor_msgs and the cv_bridge so that we can make the conversions between ros and opencv image data.
  
  ```
  class Webcam_cap: 
    def __init__(self): 
      self.cam = cv2.VideoCapture(0) 
      self.cam.set(3,640)
      self.cam.set(4,480)        
    def Capture_Frame(self): 
      _, self.img = self.cam.read() # Defining our image
      return self.img
  ```
  Here we create the object class, which will capture the image data from the camera. First is the video capturing from the camera, default cam number is 0. We then set the size of the window, we then define the Capture_frame function. 
  
  ```
  class Campublisher(Node):     
    def __init__(self):
      super().__init__('Cam_publisher')
  ```
  Since we are using OOP in ROS2, we first create a class which inherits from the Node class. Thus, the Campublisher class is created. Then, the class’s constructor is defined.  Next we call super().__init__  which, in turn calls the Node class’s constructor and gives it your node name, in this example, Cam_publisher.  
  ```
  self.publisher = self.create_publisher(Image, 'camera_msg', 10)
  self.timer = self.create_timer(0.1, self.timer_callback)
  self.obj = Webcam_cap()
  self.bridge = CvBridge()
  print("webcam running....")
  ```
  Next we initialize the ROS 2 python publisher. Using  create_publisher we declare that the node publishes messages of type Image, which we imported from the sensor_msgs module, over a topic named ‘camera_msg’, and lastly we set the “queue size” to 10. Next, we must create a timer with a callback to be executed, we set the period of suspension to a second and we call the Callback function. We define the timer_callback function right after. We also call our object class and let the user know the webcam is running. 
  
  ```
  def timer_callback(self):
    pub_frames = self.obj.Capture_Frame()
    msg = self.bridge.cv2_to_imgmsg(pub_frames)       
    self.publisher.publish(msg)
  ```
  Here we are defining the timer_callback method which will contain the data to be published. We define pub_frames as the cv2 image data collected, we then define the msg as the ROS img msg converted version of the pub_frames. Lastly we use publisher.publish(msg) to send the msg. 
  ```
  def main():
    rclpy.init()
    node = Campublisher()
    try:
       rclpy.spin(node)
    except KeyboardInterrupt:
       pass 
    node.destroy_node()
    rclpy.shutdown()
  ```
  Finally, the main function is defined. Here we simply initialize ROS2 communications, we create the node by writing our node’s name, Campublisher(), then make the node spin using rclpy.spin(node), and finally shutdown ROS2 communications.
  ```
  if __name__ == '__main__': 
    main()
  ```
  Lastly we call the entry point to be executed. The entry point here is the main() function. It’s very important that you write all your code inside the main() function. Unlike ROS, we don’t need roscore to run nodes in ROS 2. SImply run the script by `$ python3 webcam_pub.py`. To check if the images are successfully  being published, open up a new terminal, source it using `$ source/opt/ros/eloquent/setup.bash` and run `$ros2 topic echo camera_msg`. 
  
* `Checkpoint 2`: After running the  publisher, in the separate window, `$ros2 topic echo camera_msg` should display the original image data. Please redirect to `ROS2_Middleware_Trail_Python_Turtle/README.md` for any questions or issues you encounter.

### ROS 2 Subscriber:

```
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
```
We import the same first two lines as we would for any other ROS 2 node, and then we import the same modules we imported in the publisher. 
```
class Camsubscriber(Node):
   def __init__(self):
     super().__init__('cam_subscriber')
     self.subscriber = self.create_subscription(Image, 'camera_msg', self.callback, 10)
     self.bridge = CvBridge()
     print("Converting ROS2 Image message to OpenCV iplimage...")
     time.sleep(2)
     print("Initializing display")
```
The constructor creates a subscriber with the same arguments as the publisher. Note that the topic name and message type used by the publisher and subscriber must match to allow them to communicate. 
```
 def callback(self, data): 
   sub_frames = self.bridge.imgmsg_to_cv2(data)
   cv2.namedWindow("User camera")
   cv2.imshow("User camera", sub_frames)
   key = cv2.waitKey(1) #Set the delay to 1 ms
   if key == 27: # Press esc key to exit 
       cv2.destroyAllWindows()
```
The subscriber’s constructor and callback don’t include any timer definition, because it doesn’t need one. Its callback gets called as soon as it receives a message. Here we define the callback, we first convert the ROS image msg back to cv2 image data, then we display to the user the webcam. 
```
def main():
    rclpy.init()
    node = Camsubscriber()
    try:
       rclpy.spin(node)
    except KeyboardInterrupt:
       pass

    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

```
Lastly, the main function is defined. Here we simply initialize ROS2 communications. We create the node by writing node’s name, `Camsubscriber()`, then make the node spin using `rclpy.spin(node)`, and finally shutdown ROS2 communications. We then call the entry point to be executed. The entry point here is the `main()` function. Again, it's essential that you write all this part of code inside the main() function.

  - Now you will run the Publisher/Subscriber nodes in the ROS 2 method. Navigate to the `setup.py` inside (`~/ROS2_Middleware_Trail_Python_Turtle/ROS2/src/webcam`). Look for lines:
  
```
 entry_points={
   'console_scripts': [
           'webcam_pub_node = webcam.webcam_pub:main',
           'webcam_sub_node = webcam.webcam_sub:main',
     ],
 },
```
Here we added an entry point for both the publisher and subscriber. Format of is: 
```
‘<name of the node> = <name of the package>.<name of the file>:main’
```
To run the nodes you must first, run the same steps in the `Build Package` section, except replace `interfaces` with  `webcam`. Then run `$ cd`, you can now run the node:

```
$ ros2 run webcam webcam_pub_node
```
In another terminal, first source it by `$ source/opt/ros/eloquent/setup.bash` then run:
```
$ ros2 run webcam webcam_sub_node
```

  - `Checkpoint 3`: There should be a window popup displaying the real time image from webcam. Please direct to `ROS2_Middleware_Trail_Python_Turtle/readme.md` for any questions or issues you encounter.
  

## Client/Service Nodes:

### Developing a Server node:

All of the files related to the python turtle module should be inside ` ~/ROS2_Middleware_Trail_Python_Turtle/ROS2/src /python_turtle/python_turtle/  ` directory. Let’s create a new file called `turtle_server.py`. The function of this script is to keep track of the remaining distance till the goal is reached. The concept will be further explained later in the tasks.  

```
import rclpy
from rclpy.node import Node
import math
import time
from interfaces.srv import Distance 
from std_msgs.msg import Float64
```
To start off, it is essential to import the rclpy and the Node modules. We will be using math and time so we must import those too. Next we will import the `Distance` service type we created earlier and lastly, `Float64` message type from std_msgs which is part of the common_interfaces(ready to use msg and srv files).

```
class Turtle_Service(Node):
  def __init__(self):
   super().__init__('<random node name>')
```
Since we are using OOP in ROS2, we must first create a class which inherits from the Node class.  Next we call super().__init__  which, in turn calls the Node class’s constructor and gives it your node name.

```
   self.srv = self.create_service(<service type>, '<service name', self.Find_Distance) 
   self.publisher = self.create_publisher(<message type>, '<topic name', 10)
   self.timer = self.create_timer(1, self.timer_callback)
   self.res = 0.0
```
Next we initialize the ROS 2 python service, using the self.create_service. Inside we have three arguments. We then initialize the publisher. Using  create_publisher we declare that the node publishes messages, over a topic, and lastly we set the “queue size” to 10. Next, we must create a timer with a callback to be executed, we set the period of suspension to a second and we call the Callback function. We will define the timer_callback function right after. We set the self.res to 0.0, but it will update when receiving new data. 

```
def Find_Distance(self, request, response):
   response.dist = <distance formula using the 4 requests>
   self.res = response.dist
   print('Turtle:', request.x1, request.y1 )
   print('Goal:' , request.x2, request.y2 ) 
   return response
```

Here we define the Find_Distance class for our server. It will receive the 2 sets of coordinates. Use the distance formula by utilizing the math library to find the remaining distance. Let the user know the current coordinates of both.  

```
def timer_callback(self):
   msg = <message type>()        
   msg.data = self.res
   self.publisher.publish(msg)
```

Here we are defining the timer_callback method which will contain the data to be published. The message will be a `Float64`, by publishing this every second we constantly update our client with the remaining distance. We call `publisher.publish(msg)` to publish our msg. 

```
def main(args=None):
  rclpy.init(args=args) 
  <random node name> = Turtle_Service() 
  try:
     rclpy.spin(<random node name>)
  except KeyboardInterrupt:
     pass 
  rclpy.shutdown()
```
Finally, the main function is defined. Here we simply initialize ROS2 communications, we create the node by writing our node’s name, `Talker()`, then make the node spin using rclpy.spin(node), and finally shutdown ROS2 communications.

```
if __name__ == '__main__':
    main()
```
Lastly we call the entry point to be executed. The entry point here is the main() function. It’s very important that you write all your code inside the main() function. *By filling up the `<>` sections above, you should have a complete turtle server.

  - `Checkpoint 4`: Run the code with $ python3 turtle_server.py , it should be running with no error messages. Please redirect to `ROS2_Middleware_Trail_Python_Turtle/README.md` for any questions or issues you encounter.
  
### Developing a Client node: 

At this point, we learned how to create a publisher, subscriber, and service node. Now let’s create the final type, the Client node. Create a new file, and name it `Client_turtle.py`.

```
import <ROS 2 python client library>
from rclpy.node import Node
import turtle
from interfaces.srv import <service type> 
from std_msgs.msg import <message type>
import time
```
Inside the file we will first import the needed libraries. 
```
<turtle name> = turtle.Turtle()
<turtle name>.shape("turtle")
<turtle name>.color("red")
<turtle name>.penup()
<turtle name>.goto(-300,-300)
<turtle name>.pendown()

< second turtle name> = turtle.Turtle()
< second turtle name>.shape("square")
< second turtle name>.color("black")
< second turtle name>.penup()
< second turtle name>.speed(0)
< second turtle name>.goto(200,100)
```
Next we initialize the turtle. We call two new turtles, give a name to both. 

```
def move():
   for i in range(2):
      <turtle name>.fd(15)
      <turtle name>.left(90)
      <turtle name>.fd(15)
      <turtle name>.right(90)
```
We define a simple move function, in which the turtle will move in a stair like motion. 
```

class Turtle_Client(Node):    
  def __init__(self):
   super().__init__(‘<random node name>’)
```
We first create a class which inherits from the Node class.  Next we call super().__init__  which, in turn calls the Node class’s constructor and gives it your node name. 

```
 self.subscriber = self.create_subscription(<message type>,'<topic name>',self.callback, 10)
 self.cli = self.create_client(<service type>, '<service name') 
 self.timer = self.create_timer(1, self.send_request)  
 self.req = Distance.Request()
```
The constructor creates a subscriber with the same arguments as the publisher. Note that the topic name and message type used by the publisher and subscriber must match to allow them to communicate. Same thing applies to the Client, it must have the same service type and service name as the server we created earlier. The subscriber’s constructor and callback don’t include any timer definition, because it doesn’t need one. Its callback gets called as soon as it receives a message. However, we do set a timer for the Client, so that we can update the coordinates sent every second.  

```
  def callback(self, msg):  
   print(msg.data)
   move()
```
Here we define the callback, it will receive data from the topic we published to and run the function.

```
  def send_request(self):
   self.req.x1 = float(<turtle name>.xcor())
   self.req.y1 = float(<turtle name>.ycor())
   self.req.x2 = float(<second turtle name>.xcor())
   self.req.y2 = float(<second turtle name>.ycor())
   self.future = self.cli.call_async(self.req)
```
Here we define the request definition. We set the first two requests are the turtle’s x and y pos and the third and forth as the goal’s x and y coordinates. We do so using the method turtle.xcor() which will return the x coordinate, etc… The last line of code is used to publish the request data. Similar to using the publish() function from the publisher object to publish on the topic.

```
def main():
 rclpy.init()
 <random node name> = Turtle_Client() 
 try:
   rclpy.spin(<random node name>)
   
 except KeyboardInterrupt:
   pass 
 node.destroy_node()
 rclpy.shutdown()
if __name__ == '__main__':
   main()
```
Finally, we have the main. The exact same as any other main we ran across in this trail, just give it a random node name of your choice, and we are calling the Node class `Turtle_Client()`. *By filling up the `<>` sections above, you should have a complete Client turtle.

  - `Checkpoint 5`: Run the `turtle_server.py` code, then in a new terminal run `$ python3 Client_turtle.py` , it should  bring up a window that displays the turtle move in a stair like motion, take a look at the shells, there should be the turtle’s coordinates displaying in one, and the remaining distance on the Client shell. Please redirect to `ROS2_Middleware_Trail_Python_Turtle/README.md` for any questions or issues you encounter.





