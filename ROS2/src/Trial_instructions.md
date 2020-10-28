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

* Checkpoint 1: The Build should be successful without any errors. Please direct to ROS2_Middleware_Trail_Python_Turtle/README.md for any questions or issues you encounter.


