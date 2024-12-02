# **What is ROS?**
---

ROS (Robot Operating System) is an open-source software framework designed for writing robot software, much like how Sherlock Holmes uses his toolkit to solve mysteries. Just as Holmes relies on his magnifying glass and sharp reasoning to uncover hidden details, ROS equips developers with the essential tools and libraries to navigate the complexities of robot development. Its primary goal is to help developers reuse software across the globe, making it easier to create complex robot behaviors on a variety of platforms.

ROS offers services like hardware abstraction, low-level device control, message-passing between processes, and package management. With its set of tools and libraries for obtaining, building, writing, and running code across multiple computers, ROS enables developers to seamlessly construct sophisticated robotic systems. While it shares similarities with other robotic frameworks like Player, YARP, and Microsoft Robotics Studio, its open-source nature and active community support make it stand out as the go-to toolkit for robotic development.

This Week will cover the core ROS 2 concepts including working with nodes, topics, services, parameters and more through practical examples using the ROS 2 Humble release.

<img src="W0_Images/ros2.png " width=400 height=100>

---
## **Basics of ROS**

Before we get into the action, it’s essential to understand the basics of ROS. It’s always best to write the code yourself rather than copying and pasting, as it helps you grasp the concepts more thoroughly. Python is recommended over C++ for beginners because of its easier syntax and readability.

To get started, you'll need to create a ROS workspace and navigate the ROS filesystem. This may seem like an investigation at first, but don’t worry — just like Holmes solves complex cases step by step, you'll get there. We’ll walk you through some key tools, and the next few tutorials will focus on setting up your environment.


To start off, these two tutorials will cover aspects such as **creating a ROS Workspace** and
[**navigating the ROS Filesystem**](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html). 
To help you a little bit more, we will also explain you some command line tools and will highlight the sections you should focus more when you open these links......

Sections to focus in "Configuring your ROS environment":

- Sourcing of setup.bash file : ```source /install/setup.bash  # donot run ```
The command source /install/setup.bash is used in ROS 2 to set up your environment so that your terminal can correctly locate ROS 2 packages, executables, and dependencies.Ensures your workspace and its packages are available to ROS 2 commands like ros2 run, ros2 topic, etc.
Dont worry if you dont get it now , you will understand , when we practically use it.
- Its an advise to not panic if you don't grasp things in one go, try to go completely by this twice or thrice for more clarity and try to connect dots while reading the second or third time.

Next, we shall look at packages.

### **What is a package?** 
---
In the world of ROS, a package is the key to organizing your programs, much like a case file contains all the details needed for investigation. Every ROS program you write is contained within a package, which can include source code (either Python or C++), configuration files, launch files, and dependencies. The package structure is organized into directories such as:

- launch: Contains launch files.
- scripts: Contains source files (Python).
- package.xml: Information about the package and its dependencies.

In ROS2, Python or C++ are typically used for writing the script files in the package. If you want to move forward, understanding how to work with packages will be essential, just like Sherlock understanding the importance of each piece of evidence.

### **Colcon**
---
In the ROS ecosystem, software is organized into many packages. Unlike workflows where a developer works on just one package at a time, ROS developers often handle multiple packages simultaneously. To manage this, Colcon is used as a build tool to compile all the packages together with a single command. It's akin to Holmes managing multiple clues from different locations and tying them all together to form a cohesive solution.

> **Note**: remember ` s++ `  in CS101 , colcon is also a similar build tool , but builds  several linked pacakges/files at once.

To install and configure Colcon, run the following command:
```bash
sudo apt install python3-colcon-common-extensions
```
#### **Why Colcon?**

Imagine you’re working on a robot project, and you’ve divided the software into multiple packages:
- Package A: Reads sensor data.
- Package B: Processes that data.
- Package C: Controls the motors based on the processed data.

These packages depend on each other:
- If you make a change to Package A, Package B might need to adapt to the updated data structure.
- If you modify Package B, you’d need to ensure Package C works correctly with the updated processing logic.

Instead of manually building each package one by one and resolving dependencies yourself, Colcon automates this process. It:
- Detects all the packages in your workspace.
- Figures out the correct build order based on dependencies.
- Compiles everything with a single command: colcon build.

#### The Sherlock Holmes Analogy

Colcon’s role is similar to Sherlock Holmes solving a mystery:
- Holmes gathers clues (analogous to packages).
- Each clue is part of a larger puzzle (packages depend on and contribute to the whole system).
- Holmes analyzes and ties them all together to form a complete solution (Colcon builds all the interdependent packages into a cohesive system).

### **Create a workspace**
---
A workspace in ROS 2 is like Sherlock Holmes' case file—a place where all the necessary clues (ROS 2 packages) are stored. Before starting your investigation, you must "source" your ROS 2 workspace, just like Holmes prepares his tools before solving a case. This step ensures your terminal knows where to find the packages you need.

For clarity and organization, it's best practice to create a new directory for each new workspace—just as Holmes keeps each case separate to avoid confusion.



```bash
mkdir -p erc_ws/src
cd erc_ws/src
```
From the root of your workspace (erc_ws), you can now build your packages using the command:

```bash
cd ..
colcon build
```
Now, let’s add the erc_ws path. Execute the following command:

```bash
echo "source ~/erc_ws/install/setup.bash" >> ~/.bashrc
```

When you make changes to your .bashrc file, run:
```bash
source ~/.bashrc
```
This command applies the changes immediately without restarting your terminal.


### **Create a new package**
---
A package is an organizational unit for your ROS 2 code. If you want to be able to install your code or share it with others, then you’ll need it to be organized in a package. With packages, you can release your ROS 2 work and allow others to build and use it easily.

Package creation in ROS 2 uses ament as its build system and colcon as its build tool. You can create a package using either CMake or Python.

Recall that packages should be created in the src directory, not the root of the workspace. So, navigate into ```erc_ws/src```, and run the package creation command:

```bash
cd src
ros2 pkg create --build-type ament_python week0_tutorials
```

### Nodes
---
In ROS, each program is called a node—think of it like a clue in a Sherlock Holmes mystery. Each node does a specific job, like collecting evidence or analyzing a suspect. These nodes talk to each other through topics, which are like channels for sharing information.

For example, one node might capture images from a camera, sending them to another node for analysis, like Holmes looking over a piece of evidence. Then, the second node might send a command to a third node, like Holmes taking action based on his findings. The nodes communicate by sending messages—one node can publish a message to a topic, or another node can subscribe to a topic to receive the information, helping to solve the task, just like Sherlock solving the case.

> **Point to Ponder**: How is a Node differnt from a package ??



![Nodes-TopicandService](https://github.com/user-attachments/assets/5ba624ed-6b3d-4de1-b738-a2fbfc2d63a4)

### Topics
---
In ROS, a topic acts like a channel for exchanging data, with nodes serving as either Publishers, sending data, or Subscribers, receiving it. Each topic has a specific message type, similar to how Sherlock Holmes might focus on a specific type of clue in a case. To communicate effectively, both publishers and subscribers must handle the same message type.

Nodes can create publishers and subscribers in any ROS-supported language. When a node publishes data, it notifies the ROS master, while a subscriber asks the master where to find the data—like Sherlock and Watson sharing clues. Just as Holmes tracks multiple leads, a node can have multiple publishers and subscribers for different topics.



![Topic-MultiplePublisherandMultipleSubscriber](https://github.com/user-attachments/assets/3a6fd12b-35f9-4236-9e07-82b41e87b22a)

PS:

- ```rqt_graph```:
Reveals communication between nodes through topics.
Essential for understanding node interactions.

- ```ros2 topic echo```:
Displays real-time data published on a specific topic.
Useful for monitoring topic content.

- ```ros2 topic pub```:
Enables manual publication on desired topics.
Handy for testing and injecting data into the system.

Try running these commands and viewing the output.

<img src="W0_Images/mind-palace-sherlock-holmes.gif ">

## Introducing TurtleSim (hehe remember me from CS101)
---
To demonstrate how to run nodes, let us run 'turtlesim_node' node from a pre-installed package, 'turtlesim':

To run the 'turtlesim_node' node, run this **in a terminal**:
```bash
ros2 run turtlesim turtlesim_node
```
You'll see a new turtlesim window.

![turtlesim](https://github.com/user-attachments/assets/3cb5f1c4-f091-4677-9080-fde000136fe6)

#### How to move it ??

Open a new terminal

```bash
ros2 run turtlesim turtle_teleop_key
```

Arrange three windows: a terminal running turtlesim_node, a terminal running turtle_teleop_key, and the turtlesim window. Ensure you can see the turtlesim window while the turtle_teleop_key terminal is active to control the turtle in turtlesim.

Use the arrow keys on your keyboard to control the turtle. It will move around the screen, using its attached “pen” to draw the path it followed so far.

After playing a bit with teh turtle , lets ponder whats happening ?

for that first lets see the "command-onology" of ROS here 

`ros2 run turtlesim turtle_teleop_key` : 
-  `ros2 run` runs a exceutable ROS file
-  `turtlesim` the package which have to run
-  `turtle_teleop_key` The name of the executable node to be run , here 'turtle_teleop_key' which is responsible in controling the turtle

summing up below 
```bash
ros2 run <package name> <executable node of the package> 
```

## Learning about launch files from turtlesim mimic example
---

ROS 2 Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.

Create a new directory in ```erc_ws/src/week0_tutorials``` to store your launch files:
```bash
cd week0_tutorials
mkdir launch
```

Let’s put together a ROS 2 launch file using the turtlesim package and its executables.

```bash
cd launch
touch turtlesim_mimic_launch.py
chmod +x turtlesim_mimic_launch.py
```
Open this directory with vs code (if you want to use vs codium then instead of `code` use `codium` ) using the following command
```bash
code .
```

Copy and paste the complete code into the ```launch/turtlesim_launch.py``` file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```
Take your time and understand the the above , how we can calling the multiple node files with there respective packages.

Then, hit <kbd>CTRL</kbd>+<kbd>S</kbd>, to save the changes to the file, then close the window.

To run the launch file created above, enter into the directory you created earlier and run the following command:

```bash
ros2 launch turtlesim_mimic_launch.py
```
Now What , just two identical turtles ... hmm lets make them to move 

### Publishing a message 

To see the system in action, open a new terminal and run the ros2 topic pub command on the /turtlesim1/turtle1/cmd_vel topic to get the first turtle moving 

- the topic /turtlesim1/turtle1/cmd_vel is responsible for moving the turtle (how do i know that , from the offical documentation , dont worry if you dont understand it know , you will get it , as we go on )

```bash
cd ~
cd erc_ws/src/week0_tutorials/launch
```

```bash
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```
Lets decode the command: 
- `ros2 topic pub -r 1` publishes a message to the topic at rate 1Hz
- `/turtlesim1/turtle1/cmd_vel` the topic to which the message is published
-  `geometry_msgs/msg/Twist` the type of the message being sent
-  `linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}` the data that has to be published

  This was a easy example of pulisher and subscriber using the builtin topics of ROS , now later in this week we will know , how to set the Publisher-Subscriber Interface from scratch.

![image](https://github.com/user-attachments/assets/537746fc-d2f7-47f4-97b7-709e3b524dfc)


File structure is something like this

```bash
erc_ws
  |_ Install
  |_src
       |_week0_tutorials(pkg)
               |_src
               |_scripts
               |_launch
               |_config
```
*Launch files in a particular package can launch nodes from other packages as well as launch files from other packages.

```PS:``` In the turtlesim_launch we run the launch file from the same directory, if we want to run it from any directory using the generalized command, i.e.
```bash
ros2 launch <package name> <launch file name> 
```
 then we have to specify the path in setup.py and then go to workspace directory and then colcon build, and run this. Here we exceuted the run file directly from the directory, though we will do this later in the end of the tutorial in the ```pubsub.launch.py``` case.



## Publisher-Subscriber Interface <a name="PubSub"></a>
---

Message passing in ROS happens with the Publisher-Subscriber Interface provided by ROS library functions.

Creating a publisher or subscriber node is just like creating any other node. <br />

1. Go to the package where you want to create these nodes 

2. Make a new directory ```scripts```
 
3. Create python script files for a publisher and a subscriber


### Create a executeable python file

Navigate into ```erc_ws/src/week0_tutorials/week0_tutorials``` and then create a python file 

```bash
cd ~
cd erc_ws/src/week0_tutorials/week0_tutorials
touch talker.py
chmod +x talker.py #Making the python file executable
```

### Writing a simple Publisher Node <a name="Publisher"></a>

This is a basic publisher node python script ```talker.py```  (taken from the official ROS tutorials from the website, and comments are added to help you understand the working of each line):

To open VS code.
```bash
code .
```
Paste the following in the ```talker.py```

```python
#!/usr/bin/env python
import rclpy
from std_msgs.msg import String

def timer_callback(timer, i):
    # Create a String message
    msg = String()
    msg.data = 'Hello World'

    # Publish the message using the global publisher
    publisher.publish(msg)

    # Print a message indicating what is being published
    print('Publishing: "%s"' % msg.data)

def main(args=None):
    # Initialize the ROS 2 system
    rclpy.init(args=args)

    # Create a ROS 2 node named 'minimal_publisher'
    node = rclpy.create_node('minimal_publisher')

    # Create a global publisher for the 'topic' with a message type of String
    global publisher
    publisher = node.create_publisher(String, 'topic', 10)

    # Set the timer period to 0.5 seconds
    timer_period = 0.5

    # Initialize a counter variable
    i = 0

    # Create a timer that calls the timer_callback function every timer_period seconds
    timer = node.create_timer(timer_period, lambda: timer_callback(timer, i))

    # Increment the counter
    i += 1

    try:
        # Start spinning the ROS 2 node
        rclpy.spin(node)
    finally:
        # Destroy the node explicitly when done spinning
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()

        # Shutdown the ROS 2 system
        rclpy.shutdown()

# Entry point to the script
if __name__ == '__main__':
    # Call the main function if this script is the main module
    main()
```
again take your time , look into the code and try to understand ( you can always use the internet/chatgpt .. hehe)

Then, hit <kbd>CTRL</kbd>+<kbd>S</kbd>, to save the changes to the file.

### Add dependencies
> **Quick Question: what are dependencies?** - dependencies are packages or libraries that a ROS package needs to compile and run. These are often declared in the package.xml file or the CMakeLists.txt file of a package.

Navigate one level back to the ```erc_ws/src/week0_tutorials``` directory, where the setup.py, setup.cfg, and package.xml files have been created for you.

```bash
cd ..
code .
```

Open package.xml with your text editor. Add the following dependencies corresponding to your node’s import statements:

```bash
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```
Then, hit <kbd>CTRL</kbd>+<kbd>S</kbd>, to save the changes to the file.

This declares the package needs rclpy and std_msgs when its code is executed.


## Add an entry point

Open the setup.py file, and add the following line within the console_scripts brackets of the entry_points field:

```bash
entry_points={
        'console_scripts': [
                'publisher = week0_tutorials.talker:main',
        ],
},
```
Then, hit <kbd>CTRL</kbd>+<kbd>S</kbd>, to save the changes to the file.


### Writing a simple Subscriber Node <a name="Subscriber"></a>
---

Make the listener.py file similarly as we have done for talker.py.

This is a basic subscriber node python script ```listener.py``` (taken from the official ROS tutorials from the website, and comments are added to help you understand the working of each line):

```bash
cd ~
cd erc_ws/src/week0_tutorials/week0_tutorials
touch listener.py
chmod +x listener.py
```

Paste the following in the ```listener.py```

```python
#!/usr/bin/env python
import rclpy
from std_msgs.msg import String

def listener_callback(msg):
    print('I heard: "%s"' % msg.data)

def main(args=None):
    # Initialize the ROS 2 system
    rclpy.init(args=args)

    # Create a ROS 2 node named 'minimal_subscriber'
    node = rclpy.create_node('minimal_subscriber')

    # Create a subscription to the 'topic' with a message type of String
    subscription = node.create_subscription(String, 'topic', listener_callback, 10)

    # Prevent unused variable warning
    subscription

    try:
        # Start spinning the ROS 2 node
        rclpy.spin(node)
    finally:
        # Destroy the node explicitly when done spinning
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()

        # Shutdown the ROS 2 system
        rclpy.shutdown()

# Entry point to the script
if __name__ == '__main__':
    # Call the main function if this script is the main module
    main()
```

Then, hit <kbd>CTRL</kbd>+<kbd>S</kbd>, to save the changes to the file.

Now similarly for subscriber add entry points as in publisher node.

```bash
cd ..
code .
```
Now change the following in ```setup.py```

```bash
entry_points={
        'console_scripts': [
                'publisher = week0_tutorials.talker:main',
                'subscriber = week0_tutorials.listener:main',
        ],
},
```
Then, hit <kbd>CTRL</kbd>+<kbd>S</kbd>, to save the changes to the file.

## Build and run

You likely already have the ```rclpy``` and ```std_msgs``` packages installed as part of your ROS 2 system. It’s good practice to run rosdep in the root of your workspace (erc_ws) to check for missing dependencies before building:

```bash
cd ..
```
repeet till erc_ws directory 

```(optional)```

```bash
rosdep install -i --from-path src --rosdistro humble -y
```



Finally, go to erc_ws and build the package

```bash
colcon build
```

Now source the setup files

```bash
source install/setup.bash
```

Now run the publisher node:

```bash
ros2 run week0_tutorials publisher
```

Similarly run the subscriber node in a new terminal. Remember to source the workspace if you haven't already.

```bash
ros2 run week0_tutorials subscriber
```

You can see that 'Hello World: n' is being printed. The Publisher Node is up and running!

You can see that 'I heard: "Hello World: n' is being printed. The Subscriber Node is running as well.

Note that once you stop running the Publisher Node ( Press `Ctrl`+`C` while you're in the terminal that is running the Publisher Node), the Subscriber Node stops running as well. 

### Running the publisher and subscriber using a launch file
---

Create a file ```pubsub.launch.py``` in the ```launch``` folder of ```erc_ws/src/week0_tutorials``` 

```bash
cd launch
touch pubsub.launch.py
code .
```

Add the following code.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='week0_tutorials',
            executable='publisher',
        ),
        Node(
            package='week0_tutorials',
            executable='subscriber',
        ),
    ])
```

```package``` refers to the name of the package in which the node is present in, ```executable``` is the name of the node file

```bash
cd ..
code .
```
Now add the following line in ```setup.py``` in the ```data_files```


```python
(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'pubsub.launch.py'))),
```

and add the follwing on the top of ```setup.py```

```python 
import os
from glob import glob
```

On executing ```ros2 launch week0_tutorials pubsub.launch.py```, you will be able to see **Publisher** and **Subscriber** in the list of Nodes. 

While the system is still running, open a new terminal and run ```rqt_graph``` to get a better idea of the relationship between the nodes in your launch file.

## Now, enough chatter. Time to do ...

Create a new package ```sherlock``` in ```erc_ws```, which will contain three nodes and a launch file.

1) The first node will publish the following text to the topic ```listen_1```.

	**I am not a psychopath, Anderson.**

2) The second node will publish the following text to the topic ```listen_2```.

 	**I am a high-functioning sociopath.**

3) The third node will subscribe to ```listen_1``` and ```listen_2```.

4) The third node should display the following statement on the terminal at some frequency.

	**I am not a psychopath, Anderson. I am a high-functioning sociopath.**

5) The launch file will launch all the three nodes.

<img src="W0_Images/Sociopath.gif" width=400 height=220>

Have fun !
