# Drone Vision

AR.Drone camera based tracking. In the actual phase it has 4 modes, a green laser pointer tracking, which can use any of the two cameras, a led strip follower with the bottom camera (like a line follower robot, but with a drone), and a path follower with user defined paths.

## Getting Started

This will give you an overview of the project.

### ROS
The foundation of the project is based on [ROS(Robot Operating System)](http://www.ros.org/). It manages the communication of the algorithm on the PC with the drone, you can learn more about it on their [wiki page](http://wiki.ros.org/).
In the ROS environment there is one thing called packages, you can think of them as libraries, that make one's life easier. This project uses some of them, a interesting one being used is the [ardrone_autonomy package](https://github.com/AutonomyLab/ardrone_autonomy). It manages the AR.Drone SDK, making the control of the drone by software to be very easy.

### Python(OpenCV)
Since this is a camera based tracking project it needs to do image processing, which can be quite easy with [OpenCV](https://opencv.org/). The image processing algorithm it is using right now is based on a color tracking approach. It creates a mask based on a RGB threshold then applies it on the original image, as a result we have only the green dot. As a final step it recognizes the green dot as a circle and sends the x,y position to the control algorithm.

### Control(PID)
For the tracking it uses a simple [PID controller](https://en.wikipedia.org/wiki/PID_controller). The constants are optimized for the AR.Drone we used, so you may need to change them (just one line of code in controller.py).


## Prerequisites

The things you need to run the project and how to install them. The project was developed and tested in **Ubuntu 16.04 LTS**, if your operating system is different it may need some workarounds.

### ROS Kinetic
To install ROS Kinetic you just need to follow the instructions in the [ROS installation page](http://wiki.ros.org/kinetic/Installation/Ubuntu).

### ardrone_autonomy package
```
sudo apt-get install ros-kinetic-ardrone-autonomy
```

### joy package
```
sudo apt-get install ros-kinetic-joy ros-kinetic-joystick-drivers
```

### cv_bridge
```
sudo apt-get install ros-kinetic-cv-bridge
```

### Python 2.7
If you are using Ubuntu 16.04 Python comes pre-installed.

You also need to install Qt.
```
sudo apt-get install python-pyside
```

### OpenCV 3.4
To install OpenCV you just need to follow the instructions in this [page](https://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/).

## Install
To install this package run these commands in the terminal.

Navigate to your catkin workspace. If you do not have a catkin workspace, follow this [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
```
cd ~/catkin_ws/src
```

Clone this repository
```
git clone https://github.com/nishiratavo/Drone_vision.git
```

Once it is completed
```
cd ~/catkin_ws
catkin_make
```

## How to use
First turn on the AR.Drone and connect to it via wifi.
Then open the terminal and navigate to your Drove_vision source folder (Probably this path if you're using catkin workspace).
```
cd ~/catkin_ws/src/Drone_vision/src/
```
Initialize the program with the bash script.
```
./drone.sh
``` 
Wait for everything to open and then select the mode.

Wait for the camera's image to open (the path follower doesn't use the camera) and then send the takeoff command.

To land the drone send the land command.

In case of emergency send the emergency command.

### Front Camera

Wait for the drone to stabilise then use the green pointer and the drone will follow.

### Bottom Camera

Wait for the drone to stabilise and wait some seconds for the drone to achieve his final altitude. After that use the green pointer and the drone will follow.

### Line Follower

Wait for the drone to stabilise and it will begin to follow the led strip.

### Path Follower

Wait for the drone to stabilise and send the path it should follow.

The path coordinates are in meters and they say how many meters the drone should fly in each direction.

Example :

1,0,0 => one meter to the left

0,1,0 => one meter forward

0,0,1 => one meter up

To send more than one vector you need to separate the vectors with one space.

To fly in a square beginning in the bottom left corner send :
```
0,1,0 -1,0,0 0,-1,0 1,0,0
```

### Keyboard mode

Use the keyboard to control the drone.

W -> forward
A -> left
S -> backward
D -> right
Q -> counter clockwise
E -> clockwise
R -> up
F -> down

### Save data

Click the save data button to start saving data, click again to stop. It will generate a data.txt file in the package directory. 

**CAUTION**

If you press the save data button again it will overwrite the previous data.


## Authors

* **Gustavo Nishihara** - [Github](https://github.com/nishiratavo)
* **Igor Novais**  - [Github](https://github.com/igor98)

