# simple_manipulator -> bot_movit 

2 DOF simple bot using MoveIt2  

## Launch Command:

### Change the library for running GUI command with PyQt  
Go to: `~ros_ws/src/bot_movit/launch`  
Open the file: `gedit bot.launch.py`  

At line 119:  
"( "/home/naor/Desktop/naor/study/naor_task2/install/bot_movit/lib/bot_movit", "qt_gui.py")],"
Change this line to your global path.  

## Running the Workspace  
Open the first terminal and execute the following commands:  
1) `colcon build`  
2) `source install/setup.bash`  
3) `ros2 launch bot_movit bot.launch.py`  

## Running the Services (Two Methods)  

### Using PyQt  
The PyQt GUI will open at launch if you have updated the global location.  
If PyQt does not open, start the service manually:  
Open another terminal and run:  
`cd ~/ros2_ws/src/bot_movit/scripts`  
`python3 qt_gui.py`  
Inside the GUI, provide the desired position for the bot.  

### Using Direct Service Command  
Open another terminal and run:  
`ros2 service call /move_robot bot_movit/srv/SetJointP "{joint_positions: [float, float]}"`  

### using the node 
Open another terminal and run:  (dont forget to source )
ros2 run bot_movit node_run.py --ros-args -p joint1:=x -p joint2:=x


### Service Location  
The move bot service is located at:   (dont forget to source )
`~ros2_ws/src/bot_movit/src/move_bot_service.cpp`  



### Bot Movement  
- The base link can rotate from 180° to -180° (equivalent to 3.14 to -3.14 radians).  
- The second link can rotate from 90° to -90° (equivalent to 1.57 to -1.57 radians).  

## Additional Notes  
This is my first time using MoveIt2.  
The initial setup did not work as expected, so I explored other projects and packages for reference.  

I have also worked on other projects:  

1) **DIY Drone Project** (Thesis in Mechatronics)  
   - Repository: [https://github.com/naorwaiss/diy_drone](https://github.com/naorwaiss/diy_drone)  
   - This is not the full dataset; I worked locally.  
   - Includes PCB design with EasyEDA, ROS2, and PlotJuggler for IMU filtering.  
   - The drone is in the PID tuning phase. ROS helped in building notch filters and other critical flight filters.  
   - I can provide ROS bag data and other resources if needed.  

2) **AI GPS ROS2 Project**  
   - Repository: [https://github.com/naorwaiss/Ai_gps_ros2](https://github.com/naorwaiss/Ai_gps_ros2)  
   - This project integrates MAVROS data, real drone telemetry, motor RPM readings, and optical flow from an IR camera for drone localization.  
   - Implemented with PyTorch and non-linear functions.  

3) **FHSBG ROS Drone**  
   - Repository: [https://github.com/naorwaiss/fhsbg_ros](https://github.com/naorwaiss/fhsbg_ros)  
   - This was my first ROS2 project, focusing on image processing with ArUco markers for drone navigation.  

Other projects:  

1) **VTOL Drone (3D Printed)**  
   - I worked on modifying an existing VTOL flight controller to a more cost-effective solution.  
   - Based on [MiniHawk-VTOL](https://github.com/StephenCarlson/MiniHawk-VTOL).  
   - I had discussions with the project creator regarding modifications.  

2) **RF-Based Combined Controller for Drones & RC Planes**  
   - Used Arduino, Raspberry Pi, and ROS2's `joy` node to control both drones and fixed-wing planes with a unified RF controller.  

3) **Ground Robot (Tank Bot with ROS2)**  
   - This is another part of my thesis.  
   - The bot is already operational and will eventually carry the drone.  
   - Initially, I started building a TF map with Nav2, but the project paused due to drone development.  

Thanks for your time!
