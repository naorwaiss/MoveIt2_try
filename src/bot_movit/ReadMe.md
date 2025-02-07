

# simple_manipulator - > bot_movit 

    2 DOF simple bot with useing movit2  

## launch command:

### change at the lib for runing gui command with pyqt 
    got to:
    ~ros_ws/src/bot_movit/launch
    gedit bot.launch.py

    at line 119:
            "( "/home/naor/Desktop/naor/study/naor_task2/install/bot_movit/lib/bot_movit", "qt_gui.py")],)"
    cahnge this line to your global patch 


## runing the work sapce  
    at the first terminal 
    1) colcon build 
    2) source install/setup.bash
    3) ros2 launch bot_movit bot.launch.py

## to run the servics have 2 metude 
### useing the pyqt 
    the py qt open  at the launch if you change the global location 
    if the pyqt didnt open go to service and run it menualy:
    open another terminal:
    ~/ros2_ws/src/bot_movit/scripts
    and then python3 qt_gui.py

    at the qt gave the rad for the bot 

### using direct service command 
    open another termional 
    ros2 service call /move_robot bot_movit/srv/SetJointP "{joint_positions: [float, float]}"




### the servis locate at:
    ~ros2_ws/src/bot_movit/src/move_bot_service.cpp


### bot movment:
    the base link can spin from 180 <->-180 aka (3.14 <-> -3.14) 
    the secend link spin from 90<->-90 aka (1.57<->-1.57)





### another note 
    this is my first time that i use movit2 
    the deco didnt work at the first day of the assyment so i try to find another project and pkg at copy things 

    i like to add another roject that i do 
    https://github.com/naorwaiss/diy_drone - > my thisis at mechatronics -> this isnt all the data of work -> i work localy (i do at this work some pcb with easyeda and using ros2 and plotjuggler to build the filter of the imu ) -> the drone is at the pid part and the ros part help us to build the notch filter and another flter that important to the flght (i can see you rosbag and another data )
    https://github.com/naorwaiss/Ai_gps_ros2 -> some project that i try to make with data from mavros with real drone + data from the motor (rpm of each motor) and optical flow with ir camera that i build to get the location of the drone - > i use pytorch and not linear function 
    https://github.com/naorwaiss/fhsbg_ros - > this is my last project of drone that flight with image processing with aruco and ros2 (my first project with ros2)

    i have more project that i can gave you image about and explain
    1) vtol drone that i print at 3d - > i try to cahnge the flight controller of the aricraft to somthing moe cheep base of his project https://github.com/StephenCarlson/MiniHawk-VTOL -> i make some converstation with the make of the project 
    2) combine controller base rf comunicated for read drone and rc plane with fix wind -> some project with arduino pi and joy node ros2 to control real drone and plain with exotic controller
    3) gnd bot - > another part of my thisis that base on ros2 and arduin - this is a tank bot (it drive alrady) that at the end cary the drone -> at the project i start to build the tf map with nav2 but stop becuse of the build of the drone 
    
    
thanks for your time 


