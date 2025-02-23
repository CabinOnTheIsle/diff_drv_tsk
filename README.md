# ROS2 Custom Differential Drive Controller and Waypoint Navigation

This is a ROS2 task submission given by Accleration Robotics in order to implement a Differential Drive Robot Controller with RPM Computation and Waypoint Navigation.

## Given Objectives
- Develop a ROS 2 C++ node for a differential drive robot that:
- Processes velocity commands to compute RPM for each wheel.
- Publishes computed RPMs for motor control.
- Implements a separate Python script to navigate the robot between two waypoints using odometry and PID control.
- Works with Gazebo Harmonic or later and ROS 2 Humble or later.

## System Configuration:

* Ubuntu 22.04

* ROS2 Humble

* Gazebo Fortress

## Part 1: Differential Drive Controller (C++)

- Using the specified dependencies from the task, I was able create a Differential Drive Controller that:
    - Subscribes to /cmd_vel.
    - Publishes /left_wheel_rpm and /right_wheel_rpm messages.
    - Computed wheel speed into RPM:
      
        $$
        v_{\text{left}} = \frac{v_x - \bigl(\omega \times \frac{L}{2}\bigr)}{r}
        $$

        $$
        v_{\text{right}} = \frac{v_x + \bigl(\omega \times \frac{L}{2}\bigr)}{r}
        $$

        $$
        \text{RPM} = \omega_{\text{rad/s}} \times \frac{60}{2 \pi}
        $$

### This program was run using:
  [Launching the Simulated Robot...]
  
  `ros2 run diff_drv_tsk diff_drive_controller`
  
  `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"`
  
  `ros2 topic echo /left_wheel_rpm`
  
  `ros2 topic echo /right_wheel_rpm`

![Task 1](/task_demo_videos/Part_1.gif)

## Part 2: Waypoint Navigation (Python Script)

- Using python I was able to create a script that receives --ros-args to pass in the waypoint 1 and 2 and also PID values to fine tune the robot.
  - Subscribes /diff_cont/odom to get robot's current odometry. (Simulated Robot odom messages were being passed through /diff_cont/odom instead of /odom)
  - Publishes /cmd_vel to give the linear and angular velocity messages.
  - Implemented PID tuning.
  - Waypoint 1 and 2 parameters to traverse the robot sequentially and stops once waypoint 2 has been reached.

### This program was run using:
  [Launching the Simulated Robot...]
  
  `ros2 run diff_drv_tsk diff_drive_controller`
  
  ``ros2 run differential_drive_controller waypoint_navigation.py --ros-args \
    -p waypoint_1_x:=2.0 -p waypoint_1_y:=1.0 \
    -p waypoint_2_x:=4.0 -p waypoint_2_y:=3.0 \
    -p kp:=0.30 -p ki:=0.0003 -p kd:=0.20``
    
![Task 2](/task_demo_videos/Part_2.gif)
