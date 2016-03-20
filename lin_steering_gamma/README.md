# lin_steering_gamma
Steering algorithm using odometry.  For mobot, "odom" is perfect.  Neede to relax this
assumption.

If start with good initial conditions, linear steering algorithm will do a good job.
Can compare feedback controller to open-loop controller.

## Example usage
Start up gazebo, load the mobot model, desired-state publisher, desired-state client,
and linear-steering algorithm.

`roslaunch gazebo_ros empty_world.launch`

`roslaunch mobot_urdf mobot_w_lidar.launch`

Drag in starting pen

`roslaunch lin_steering_gamma lin_steering_gamma.launch`





`rosrun mobot_pub_des_state_gamma lidar_alert`

`rosrun mobot_pub_des_state_gamma estop_alert`

`rosrun mobot_pub_des_state_gamma pub_des_state_main_gamma`

`rosrun mobot_pub_des_state_gamma pub_des_state_path_client_gamma`

`rosrun lin_steering_gamma lin_steering_wrt_odom_gamma`


    
