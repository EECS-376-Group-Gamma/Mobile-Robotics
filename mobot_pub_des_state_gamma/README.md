# mobot_pub_des_state_gamma
This package illustrates a desired-state publisher that creates and publishes
sequences of states that are dynamically feasible and which lead a robot through
a sequence of subgoals, treated as a polyline.  The publisher exploits the
traj_builder library to construct dynamically-feasible trajectories.  It accepts
subgoals via the service "append_path_queue_service", which appends subgoals to
the current queue of subgoals.  The service "estop_service" invokes an e-stop
state, causing the robot to come to a halt with a dynamically-feasible trajectory.
The service "clear_estop_service" allows the robot to resume visiting subgoals.
The current path queue can be flushed via the service "flush_path_queue_service".
When there are no subgoals left in the queue, the robot halts at its lasts subgoal.
It will resume motion if/when new subgoals are added to the queue.

## Example usage
`roslaunch gazebo_ros empty_world.launch`

`roslaunch mobot_urdf mobot_w_lidar.launch`

`rosrun mobot_pub_des_state_gamma open_loop_controller_gamma`

`rosrun mobot_pub_des_state_gamma lidar_alert`

`rosrun mobot_pub_des_state_gamma estop_alert`

`rosrun mobot_pub_des_state_gamma pub_des_state_main_gamma`

`rosrun mobot_pub_des_state_gamma pub_des_state_path_client_gamma`


Perform e-stop and e-stop reset with:

`rosservice call estop_service`

`rosservice call flush_path_queue_service`

`rosservice call clear_estop_service`