# coordinator_gamma

Top-level node that is a client of: navigator_gamma, object_grabber_gamma, and object_finder_gamma.
Waits for an Alexa trigger, then starts entire process of: navigation to table,
recognition of object, grasp of object, return to home

## Example usage
Start up the simulator with:
`roslaunch baxter_launch beer_world.launch`
Enable the robot with:
`rosrun baxter_tools enable_robot.py -e`
Start up CWRU nodes with:
`roslaunch baxter_launch_files baxter_nodes.launch`

Then trigger the behavior with:
`rostopic pub Alexa_codes std_msgs/UInt32 100`



    
