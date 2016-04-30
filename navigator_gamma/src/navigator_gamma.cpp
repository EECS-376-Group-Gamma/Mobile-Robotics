// example_action_server: a simple action server
// Wyatt Newman

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <navigator_gamma/navigatorAction.h>
#include <mobot_pub_des_state_gamma/path.h>  //DELETE IF NOT USING OPEN LOOP
#include <geometry_msgs/PoseStamped.h> //DELETE IF NOT USING OPEN LOOP
#include <geometry_msgs/Twist.h>

class Navigator {
private:
    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation
    actionlib::SimpleActionServer<navigator_gamma::navigatorAction> navigator_as_;
    ros::ServiceClient open_loop_client;
    ros::Publisher twist_commander;
    
    // here are some message types to communicate with our client(s)
    navigator_gamma::navigatorGoal goal_; // goal message, received from client
    navigator_gamma::navigatorResult result_; // put results here, to be sent back to the client when done w/ goal
    navigator_gamma::navigatorFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client

    geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);
    int navigate_home();
    int navigate_to_table();
    int navigate_to_pose(geometry_msgs::PoseStamped goal_pose);

public:
    Navigator(); //define the body of the constructor outside of class definition

    ~Navigator(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<navigator_gamma::navigatorAction>::GoalConstPtr& goal);
};



Navigator::Navigator() :
   navigator_as_(nh_, "navigatorActionServer", boost::bind(&Navigator::executeCB, this, _1),false) {
    ROS_INFO("in constructor of navigator...");
    // do any other desired initializations here...specific to your implementation
    open_loop_client = nh_.serviceClient<mobot_pub_des_state_gamma::path>("append_path_queue_service");
    twist_commander = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    navigator_as_.start(); //start the server running
}

geometry_msgs::Quaternion Navigator::convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}


int Navigator::navigate_home() {
    /* OPEN LOOP APPROACH */

    
    // MOVE BACKWARDS FOR 2 SECONDS
    ROS_INFO("MOVING BACKWARDS");
    geometry_msgs::Twist twist_cmd;
    twist_cmd.linear.x = -0.5;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = 0.0;
    double timer = 0.0;
    ros::Rate loop_timer(1/0.01);

    for(double i = 0.0; i < 2.0; i += 0.01){
    //for(int i = 0; i < 2; i++){
        twist_commander.publish(twist_cmd);
        loop_timer.sleep();
    }
    ROS_INFO("FINISHED MOVING BACKWARDS");
    

    // RETURN TO HOME
    mobot_pub_des_state_gamma::path path_srv;
    geometry_msgs::Quaternion quat;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;

    quat = convertPlanarPhi2Quaternion(0.0);
    pose.orientation = quat;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);

    bool success = open_loop_client.call(path_srv);
    ROS_INFO("SENDING HOME POSE TO SERVER");

    if(success) {
        return navigator_gamma::navigatorResult::DESIRED_POSE_ACHIEVED;
    } else {
        return navigator_gamma::navigatorResult::FAILED_CANNOT_REACH_DES_POSE;
    }
    /* OPEN LOOP APPROACH */
}

int Navigator::navigate_to_table() { 
    /* OPEN LOOP APPROACH */
    mobot_pub_des_state_gamma::path path_srv;
    geometry_msgs::Quaternion quat;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    geometry_msgs::Pose pose;
    pose.position.x = 5.0;  // move three meters forward
    pose.position.y = 0.0;
    pose.position.z = 0.0;

    quat = convertPlanarPhi2Quaternion(0.0);
    pose.orientation = quat;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);

    ROS_INFO("SENDING TABLE POSE TO SERVER");
    bool success = open_loop_client.call(path_srv);

    ros::Duration(30.0).sleep();

    if(success) {
        return navigator_gamma::navigatorResult::DESIRED_POSE_ACHIEVED;
    } else {
        return navigator_gamma::navigatorResult::FAILED_CANNOT_REACH_DES_POSE;
    }
    /* OPEN LOOP APPROACH */
}

int Navigator::navigate_to_pose(geometry_msgs::PoseStamped goal_pose) {
    return navigator_gamma::navigatorResult::DESIRED_POSE_ACHIEVED;
}


void Navigator::executeCB(const actionlib::SimpleActionServer<navigator_gamma::navigatorAction>::GoalConstPtr& goal) {
    int destination_id = goal->location_code;
    geometry_msgs::PoseStamped destination_pose;
    int navigation_status;

    if (destination_id == navigator_gamma::navigatorGoal::COORDS) {
        destination_pose = goal->desired_pose;
    }
    
    switch(destination_id) {
        case navigator_gamma::navigatorGoal::HOME: 
            //specialized function to navigate to pre-defined HOME coords
            navigation_status = navigate_home();
            if (navigation_status == navigator_gamma::navigatorResult::DESIRED_POSE_ACHIEVED) {
                ROS_INFO("reached home");
                result_.return_code = navigator_gamma::navigatorResult::DESIRED_POSE_ACHIEVED;
                navigator_as_.setSucceeded(result_);
            } else {
                ROS_WARN("could not navigate home!");
                result_.return_code = navigator_gamma::navigatorResult::FAILED_CANNOT_REACH_DES_POSE;
                navigator_as_.setAborted(result_);
            }
            break;

        case navigator_gamma::navigatorGoal::TABLE: 
            //specialized function to navigate to pre-defined TABLE coords
            navigation_status = navigate_to_table(); 
            if (navigation_status == navigator_gamma::navigatorResult::DESIRED_POSE_ACHIEVED) {
                ROS_INFO("reached table");
                result_.return_code = navigator_gamma::navigatorResult::DESIRED_POSE_ACHIEVED;
                navigator_as_.setSucceeded(result_);
            } else {
                ROS_WARN("could not navigate to table!");
                result_.return_code = navigator_gamma::navigatorResult::FAILED_CANNOT_REACH_DES_POSE;
                navigator_as_.setAborted(result_);
            }
            break;

        case navigator_gamma::navigatorGoal::COORDS:
            //more general function to navigate to specified pose:
            destination_pose = goal->desired_pose;
            navigation_status = navigate_to_pose(destination_pose); 
            if (navigation_status == navigator_gamma::navigatorResult::DESIRED_POSE_ACHIEVED) {
                ROS_INFO("reached desired pose");
                result_.return_code = navigator_gamma::navigatorResult::DESIRED_POSE_ACHIEVED;
                navigator_as_.setSucceeded(result_);
            } else {
                ROS_WARN("could not navigate to desired pose!");
                result_.return_code = navigator_gamma::navigatorResult::FAILED_CANNOT_REACH_DES_POSE;
                navigator_as_.setAborted(result_);
            }
            break;               

        default:
            ROS_WARN("this location ID is not implemented");
            result_.return_code = navigator_gamma::navigatorResult::DESTINATION_CODE_UNRECOGNIZED; 
            navigator_as_.setAborted(result_);
            break;
    }
  
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_action_server"); // name this node

    // while (!open_loop_client.exists()) {
    //     ROS_INFO("waiting for service...");
    //     ros::Duration(1.0).sleep();
    // }
    // ROS_INFO("connected to open loop service");

    ROS_INFO("instantiating the navigation action server: ");

    Navigator navigator_as; // create an instance of the class "ObjectFinder"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        // for debug, induce a halt if we ever get our client/server communications out of sync
    }

    return 0;
}

