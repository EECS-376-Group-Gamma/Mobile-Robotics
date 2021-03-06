// example_navigator_action_client: 
// wsn, April, 2016
// illustrates use of navigator action server called "navigatorActionServer"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <navigator_gamma/navigatorAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

geometry_msgs::PoseStamped g_desired_pose;
int g_navigator_rtn_code;
void navigatorDoneCb(const actionlib::SimpleClientGoalState& state,
        const navigator_gamma::navigatorResultConstPtr& result) {
    ROS_INFO("navigatorDoneCb: server responded with state [%s]", state.toString().c_str());
    g_navigator_rtn_code = result->return_code;
    ROS_INFO("got object code response = %d; ",g_navigator_rtn_code);
    if (g_navigator_rtn_code == navigator_gamma::navigatorResult::DESTINATION_CODE_UNRECOGNIZED) {
        ROS_WARN("destination code not recognized");
    } else if (g_navigator_rtn_code == navigator_gamma::navigatorResult::DESIRED_POSE_ACHIEVED) {
        ROS_INFO("reached desired location!");
    } else {
        ROS_WARN("desired pose not reached!");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_navigator_action_client"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    
    
    actionlib::SimpleActionClient<navigator_gamma::navigatorAction> navigator_ac("navigatorActionServer", true);
    
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists) && (ros::ok())) {
        server_exists = navigator_ac.waitForServer(ros::Duration(0.5));
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to navigator action server"); // if here, then we connected to the server; 
     
    navigator_gamma::navigatorGoal navigation_goal;
    
    // MOVE TO TABLE
    navigation_goal.location_code = navigator_gamma::navigatorGoal::TABLE;
    ROS_INFO("sending goal to go to table: ");
    navigator_ac.sendGoal(navigation_goal,&navigatorDoneCb);
    
    //bool finished_before_timeout = navigator_ac.waitForResult(ros::Duration(30.0));
    bool finished_before_timeout = navigator_ac.waitForResult(); // wait forever...
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result ");
        return 1;
    }

    // MOVE TO HOME
    navigation_goal.location_code = navigator_gamma::navigatorGoal::HOME;
    ROS_INFO("sending goal to return home: ");
    navigator_ac.sendGoal(navigation_goal,&navigatorDoneCb);
    finished_before_timeout = navigator_ac.waitForResult();
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result ");
        return 1;
    }
        
    return 0;
}

