#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <std_srvs/Trigger.h>

ros::ServiceClient client_set_;
ros::ServiceClient client_rst_;
std_srvs::Trigger srv_set_;
std_srvs::Trigger srv_rst_;

bool stopped_;

void estopCallback(const std_msgs::Bool& estoppery){

	bool stopped = !(estoppery.data);

    if(stopped && !stopped_){
    	stopped_ = true;
    	ROS_WARN("STOP.");
    	while(!client_set_.call(srv_set_)){
    		ROS_ERROR("STOP I SAY!");
    	}
    	ROS_INFO("Good boy.");
    }
    else if(!stopped && stopped_){
    	stopped_ = false;
    	ROS_INFO("You're free to go.");
    	while(!client_rst_.call(srv_rst_)){
    		ROS_ERROR("Go on then...");
    	}
    	ROS_INFO("On with you.");
    }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "estop_alert");
	ros::NodeHandle nh;
	stopped_ = false;

	ros::Subscriber estopped_subscriber = nh.subscribe("/motors_enabled", 1, estopCallback);

	client_set_ = nh.serviceClient<std_srvs::Trigger>("estop_service");
	client_rst_ = nh.serviceClient<std_srvs::Trigger>("clear_estop_service");

	ros::spin();

	return 0;
}
