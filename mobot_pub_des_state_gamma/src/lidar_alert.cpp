#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <std_srvs/Trigger.h>

const double LIDAR_SAFE_DIST = 2.0;
const double ONE_SIDE_FANOUT = 0.523598776;//in radians

bool obstruction_;
int pindex_;

double angle_min_;
double angle_max_;
double angle_inc_;
double range_min_;
double range_max_;

int ind_min_;
int ind_max_;

ros::ServiceClient client_set_;
ros::ServiceClient client_rst_;
std_srvs::Trigger srv_set_;
std_srvs::Trigger srv_rst_;


void laserCallback(const sensor_msgs::LaserScan& laser_scan){
	if (pindex_<0){
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_inc_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        pindex_ = (int) ((0.0 - angle_min_)/angle_inc_);

        double halfway = angle_min_ + angle_max_ / 2.0;
        double ang_upper_bound = halfway - ONE_SIDE_FANOUT;
        double ang_lower_bound = halfway + ONE_SIDE_FANOUT;
        ind_min_ = (int) ((ang_upper_bound - angle_min_)/angle_inc_);
        ind_max_ = (int) ((ang_lower_bound - angle_min_)/angle_inc_);
        ROS_INFO("LIDAR setup: ping_index = %d, bound indices (%d, %d)",pindex_, ind_min_, ind_max_);
    }

    bool obstruction = false;
    for(int i = ind_min_; i <= ind_max_; i++){
        //ROS_INFO("  %d: %f",i, laser_scan.ranges[i]);
    	if(laser_scan.ranges[i] < LIDAR_SAFE_DIST){
    		obstruction = true;
    		ROS_WARN("I AM SO TRIGGERED RIGHT NOW...");
    		break;
    	}
    }

    if(obstruction && !obstruction_){
    	obstruction_ = true;
    	ROS_WARN("STOP.");
    	while(!client_set_.call(srv_set_)){
    		ROS_ERROR("STOP I SAY!");
    	}
    }
    else if(!obstruction && obstruction_){
    	obstruction_ = false;
    	ROS_INFO("You're free to go.");
    	while(!client_rst_.call(srv_rst_)){
    		ROS_ERROR("Go on then...");
    	}
    }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "lidar_alert");
	ros::NodeHandle nh;
	obstruction_ = false;
	pindex_ = -1;
	angle_min_ = 0.0;
	angle_max_ = 0.0;
	angle_inc_ = 0.0;
	range_min_ = 0.0;
	range_max_ = 0.0;
	ind_min_ = 0;
	ind_max_ = 0;

	ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, laserCallback);

	client_set_ = nh.serviceClient<std_srvs::Trigger>("lidar_service");
	client_rst_ = nh.serviceClient<std_srvs::Trigger>("clear_lidar_service");

	ros::spin();

	return 0;
}