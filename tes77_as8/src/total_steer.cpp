#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
//A bunch of probably mostly irrelevant dependencies from old projects.

#include "../include/simple_point.h"

//Since the objective here is steering and not obsticle avoidance or ROS communication,
//I have just made a blind steering algorithm in a single file.
//It writes my name, which is only really possible because my name is three letters long.
//It was originally going to do that by writing one letter after another cursive-style, but
//I ran out of room in the pen so it just does them overtop of one another.
//For some reason, that just strikes me as incredibly lame, but what can I do?
//Based off of code found in mobot_nl_steering.

const double GOAL_EPSILON = 0.3;
const double DTHRESH = 1.0;
const double W_MAX = 2.0;
const double LMAX = 0.5;
const double K = 2.0;
const double DT = 0.01;
const ros::Duration REACTION_TIME(1.0);

const int wp_siz = 15;
const simple_point waypoints[wp_siz]{
	//T
	simple_point(	3.0,	3.0		),
	simple_point(	-3.0,	3.0		),
	simple_point(	-3.0,	1.0		),
	simple_point(	-3.0,	6.0		),
	//O
	simple_point(	3.0,	6.0		),
	simple_point(	3.0,	0.0		),
	simple_point(	-3.0,	0.0		),
	simple_point(	-3.0,	6.0		),
	//M
	simple_point(	3.0,	0.0		),
	simple_point(	-3.0,	0.0		),
	simple_point(	-3.0,	3.0		),
	simple_point(	3.0,	3.0		),
	simple_point(	-3.0,	3.0		),
	simple_point(	-3.0,	6.0		),
	simple_point(	3.0,	6.0		)
};

int wp_pos;

simple_point oldpos;//Where we were last.

ros::Publisher helm;
ros::Subscriber nvgt;

nav_msgs::Odometry lns;//Last kNown State
ros::Time lst;//Last State Time

bool there(simple_point wp){
	simple_point now = simple_point(lns.pose.pose.position.x, lns.pose.pose.position.y);
	return simple_point::eucDist(wp, now) < GOAL_EPSILON;
	//This could be more sophisticated- I quite liked the better-off check I did on the last assignment.
	//However, once we start getting into curved trajectories and acknolwegdement of wobble and
	//the robot maybe not moving at the same speed it is now in five seconds and all that
	//the better-off check gets INSANELY complicated.
}

//Utility functions from mobot_nl_steering.
double min_dang(double dang) {
    while (dang > M_PI) dang -= 2.0 * M_PI;
    while (dang < -M_PI) dang += 2.0 * M_PI;
    return dang;
}
double sat(double x) {
    if (x>1.0) {
        return 1.0;
    }
    if (x< -1.0) {
        return -1.0;
    }
    return x;
}
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//Not a lot to it.
void odoCB(const nav_msgs::Odometry & odometric){
	lns = odometric;
	lst = ros::Time::now();
}

//Self-contained movement functions.
void stop(){
	geometry_msgs::Twist tc;
	tc.linear.x = 0.0;
    tc.linear.y = 0.0;
    tc.linear.z = 0.0;
    tc.angular.x = 0.0;
    tc.angular.y = 0.0;
    tc.angular.z = 0.0;
    helm.publish(tc);
    ros::Duration(1.0).sleep();
}
void go(double lspeed, double aspeed){
	geometry_msgs::Twist tc;
	tc.linear.x = lspeed;
    tc.linear.y = 0.0;
    tc.linear.z = 0.0;
    tc.angular.x = 0.0;
    tc.angular.y = 0.0;
    tc.angular.z = aspeed;
    helm.publish(tc);
}

//One steering calculation.
void steer_step(double phiPath, double phiCur, simple_point loc){
	double derr = (loc.x - oldpos.x) * -sin(phiPath) + (loc.y - oldpos.y) * cos(phiPath);
	double phiStrategy = min_dang(-0.5 * M_PI * sat(derr / DTHRESH));
	double phiCommand = min_dang(phiPath + phiStrategy);
	double omega = W_MAX * sat(K * min_dang(phiCommand - phiCur));
	go(LMAX, omega);
}

//All the steering calculations.
void steer_to_point(simple_point wp){
	ROS_INFO("Let's go to (%f, %f)!", wp.x, wp.y);
	double phiPath = min_dang(atan2(wp.y - oldpos.y, wp.x - oldpos.x));
	ROS_INFO("Got angle %f", phiPath);
	while(!there(wp) && ros::ok()){
		while((ros::Time::now() - lst).toSec() > REACTION_TIME.toSec() && ros::ok()){
			stop();

			ROS_WARN("CANNOT GET UP-TO-DATE ODOMETRIC INFO. PAUSED.");

			ros::spinOnce();
		}
		double pc = convertPlanarQuat2Phi(lns.pose.pose.orientation);
		simple_point location(lns.pose.pose.position.x, lns.pose.pose.position.y);
		steer_step(phiPath, pc, location);
		ros::spinOnce();
		ros::Duration(DT).sleep();
	}
	ROS_INFO("We made it!");
	stop();
	wp_pos ++;
	oldpos = wp;
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "as8_steer");
	ros::NodeHandle nh;

	ROS_INFO("Initializing...");
	wp_pos = 0;
	lst = ros::Time(0.000);//Times, unfortunately, cannot be negative.
	oldpos.x = 0.0;
	oldpos.y = 0.0;
	ros::Duration(1.0).sleep();
	ROS_INFO("Done.");

	helm = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	nvgt = nh.subscribe("/odom", 1, odoCB);

	while(wp_pos < wp_siz && ros::ok()){
		steer_to_point(waypoints[wp_pos]);
	}
	stop();
	return 0;
}
