#include <ros/ros.h> 
#include <stdlib.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "yes_we_can");
	ros::NodeHandle nh;

	ROS_INFO("Initted.");

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	ROS_INFO("DONE");

	if(pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/tes77/ros_ws/src/learning_ros/Part_3/pcd_images", *pclKinect_clr_ptr) == -1){
		ROS_ERROR("ERRORRRRRRR...");
		return 1;
	}

	ROS_INFO("Double done.");

	pclKinect_clr_ptr->header.frame_id = "camera_depth_optical_frame";


	ROS_INFO("***Initialization Complete***");

	return 0;
}