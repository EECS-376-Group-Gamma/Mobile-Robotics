//find_plane_pcd_file.cpp
// prompts for a pcd file name, reads the file, and displays to rviz on topic "pcd"
// can select a patch; then computes a plane containing that patch, which is published on topic "planar_pts"
// illustrates use of PCL methods: computePointNormal(), transformPointCloud(), 
// pcl::PassThrough methods setInputCloud(), setFilterFieldName(), setFilterLimits, filter()
// pcl::io::loadPCDFile() 
// pcl::toROSMsg() for converting PCL pointcloud to ROS message
// voxel-grid filtering: pcl::VoxelGrid,  setInputCloud(), setLeafSize(), filter()
//wsn March 2016

#include <ros/ros.h> 
#include <stdlib.h>
#include <math.h>
#include <iterator>

#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl/ros/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 

#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 

#include <point_cloud_analysis_gamma/pcl_utils_gamma.h>  //a local library with some utility fncs.
//We DID end up changing some things, so it would NOT be a good idea to try to use regular pcl_utils

#include <tf/transform_listener.h>

using namespace std;

const double MIN_HEIGHT = 0.8;
const double MAX_HEIGHT = 1.6;

const double TABLE_TOLERANCE = 0.4;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr incoming(new pcl::PointCloud<pcl::PointXYZRGB>);
bool ready;

void kinectCB(const sensor_msgs::PointCloud2ConstPtr& c) {//Well THIS is unnecessarily complicated...
    if (!ready){
        pcl::fromROSMsg(*c, *incoming);
        ready = true;
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "can_finder"); //node name
    ros::NodeHandle nh;

    //Are all of these ACTUALLY needed?
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pts_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for pointcloud of planar points found
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_pts_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>); //ptr to selected pts from Rvis tool
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image

    vector<int> indices;

    tf::TransformListener lambda;
    tf::StampedTransform tau;
    ROS_INFO("Waiting for transform...");
    while(ros::ok()){
	    try{
	    	lambda.lookupTransform("base", "kinect_pc_frame", ros::Time(0), tau);
	    	break;
	    }
	    catch (tf::TransformException ex){
   		 	ROS_ERROR("%s",ex.what());
   		 	ros::Duration(1.0).sleep();
    	}
    }
    ROS_INFO("GOT IT!");

    //Grab a snapshot of the incoming point cloud.
    ready = false;
    ros::Subscriber pcs = nh.subscribe("/kinect/depth/points", 1, kinectCB);
    ROS_INFO("Waiting for a message from the Kinect...");
    while(!ready && ros::ok()){
    	ros::spinOnce();
    }
    ROS_INFO("GOT IT!");

    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    ros::Publisher cloudIn = nh.advertise<sensor_msgs::PointCloud2> ("/pcd_original", 1);
    ros::Publisher cloudOut = nh.advertise<sensor_msgs::PointCloud2> ("/pcd_final", 1);
    sensor_msgs::PointCloud2 input_cloud, output_cloud; //here are ROS-compatible messages

    //use voxel filtering to downsample the original cloud:
    cout << "starting voxel filtering" << endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(incoming);

    vox.setLeafSize(0.02f, 0.02f, 0.02f);
    vox.filter(*downsampled_kinect_ptr);
    cout << "done voxel filtering" << endl;

    cout << "num bytes in original cloud data = " << incoming->points.size() << endl;
    cout << "num bytes in filtered cloud data = " << downsampled_kinect_ptr->points.size() << endl; // ->data.size()<<endl;    

    // Transform into the world frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl_ros::transformPointCloud(*downsampled_kinect_ptr, *transformed_cloud_ptr, tau);

    //Cut off everything that can't possibly be a table (tatami need not apply)
	pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object
    pass.setInputCloud(transformed_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z"); // we will "filter" based on points that lie within some range of z-value
    pass.setFilterLimits(MIN_HEIGHT, MAX_HEIGHT); //here is the range: z value near zero, -0.02<z<0.02
    //std::vector<int> filter_output_indices;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pass.filter(*sliced_cloud_ptr); //  this will return the indices of the points in transformed_cloud_ptr that pass our test

    //pcl::copyPointCloud(*transformed_cloud_ptr, filter_output_indices, *plane_pts_ptr); //extract these pts into new cloud


    //the new cloud is a set of points from original cloud, coplanar with selected patch; display the result
    pcl::toROSMsg(*downsampled_kinect_ptr, input_cloud);
    pcl::toROSMsg(*transformed_cloud_ptr, output_cloud); //convert to ros message for publication and display

    input_cloud.header.frame_id = "kinect_pc_frame";
    output_cloud.header.frame_id = "world";

    while(true && ros::ok()){
    cloudIn.publish(input_cloud); // will not need to keep republishing if display setting is persistent
    cloudOut.publish(output_cloud); //can directly publish a pcl::PointCloud2!!
    ros::spinOnce(); //PclUtilsGamma needs some spin cycles to invoke callbacks for new selected points
    ros::Duration(0.1).sleep();
}

_exit(0);

   /* //Now, get the height of the table.
    Eigen::Vector3f ideal(0.0, 0.0, 1.0);
    PclUtilsGamma p = PclUtilsGamma(&nh);
    //While we still have points that we can cull:
    while(PclUtilsGamma::epsilon_tolerance(filter_output_ptr) > TABLE_TOLERANCE && ros::ok()){
    	ROS_INFO("Tolerance of %f, %lu points remain.",PclUtilsGamma::epsilon_tolerance(filter_output_ptr), filter_output_ptr->size());
        Eigen::Vector3f current_normal;
        double d;
        p.fit_points_to_plane(filter_output_ptr, current_normal, d);
        for(
            pcl::PointCloud<pcl::PointXYZRGB>::iterator i = filter_output_ptr->begin();
            i != filter_output_ptr->end();
        ){
            pcl::PointXYZRGB ptemp = pcl::PointXYZRGB(*i);
            i = filter_output_ptr -> erase(i);
            Eigen::Vector3f experimental_normal;
            p.fit_points_to_plane(filter_output_ptr, experimental_normal, d);
            if(experimental_normal.dot(ideal) <= current_normal.dot(ideal)){
                //It is bad that we removed
                i = filter_output_ptr->insert(i, ptemp);
                i++;
            }
        }
    }

    float sum_z = 0;
    for(int i = 0; i < filter_output_ptr -> size(); i++){
        sum_z = sum_z + filter_output_ptr->at(i).z;
    }
    sum_z = sum_z / filter_output_ptr->size();

    ROS_ERROR("WE GOT A TABLE HEIGHT OF %f", sum_z);

    //Then look the can height above.

    // TODO: find coplanar points with top of can

    // TODO: put those coplanaer points into this variable:

    cout << "num bytes in original cloud data = " << pclKinect_clr_ptr->points.size() << endl;
    cout << "num bytes in filtered cloud data = " << downsampled_kinect_ptr->points.size() << endl; // ->data.size()<<endl;    
    pcl::toROSMsg(*downsampled_kinect_ptr, downsampled_cloud); //convert to ros message for publication and display

    pcl::copyPointCloud(*downsampled_kinect_ptr, indices, *plane_pts_ptr); //extract these pts into new cloud
    //the new cloud is a set of points from original cloud, coplanar with selected patch; display the result
    pcl::toROSMsg(*plane_pts_ptr, ros_planar_cloud); //convert to ros message for publication and display
        
    pubCloud.publish(ros_cloud); // will not need to keep republishing if display setting is persistent
    pubPlane.publish(ros_planar_cloud); // display the set of points computed to be coplanar w/ selection
    pubDnSamp.publish(downsampled_cloud); //can directly publish a pcl::PointCloud2!!
    ros::spinOnce(); //PclUtilsGamma needs some spin cycles to invoke callbacks for new selected points
    ros::Duration(0.1).sleep();*/

    return 0;
}
