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

#include <pcl/segmentation/sac_segmentation.h>

using namespace std;

const double MIN_HEIGHT = -0.5;
const double MAX_HEIGHT = 0.7;//Gotta make it a little taller to accomodate the can itself...
const double MIN_SIDE= -0.5;
const double MAX_SIDE = 0.5;
const double MIN_DIST = 0.2;
const double MAX_DIST = 1.0;

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
	    	lambda.lookupTransform("world", "kinect_pc_frame", ros::Time(0), tau);
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

    //Cut off everything that can't possibly be a table (tatamis need not apply)
	pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object
    pass.setInputCloud(transformed_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z"); // we will "filter" based on points that lie within some range of z-value
    pass.setFilterLimits(MIN_HEIGHT, MAX_HEIGHT); //here is the range: z value near zero, -0.02<z<0.02
    //std::vector<int> filter_output_indices;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pass.filter(*sliced_cloud_ptr); //  this will return the indices of the points in transformed_cloud_ptr that pass our test

    /*//Cut off everything that can't possibly be a table (tatamis need not apply)
    pass.setInputCloud(sliced_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("y"); // we will "filter" based on points that lie within some range of z-value
    pass.setFilterLimits(MIN_SIDE, MAX_SIDE); //here is the range: z value near zero, -0.02<z<0.02
    //std::vector<int> filter_output_indices;
    pass.filter(*sliced_cloud_ptr); //  this will return the indices of the points in transformed_cloud_ptr that pass our test

    //Cut off everything that can't possibly be a table (tatamis need not apply)
    pass.setInputCloud(sliced_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("x"); // we will "filter" based on points that lie within some range of z-value
    pass.setFilterLimits(MIN_DIST, MAX_DIST); //here is the range: z value near zero, -0.02<z<0.02
    //std::vector<int> filter_output_indices;
    pass.filter(*sliced_cloud_ptr); //  this will return the indices of the points in transformed_cloud_ptr that pass our test*/

 	pcl::PointCloud<pcl::PointXYZ>::Ptr mono_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //copyPointCloud(*sliced_cloud_ptr, *mono_cloud_ptr);

    //Now, get the plane for the table
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;

    segmentor.setModelType(pcl::SACMODEL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    segmentor.setAxis(Eigen::Vector3f(0.0, 0.0, -1.0));
    segmentor.setEpsAngle(30 * (M_PI / 180));
    segmentor.setDistanceThreshold (0.01);


    segmentor.setInputCloud(sliced_cloud_ptr);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	segmentor.segment (*inliers, *coefficients);

	/*pcl::PointCloud<pcl::PointXYZ>::Ptr table_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	table_cloud_ptr->header = sliced_cloud_ptr->header;
    table_cloud_ptr->is_dense = sliced_cloud_ptr->is_dense;
    table_cloud_ptr->width = inliers->indices.size();
    table_cloud_ptr->height = 1;
    table_cloud_ptr->points.resize(inliers->indices.size());
	for(int i = 0; i < inliers->indices.size(); i++){//There HAS to be a better way to do this...
		table_cloud_ptr->points[i].getVector3fMap() = sliced_cloud_ptr->points[inliers->indices[i]].getVector3fMap();
	}*/

	//Figure out the table's height...
	float z_avg = 0.0;
	for(int i = 0; i < inliers->indices.size(); i++){
		z_avg += sliced_cloud_ptr->points[inliers->indices[i]].z;
	}
	z_avg = z_avg / inliers->indices.size();
	ROS_INFO("Table is about %f", z_avg);

	//Go ahead and get rid of the table...
	pcl::ExtractIndices<PointXYZRGB> extract;
	extract.setInputCloud(sliced_cloud_ptr);
	extract.setIndices(inliers);
 	extract.setNegative(true);
 	extract.filter(*sliced_cloud_ptr);

 	//Can will be ABOVE the table...
 	pass.setInputCloud(sliced_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
 	pass.setFilterLimits(z_avg, MAX_HEIGHT); //here is the range: z value near zero, -0.02<z<0.02
    pass.filter(*sliced_cloud_ptr); //  this will return the indices of the points in transformed_cloud_ptr that pass our test

    //Get the normals
    pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod (tree);
	ne.setInputCloud (sliced_cloud_ptr);
	ne.setKSearch (50);
	ne.compute(*cloud_normals);
    ROS_INFO("Got normals");

    //Find a cylindrical thing matching that description:
    pcl::SACSegmentationFromNormals<PointXYZRGB, pcl::Normal> nseg;
    nseg.setOptimizeCoefficients (true);
    nseg.setModelType (pcl::SACMODEL_CYLINDER);
  	nseg.setMethodType (pcl::SAC_RANSAC);
  	nseg.setNormalDistanceWeight (0.1);
  	nseg.setMaxIterations (10000);
  	nseg.setDistanceThreshold (0.05);
  	nseg.setRadiusLimits (0, 0.1);
  	nseg.setInputCloud (sliced_cloud_ptr);
  	nseg.setInputNormals (cloud_normals);

  	nseg.segment (*inliers, *coefficients);

    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*sliced_cloud_ptr);

    //And find the centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*sliced_cloud_ptr, centroid);

    ROS_WARN("CALCULATED CENTER OF CAN TO BE (%f, %f, %f)",centroid[0], centroid[1], centroid[2]);

    //the new cloud is a set of points from original cloud, coplanar with selected patch; display the result
    pcl::toROSMsg(*downsampled_kinect_ptr, input_cloud);
    pcl::toROSMsg(*sliced_cloud_ptr, output_cloud); //convert to ros message for publication and display

    input_cloud.header.frame_id = "kinect_pc_frame";
    output_cloud.header.frame_id = "world";

    cloudIn.publish(input_cloud); // will not need to keep republishing if display setting is persistent
    cloudOut.publish(output_cloud); //can directly publish a pcl::PointCloud2!!
    ros::spinOnce(); //PclUtilsGamma needs some spin cycles to invoke callbacks for new selected points

	_exit(0);

    return 0;
}
