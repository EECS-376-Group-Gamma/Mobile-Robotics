// example_action_server: a simple action server
// Wyatt Newman

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../example_action_server/action/demo.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (demo) and appended name (Action)
#include <pcl_utils/pcl_utils.h>
#include <object_finder_gamma/objectFinderAction.h>

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

#include <tf/transform_listener.h>

#include <pcl/segmentation/sac_segmentation.h>


class ObjectFinder {
private:

    static const double MIN_HEIGHT = -0.5;
    static const double MAX_HEIGHT = 0.7;//Gotta make it a little taller to accomodate the can itself...
    static const double MIN_SIDE= -0.5;
    static const double MAX_SIDE = 0.5;
    static const double MIN_DIST = 0.2;
    static const double MAX_DIST = 1.0;
    static const double TABLE_TOLERANCE = 0.4;

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<object_finder_gamma::objectFinderAction> object_finder_as_;
    
    // here are some message types to communicate with our client(s)
    object_finder_gamma::objectFinderGoal goal_; // goal message, received from client
    object_finder_gamma::objectFinderResult result_; // put results here, to be sent back to the client when done w/ goal
    object_finder_gamma::objectFinderFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client

    PclUtils pclUtils_;
    //specialized function to find an upright Coke can on known height of horizontal surface;
    // returns true/false for found/not-found, and if found, fills in the object pos

    bool find_upright_coke_can(float surface_height,geometry_msgs::PoseStamped &object_pose);

    static void kinectCB(const sensor_msgs::PointCloud2ConstPtr& c);

public:
    ObjectFinder(); //define the body of the constructor outside of class definition

    ~ObjectFinder(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<object_finder_gamma::objectFinderAction>::GoalConstPtr& goal);

    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr incoming;
    static bool ready;
};

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectFinder::incoming = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
bool ObjectFinder::ready = false;

ObjectFinder::ObjectFinder() :
   object_finder_as_(nh_, "objectFinderActionServer", boost::bind(&ObjectFinder::executeCB, this, _1),false),pclUtils_(&nh_) {
    ROS_INFO("in constructor of ObjectFinder...");
    // do any other desired initializations here...specific to your implementation

    object_finder_as_.start(); //start the server running
}

void ObjectFinder::kinectCB(const sensor_msgs::PointCloud2ConstPtr& c) {//Well THIS is unnecessarily complicated...
    if (!ready){
        pcl::fromROSMsg(*c, *incoming);
        ready = true;
    }
}

bool ObjectFinder::find_upright_coke_can(float surface_height,geometry_msgs::PoseStamped &object_pose) {

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
            lambda.lookupTransform("torso", "camera_depth_optical_frame", ros::Time(0), tau);
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
    ros::Subscriber pcs = nh_.subscribe("/camera/depth_registered/points", 1, kinectCB);
    ROS_INFO("Waiting for a message from the Kinect...");
    while(!ready && ros::ok()){
        ros::spinOnce();
    }
    ROS_INFO("GOT IT!");

    bool found_object=true;

    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    ros::Publisher cloudIn = nh_.advertise<sensor_msgs::PointCloud2> ("/pcd_original", 1);
    ros::Publisher cloudOut = nh_.advertise<sensor_msgs::PointCloud2> ("/pcd_final", 1);
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

    if(inliers->indices.size() < 10){
        found_object =  false;
    }

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

    ROS_INFO("Cut off everything below table...");

    //Get the normals
    /*pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod (tree);
    ne.setInputCloud (sliced_cloud_ptr);
    ne.setKSearch (50);
    ne.compute(*cloud_normals);
    ROS_INFO("Got normals");*/

    /*//Find a cylindrical thing matching that description:
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

    if(inliers->indices.size() < 5){
        found_object = false;
    }

    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*sliced_cloud_ptr);*/

    //Can will be ABOVE the table...
    pass.setInputCloud(sliced_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterLimits(0.5, 2.0); //here is the range: z value near zero, -0.02<z<0.02
    pass.setFilterFieldName("x"); // we will "filter" based on points that lie within some range of z-value
    pass.filter(*sliced_cloud_ptr); //  this will return the indices of the points in transformed_cloud_ptr that pass our test

    ROS_INFO("SLICED OUT THE ROBOT ITSELF...");

    segmentor.setModelType(pcl::SACMODEL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    segmentor.setAxis(Eigen::Vector3f(0.0, 0.0, -1.0));
    segmentor.setEpsAngle(30 * (M_PI / 180));
    segmentor.setDistanceThreshold (0.01);


    segmentor.setInputCloud(sliced_cloud_ptr);

    segmentor.segment (*inliers, *coefficients);

    //And find the centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*sliced_cloud_ptr, centroid);

    ROS_WARN("CALCULATED CENTER OF CAN TO BE (%f, %f, %f)",centroid[0], centroid[1], centroid[2]);

    //the new cloud is a set of points from original cloud, coplanar with selected patch; display the result
    pcl::toROSMsg(*transformed_cloud_ptr, input_cloud);
    pcl::toROSMsg(*sliced_cloud_ptr, output_cloud); //convert to ros message for publication and display

    input_cloud.header.frame_id = "torso";
    output_cloud.header.frame_id = "torso";

    cloudIn.publish(input_cloud); // will not need to keep republishing if display setting is persistent
    cloudOut.publish(output_cloud); //can directly publish a pcl::PointCloud2!!
    ros::spinOnce(); //PclUtilsGamma needs some spin cycles to invoke callbacks for new selected points

    object_pose.header.frame_id = "base_link";
    object_pose.pose.position.x = 0.8;//centroid[0];
    object_pose.pose.position.y = -0.25;//centroid[1];
    object_pose.pose.position.z = -0.053; //centroid[2] + 0.15;
    object_pose.pose.orientation.x = -0.699; //0.166; //0;
    object_pose.pose.orientation.y = 0.266;//0.648; //0;
    object_pose.pose.orientation.z = -0.033; //0.702; //0;
    object_pose.pose.orientation.w = 0.663; //0.109; //1;
    return found_object;
    
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <example_action_server::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "example_action_server", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void ObjectFinder::executeCB(const actionlib::SimpleActionServer<object_finder_gamma::objectFinderAction>::GoalConstPtr& goal) {
    int object_id = goal->object_id;
    geometry_msgs::PoseStamped object_pose;
    bool known_surface_ht = goal->known_surface_ht;
    float surface_height;
    if (known_surface_ht) {
        surface_height=goal->surface_ht;
    }
    bool found_object=false;
    switch(object_id) {
        case object_finder_gamma::objectFinderGoal::COKE_CAN_UPRIGHT: 
              //specialized function to find an upright Coke can on a horizontal surface of known height:
               found_object = find_upright_coke_can(surface_height,object_pose); //special case for Coke can;
               if (found_object) {
                   ROS_INFO("found upright Coke can!");
                   result_.found_object_code = object_finder_gamma::objectFinderResult::OBJECT_FOUND;
                   result_.object_pose = object_pose;
                   object_finder_as_.setSucceeded(result_);
               }
               else {
                   ROS_WARN("could not find requested object");
                   object_finder_as_.setAborted(result_);
               }
               break;
        default:
            found_object = find_upright_coke_can(surface_height,object_pose); //special case for Coke can;
             ROS_WARN("this object ID is not implemented");
             result_.found_object_code = object_finder_gamma::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED; 
             object_finder_as_.setAborted(result_);
            }
  
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_action_server_node"); // name this node 

    ROS_INFO("instantiating the demo action server: ");

    ObjectFinder object_finder_as; // create an instance of the class "ObjectFinder"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        // for debug, induce a halt if we ever get our client/server communications out of sync
    }

    return 0;
}

