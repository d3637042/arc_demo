#include <iostream> 
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <fstream>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <image_transport/image_transport.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/people/height_map_2d.h>
#include <pcl/filters/voxel_grid.h>
//ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/SetBool.h"
#include <tf/transform_listener.h>
//opencv
#include <opencv2/opencv.hpp>

using namespace std;
using namespace pcl;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
double* extrinsic;
double extrinsic_array[7];
ros::Subscriber pointcloud_subscriber;
ros::Publisher pointcloud_publisher;
ros::Publisher voxel_pub;
ros::Publisher model_coeff_pub;
ros::Publisher pcl2trans_pub;
ros::Publisher pcl2transerror_pub;
void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
double* read_camera_extrinsic();
tf::StampedTransform transformtf;
tf::TransformListener* listener;
PointCloud<PointXYZRGB>::Ptr realcloud (new pcl::PointCloud<pcl::PointXYZRGB> ()); 
bool get_height_map(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
const char* name;
int main(int argc, char** argv){
//ros node init

	//extrinsic = read_camera_extrinsic();
	/*
	int i=0;
	for (i=0;i<7;i++){
		cout << extrinsic[i] << endl;
		extrinsic_array[i]=extrinsic[i];
	}
	*/
	ros::init (argc, argv, "get_transformed_point");
	ros::NodeHandle nh; 
	tf::TransformListener lr(ros::Duration(10));
	listener = &lr; 
	ros::Rate rate(1.0);
	rate.sleep();
	rate.sleep();
	pointcloud_subscriber = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, pointcloud_cb);

	//pointcloud_publisher = nh.advertise<PointCloudXYZRGB> ("/cropedpointcloud", 1);

	pcl2trans_pub = nh.advertise<sensor_msgs::PointCloud2> ("/pcl2trans", 1);
	//pcl2transerror_pub = nh.advertise<sensor_msgs::PointCloud2> ("/pcl2transerror", 1);

	ros::spin ();
	return 0;
}
double* read_camera_extrinsic(){
	double data[7];
	ifstream infile("/home/nctuece/catkin_ws/src/heightmap_creator/camera.txt");
	char line[1000];
	while(infile) {
		infile.getline(line, 1000); 

		char* pch;
		const char* delim = " ";
		int count = 0;
		pch = strtok(line,delim);
		while (pch != NULL && count < 7)
		{
			cout << pch << endl;
			data[count] = atof(pch);
  //cout << data[count] << endl;
			pch = strtok (NULL, delim);
			count++;
		} 

	}
	infile.close();
	cout << endl; 
	return data;
}
void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){
	//cout << input->header.frame_id << endl;
	//PointCloud<PointXYZRGB>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	//PointCloud<PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	
	////////////////////get pointcloud from rostopic
	//pcl::fromROSMsg (*input, *source_cloud);
	
    ////////////////////TRANSFORM(by APi) pointcloud from camera to map////////////////////
	sensor_msgs::PointCloud2 transcloud, outputcloud;
	const string center_camera = "/center_camera";
	pcl_ros::transformPointCloud(center_camera, *input, transcloud, *listener);
	pcl2trans_pub.publish(transcloud);
	

	////////////////////translate error////////////////////
	//Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	//if needed
	//transform (0,3) = -0.012;
	//pcl_ros::transformPointCloud(transform, transcloud, outputcloud);
	//pcl2transerror_pub.publish(outputcloud);
	
	////////////////////crop pointcloud////////////////////
	// (x/2, y/2) = (0.12,0.20)
	
	//pcl::fromROSMsg (outputcloud, *realcloud);

	//pointcloud_publisher.publish(*realcloud);

	
}
