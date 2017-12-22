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
#include "pcl_ros/point_cloud.h"
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
#include <tf/transform_listener.h>

using namespace std;
using namespace pcl;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
double* extrinsic;
double extrinsic_array[7];
ros::Subscriber pointcloud_subscriber;
ros::Publisher pointcloud_publisher;
ros::Publisher voxel_pub;
ros::Publisher model_coeff_pub;
void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
double* read_camera_extrinsic();
tf::StampedTransform transformtf;


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
	ros::init (argc, argv, "get_height_map");
	ros::NodeHandle nh; 

	
	tf::TransformListener listener; 
	ros::Rate rate(1.0);
	rate.sleep();
	rate.sleep();
	listener.lookupTransform( "/camera_rgb_optical_frame", "/map", ros::Time(0), transformtf);
	pointcloud_subscriber = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, pointcloud_cb);
	//pointcloud_subscriber = nh.subscribe<sensor_msgs::PointCloud2> ("/camera_link/moving_object", 1, pointcloud_cb);
	pointcloud_publisher = nh.advertise<PointCloudXYZRGB> ("/camera/depth_registered/transformed_points", 1);
    voxel_pub = nh.advertise<sensor_msgs::PointCloud2> ("/voxel", 1);
	model_coeff_pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);
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
	PointCloud<PointXYZRGB>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	PointCloud<PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
//get pointcloud from rostopic
	pcl::fromROSMsg (*input, *source_cloud);
	int i=0;
	//for (i=0;i<7;i++)
		//cout << extrinsic_array[i] << endl;
	/*
	Eigen::Matrix3f rotmat = Eigen::Quaternionf(extrinsic_array[6], extrinsic_array[3], extrinsic_array[4], extrinsic_array[5]).toRotationMatrix();
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform.block(0,0,3,3) = rotmat;
	transform (0,3) = extrinsic_array[0];
	transform (1,3) = extrinsic_array[1];
	transform (2,3) = extrinsic_array[2];
	std::cout << transform << std::endl;
	Eigen::Matrix4f inv_transform = transform.inverse();
	std::cout << inv_transform << std::endl;
	Eigen::Matrix3f rotmat2 = Eigen::Quaternionf(-0.5, 0.5, 0.5, 0.5).toRotationMatrix();
	Eigen::Matrix4f transform2 = Eigen::Matrix4f::Identity();
	transform2.block(0,0,3,3) = rotmat2;
	Eigen::Matrix4f inv_transform2 = transform2.inverse();
	Eigen::Matrix4f final_transform = transform2 * inv_transform;
	*/
    
	Eigen::Matrix3f rotmat = Eigen::Quaternionf(transformtf.getRotation().w(), transformtf.getRotation().x(), transformtf.getRotation().y(), transformtf.getRotation().z()).toRotationMatrix();
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform.block(0,0,3,3) = rotmat;
	transform (0,3) = transformtf.getOrigin().x();
	transform (1,3) = transformtf.getOrigin().y();
	transform (2,3) = transformtf.getOrigin().z();
	//cout << transformtf.getOrigin().x() << " " << transformtf.getOrigin().y() << " " << transformtf.getOrigin().z() << endl;
	//cout << transformtf.getRotation().x() << " " << transformtf.getRotation().y() << " " << transformtf.getRotation().z() << " " << transformtf.getRotation().w() << endl;
	transformPointCloud (*source_cloud, *transformed_cloud, transform);
	transformed_cloud->header.frame_id = "/map";
	cout << transformed_cloud->width << " " << transformed_cloud->height << endl;
	

	//Eigen::Quaternionf q = transformed_cloud->sensor_orientation_;
	//cout << transformed_cloud->sensor_origin_ << " " <<  q.x() << " " <<  q.y() << " " <<  q.z() << " " << q.w() << endl;
	
	for (int i=0; i<transformed_cloud->size(); i++){
		if(transformed_cloud->points[i].z<0.08){
			transformed_cloud->points[i].r=0;
			transformed_cloud->points[i].g=255;
			transformed_cloud->points[i].r=0;
		}
	
	}
	
	pointcloud_publisher.publish(*transformed_cloud);

	

	//heightmap api still not working
	/*
	pcl::people::HeightMap2D<pcl::PointXYZRGB> heightmap;
	heightmap.setInputCloud(transformed_cloud);
	vector<int> heightmapvec = heightmap.getHeightMap();
	for(int i=0; i<heightmapvec.size(); i++)
 		cout << heightmapvec[i] << endl;
 	*/
	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
	pcl_conversions::toPCL(*input, *cloud);

  // Perform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (0.01, 0.01, 0.01);
	sor.filter (cloud_filtered);

  // Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
	voxel_pub.publish (output);
}
