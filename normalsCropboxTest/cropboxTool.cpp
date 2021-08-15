#include <pcl/io/ply_io.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <chrono>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

int
main (int argc, char** argv)
{
	cout << "Starting program" << endl;

	////////////////////////////////
	// Read the input parameters
	std::string fileName = argv[1];
	cout << "Opening model file: " << fileName << endl;
	double minX = atof(argv[2]); 
	double minY = atof(argv[3]); 
	double minZ = atof(argv[4]); 
	double maxX = atof(argv[5]);
	double maxY = atof(argv[6]); 
	double maxZ = atof(argv[7]); 

    ////////////////////////////////
    // Read cloud
	// Create necessary point clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>() );
	pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud);

    /////////////////////////////////
	// Calculate normals
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;

    // Estimate the normals
    normalEstimation.setSearchMethod (tree);
    normalEstimation.setInputCloud (cloud);
    normalEstimation.setKSearch (40);
    normalEstimation.compute (*cloudNormals);

    ////////////////////////////////
	// Print amount of points in clouds
	cout << "Input file has size: " << cloud->points.size() << endl;
	cout << "Clouds with normals has size: " << cloudNormals->points.size() << endl;


	////////////////////////////////
	// Define minimum and maximum points
	Eigen::Vector4f minPoint = Eigen::Vector4f(minX, minY, minZ, 0);
	Eigen::Vector4f maxPoint = Eigen::Vector4f(maxX, maxY, maxZ, 0);
	Eigen::Vector3f translation = Eigen::Vector3f(0,0,0);
	Eigen::Vector3f rotation = Eigen::Vector3f(0,0,0);

	////////////////////////////////
    // Do cropbox point cloud
    pcl::CropBox<pcl::PointXYZ> cropBoxFilter;
    // Set the cloud
    cropBoxFilter.setInputCloud(cloud);
    
    // Define the minimum and maximum point. 
    // It could be directly in the wanted position from world origin with empty translation and rotation
    // or with the position in the origin (just define dimensions) and moved with the translation and rotation.
    cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);

    // Translate the box
    cropBoxFilter.setTranslation(translation);
    // Rotate the box
    cropBoxFilter.setRotation(rotation);

    // Get the filtered cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
    cropBoxFilter.setNegative (false);
    cropBoxFilter.filter(*tempCloud);

	////////////////////////////////
    // Do cropbox point cloud of normals
    pcl::CropBox<pcl::Normal> cropBoxFilterNormals;
    // Set the cloud
    cropBoxFilterNormals.setInputCloud(cloudNormals);
    
    // Define the minimum and maximum point. 
    // It could be directly in the wanted position from world origin with empty translation and rotation
    // or with the position in the origin (just define dimensions) and moved with the translation and rotation.
    cropBoxFilterNormals.setMin(minPoint);
    cropBoxFilterNormals.setMax(maxPoint);

    // Translate the box
    cropBoxFilterNormals.setTranslation(translation);
    // Rotate the box
    cropBoxFilterNormals.setRotation(rotation);

    // Get the filtered cloud
    pcl::PointCloud<pcl::Normal>::Ptr tempCloudNormals (new pcl::PointCloud<pcl::Normal>() );
    cropBoxFilterNormals.setNegative (false);
    cropBoxFilterNormals.filter(*tempCloudNormals);

    *cloud = *tempCloud;
    *cloudNormals = *tempCloudNormals;

	cout << "Output cloud has size: " << cloud->points.size() << endl;
	cout << "Output cloud of normals has size: " << cloudNormals->points.size() << endl;

	cout << "Finished program" << endl;
	return 0;
}