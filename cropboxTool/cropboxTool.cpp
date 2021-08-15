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
	pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud);

	cout << "Input file has size: " << cloud->points.size() << endl;

	////////////////////////////////
	// Define minimum and maximum points
	Eigen::Vector4f minPoint = Eigen::Vector4f(minX, minY, minZ, 0);
	Eigen::Vector4f maxPoint = Eigen::Vector4f(maxX, maxY, maxZ, 0);
	Eigen::Vector3f translation = Eigen::Vector3f(0,0,0);
	Eigen::Vector3f rotation = Eigen::Vector3f(0,0,0);


	////////////////////////////////
    // Do cropbox
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
	// Create visualizer
  	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
  	viewer->addCoordinateSystem();

  	// Color point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorInputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorOutputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *colorInputCloud);
	for (size_t i = 0; i < colorInputCloud->points.size(); i++) {
	    colorInputCloud->points[i].r = 0;
	    colorInputCloud->points[i].g = 255;
	    colorInputCloud->points[i].b = 0;
	}
	pcl::copyPointCloud(*tempCloud, *colorOutputCloud);
	for (size_t i = 0; i < colorOutputCloud->points.size(); i++) {
	    colorOutputCloud->points[i].r = 255;
	    colorOutputCloud->points[i].g = 0;
	    colorOutputCloud->points[i].b = 0;
	}

  	// Add model point cloud
  	viewer->addPointCloud<pcl::PointXYZRGB>(colorInputCloud, "Point Cloud");
  	viewer->addPointCloud<pcl::PointXYZRGB>(colorOutputCloud, "Point Cloud Output");
  	viewer->spin();

    *cloud = *tempCloud;

	cout << "Output file has size: " << cloud->points.size() << endl;

	pcl::io::savePCDFile( "outputCloud.pcd", *cloud, true );

	cout << "Finished program" << endl;
	return 0;
}