#include <pcl/io/ply_io.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <chrono>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

using namespace std;

int
main (int argc, char** argv)
{
	cout << "Starting program" << endl;

	////////////////////////////////
	// Read the model point cloud
	std::string fileName = argv[1];
	cout << "Opening model file: " << fileName << endl;
	// Create necessary point clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelColorCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	// Read
	// pcl::PLYReader Reader;
	// Reader.read(fileName, *modelCloud);
	// Change to red pointcloud
	pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *modelCloud);

	pcl::copyPointCloud(*modelCloud, *modelColorCloud);

	cout << "Model file has size: " << modelCloud->points.size() << endl;

	float x, y, z;
	for (size_t i = 0; i < modelColorCloud->points.size(); i++) {
	    modelColorCloud->points[i].r = 255;
	    modelColorCloud->points[i].g = 0;
	    modelColorCloud->points[i].b = 0;
	 //    x = modelColorCloud->points[i].x;
	 //    y = modelColorCloud->points[i].y;
	 //    z = modelColorCloud->points[i].z;
	 //    modelColorCloud->points[i].x = z/1000;
		// modelColorCloud->points[i].y = y/1000;
		// modelColorCloud->points[i].z = x/1000; 
	}

	 //Load point clouds here

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    double translationX = std::stod(argv[3]);
    double translationY = std::stod(argv[4]);
    double translationZ = std::stod(argv[5]);
    double rotationX = std::stod(argv[6]);
    double rotationY = std::stod(argv[7]);
    double rotationZ = std::stod(argv[8]);
    transform.translation() << translationX, translationY, translationZ;
    transform.rotate(Eigen::AngleAxisf((rotationX*M_PI) / 180, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf((rotationY*M_PI) / 180, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf((rotationZ*M_PI) / 180, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*modelColorCloud, *modelColorCloud, transform); //Only rotate target cloud


    // Save file
    pcl::io::savePCDFile( "outputModelCloud.pcd", *modelColorCloud, true );

	////////////////////////////////
	// Read the point cloud
	fileName = argv[2];
	cout << "Opening big file: " << fileName << endl;
	// Create necessary point clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr bigCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr bigColorCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	// Read
	pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *bigCloud);
	// Change to red pointcloud
	pcl::copyPointCloud(*bigCloud, *bigColorCloud);

	cout << "Big file has size: " << bigCloud->points.size() << endl;

	for (size_t i = 0; i < bigColorCloud->points.size(); i++) {
	    bigColorCloud->points[i].r = 255;
	    bigColorCloud->points[i].g = 255;
	    bigColorCloud->points[i].b = 255;
	}

	// Pass through filter
	// Create tools
    pcl::PassThrough<pcl::PointXYZRGB> passThrough;

    // Create needed clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Create and do the filter
    passThrough.setInputCloud (bigColorCloud);
    passThrough.setFilterFieldName ("x");
    passThrough.setFilterLimits (0, 3);
    passThrough.filter (*tempCloud);

    *bigColorCloud = *tempCloud;

    pcl::io::savePCDFile( "outputCloud.pcd", *bigColorCloud, true );

	////////////////////////////////
	// Create visualizer
  	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
  	viewer->addCoordinateSystem();
  	// Add model point cloud
  	viewer->addPointCloud<pcl::PointXYZRGB>(modelColorCloud, "Model Point Cloud");
  	viewer->addPointCloud<pcl::PointXYZRGB>(bigColorCloud, "Big Point Cloud");
  	viewer->spin();


	cout << "Finished program" << endl;
	return 0;
}