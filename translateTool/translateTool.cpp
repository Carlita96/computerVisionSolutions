#include <pcl/io/ply_io.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <chrono>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

using namespace std;

int
main (int argc, char** argv)
{
	cout << "Starting program" << endl;

	////////////////////////////////
	// Read the input parameters
	std::string fileName = argv[1];
	cout << "Opening model file: " << fileName << endl;
	double translate = 2.78; // 0.04 usually
	cout << "Translate: Doing translation of " << translate << "meters in X." << endl;

    ////////////////////////////////
    // Read cloud
	// Create necessary point clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud);

	cout << "Input file has size: " << cloud->points.size() << endl;

  	// Move it as much as needed
	for (size_t i = 0; i < cloud->points.size(); i++) {
	    cloud->points[i].x -= translate;
	    cloud->points[i].x += 0.5;
	    cloud->points[i].x -= 2.04;
	    cloud->points[i].y += 0.5;
	    cloud->points[i].z += 0.11;
	    cloud->points[i].z -= 0.2;
	}

	cout << "Output file has size: " << cloud->points.size() << endl;

	pcl::io::savePCDFile( "outputCloud.pcd", *cloud, true );

	cout << "Finished program" << endl;
	return 0;
}