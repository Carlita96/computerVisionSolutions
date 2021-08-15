#include <pcl/io/ply_io.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <chrono>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

int
main (int argc, char** argv)
{
	cout << "Starting program" << endl;

	////////////////////////////////
	// Read the input parameters
	std::string fileName = argv[1];
	cout << "Opening model file: " << fileName << endl;
	int sorKMean = atoi(argv[2]); // 30 usually
	double sorStandardDeviation = atof(argv[3]); // 3 usually
	cout << "NOISE REMOVAL: Doing outlier statistical removal with K mean: " 
        << sorKMean << " and standar deviation: " << sorStandardDeviation << "." << fileName << endl;

    ////////////////////////////////
    // Read cloud
	// Create necessary point clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud);

	cout << "Input file has size: " << cloud->points.size() << endl;

	////////////////////////////////
    // Do a statistical outlier removal
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statisticalOutlierRemovalFilter;

    statisticalOutlierRemovalFilter.setInputCloud (cloud);
    statisticalOutlierRemovalFilter.setMeanK (sorKMean);
    statisticalOutlierRemovalFilter.setStddevMulThresh (sorStandardDeviation);
    statisticalOutlierRemovalFilter.filter (*tempCloud);

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