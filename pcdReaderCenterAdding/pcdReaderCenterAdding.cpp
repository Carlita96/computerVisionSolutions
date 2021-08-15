#include <pcl/io/ply_io.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <chrono>

using namespace std;

int
main (int argc, char** argv)
{
	cout << "Starting program" << endl;

	// Read the input paramters
	std::string fileName = argv[1];
	float centerX = atof(argv[2]);
	float centerY = atof(argv[3]);
	float centerZ = atof(argv[4]);

	// Open PCD file measuring time
	cout << "Opening file: " << fileName << endl;
	auto start = std::chrono::high_resolution_clock::now();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud);
    auto end = std::chrono::high_resolution_clock::now(); 
    std::chrono::duration<double> elapsed;
    elapsed = end - start;
	cout << "Time spent opening file is : " << elapsed.count() << endl;

	// Create a point to show in the visualizer
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr centerCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointXYZRGB center;
	center.x = centerX;
	center.y = centerY;
	center.z = centerZ;
	center.r = 255;
	center.g = 0;
	center.b = 0;
	centerCloud->push_back(center);

	// Show visualizer
  	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
  	viewer->addCoordinateSystem();
  	viewer->addPointCloud<pcl::PointXYZ>(cloud, "Point Cloud");
  	viewer->addPointCloud<pcl::PointXYZRGB>(centerCloud, "Center");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "Center");
  	viewer->spin();


	cout << "Finished program" << endl;
	return 0;
}