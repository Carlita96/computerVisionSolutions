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

	std::string fileName = argv[1];
	cout << "Opening file: " << fileName << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud);
	pcl::io::savePLYFileBinary("output.ply", *cloud);

	cout << "Finished program" << endl;
	return 0;
}