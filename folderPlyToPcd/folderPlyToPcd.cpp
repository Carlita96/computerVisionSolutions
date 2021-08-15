#include <pcl/io/ply_io.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <experimental/filesystem>
// #include <filesystem>
#include <chrono>

using namespace std;

int
main (int argc, char** argv)
{
	cout << "Starting program" << endl;

	namespace stdfs = std::experimental::filesystem;
	const stdfs::directory_iterator end{};
	std::vector<std::string> filenames;

	std::string fileName = argv[1];
	cout << "Opening ply file: " << fileName << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PLYReader Reader;
	Reader.read(fileName, *cloud);
	pcl::io::savePCDFile( "outputCloud.pcd", *cloud, true );


	cout << "Finished program" << endl;
	return 0;
}