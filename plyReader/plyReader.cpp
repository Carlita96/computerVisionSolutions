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
	auto start = std::chrono::high_resolution_clock::now();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PLYReader Reader;
	Reader.read(fileName, *cloud);

    auto end = std::chrono::high_resolution_clock::now(); 
    std::chrono::duration<double> elapsed;
    elapsed = end - start;
	cout << "Time spent opening file is : " << elapsed.count() << endl;

  	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
  	viewer->addCoordinateSystem();
  	viewer->addPointCloud<pcl::PointXYZ>(cloud, "Point Cloud");
  	viewer->spin();


	cout << "Finished program" << endl;
	return 0;
}