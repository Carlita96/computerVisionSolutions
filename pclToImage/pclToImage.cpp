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

    ////////////////////////////////
    // Read cloud
	// Create necessary point clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud);

// 	cout << "Input file has size: " << cloud->points.size() << endl;

// 	////////////////////////////////
//     // Do a statistical outlier removal
//     pcl::PCLPointCloud2::Ptr pc2ForDownsample (new pcl::PCLPointCloud2 ());
//     pcl::PCLPointCloud2::Ptr pc2Downsampled (new pcl::PCLPointCloud2 ());
//     // Transform pointcloud
//     pcl::toPCLPointCloud2(*cloud , *pc2ForDownsample);

//     // Create the filter
//     pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGridCloud;
//     voxelGridCloud.setInputCloud (pc2ForDownsample);
//     voxelGridCloud.setLeafSize (downsample, downsample, downsample);
//     voxelGridCloud.filter (*pc2Downsampled);

//     // Transform pointcloud
//     pcl::fromPCLPointCloud2(*pc2Downsampled , *tempCloud);

// 	////////////////////////////////
// 	// Create visualizer
//   	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
//   	viewer->addCoordinateSystem();

//   	// Color point clouds
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorInputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorOutputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::copyPointCloud(*cloud, *colorInputCloud);
// 	for (size_t i = 0; i < colorInputCloud->points.size(); i++) {
// 	    colorInputCloud->points[i].r = 0;
// 	    colorInputCloud->points[i].g = 255;
// 	    colorInputCloud->points[i].b = 0;
// 	}
// 	pcl::copyPointCloud(*tempCloud, *colorOutputCloud);
// 	for (size_t i = 0; i < colorOutputCloud->points.size(); i++) {
// 	    colorOutputCloud->points[i].r = 255;
// 	    colorOutputCloud->points[i].g = 0;
// 	    colorOutputCloud->points[i].b = 0;
// 	}

//   	// Add model point cloud
//   	viewer->addPointCloud<pcl::PointXYZRGB>(colorInputCloud, "Point Cloud");
//   	viewer->addPointCloud<pcl::PointXYZRGB>(colorOutputCloud, "Point Cloud Output");
//   	viewer->spin();

// 	*cloud = *tempCloud;

// 	cout << "Output file has size: " << cloud->points.size() << endl;

// 	pcl::io::savePCDFile( "outputCloud.pcd", *cloud, true );

// 	cout << "Finished program" << endl;
// 	return 0;
// }

#include <pcl/range_image/range_image.h>

int main (int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ> pointCloud;
  
  // Generate the data
  for (float y=-0.5f; y<=0.5f; y+=0.01f) {
    for (float z=-0.5f; z<=0.5f; z+=0.01f) {
      pcl::PointXYZ point;
      point.x = 2.0f - y;
      point.y = y;
      point.z = z;
      pointCloud.push_back(point);
    }
  }
  pointCloud.width = pointCloud.size();
  pointCloud.height = 1;
  
  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
  float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  float noiseLevel=0.00;
  float minRange = 0.0f;
  int borderSize = 1;
  
  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  
  std::cout << rangeImage << "\n";
}