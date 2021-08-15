#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZ PointT;

using namespace std;

int
main (int argc, char** argv)
{

    ////////////////////////////////
    // Read the input parameters
    std::string fileName = argv[1];
    std::string axis = argv[2];
	  double minAxis = atof(argv[3]); 
	  double maxAxis = atof(argv[4]); 
    cout << "Opening model file: " << fileName << endl;

    // All the objects needed
    pcl::PCDReader reader;
    pcl::PassThrough<PointT> pass;
    pcl::PCDWriter writer;

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloudOutput (new pcl::PointCloud<PointT>);

    // Read in the cloud data
    reader.read (fileName, *cloud);
    cout << "PointCloud has: " << cloud->size () << " data points." << endl;

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName (axis);
    pass.setFilterLimits (minAxis, maxAxis);
    pass.filter (*cloudOutput);
    cout << "PointCloud after filtering has: " << cloudOutput->size () << " data points." << endl;

    writer.write ("output.pcd", *cloudOutput, false);

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
    pcl::copyPointCloud(*cloudOutput, *colorOutputCloud);
    for (size_t i = 0; i < colorOutputCloud->points.size(); i++) {
        colorOutputCloud->points[i].r = 255;
        colorOutputCloud->points[i].g = 0;
        colorOutputCloud->points[i].b = 0;
    }

    // Add model point cloud
  	viewer->addPointCloud<pcl::PointXYZRGB>(colorInputCloud, "Point Cloud");
  	viewer->addPointCloud<pcl::PointXYZRGB>(colorOutputCloud, "Cylinder Point Cloud");
  	viewer->spin();


    return (0);
}