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
    cout << "Opening model file: " << fileName << endl;

    // Create clouds
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloudPlane (new pcl::PointCloud<PointT> ());
    pcl::PointCloud<PointT>::Ptr cloudSphere (new pcl::PointCloud<PointT> ());
    pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloudFiltered2 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficientsPlane (new pcl::ModelCoefficients), coefficientsSphere (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliersPlane (new pcl::PointIndices), inliersSphere (new pcl::PointIndices);

    // Read point cloud
    pcl::PCDReader reader;
    reader.read (fileName, *cloud);
    cout << "PointCloud has: " << cloud->size () << " data points." << endl;

    // Pass Through Filter
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0, 10);
    pass.filter (*cloudFiltered);
    cout << "PointCloud after filtering has: " << cloudFiltered->size () << " data points." << endl;

    // Estimate point normals
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloudFiltered);
    ne.setKSearch (50);
    ne.compute (*cloudNormals);

    // Plane segmentation
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    pcl::ExtractIndices<PointT> extract;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloudFiltered);
    seg.setInputNormals (cloudNormals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliersPlane, *coefficientsPlane);
    cout << "Plane coefficients: " << *coefficientsPlane << endl;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloudFiltered);
    extract.setIndices (inliersPlane);
    extract.setNegative (false);

    // Write the planar inliers to disk
    extract.filter (*cloudPlane);
    
    // Remove the planar inliers, extract the rest
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract.setNegative (true);
    extract.filter (*cloudFiltered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloudNormals);
    extract_normals.setIndices (inliersPlane);
    extract_normals.filter (*cloudNormals2);

    // Sphere Segmentation
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_SPHERE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0.2, 0.8);
    seg.setInputCloud (cloudFiltered2);
    seg.setInputNormals (cloudNormals2);

    // Obtain the sphere inliers and coefficients
    seg.segment (*inliersSphere, *coefficientsSphere);
    cout << "Sphere coefficients: " << *coefficientsSphere << endl;

    // Write the sphere inliers to disk
    extract.setInputCloud (cloudFiltered2);
    extract.setIndices (inliersSphere);
    extract.setNegative (false);
    extract.filter (*cloudSphere);

    ////////////////////////////////
	  // Create visualizer
  	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
  	viewer->addCoordinateSystem();

    // Color point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorInputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorPlaneCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorSphereCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *colorInputCloud);
    for (size_t i = 0; i < colorInputCloud->points.size(); i++) {
        colorInputCloud->points[i].r = 0;
        colorInputCloud->points[i].g = 0;
        colorInputCloud->points[i].b = 255;
    }
    pcl::copyPointCloud(*cloudPlane, *colorPlaneCloud);
    for (size_t i = 0; i < colorPlaneCloud->points.size(); i++) {
        colorPlaneCloud->points[i].r = 255;
        colorPlaneCloud->points[i].g = 0;
        colorPlaneCloud->points[i].b = 0;
    }
    pcl::copyPointCloud(*cloudSphere, *colorSphereCloud);
    for (size_t i = 0; i < colorSphereCloud->points.size(); i++) {
        colorSphereCloud->points[i].r = 0;
        colorSphereCloud->points[i].g = 255;
        colorSphereCloud->points[i].b = 0;
    }

    // Add model point cloud
  	viewer->addPointCloud<pcl::PointXYZRGB>(colorInputCloud, "Point Cloud");
  	viewer->addPointCloud<pcl::PointXYZRGB>(colorPlaneCloud, "Plane Point Cloud");
  	viewer->addPointCloud<pcl::PointXYZRGB>(colorSphereCloud, "Sphere Point Cloud");
  	viewer->spin();


    return (0);
}
