#include <pcl/io/ply_io.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <chrono>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <math.h>

using namespace std;

int main (int argc, char** argv)
{
cout << "Starting program" << endl;

	////////////////////////////////
	// Read the input parameters
    if (argc != 6) {
		cout << "Input number of parameters is wrong. Input needed:\n\t- Directory of PCD file." 
			<< "\n\t- X of center right target.\n\t- Y of center right target.\n\t- Z of center right target."
			<< "\n\t- Size of cropbox for the target.\n\t- Boolean to set if depth is Z axis (1) or X axis (0). 1 is default value." << endl;
		return 0;
	} else {	
        std::string fileName = argv[1];
        cout << "Opening model file: " << fileName << endl;
        double centerCropboxX = atof(argv[2]); 
        double centerCropboxY = atof(argv[3]); 
        double centerCropboxZ = atof(argv[4]);
        double sizeBox = atof(argv[5]);
        bool isDepthZ = true;
        if (argc == 7 && atoi(argv[6]) == 0) {
			cout << "Depth axis is X" << endl;
			isDepthZ = false;
		} else {
			cout << "Depth axis is Z" << endl;
		}

        ////////////////////////////////
        // Read cloud
        // Create necessary point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr firstCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *firstCloud);
        cout << "Input cloud has size: " << firstCloud->points.size() << endl;

        ////////////////////////////////
        // Define minimum and maximum points
        Eigen::Vector4f minPoint = Eigen::Vector4f(-3, -3, -3, 0);
        Eigen::Vector4f maxPoint = Eigen::Vector4f(3, 3, 3, 0);
        Eigen::Vector3f translation = Eigen::Vector3f(0,0,0);
        Eigen::Vector3f rotation = Eigen::Vector3f(0,0,0);

        ////////////////////////////////
        // Do cropbox
        pcl::CropBox<pcl::PointXYZ> cropBoxFilter;
        // Set the cloud
        cropBoxFilter.setInputCloud(firstCloud);
        
        // Define the minimum and maximum point. 
        // It could be directly in the wanted position from world origin with empty translation and rotation
        // or with the position in the origin (just define dimensions) and moved with the translation and rotation.
        cropBoxFilter.setMin(minPoint);
        cropBoxFilter.setMax(maxPoint);

        // Translate the box
        cropBoxFilter.setTranslation(translation);
        // Rotate the box
        cropBoxFilter.setRotation(rotation);

        // Get the filtered cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        cropBoxFilter.setNegative (false);
        cropBoxFilter.filter(*cloud);
        cout << "Output cloud has size: " << cloud->points.size() << endl;

        ////////////////////////////////
        // Define minimum and maximum points
        minPoint = Eigen::Vector4f(centerCropboxX - sizeBox/2, centerCropboxY - sizeBox/2, centerCropboxZ - sizeBox/2, 0);
        maxPoint = Eigen::Vector4f(centerCropboxX + sizeBox/2, centerCropboxY + sizeBox/2, centerCropboxZ + sizeBox/2, 0);
        translation = Eigen::Vector3f(0,0,0);
        rotation = Eigen::Vector3f(0,0,0);

        ////////////////////////////////
        // Do cropbox
        // Set the cloud
        cropBoxFilter.setInputCloud(cloud);
        
        // Define the minimum and maximum point. 
        // It could be directly in the wanted position from world origin with empty translation and rotation
        // or with the position in the origin (just define dimensions) and moved with the translation and rotation.
        cropBoxFilter.setMin(minPoint);
        cropBoxFilter.setMax(maxPoint);

        // Translate the box
        cropBoxFilter.setTranslation(translation);
        // Rotate the box
        cropBoxFilter.setRotation(rotation);

        // Get the filtered cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr croppedCloud (new pcl::PointCloud<pcl::PointXYZ>);
        cropBoxFilter.setNegative (false);
        cropBoxFilter.filter(*croppedCloud);
        cout << "Output cloud has size: " << croppedCloud->points.size() << endl;

        // Color point clouds
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorInputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCroppedCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*cloud, *colorInputCloud);
        for (size_t i = 0; i < colorInputCloud->points.size(); i++) {
            colorInputCloud->points[i].r = 255;
            colorInputCloud->points[i].g = 255;
            colorInputCloud->points[i].b = 255;
        }
        pcl::copyPointCloud(*croppedCloud, *colorCroppedCloud);
        for (size_t i = 0; i < colorCroppedCloud->points.size(); i++) {
            colorCroppedCloud->points[i].r = 255;
            colorCroppedCloud->points[i].g = 0;
            colorCroppedCloud->points[i].b = 0;
        }

        ////////////////////////////////
        // Create visualizer
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
        viewer->addCoordinateSystem();
        // Add model point cloud
        viewer->addPointCloud<pcl::PointXYZRGB>(colorInputCloud, "Point Cloud");
        viewer->addPointCloud<pcl::PointXYZRGB>(colorCroppedCloud, "Point Cloud Output");
        viewer->spin();

        if (isDepthZ) {
            /////////////////////////////
            // Calculate mean
            int count = 0;
            double mean = 0.0;
            for (size_t i = 0; i < croppedCloud->points.size(); i++) {
                mean += croppedCloud->points[i].z;
                count ++;
            }
            mean = mean / count;
            cout << "Mean Z is: " << mean << " meters." << endl; 
            // Calculate standard deviation
            double standardDeviation = 0.0;
            for (size_t i = 0; i < croppedCloud->points.size(); i++) {
                standardDeviation += pow(croppedCloud->points[i].z - mean, 2);
            }
            standardDeviation = sqrt(standardDeviation/(count-1));
            cout << "Standard deviation is: " << standardDeviation << " meters." << endl; 
            // Calculate dimensions
            double minX = 1000;
            double minY = 1000;
            double maxX = -1000;
            double maxY = -1000;
            for (size_t i = 0; i < croppedCloud->points.size(); i++) {
                if (croppedCloud->points[i].x < minX) {
                    minX = croppedCloud->points[i].x;
                }
                if (croppedCloud->points[i].y < minY) {
                    minY = croppedCloud->points[i].y;
                }
                if (croppedCloud->points[i].x > maxX) {
                    maxX = croppedCloud->points[i].x;
                }
                if (croppedCloud->points[i].y > maxY) {
                    maxY = croppedCloud->points[i].y;
                }
            }
            cout << "Dimension of target is X: " << maxX-minX << " meters and Y: " << maxY-minY << " meters." << endl; 
        } else {
            /////////////////////////////
            // Calculate mean
            int count = 0;
            double mean = 0.0;
            for (size_t i = 0; i < croppedCloud->points.size(); i++) {
                mean += croppedCloud->points[i].x;
                count ++;
            }
            mean = mean / count;
            cout << "Mean X is: " << mean << " meters." << endl; 
            // Calculate standard deviation
            double standardDeviation = 0.0;
            for (size_t i = 0; i < croppedCloud->points.size(); i++) {
                standardDeviation += pow(croppedCloud->points[i].x - mean, 2);
            }
            standardDeviation = sqrt(standardDeviation/(count-1));
            cout << "Standard deviation is: " << standardDeviation << " meters." << endl; 
            // Calculate dimensions
            double minZ = 1000;
            double minY = 1000;
            double maxZ = -1000;
            double maxY = -1000;
            for (size_t i = 0; i < croppedCloud->points.size(); i++) {
                if (croppedCloud->points[i].x < minZ) {
                    minZ = croppedCloud->points[i].z;
                }
                if (croppedCloud->points[i].y < minY) {
                    minY = croppedCloud->points[i].y;
                }
                if (croppedCloud->points[i].x > maxZ) {
                    maxZ = croppedCloud->points[i].z;
                }
                if (croppedCloud->points[i].y > maxY) {
                    maxY = croppedCloud->points[i].y;
                }
            }
            cout << "Dimension of target is Z: " << maxZ-minZ << " meters and Y: " << maxY-minY << " meters." << endl;
        }


        cout << "Finished program" << endl;
        return 0;
    }
}