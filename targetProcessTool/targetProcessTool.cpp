/*
This code can get several parameters from a target of specific shape (check README).
Parameters printed in terminal:
	- Distance error of each target.
	- Standard deviation for each target.
	- X and Y size calculation.
It will show a window with a PCD viewer showing how the targets have been found in red.
The inputs to the program are:
	- Path to PCD file: path to the PCD file to treat.
	- X of the center right target.
	- Y of the center right target.
	- Z of the center right target.
	- Size of the cropbox to get the target.
	- Boolean to set if depth is in Z axis (1) or X axis (0). 1 as default.
*/

#include "targetProcessTool.hpp"

using namespace std;

TargetProcessTool::TargetProcessTool(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	double rightCenterXParam, double rightCenterYParam, double rightCenterZParam, double sizeParam)
{
	// Define target parameters
	upperRow = 0.12;
	lowerRow = -0.12;
	leftCenterColumn = -0.12;
	leftColumn = -0.12 - 0.11;
	rightCenterColumn = 0;
	rightColumn = 0.11;
	
	// Input cloud
	inputCloud = cloud;

	// Set the cloud
    cropBoxFilter.setInputCloud(cloud);

	// Define center of target
	rightCenterX = rightCenterXParam;
	rightCenterY = rightCenterYParam;
	rightCenterZ = rightCenterZParam;
	size = sizeParam;

	defineTranslationRotation(0, 0, 0, 0, 0, 0);
}

TargetProcessTool::~TargetProcessTool()
{
}


void TargetProcessTool::defineTranslationRotation(float translationX, float translationY, 
	float translationZ, float rotationX, float rotationY, float rotationZ)
{
	// Define paramters
	Eigen::Vector3f translation = Eigen::Vector3f(0,0,0);
	Eigen::Vector3f rotation = Eigen::Vector3f(0,0,0);
	// Translate the box
    cropBoxFilter.setTranslation(translation);
    // Rotate the box
    cropBoxFilter.setRotation(rotation);

	return;
}

void TargetProcessTool::showTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr croppedCloud)
{
	// Color point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorInputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCroppedCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*inputCloud, *colorInputCloud);
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

	return;
}

void TargetProcessTool::processTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr croppedCloud)
{
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

		return;
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

		return;

	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TargetProcessTool::getRightCenterTarget()
{
	cout << "Right center target" << endl;
	// Create cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Define points
	Eigen::Vector4f minPoint = Eigen::Vector4f(rightCenterX-size/2, rightCenterY-size/2, rightCenterZ-size/2, 0);
	Eigen::Vector4f maxPoint = Eigen::Vector4f(rightCenterX+size/2, rightCenterY+size/2, rightCenterZ+size/2, 0);

	// Change cropbox
	cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);

    // Get the filtered cloud
    cropBoxFilter.setNegative (false);
    cropBoxFilter.filter(*outputCloud);

	showTarget(outputCloud);
	processTarget(outputCloud);

	return outputCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TargetProcessTool::getLeftCenterTarget()
{
	cout << "Left center target" << endl;
	// Create cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Define points
	Eigen::Vector4f minPoint = Eigen::Vector4f(rightCenterX-size/2+leftCenterColumn, rightCenterY-size/2, rightCenterZ-size/2, 0);
	Eigen::Vector4f maxPoint = Eigen::Vector4f(rightCenterX+size/2+leftCenterColumn, rightCenterY+size/2, rightCenterZ+size/2, 0);

	// Change cropbox
	cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);

    // Get the filtered cloud
    cropBoxFilter.setNegative (false);
    cropBoxFilter.filter(*outputCloud);

	showTarget(outputCloud);
	processTarget(outputCloud);

	return outputCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TargetProcessTool::getLeftTarget()
{
	cout << "Left target" << endl;
	// Create cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Define points
	Eigen::Vector4f minPoint = Eigen::Vector4f(rightCenterX-size/2+leftColumn, rightCenterY-size/2, rightCenterZ-size/2, 0);
	Eigen::Vector4f maxPoint = Eigen::Vector4f(rightCenterX+size/2+leftColumn, rightCenterY+size/2, rightCenterZ+size/2, 0);

	// Change cropbox
	cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);

    // Get the filtered cloud
    cropBoxFilter.setNegative (false);
    cropBoxFilter.filter(*outputCloud);

	showTarget(outputCloud);
	processTarget(outputCloud);

	return outputCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TargetProcessTool::getRightTarget()
{
	cout << "Right target" << endl;
	// Create cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Define points
	Eigen::Vector4f minPoint = Eigen::Vector4f(rightCenterX-size/2+rightColumn, rightCenterY-size/2, rightCenterZ-size/2, 0);
	Eigen::Vector4f maxPoint = Eigen::Vector4f(rightCenterX+size/2+rightColumn, rightCenterY+size/2, rightCenterZ+size/2, 0);

	// Change cropbox
	cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);

    // Get the filtered cloud
    cropBoxFilter.setNegative (false);
    cropBoxFilter.filter(*outputCloud);

	showTarget(outputCloud);
	processTarget(outputCloud);

	return outputCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TargetProcessTool::getRightCenterUpTarget()
{
	cout << "Right center up target" << endl;
	// Create cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Define points
	Eigen::Vector4f minPoint = Eigen::Vector4f(rightCenterX-size/2, rightCenterY-size/2+upperRow, rightCenterZ-size/2, 0);
	Eigen::Vector4f maxPoint = Eigen::Vector4f(rightCenterX+size/2, rightCenterY+size/2+upperRow, rightCenterZ+size/2, 0);

	// Change cropbox
	cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);

    // Get the filtered cloud
    cropBoxFilter.setNegative (false);
    cropBoxFilter.filter(*outputCloud);

	showTarget(outputCloud);
	processTarget(outputCloud);

	return outputCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TargetProcessTool::getLeftLowTarget()
{
	cout << "Left low target" << endl;
	// Create cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Define points
	Eigen::Vector4f minPoint = Eigen::Vector4f(rightCenterX-size/2+leftColumn, rightCenterY-size/2+lowerRow, rightCenterZ-size/2, 0);
	Eigen::Vector4f maxPoint = Eigen::Vector4f(rightCenterX+size/2+leftColumn, rightCenterY+size/2+lowerRow, rightCenterZ+size/2, 0);

	// Change cropbox
	cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);

    // Get the filtered cloud
    cropBoxFilter.setNegative (false);
    cropBoxFilter.filter(*outputCloud);

	showTarget(outputCloud);
	processTarget(outputCloud);

	return outputCloud;
}

int main (int argc, char** argv)
{
	cout << "Starting program" << endl;

	////////////////////////////////
	// Read the input parameters
	if (argc < 6 || argc > 7) {
		cout << "Input number of parameters is wrong. Input needed:\n\t- Directory of PCD file." 
			<< "\n\t- X of center right target.\n\t- Y of center right target.\n\t- Z of center right target."
			<< "\n\t- Size of cropbox for the target.\n\t- Boolean to set if depth is Z axis (1) or X axis (0). 1 is default value." << endl;
		return 0;
	} else {
		std::string fileName = argv[1];
		cout << "Opening model file: " << fileName << endl;
		double rightCenterX = atof(argv[2]); 
		double rightCenterY = atof(argv[3]); 
		double rightCenterZ = atof(argv[4]);
		double size = atof(argv[5]);
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
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud);

		cout << "Input file has size: " << cloud->points.size() << endl;

		////////////////////////////////
		// Create the Target Process Tool
		TargetProcessTool targetProcessTool(cloud, rightCenterX, rightCenterY, rightCenterZ, size);
		targetProcessTool.isDepthZ = isDepthZ;
		targetProcessTool.getRightCenterTarget();
		targetProcessTool.getLeftCenterTarget();
		targetProcessTool.getLeftTarget();
		targetProcessTool.getRightTarget();
		targetProcessTool.getRightCenterUpTarget();
		targetProcessTool.getLeftLowTarget();

		cout << "Finished program" << endl;
		return 0;
	}
}