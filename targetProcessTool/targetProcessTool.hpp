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

using namespace std;

#ifndef TARGETPROCESSTOOL_HPP
#define TARGETPROCESSTOOL_HPP

class TargetProcessTool
{
	public:
		TargetProcessTool(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
			double rightCenterXParam, double rightCenterYParam, double rightCenterZParam, double size);

		~TargetProcessTool();

		void defineTranslationRotation(float translationX, float translationY, 
			float translationZ, float rotationX, float rotationY, float rotationZ);
	
		pcl::PointCloud<pcl::PointXYZ>::Ptr getLeftTarget();

		pcl::PointCloud<pcl::PointXYZ>::Ptr getLeftCenterTarget();

		pcl::PointCloud<pcl::PointXYZ>::Ptr getRightTarget();

		pcl::PointCloud<pcl::PointXYZ>::Ptr getRightCenterTarget();

		pcl::PointCloud<pcl::PointXYZ>::Ptr getRightCenterUpTarget();

		pcl::PointCloud<pcl::PointXYZ>::Ptr getLeftLowTarget();
		
		void showTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr croppedCloud);

		void processTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr croppedCloud);

		// Depth
		bool isDepthZ = true;

	private:
		// Cropbox filter
		pcl::CropBox<pcl::PointXYZ> cropBoxFilter;

		// Input cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud;

		// Target positions
		double rightCenterX;
		double rightCenterY;
		double rightCenterZ;
		double size;

		// Target dimensions
		double upperRow;
		double lowerRow;
		double leftCenterColumn;
		double leftColumn;
		double rightCenterColumn;
		double rightColumn;
};

#endif // TARGETPROCESSTOOL_HPP