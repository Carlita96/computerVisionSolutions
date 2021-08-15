#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/transformation_estimation_svd.h>

typedef pcl::PointXYZ PointT;

double TranslationX = 1.498;
double TranslationY = -0.594;
double TranslationZ = -0.219;
double RotationQuaternionX = 0.997;
double RotationQuaternionY = 0.001;
double RotationQuaternionZ = -0.072;
double RotationQuaternionW = 0.017;

using namespace std;

Eigen::Matrix3d getRotationMatrixFromQuaternions(const Eigen::Vector4d& quaternions)
{
    /*
    Method that can calculate the rotation matrix from quaternions.
    * Input:
        - quaternions: vector containing qx[0], qy[0], qz[0] and qw[0].
    * Output:
        - rotation matrix: of size 3x3.
    */
    // Get rotation matrix
    // Row 0
    double r00 = 1 - (2 * (quaternions[1]  * quaternions[1] + quaternions[2] * quaternions[2]));
    double r01 = 2 * (quaternions[0]  * quaternions[1] - quaternions[2] * quaternions[3]);
    double r02 = 2 * (quaternions[0]  * quaternions[2] + quaternions[1] * quaternions[3]);
    // Row 1
    double r10 = 2 * (quaternions[0]  * quaternions[1] + quaternions[2] * quaternions[3]);
    double r11 = 1 - (2 * (quaternions[0]  * quaternions[0] + quaternions[2] * quaternions[2]));
    double r12 = 2 * (quaternions[1]  * quaternions[2] - quaternions[0] * quaternions[3]);
    // Row 2
    double r20 = 2 * (quaternions[0]  * quaternions[2] - quaternions[1] * quaternions[3]);
    double r21 = 2 * (quaternions[1]  * quaternions[2] + quaternions[0] * quaternions[3]);
    double r22 = 1 - (2 * (quaternions[0]  * quaternions[0] + quaternions[1] * quaternions[1]));

    // Calculate rotation matrix
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix << r00, r01, r02, r10, r11, r12, r20, r21, r22;
    return rotationMatrix;
}

pcl::PointXYZ getTransformedPoint(pcl::PointXYZ input_point)
{
    Eigen::Matrix3d rotationMatrix = getRotationMatrixFromQuaternions(Eigen::Vector4d(RotationQuaternionX, 
                    RotationQuaternionY, RotationQuaternionZ, RotationQuaternionW));
    Eigen::Vector3d input_point_vector = Eigen::Vector3d(input_point.x, input_point.y, input_point.z);
    Eigen::Vector3d translation_vector = Eigen::Vector3d(TranslationX, TranslationY, TranslationZ);

    Eigen::Vector3d output_point_vector = rotationMatrix * input_point_vector + translation_vector;
    pcl::PointXYZ output_point;
    output_point.x = output_point_vector[0];
    output_point.y = output_point_vector[1];
    output_point.z = output_point_vector[2];
    return output_point;
}

Eigen::Affine3f getRotationMatrix(Eigen::Vector3f source,
                                  Eigen::Vector3f target) {
  Eigen::Vector3f rotation_vector = target.cross(source);
  rotation_vector.normalize();
  double theta = acos(source[2] / sqrt(pow(source[0], 2) + pow(source[1], 2) +
                                       pow(source[2], 2)));

  Eigen::Matrix3f rotation =
      Eigen::AngleAxis<float>(theta, rotation_vector) * Eigen::Scaling(1.0f);
  Eigen::Affine3f rot(rotation);
  return rot;
}

int
main (int argc, char** argv)
{
    ///////////////////////////////////////////
    // READ CLOUD
    ///////////////////////////////////////////
    // Create clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilter (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPlane (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloudPlaneNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCircle (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudEdges (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXyPlane (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXyCircles (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXyCircleCenters (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCircleCenters (new pcl::PointCloud<pcl::PointXYZ>);
    // Read point cloud
    pcl::PCDReader reader;
    reader.read ("target.pcd", *cloud);
    cout << "PointCloud has: " << cloud->size () << " data points." << endl;

    ///////////////////////////////////////////
    // PASSTHROUGH FILTER
    ///////////////////////////////////////////
    // Remove depth 
    pcl::PassThrough<pcl::PointXYZ> passthroughFilter;
    passthroughFilter.setInputCloud (cloud);
    passthroughFilter.setFilterFieldName ("x");
    passthroughFilter.setFilterLimits (0, 10);
    passthroughFilter.filter (*cloudFilter);

    ///////////////////////////////////////////
    // FIND PLANAR TARGET
    ///////////////////////////////////////////
    // Plane segmentation
    pcl::ModelCoefficients::Ptr coefficientsPlane(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliersPlane(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> plane_segmentation;
    plane_segmentation.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    plane_segmentation.setDistanceThreshold(0.01);
    plane_segmentation.setMethodType(pcl::SAC_RANSAC);
    plane_segmentation.setAxis(Eigen::Vector3f(0, 0, 1));
    plane_segmentation.setEpsAngle(10 * 3.14159/180);
    plane_segmentation.setOptimizeCoefficients(true);
    plane_segmentation.setMaxIterations(1000);
    plane_segmentation.setInputCloud(cloudFilter);
    plane_segmentation.segment(*inliersPlane, *coefficientsPlane);

    // Get cloud of plane
    pcl::ExtractIndices<pcl::PointXYZ> extractPlane;
    extractPlane.setInputCloud (cloudFilter);
    extractPlane.setIndices (inliersPlane);
    extractPlane.setNegative (false);
    extractPlane.filter (*cloudPlane);

    ///////////////////////////////////////////
    // COMPUTE NORMALS
    ///////////////////////////////////////////
    // Normal estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ> ());
    normalEstimator.setInputCloud(cloudPlane);
    normalEstimator.setSearchMethod(kdtree);
    normalEstimator.setRadiusSearch(0.03);
    normalEstimator.compute(*cloudPlaneNormals);

    ///////////////////////////////////////////
    // EDGE DETECTION
    ///////////////////////////////////////////
    // Normal estimation
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundaryEstimator;
    boundaryEstimator.setInputCloud(cloudPlane);
    boundaryEstimator.setInputNormals(cloudPlaneNormals);
    boundaryEstimator.setRadiusSearch(0.03);
    boundaryEstimator.setSearchMethod(kdtree);
    boundaryEstimator.compute(boundaries);
    for (int i=0; i<cloudPlane->points.size(); i++) {
        if (boundaries.points[i].boundary_point == 1) {
            cloudEdges->push_back(cloudPlane->points[i]);
        }
    }

    ///////////////////////////////////////////
    // PROJECT TO PLANE
    ///////////////////////////////////////////
    // Calculate rotation matrix from planar target to XY plane
    Eigen::Vector3f xyPlaneNormalVector, floorPlaneNormalVector;
    xyPlaneNormalVector[0] = 0.0;
    xyPlaneNormalVector[1] = 0.0;
    xyPlaneNormalVector[2] = -1.0;
    floorPlaneNormalVector[0] = coefficientsPlane->values[0];
    floorPlaneNormalVector[1] = coefficientsPlane->values[1];
    floorPlaneNormalVector[2] = coefficientsPlane->values[2];

    // Get rotation
    Eigen::Affine3f rotation =
        getRotationMatrix(floorPlaneNormalVector, xyPlaneNormalVector);
    pcl::transformPointCloud(*cloudEdges, *cloudXyPlane, rotation);
    xyPlaneNormalVector[0] = 0.0;
    xyPlaneNormalVector[1] = 0.0;
    xyPlaneNormalVector[2] = -1.0;
    floorPlaneNormalVector[0] = coefficientsPlane->values[0];
    floorPlaneNormalVector[1] = coefficientsPlane->values[1];
    floorPlaneNormalVector[2] = coefficientsPlane->values[2];
    // Get rotation
    Eigen::Affine3f rotationBack = rotation.inverse();

    // Force pattern points to belong to computed plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr auxCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr auxCloudRotated (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ auxPoint;
    auxPoint.x = 0;
    auxPoint.y = 0;
    auxPoint.z = (-coefficientsPlane->values[3] / coefficientsPlane->values[2]);
    auxCloud->push_back(auxPoint);
    pcl::transformPointCloud(*auxCloud, *auxCloudRotated, rotation);
    double zCoordXyPlane = auxCloudRotated->at(0).z;

    ///////////////////////////////////////////
    // GET CIRCLES
    ///////////////////////////////////////////
    // RANSAC circle detection
    while(cloudXyCircleCenters->points.size() < 4) {
        // Get circle
        pcl::ModelCoefficients::Ptr coefficientsCircles(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliersCircles(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> circleSegmentation;
        circleSegmentation.setModelType(pcl::SACMODEL_CIRCLE2D);
        circleSegmentation.setDistanceThreshold(0.01);
        circleSegmentation.setMethodType(pcl::SAC_RANSAC);
        circleSegmentation.setOptimizeCoefficients(true);
        circleSegmentation.setMaxIterations(10000);
        circleSegmentation.setRadiusLimits(0.09, 0.15);
        circleSegmentation.setInputCloud(cloudXyPlane);
        circleSegmentation.segment(*inliersCircles, *coefficientsCircles);

        // Get cloud of circle
        pcl::PointCloud<pcl::PointXYZ> cloudCircle;
        pcl::ExtractIndices<pcl::PointXYZ> extractCircles;
        extractCircles.setInputCloud(cloudXyPlane);
        extractCircles.setIndices(inliersCircles);
        extractCircles.setNegative(false);
        extractCircles.filter(cloudCircle);
        *cloudXyCircles = *cloudXyCircles + cloudCircle;

        // Delete cloud of circle
        extractCircles.setInputCloud(cloudXyPlane);
        extractCircles.setIndices(inliersCircles);
        extractCircles.setNegative(true);
        extractCircles.filter(*cloudXyPlane);

        // Add to list
        if (inliersCircles->indices.size() == 0) {
            cout << "Circle segmentation failed... Exiting" << endl;
            return 1;
        }

        // Save centers
        pcl::PointXYZ point;
        point.x = coefficientsCircles->values[0];
        point.y = coefficientsCircles->values[1];
        point.z = zCoordXyPlane;
        cloudXyCircleCenters->push_back(point);
    }
    // Rotate center of circles
    pcl::transformPointCloud(*cloudXyCircleCenters, *cloudCircleCenters, rotationBack);


    ///////////////////////////////////////////
    // FIND CALIBRATION PARAMETERS
    ///////////////////////////////////////////
    // Transform points
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ point;
    for (int i=0; i<cloudCircleCenters->points.size(); i++) {
        point = cloudCircleCenters->points[i];
        pcl::PointXYZ transformedPoint = getTransformedPoint(point);
        transformedCloud->push_back(transformedPoint);
    }

    // Get final transformation
    Eigen::Matrix4f final_transformation;
    const pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,
                                                        pcl::PointXYZ>
        trans_est_svd(true);
    trans_est_svd.estimateRigidTransformation(*cloudCircleCenters, *transformedCloud,
                                                final_transformation);

    // Get quaterions
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix  << (double)final_transformation(0, 0), (double)final_transformation(0, 1),
                    (double)final_transformation(0, 2), (double)final_transformation(1, 0),
                    (double)final_transformation(1, 1), (double)final_transformation(1, 2),
                    (double)final_transformation(2, 0), (double)final_transformation(2, 1),
                    (double)final_transformation(2, 2);
    Eigen::Quaterniond quaternions(rotation_matrix);

    // Translation
    Eigen::Vector3d translation;
    translation = Eigen::Vector3d(final_transformation(0, 3), final_transformation(1, 3),
                    final_transformation(2, 3));

    // Print
    cout << "Calibration finished." << endl;
    cout << "Extrinsic parameters:" << endl;
    cout << "x = " << translation[0] << "\ty = " << translation[1] << "\tz = " << translation[2] << endl;
    cout << "x = " << quaternions.x() << "\ty = " << quaternions.y() << "\tz = " 
            << quaternions.z() << "\tw = " << quaternions.w() << endl;
    cout << "Error:" << endl;
    cout << "x = " << abs(translation[0]-TranslationX) << "\ty = " << abs(translation[1]-TranslationY)
            << "\tz = " << abs(translation[2]-TranslationZ) << endl;
    cout << "x = " << abs(quaternions.x()-RotationQuaternionX) << "\ty = " << abs(quaternions.y()-RotationQuaternionY) << "\tz = " 
            << abs(quaternions.z()-RotationQuaternionZ) << "\tw = " << abs(quaternions.w()-RotationQuaternionW) << endl;


    ////////////////////////////////
    // Create visualizer
    ////////////////////////////////
  	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
  	viewer->addCoordinateSystem();
    
    // Color point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorInputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorPlaneCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorEdgesCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorXyPlaneCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorXyCircleCentersCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorXyCirclesClouds (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCircleCentersCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloudFilter, *colorInputCloud);
    for (size_t i = 0; i < colorInputCloud->points.size(); i++) {
        colorInputCloud->points[i].r = 0;
        colorInputCloud->points[i].g = 0;
        colorInputCloud->points[i].b = 0;
    }
    pcl::copyPointCloud(*cloudPlane, *colorPlaneCloud);
    for (size_t i = 0; i < colorPlaneCloud->points.size(); i++) {
        colorPlaneCloud->points[i].r = 0;
        colorPlaneCloud->points[i].g = 0;
        colorPlaneCloud->points[i].b = 255;
    }
    pcl::copyPointCloud(*cloudEdges, *colorEdgesCloud);
    for (size_t i = 0; i < colorEdgesCloud->points.size(); i++) {
        colorEdgesCloud->points[i].r = 0;
        colorEdgesCloud->points[i].g = 255;
        colorEdgesCloud->points[i].b = 0;
    }
    pcl::copyPointCloud(*cloudXyPlane, *colorXyPlaneCloud);
    for (size_t i = 0; i < colorXyPlaneCloud->points.size(); i++) {
        colorXyPlaneCloud->points[i].r = 0;
        colorXyPlaneCloud->points[i].g = 0;
        colorXyPlaneCloud->points[i].b = 255;
    }
    pcl::copyPointCloud(*cloudXyCircleCenters, *colorXyCircleCentersCloud);
    for (size_t i = 0; i < colorXyCircleCentersCloud->points.size(); i++) {
        colorXyCircleCentersCloud->points[i].r = 255;
        colorXyCircleCentersCloud->points[i].g = 0;
        colorXyCircleCentersCloud->points[i].b = 0;
    }
    pcl::copyPointCloud(*cloudXyCircles, *colorXyCirclesClouds);
    for (size_t i = 0; i < colorXyCirclesClouds->points.size(); i++) {
        colorXyCirclesClouds->points[i].r = 0;
        colorXyCirclesClouds->points[i].g = 255;
        colorXyCirclesClouds->points[i].b = 255;
    }
    pcl::copyPointCloud(*cloudCircleCenters, *colorCircleCentersCloud);
    for (size_t i = 0; i < colorCircleCentersCloud->points.size(); i++) {
        colorCircleCentersCloud->points[i].r = 255;
        colorCircleCentersCloud->points[i].g = 0;
        colorCircleCentersCloud->points[i].b = 0;
    }

    // Add model point cloud
  	viewer->addPointCloud<pcl::PointXYZRGB>(colorInputCloud, "Point Cloud");
  	viewer->addPointCloud<pcl::PointXYZRGB>(colorPlaneCloud, "Plane Point Cloud");
  	viewer->addPointCloud<pcl::PointXYZRGB>(colorEdgesCloud, "Edge Point Cloud");
  	viewer->addPointCloud<pcl::PointXYZRGB>(colorXyPlaneCloud, "XY Plane Point Cloud");
  	viewer->addPointCloud<pcl::PointXYZRGB>(colorXyCircleCentersCloud, "Circle Centers Point Cloud");
  	viewer->addPointCloud<pcl::PointXYZRGB>(colorXyCirclesClouds, "Circle Point Cloud");
  	viewer->addPointCloud<pcl::PointXYZRGB>(colorCircleCentersCloud, "Circles Centers Point Cloud");
  	viewer->spin();

    return (0);
}
