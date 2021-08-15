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

float getDistanceBetweenPoints(pcl::PointXYZ point1, pcl::PointXYZ point2) 
{
    float difX = point1.x - point2.x;
    float difY = point1.y - point2.y;
    float difZ = point1.z - point2.z;
    float distance = sqrt(pow(difX,2) + pow(difY,2) + pow(difZ,2));
    return abs(distance);
}

int
main (int argc, char** argv)
{

    ////////////////////////////////
    // Read the input parameters
    std::string fileName = argv[1];
    cout << "Opening model file: " << fileName << endl;

    // Create clouds
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    // Read point cloud
    pcl::PCDReader reader;
    reader.read (fileName, *cloud);
    cout << "PointCloud has: " << cloud->size () << " data points." << endl;

    ////////////////////////////////
	// Create visualizer
  	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
  	viewer->addCoordinateSystem();

    // Color point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorInputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorSegmentCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *colorInputCloud);
    for (size_t i = 0; i < colorInputCloud->points.size(); i++) {
        colorInputCloud->points[i].r = 255;
        colorInputCloud->points[i].g = 255;
        colorInputCloud->points[i].b = 255;
    }
    viewer->addPointCloud<pcl::PointXYZRGB>(colorInputCloud, "Point Cloud ");

    // Get from outside of the file
    float maxLenghtSegment = M_PI * 0.5; // Perimeter/2 = 2/2 * PI * r = PI * r
    float maxDepthDiffSegment = 0.2;
    float maxDistanceBetweenCentersSegmentForCluster = maxLenghtSegment;

    // Go through rows in point cloud and divide in segments with similar depth
    // Information of last point read in row
    pcl::PointXYZ lastPoint;
    // Segment information
    float lengthSegment = 0;
    pcl::PointCloud<pcl::PointXYZ> segmentCloud;
    // First segment could be connected to last. Information to keep about first segment
    pcl::PointXYZ firstSegmentRowPoint;
    bool isFirstSegmentRow = true;
    float lengthFirstSegmentRow = 0;
    pcl::PointCloud<pcl::PointXYZ> firstSegmentRowCloud;
    // List where segments will be saved
    std::vector<pcl::PointCloud<pcl::PointXYZ>> listSegmentClouds;

    // Go through rows and columns
    for(std::size_t i=0; i<cloud->width; i++){
        // Clean parameters about last row
        lastPoint.x = 1000;
        lastPoint.y = 1000;
        lastPoint.z = 1000;
        firstSegmentRowPoint.x = 1000;
        firstSegmentRowPoint.y = 1000;
        firstSegmentRowPoint.z = 1000;
        lengthSegment = 0;
        lengthFirstSegmentRow = 0;
        isFirstSegmentRow = true;
        for(std::size_t j=0; j<cloud->height; j++){
            // Not consider NaN values of points
            if (isnan(cloud->at(i,j).x) || isnan(cloud->at(i,j).y) || isnan(cloud->at(i,j).z) ) {
                continue;
            }
            // First point saves parameters and defines last point
            if (firstSegmentRowPoint.x == 1000) {
                firstSegmentRowPoint = cloud->at(i,j);
                lastPoint = cloud->at(i,j);
                continue;
            }

            // Check if difference of depth is small enough to consider the point in the same segment as the point before
            if ((cloud->at(i,j).x > lastPoint.x - maxDepthDiffSegment) && (cloud->at(i,j).x < lastPoint.x + maxDepthDiffSegment)) {
                lengthSegment += getDistanceBetweenPoints(lastPoint, cloud->at(i,j));
                segmentCloud.push_back(cloud->at(i,j));
            } else {
                // If distance is too large, close the segment and add it to the list

                // If it is the first segment of the row, do not add to list and wait for end of row
                if (isFirstSegmentRow) {
                    firstSegmentRowCloud = segmentCloud;
                    isFirstSegmentRow = false;
                    lengthFirstSegmentRow = lengthSegment;
                } else if (lengthSegment < maxLenghtSegment && lengthSegment > 0) {
                    // Add to the list if it has a good length
                    listSegmentClouds.push_back(segmentCloud);
                }

                // Clean the segment information
                segmentCloud.clear();
                lengthSegment = 0;
            }

            // Only in the last point of row. Check if first and last segments can merge together
            if (j == cloud->height-1) {
                // Add them together if they have right length and are similar in depth
                if ((cloud->at(i,j).x > firstSegmentRowPoint.x - maxDepthDiffSegment) && (cloud->at(i,j).x < firstSegmentRowPoint.x + maxDepthDiffSegment)) {
                    lengthSegment += lengthFirstSegmentRow + getDistanceBetweenPoints(firstSegmentRowPoint, cloud->at(i,j));
                    if (lengthSegment < maxLenghtSegment && lengthSegment > 0) {
                        firstSegmentRowCloud += segmentCloud;
                        listSegmentClouds.push_back(firstSegmentRowCloud);
                    }
                } else { // Add them separately if they have right size independently
                    if (lengthFirstSegmentRow < maxLenghtSegment && lengthFirstSegmentRow > 0) {
                        listSegmentClouds.push_back(firstSegmentRowCloud);
                    }
                    if (lengthSegment < maxLenghtSegment && lengthSegment > 0) {
                        listSegmentClouds.push_back(segmentCloud);
                    }
                }
            }

            // Set last point
            lastPoint = cloud->at(i,j);
        }
    }


    // Cluster the segments together if they are close in distance

    //  First, get center of each segment averaging X, Y and Z
    pcl::PointXYZ center;
    float averageX, averageY, averageZ;
    // List where centers will be saved
    std::vector<pcl::PointXYZ> listSegmentCenterClouds;

    for (auto it = std::begin(listSegmentClouds); it != std::end(listSegmentClouds); ++it) {
        averageX = 0;
        averageY = 0;
        averageZ = 0;
        for (size_t i = 0; i < it->points.size(); i++) {
            averageX += it->points[i].x;
            averageY += it->points[i].y;
            averageZ += it->points[i].z;
        }
        center.x = averageX / it->points.size();
        center.y = averageY / it->points.size();
        center.z = averageZ / it->points.size();
        listSegmentCenterClouds.push_back(center);
    }

    // Second, get which center of which cloud is close to which center of which cloud and merge them
    pcl::PointCloud<pcl::PointXYZ> clusterCloud;
    int i, j;
    // Keep a list of clouds added to the list of clusters to avoid duplicates
    std::vector<int> listItAddedToCluster;
    // List of clusters found
    std::vector<pcl::PointCloud<pcl::PointXYZ>> listClusterClouds;

    for (auto it1 = std::begin(listSegmentCenterClouds); it1 != std::end(listSegmentCenterClouds); ++it1) {
        // Check if segment is already added to cluster, continue
        i = it1 - listSegmentCenterClouds.begin();
        if (std::count(listItAddedToCluster.begin(), listItAddedToCluster.end(), i)) {
            continue;
        }

        // Add cloud to cluster and list of clouds added
        clusterCloud += listSegmentClouds[i];
        listItAddedToCluster.push_back(i);
        // Find clouds close to this cloud and add to cluster
        for (auto it2 = std::begin(listSegmentCenterClouds); it2 != std::end(listSegmentCenterClouds); ++it2) {
            // Check if segment is already added to cluster, continue
            j = it2 - listSegmentCenterClouds.begin();
            if (i == j || std::count(listItAddedToCluster.begin(), listItAddedToCluster.end(), j)) {
                continue;
            }

            // Get distance between the centers. If less than specified, add to cluster
            if (getDistanceBetweenPoints(*it1, *it2) < maxDistanceBetweenCentersSegmentForCluster) {
                clusterCloud += listSegmentClouds[j];
                listItAddedToCluster.push_back(j);
            }
        }
        // Add cluster cloud to list
        listClusterClouds.push_back(clusterCloud);
        // Clean cluster information
        clusterCloud.clear();
    }


    cout << "Clusters: " << listClusterClouds.size() << endl;

    pcl::PointCloud<pcl::PointXYZ> cloudFor1;
    int r, g, b;
    for (auto it2 = std::begin(listClusterClouds); it2 != std::end(listClusterClouds); ++it2) {
        // Viewer
        cloudFor1 = *it2;
        pcl::copyPointCloud(cloudFor1, *colorSegmentCloud);
        r = rand() % 255 + 1;
        g = rand() % 255 + 1;
        b = rand() % 255 + 1;
        for (size_t i = 0; i < colorSegmentCloud->points.size(); i++) {
            colorSegmentCloud->points[i].r = r;
            colorSegmentCloud->points[i].g = g;
            colorSegmentCloud->points[i].b = b;
        }
        std::string id = "Segment Point Cloud ";
        id += std::to_string(rand());
        viewer->addPointCloud<pcl::PointXYZRGB>(colorSegmentCloud, id);
    }
    viewer->spin();

    return (0);
}
