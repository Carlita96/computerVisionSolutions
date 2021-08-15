#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>
# include <pcl/io/png_io.h>

int
main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// Parameters needed by the range image object:

	// Angular resolution is the angular distance between pixels.
	// Kinect: 57° horizontal FOV, 43° vertical FOV, 640x480 (chosen here).
	// Xtion: 58° horizontal FOV, 45° vertical FOV, 640x480.
	float angularResolutionX = (float)(57.0f / 640.0f * (M_PI / 180.0f));
	float angularResolutionY = (float)(43.0f / 480.0f * (M_PI / 180.0f));
	// Maximum horizontal and vertical angles. For example, for a full panoramic scan,
	// the first would be 360º. Choosing values that adjust to the real sensor will
	// decrease the time it takes, but don't worry. If the values are bigger than
	// the real ones, the image will be automatically cropped to discard empty zones.
	float maxAngleX = (float)(60.0f * (M_PI / 180.0f));
	float maxAngleY = (float)(50.0f * (M_PI / 180.0f));
	// Sensor pose. Thankfully, the cloud includes the data.
  	Eigen::Affine3f sensorPose (Eigen::Affine3f::Identity ());
	// Noise level. If greater than 0, values of neighboring points will be averaged.
	// This would set the search radius (e.g., 0.03 == 3cm).
	float noiseLevel = 0.3f;
	// Minimum range. If set, any point closer to the sensor than this will be ignored.
	float minimumRange = 0.0f;
	// Border size. If greater than 0, a border of "unobserved" points will be left
	// in the image when it is cropped.
	int borderSize = 1;

	// Range image object.
	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(*cloud, angularResolutionX, angularResolutionY,
									maxAngleX, maxAngleY, sensorPose, pcl::RangeImage::CAMERA_FRAME,
									noiseLevel, minimumRange, borderSize);

	// pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
  	// range_image_widget.showRangeImage (rangeImage);

	// while (!range_image_widget.wasStopped ())
	// {
	// 	range_image_widget.spinOnce ();
	// 	pcl_sleep (0.01);
	// }

	// std::cout << "Width: " << rangeImage.width << ". Height: " << rangeImage.height << "." << std::endl;

	// float* ranges = rangeImage.getRangesArray ();
 
	// unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage (ranges, rangeImage.width, rangeImage.height);

	// pcl::io::saveRgbPNGFile("ha.png", rgb_image, rangeImage.width, rangeImage.height); 
}
