#include <bs/pcl_wrapper.h>

#pragma warning(disable:4819)
#include <pcl/io/ply_io.h>
#pragma warning(default:4819)

namespace bs
{

void savePointCloud(const std::string file, cv::InputArray point_cloud, const bool remove_miss_point, const bool is_binary)
{
	cv::Mat points = point_cloud.getMat();
	
	if (points.empty())
		return;

	if (points.type() != CV_32FC1)
		points.reshape(1, 3);

	if (remove_miss_point)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		for (int row = 0; row < points.rows; ++row)
		{
			const cv::Vec3f *ptr = points.ptr<cv::Vec3f>(row);

			for (int col = 0; col < points.cols; ++col)
			{
				const cv::Vec3f val = ptr[col];

				if (val[2] < 9999.9f)	cloud->push_back(pcl::PointXYZ(val[0], val[1], val[2]));
			}
		}

		if (cloud->size() == 0)
			return;
		else if (is_binary)
			pcl::io::savePLYFileBinary(file, *cloud);
		else if (!is_binary)
			pcl::io::savePLYFileASCII(file, *cloud);
	}
	else
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(points.size().area(), 1));

		for (int row = 0; row < points.rows; ++row)
		{
			const cv::Vec3f *ptr = points.ptr<cv::Vec3f>(row);

#pragma omp parallel for
			for (int col = 0; col < points.cols; ++col)
			{
				const cv::Vec3f val = ptr[col];

				(*cloud)[row * points.cols + col] = pcl::PointXYZ(val[0], val[1], val[2]);
			}
		}

		if (cloud->size() == 0)
			return;
		else if (is_binary)
			pcl::io::savePLYFileBinary(file, *cloud);
		else if (!is_binary)
			pcl::io::savePLYFileASCII(file, *cloud);
	}
	return;
}

}	// namespace bs
