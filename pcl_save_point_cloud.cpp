#include <bs/pcl_wrapper.h>

#pragma warning(disable:4819)
#include <pcl/io/ply_io.h>
#pragma warning(default:4819)

namespace bs
{

void savePointCloud(const std::string file, cv::InputArray point_cloud, const bool is_binary, cv::InputArray color_image, const bool remove_miss_point)
{
	const cv::Mat points = point_cloud.getMat();
	const bool color_cloud = (!color_image.empty() && (point_cloud.size() == color_image.size()));
	
	if (points.empty())
		return;

	if (points.type() != CV_32FC1)
		points.reshape(1, 3);

	if (color_cloud)
	{
		const cv::Mat im = color_image.getMat();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		for (int row = 0; row < points.rows; ++row)
		{
			const auto ptr = points.ptr<cv::Vec3f>(row);
			const auto ptr_image = im.ptr<cv::Vec3b>(row);

			for (int col = 0; col < points.cols; ++col)
			{
				const auto val = ptr[col];
				const auto color = ptr_image[col];

				if (!remove_miss_point || (remove_miss_point && val[2] < 9999.9f))
				{
					pcl::PointXYZRGB p;
					p.x = val[0], p.y = val[1], p.z = val[2], p.r = color[0], p.g = color[1], p.b = color[2];
					cloud->push_back(p);
				}
			}
		}

		if (cloud->size() == 0)
			return;
		else if (is_binary)
			pcl::io::savePLYFileBinary(file, *cloud);
		else
			pcl::io::savePLYFileASCII(file, *cloud);
	}
	else
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		for (int row = 0; row < points.rows; ++row)
		{
			const auto ptr = points.ptr<cv::Vec3f>(row);

			for (int col = 0; col < points.cols; ++col)
			{
				const auto val = ptr[col];

				if (!remove_miss_point || (remove_miss_point && val[2] < 9999.9f))
					cloud->push_back(pcl::PointXYZ(val[0], val[1], val[2]));
			}
		}

		if (cloud->size() == 0)
			return;
		else if (is_binary)
			pcl::io::savePLYFileBinary(file, *cloud);
		else
			pcl::io::savePLYFileASCII(file, *cloud);
	}
	return;
}

}	// namespace bs
