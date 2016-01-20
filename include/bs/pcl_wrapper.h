#pragma once
#pragma warning(disable:4819)
#include <opencv2/core/mat.hpp>
#pragma warning(default:4819)

namespace bs
{
	void savePointCloud(
		const std::string file,
		cv::InputArray point_cloud,
		const bool is_binary,
		cv::InputArray color_image = cv::Mat(),
		const bool remove_miss_point = true);
}
