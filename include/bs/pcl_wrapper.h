#pragma once
#pragma warning(disable:4819)
#include <opencv2/core/mat.hpp>
#pragma warning(default:4819)

namespace bs
{
	void savePointCloud(
		const std::string file,
		cv::InputArray point_cloud,
		const bool remove_miss_point,
		const bool is_binary);
}
