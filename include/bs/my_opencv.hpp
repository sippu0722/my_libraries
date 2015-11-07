#pragma once

#include <iostream>

#pragma warning(disable:4819)
#include <opencv2/opencv.hpp>
#pragma warning(default:4819)

namespace bs
{

	template<typename T>
	inline T max(cv::InputArray m)
	{
		double max;
		cv::minMaxLoc(m, nullptr, &max);
		return static_cast<T>(max);
	}

}	// namespace bs
