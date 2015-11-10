#pragma once

#include <iostream>

#pragma warning(disable:4819)
#include <opencv2/opencv.hpp>
#pragma warning(default:4819)

namespace bs
{

template<typename T = uchar>
inline T max(cv::InputArray m)
{
	double val;
	cv::minMaxLoc(m, nullptr, &val);
	return static_cast<T>(val);
}

template<typename T = uchar>
inline T min(cv::InputArray m)
{
	double val;
	cv::minMaxLoc(m, &val);
	return static_cast<T>(val);
}

}	// namespace bs
