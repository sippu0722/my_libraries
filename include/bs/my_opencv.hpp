#pragma once

#include <iostream>

#pragma warning(disable:4819)
#include <opencv2/opencv.hpp>
#pragma warning(default:4819)

namespace bs
{

template<typename T = uchar>
T max(cv::InputArray m)
{
	double val;
	cv::minMaxLoc(m, nullptr, &val);
	return static_cast<T>(val);
}

template<typename T = uchar>
T min(cv::InputArray m)
{
	double val;
	cv::minMaxLoc(m, &val);
	return static_cast<T>(val);
}

void imshow(
	const std::string winname,
	cv::InputArray src,
	const cv::Size dsize,
	const double fx = 0.0, const double fy = 0.0,
	const cv::InterpolationFlags interpolation = cv::INTER_LINEAR)
{
	cv::Mat im;
	cv::resize(src, im, dsize, fx, fy, interpolation);
	cv::imshow(winname, im);
}

enum Key : int
{
	KEY_ESC = 27,
	KEY_UP = 2490368,
	KEY_DOWN = 2621440,
	KEY_LEFT = 2424832,
	KEY_RIGHT = 2555904
};

}	// namespace bs
