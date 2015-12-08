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

inline void imshow(
	const std::string winname,
	cv::InputArray src,
	const cv::Size dsize = cv::Size(),
	const double fx = 0., const double fy = 0.,
	const cv::InterpolationFlags interpolation = cv::INTER_LINEAR)
{
	cv::Mat im;

	if (dsize.area() == 0)
		im = src.getMat();
	else
		cv::resize(src, im, dsize, fx, fy, interpolation);

	cv::imshow(winname, im);
	return;
}

inline void imwrite(
	const std::string file,
	cv::InputArray img,
	const cv::Size dsize = cv::Size(),
	const std::vector<int>& params = std::vector<int>(),
	const double fx = 0., const double fy = 0.,
	const cv::InterpolationFlags interpolation = cv::INTER_LINEAR)
{
	cv::Mat im;

	if (dsize.area() == 0)
		im = img.getMat();
	else
		cv::resize(img, im, dsize, fx, fy, interpolation);

	cv::imwrite(file, im, params);
	return;
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
