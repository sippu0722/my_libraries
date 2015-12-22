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

cv::String getMatTypeName(const int type_val)
{
	cv::String str;

	for (int cn = 1; cn <= 3; ++cn)
	{
		const int tmp = type_val - ((cn - 1) << 3);

		if (tmp < 0 || 6 < tmp)
			continue;

		const cv::String str_cn = (cn == 1) ? "" : "C" + std::to_string(cn);

		str =
			(tmp == CV_8U ) ? "CV_8U"  :
			(tmp == CV_8S ) ? "CV_8S"  :
			(tmp == CV_16U) ? "CV_16U" :
			(tmp == CV_16S) ? "CV_16S" :
			(tmp == CV_32S) ? "CV_32S" :
			(tmp == CV_32F) ? "CV_32F" :
			(tmp == CV_64F) ? "CV_64F" : ""
			+ str_cn;
	}
	return str.empty() ? "Unknown type!!!" : str;
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
