#pragma once

#include <iostream>
#include <random>
#include <array>

#pragma warning(disable:4819)
#include <opencv2/opencv.hpp>

namespace bs
{

namespace proj
{

cv::Mat makeRandomDot(
	const cv::Size& size, const int dot_size,
	const std::pair<int, int> range, const bool weighted_pattern,
	const std::pair<double, double> weight = { 1., 1. })
{
	cv::Mat im;

	if (!weighted_pattern)
	{
		const int low = std::min(range.first, range.second);
		const int high = std::max(range.first, range.second);
		im = cv::Mat(size, CV_8U);
		cv::randu(im, 0, 2);
		cv::threshold(im, im, 0, high - low, cv::THRESH_BINARY);
		im += low;

		if (1 < dot_size)
		{
			cv::resize(im, im, cv::Size(), (double)dot_size, (double)dot_size, cv::INTER_NEAREST);
			im = im(cv::Rect(cv::Point(0, 0), size));
		}
	}
	else
	{
		std::random_device rd;
		std::mt19937 mt(rd());
		std::vector<int> vec_r(2);
		std::vector<double> vec_w(2);

		im = cv::Mat(size, CV_8U);

		if (range.first < range.second)
		{
			vec_r[0] = range.first;
			vec_r[1] = range.second;
			vec_w[0] = weight.first;
			vec_w[1] = weight.second;
		}
		else
		{
			vec_r[1] = range.first;
			vec_r[0] = range.second;
			vec_w[1] = weight.first;
			vec_w[0] = weight.second;
		}

		size_t i = 0;
		std::discrete_distribution<> generator(
			vec_w.size(),
			0.,		// Dummy!!!!!	*std::min_element(vec_w.begin(), vec_w.end()),
			1.,		// Dummy!!!!!	*std::max_element(vec_w.begin(), vec_w.end()),
			[&vec_w, &i](double)
		{
			return vec_w[i++];
		});

#pragma omp parallel for
		for (int row = 0; row < im.rows; ++row)
		{
			uchar *ptr = im.ptr<uchar>(row);

			for (int col = 0; col < im.cols; ++col)
				ptr[col] = vec_r[generator(mt)];
		}

		if (1 < dot_size)
		{
			cv::resize(im, im, cv::Size(), (double)dot_size, (double)dot_size, cv::INTER_NEAREST);
			im = im(cv::Rect(cv::Point(0, 0), size));
		}
	}
	return im;
}

}

}
