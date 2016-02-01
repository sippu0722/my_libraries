#pragma once

#include <iostream>
#include <random>
#include <array>

#pragma warning(disable:4819)
#include <opencv2/opencv.hpp>
#pragma warning(default:4819)

// CPUのアーキテクチャ設定。
//  'winnt.h'('windef.h'が内部でincludeしている)で必要。
//  普段は'Windows.h'が設定している
#ifdef _M_IX86          // コンパイラが定義しているはず
#define _X86_
#else
#define _AMD64_
#endif

#define NOMINMAX		// マクロ min, max を定義されたくない場合アクティブにする
#include <windef.h>		// 基本的な定義類
#include <WinUser.h>	// 


namespace bs
{

namespace proj
{

cv::Mat makeRandomDot(
	const cv::Size& size, const uint dot_size,
	const std::pair<uint, uint> range = { 0, 255 },
	const std::pair<double, double> weight = { 1., 1. })
{
	cv::Mat im;
	const bool weighted_pattern = (weight.first != weight.second ? true : false);

	if (!weighted_pattern)
	{
		const uint low = std::min(range.first, range.second);
		const uint high = std::max(range.first, range.second);
		im = cv::Mat(size, CV_8U);
		cv::randu(im, 0, 2);
		cv::threshold(im, im, 0, double(high - low), cv::THRESH_BINARY);
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
		std::vector<uint> vec_r(2);
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

		int i = 0;
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
				ptr[col] = static_cast<uchar>(vec_r[generator(mt)]);
		}

		if (1 < dot_size)
		{
			cv::resize(im, im, cv::Size(), (double)dot_size, (double)dot_size, cv::INTER_NEAREST);
			im = im(cv::Rect(cv::Point(0, 0), size));
		}
	}
	return im;
}

/*!
@overload
*/
cv::Mat makeRandomDot(
	const uint rows, const uint cols,
	const uint dot_size,
	const std::pair<uint, uint> range = { 0, 255 },
	const std::pair<double, double> weight = { 1., 1. })
{
	return makeRandomDot(cv::Size(cols, rows), dot_size, range, weight);
}

cv::Mat makeStripe(
	const cv::Size& size, const uint stripe_width,
	const bool is_vertical,
	const std::pair<uint, uint> range = { 0, 255 }
	)
{
	cv::Mat im(size, CV_8U);

	const uint length = is_vertical ? size.height : size.width;

#pragma omp parallel for
	for (int i = 0; i < (int)length; ++i)
	{
		const uint value = (i % (stripe_width * 2) < stripe_width ? range.first : range.second);
		(is_vertical ? im.row(i) : im.col(i)) = cv::Scalar::all(value);
	}
	return im;

}

/*!
@overload
*/
cv::Mat makeStripe(
	const uint rows, const uint cols,
	const uint stripe_width,
	const bool is_vertical,
	const std::pair<uint, uint> range = { 0, 255 }
	)
{
	return makeStripe(cv::Size(cols, rows), stripe_width, is_vertical, range);
}

cv::Mat makeRandomStripe(
	const cv::Size& size,
	const bool is_vertical,
	const std::pair<uint, uint> width_range,
	const std::pair<uint, uint> intensity_range = { 0, 255 }
	)
{
	assert(0 < width_range.first && width_range.first < width_range.second);

	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_int_distribution<uint> gen(width_range.first, width_range.second);
	uint w, length = (is_vertical ? size.height : size.width), count = 0;
	cv::Mat im(size, CV_8U);
	bool draw_brack = true;

	while (count < length)
	{
		cv::Range range;
		
		w = gen(mt);
		range.start = count;
		count += w;
		range.end = std::min(count, length);
		(is_vertical ? im.rowRange(range) : im.colRange(range)) = cv::Scalar::all(draw_brack ? intensity_range.first : intensity_range.second);
		draw_brack = !draw_brack;
	}
	return im;
}

cv::Mat makeRandomStripe(
	const uint rows, const uint cols,
	const bool is_vertical,
	const std::pair<uint, uint> width_range,
	const std::pair<uint, uint> intensity_range = { 0, 255 }
	)
{
	return makeRandomStripe(
		cv::Size(cols, rows),
		is_vertical, width_range, intensity_range);
}

}		// namespace proj


inline void showWindowNoframe(cv::InputArray image,
	const cv::Point2i& window_pos,
	const std::string& win_name = "projection")
{
	// std::string -> wchar_t*
	//
	WCHAR ws[32];
	size_t wLength = 0;
	errno_t err = 0;
	setlocale(LC_ALL, "japanese");
	err = mbstowcs_s(&wLength, ws, 16, win_name.c_str(), _TRUNCATE);

	cv::imshow(win_name, image);

	unsigned int win_flags = (SWP_SHOWWINDOW | SWP_NOSIZE);
	cv::setWindowProperty(win_name, cv::WINDOW_FULLSCREEN, cv::WINDOW_FULLSCREEN);
	HWND win_handle = FindWindow(0, ws);
	SetWindowPos(win_handle, HWND_NOTOPMOST, window_pos.x, window_pos.y, image.getMat().cols, image.getMat().rows, win_flags);
	SetWindowLong(win_handle, GWL_STYLE, GetWindowLong(win_handle, GWL_EXSTYLE) | WS_EX_TOPMOST);
	ShowWindow(win_handle, SW_SHOW);

	return;
}

}	// namespace bs
