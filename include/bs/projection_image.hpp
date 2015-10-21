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

inline void showWindowNoframe(cv::InputArray image,
	const std::string win_name = "prj",
	const cv::Point2i& window_pos = cv::Point2i(1920, -800))
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

}
