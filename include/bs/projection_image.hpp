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
#include <Windows.h>


template<typename T, typename U>
std::pair<T, U> bs__sortPairGreater(const std::pair<T, U>& p);


void bs__makeSinWave(cv::InputOutputArray image, const uint numof_wave,
					 const size_t min,const size_t max, const uint numof_step,
					 const uint numof_step_current, const bool is_horizontal);


namespace bs
{

namespace proj
{

inline cv::Mat
makeRandomDot(const cv::Size& size, const size_t dot_size,
			  const std::pair<uchar, uchar> intensity_range = { 0u, 255u },
			  const std::pair<double, double> weight = { 1., 1. }) 
{
	cv::Mat im;
	const bool weighted_pattern = (weight.first != weight.second ? true : false);
	const double dot_size_d = static_cast<double>(dot_size);
	const auto r = bs__sortPairGreater(intensity_range);
	const auto w = bs__sortPairGreater(weight);

	// In case of waighted pattern
	if (!weighted_pattern)
	{
		const uchar min = r.first, max = r.second;

		im = cv::Mat(size, CV_8U);
		cv::randu(im, 0, 2);
		cv::threshold(im, im, 0, static_cast<double>(max - min), cv::THRESH_BINARY);
		im += min;

		if (1 < dot_size)
		{
			cv::resize(im, im, cv::Size(), dot_size_d, dot_size_d, cv::INTER_NEAREST);
			im = im(cv::Rect(cv::Point(0, 0), size));
		}
		return im;
	}

	std::random_device rd;
	std::mt19937 mt(rd());
	std::vector<uchar> vec_r(2);
	std::vector<double> vec_w(2);

	im = cv::Mat(size, CV_8U);
	vec_r[0] = intensity_range.first;
	vec_r[1] = intensity_range.second;
	vec_w[0] = weight.first;
	vec_w[1] = weight.second;

	int i = 0;
	std::discrete_distribution<> generator(
		vec_w.size(),
		0.,		// Dummy!!!!!	*std::min_element(vec_w.begin(), vec_w.end()),
		1.,		// Dummy!!!!!	*std::max_element(vec_w.begin(), vec_w.end()),
		[&vec_w, &i](double) {	return vec_w[i++];	});

#pragma omp parallel for
	for (int row = 0; row < im.rows; ++row)
	{
		uchar *ptr = im.ptr<uchar>(row);

		for (int col = 0; col < im.cols; ++col)
			ptr[col] = vec_r[generator(mt)];
	}
	if (1 < dot_size)
	{
		cv::resize(im, im, cv::Size(), dot_size_d, dot_size_d, cv::INTER_NEAREST);
		im = im(cv::Rect(cv::Point(0, 0), size));
	}
	return im;
}


inline cv::Mat
makeRandomDot(const size_t rows, const size_t cols, const size_t dot_size,
	const std::pair<uchar, uchar> intensity_range = { 0u, 255u },
	const std::pair<double, double> weight = { 1., 1. })
{
	const cv::Size sz(static_cast<int>(cols), static_cast<int>(rows));
	return makeRandomDot(sz, dot_size, intensity_range, weight);
}


inline cv::Mat
makeStripe(const cv::Size& size, const size_t stripe_width, const bool is_vertical,
		   const std::pair<uchar, uchar> intensity_range = { 0u, 255u })
{
	cv::Mat im(size, CV_8U);
	const size_t length = static_cast<size_t>(is_vertical ? size.width : size.height);

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(length); ++i)
	{
		const uchar value = (i % (stripe_width * 2) < stripe_width ? intensity_range.first : intensity_range.second);
		(is_vertical ? im.col(i) : im.row(i)) = cv::Scalar::all(value);
	}
	return im;
}


/*!
@overload
*/
inline cv::Mat
makeStripe(const size_t rows, const size_t cols, const size_t stripe_width,
		   const bool is_vertical,
		   const std::pair<uint, uint> intensity_range = { 0u, 255u })
{
	const cv::Size sz(static_cast<int>(cols), static_cast<int>(rows));
	return makeStripe(sz, stripe_width, is_vertical, intensity_range);
}


inline cv::Mat
makeRandomStripe(const cv::Size& size, const bool is_vertical,
				 const std::pair<uint, uint> width_range = { 1u, 1u },
				 const std::pair<uchar, uchar> intensity_range = { 0u, 255u })
{
	const auto w = bs__sortPairGreater(width_range);
	const size_t length = static_cast<size_t>(is_vertical ? size.height : size.width);

	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_int_distribution<size_t> gen(w.first, w.second);
	cv::Mat im(size, CV_8U);
	bool draw_brack = true;
	size_t w_curr;
	uint count = 0;

	while (count < length)
	{
		cv::Range range;
		
		w_curr = gen(mt);
		range.start = static_cast<int>(count);
		count += static_cast<uint>(w_curr);
		range.end = static_cast<int>(std::min(static_cast<size_t>(count), length));
		(is_vertical ? im.colRange(range) : im.rowRange(range)) =
			cv::Scalar::all(draw_brack ? intensity_range.first : intensity_range.second);
		draw_brack = !draw_brack;
	}
	return im;
}

inline cv::Mat
makeRandomStripe(const size_t rows, const size_t cols, const bool is_vertical,
				 const std::pair<uint, uint> width_range = { 1u, 1u },
				 const std::pair<uchar, uchar> intensity_range = { 0u, 255u })
{
	const cv::Size sz(static_cast<int>(cols), static_cast<int>(rows));
	return makeRandomStripe(sz, is_vertical, width_range, intensity_range);
}


inline void
makeSinPattern(cv::OutputArrayOfArrays projection_h,
			   cv::OutputArrayOfArrays projection_v,
			   const cv::Size& prj_sz, const uint numof_step,
			   const std::vector<uint> waves,
			   const std::pair<uchar, uchar> intensity_range = { 0u, 255u })
{
	assert(!waves.empty() && 0 < numof_step);

	const auto r = bs__sortPairGreater(intensity_range);
	std::vector<cv::Mat> prj_h(waves.size() * numof_step);
	std::vector<cv::Mat> prj_v(waves.size() * numof_step);

	for (size_t i = 0u; i < waves.size(); ++i)
	{
		for (uint j = 0u; j < numof_step; ++j)
		{
			size_t idx = i * numof_step + j;
			prj_h[idx] = cv::Mat::zeros(prj_sz, CV_8U);
			prj_v[idx] = cv::Mat::zeros(prj_sz, CV_8U);

			bs__makeSinWave(prj_h[idx], waves[i], r.first, r.second, numof_step, j, true);
			bs__makeSinWave(prj_v[idx], waves[i], r.first, r.second, numof_step, j, false);
		}
	}
	const int rows = static_cast<int>(waves.size() * numof_step);
	projection_h.create(rows, 1, prj_h[0].type());
	projection_v.create(rows, 1, prj_v[0].type());

	for (size_t i = 0; i < static_cast<size_t>(rows); ++i)
	{
		projection_h.getMatRef(static_cast<int>(i)) = prj_h[i];
		projection_v.getMatRef(static_cast<int>(i)) = prj_v[i];
	}
	return;
}

}	// namespace proj


inline void
showWindowNoframe(cv::InputArray image, const cv::Point2i& window_pos,
				  const std::string& win_name = "projection")
{
	cv::imshow(win_name, image);
	unsigned int win_flags = (SWP_SHOWWINDOW | SWP_NOSIZE);
	cv::setWindowProperty(win_name, cv::WINDOW_FULLSCREEN, cv::WINDOW_FULLSCREEN);
	HWND win_handle = FindWindow(0, win_name.c_str());
	SetWindowPos(win_handle, HWND_NOTOPMOST, window_pos.x, window_pos.y, image.getMat().cols, image.getMat().rows, win_flags);
	SetWindowLong(win_handle, GWL_STYLE, GetWindowLong(win_handle, GWL_EXSTYLE) | WS_EX_TOPMOST);
	ShowWindow(win_handle, SW_SHOW);

	return;
}

}	// namespace bs


template<typename T, typename U>
inline std::pair<T, U>
bs__sortPairGreater(const std::pair<T, U>& p)
{
	if (p.first < p.second)	return p;
	else
	{
		const T fi = static_cast<T>(p.second);
		const U se = static_cast<U>(p.first);
		return std::make_pair(fi, se);
	}
}


inline void
bs__makeSinWave(cv::InputOutputArray image, const uint numof_wave,
				const size_t min,const size_t max, const uint numof_step,
				const uint numof_step_current, const bool is_horizontal)
{
	cv::Mat img = image.getMat();
	const double bright = (max - min) / 2.;
	const double offset = (max + min) / 2.;
	const size_t wave_length = static_cast<size_t>((is_horizontal ? img.cols : img.rows) / numof_wave);

	for (size_t i = 0; i < static_cast<size_t>(is_horizontal ? img.cols : img.rows); ++i)
	{
		const double initial_phase = (2 * std::_Pi / numof_step) * numof_step_current;
		const double angle_frequency = 2 * std::_Pi * i / wave_length;
		const uchar value = static_cast<uchar>(bright * std::sin(angle_frequency + initial_phase) + offset);
		
		const int i_ = static_cast<int>(i);
		(is_horizontal ? img.col(i_) : img.row(i_)) = cv::Scalar::all(value);
	}
	//img.copyTo(image);
	return;
}
