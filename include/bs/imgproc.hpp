#pragma once

#include <random>
#include <array>

#pragma warning(disable:4819)
#include <opencv2/opencv.hpp>
#pragma warning(default:4819)

#include <bs/tips.hpp>


namespace bs
{
	enum Direction
	{
		Top, Bottom, Left, Right
	};

	namespace pattern
	{
		void shiftPattern(cv::InputArray src, cv::OutputArray dst, const int direction, const int shift);

		class RandomDotMaker
		{
		public:
			cv::Mat image;
			cv::Size image_size;
			int dot_size;
			std::pair<int, int> intensity;
			std::pair<double, double> weight;

			RandomDotMaker() {};

			void create();

			static void create(const cv::Size& _image_size, const size_t _dot_size,
							   cv::OutputArray dst,
							   const std::pair<int, int> _intensity = { 0, 255 },
							   const std::pair<double, double> _weight = { 1., 1. });
		};

		class ChessPatternMaker
		{
		public:
			cv::Mat image;
			cv::Size image_size, pattern_size;
			int square_size;

			ChessPatternMaker() {};

			void create();

			static void create(const cv::Size& _image_size, const cv::Size& _pattern_size,
							   const int _square_size, cv::OutputArray dst);

			static cv::Mat create(const cv::Size& _image_size, const cv::Size& _pattern_size,
				const int _square_size);
		};
	}

}


/*
-------------------------------------------------------
Implimantation
-------------------------------------------------------
*/

inline void bs::pattern::shiftPattern(cv::InputArray src, cv::OutputArray dst, const int direction, const int shift)
{
	cv::Mat d;
	cv::copyMakeBorder(src, d, shift, shift, shift, shift, cv::BORDER_WRAP);

	const auto direc = static_cast<Direction>(direction);

	switch (direc)
	{
	default:
		break;
	case Top:
		d = d(cv::Rect(cv::Point(shift, 2 * shift), src.size()));
		break;

	case Bottom:
		d = d(cv::Rect(cv::Point(shift, 0), src.size()));
		break;

	case Left:
		d = d(cv::Rect(cv::Point(2 * shift, shift), src.size()));
		break;

	case Right:
		d = d(cv::Rect(cv::Point(0, shift), src.size()));
		break;
	}
	d.copyTo(dst);
	return;
}


inline void bs::pattern::ChessPatternMaker::create()
{
#ifdef _DEBUG
	assert(image_size.area() != 0);
	assert(pattern_size.area() != 0);
	assert(square_size.area() != 0);
#endif
	create(image_size, pattern_size, square_size, image);
}


inline void bs::pattern::ChessPatternMaker::create(
	const cv::Size& _image_size,
	const cv::Size& _pattern_size,
	const int _square_size,
	cv::OutputArray dst)
{
	const int pat_w = _square_size * (_pattern_size.width + 1);
	const int pat_h = _square_size * (_pattern_size.height + 1);

	assert(pat_w < _image_size.width);
	assert(pat_h < _image_size.height);

	const int d_w = _image_size.width - pat_w, d_h = _image_size.height - pat_h;
	std::array<int, 4> margin;	// { t, b, l, r }

	switch (d_h % 2)
	{
	case 0:
		margin[0] = margin[1] = d_h / 2;
		break;
	case 1:
		margin[0] = (d_h + 1) / 2;
		margin[1] = (d_h - 1) / 2;
		break;
	}

	switch (d_w % 2)
	{
	case 0:
		margin[2] = margin[3] = d_w / 2;
		break;
	case 1:
		margin[2] = (d_w + 1) / 2;
		margin[3] = (d_w - 1) / 2;
		break;
	}

	cv::Mat pat(pat_h, pat_w, CV_8U);
	bool col_flag = false;
	bool row_flag = false;

	for (int row = 0; row < pat_h; ++row)
	{
		auto *p = pat.ptr<uchar>(row);

		if (row % _square_size == 0)
			row_flag = !row_flag;
		col_flag = row_flag;

		for (int col = 0; col < pat_w; ++col)
		{
			if (col % _square_size == 0)
				col_flag = !col_flag;
			p[col] = (col_flag ? 0 : 255);
		}
	}
	cv::copyMakeBorder(pat, dst, margin[0], margin[1], margin[2], margin[3],
		cv::BORDER_CONSTANT, cv::Scalar(255));
}


inline cv::Mat bs::pattern::ChessPatternMaker::create(
	const cv::Size& _image_size,
	const cv::Size& _pattern_size,
	const int _square_size)
{
	cv::Mat im;
	create(_image_size, _pattern_size, _square_size, im);
	return im;
}

inline void bs::pattern::RandomDotMaker::create(const cv::Size& _image_size, const size_t _dot_size,
	cv::OutputArray dst, const std::pair<int, int> _intensity, const std::pair<double, double> _weight)
{
	const bool is_weighted_pattern = (_weight.first != _weight.second ? true : false);
	const double dot_size_d = static_cast<double>(_dot_size);
	const auto r = bs::sortPairGreater(_intensity);
	const auto w = bs::sortPairGreater(_weight);

	cv::Mat im(_image_size, CV_8U);

	if (!is_weighted_pattern)
	{
		const int min = r.first, max = r.second;

		cv::randu(im, 0, 2);
		cv::threshold(im, im, 0, static_cast<double>(max - min), cv::THRESH_BINARY);
		im += min;
	}
	else
	{
		std::random_device rd;
		std::mt19937 mt(rd());
		std::vector<int> vec_r = { r.first, r.second };
		std::vector<double> vec_w = { w.first, w.second };

		size_t i = 0;
		std::discrete_distribution<> generator(
			vec_w.size(),
			0.,		// Dummy!!!!!	*std::min_element(vec_w.begin(), vec_w.end()),
			1.,		// Dummy!!!!!	*std::max_element(vec_w.begin(), vec_w.end()),
			[&vec_w, &i](double) {	return vec_w[i++];	});

#pragma omp parallel for
		for (int row = 0; row < im.rows; ++row)
		{
			auto *ptr = im.ptr<uchar>(row);

			for (int col = 0; col < im.cols; ++col)
				ptr[col] = vec_r[generator(mt)];
		}
	}

	if (1 < _dot_size)
	{
		cv::resize(im, im, cv::Size(), dot_size_d, dot_size_d, cv::INTER_NEAREST);
		im = im(cv::Rect(cv::Point(0, 0), _image_size));
	}
	im.copyTo(dst);
	return;
}


/*
inline cv::Mat
makeStripe(const cv::Size& size, const size_t stripe_width, const bool is_vertical,
	const std::pair<uint8_t, uint8_t> intensity_range = { 0u, 255u })
{
	cv::Mat im(size, CV_8U);
	const size_t length = static_cast<size_t>(is_vertical ? size.width : size.height);

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(length); ++i)
	{
		const uint8_t value = (i % (stripe_width * 2) < stripe_width ? intensity_range.first : intensity_range.second);
		(is_vertical ? im.col(i) : im.row(i)) = cv::Scalar(value);
	}
	return im;
}


@overload

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
	const std::pair<uint8_t, uint8_t> intensity_range = { 0u, 255u })
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
			cv::Scalar(draw_brack ? intensity_range.first : intensity_range.second);
		draw_brack = !draw_brack;
	}
	return im;
}

inline cv::Mat
makeRandomStripe(const size_t rows, const size_t cols, const bool is_vertical,
	const std::pair<uint, uint> width_range = { 1u, 1u },
	const std::pair<uint8_t, uint8_t> intensity_range = { 0u, 255u })
{
	const cv::Size sz(static_cast<int>(cols), static_cast<int>(rows));
	return makeRandomStripe(sz, is_vertical, width_range, intensity_range);
}


inline void
makeSinPattern(cv::OutputArrayOfArrays projection_h,
	cv::OutputArrayOfArrays projection_v,
	const cv::Size& prj_sz, const uint numof_step,
	const std::vector<uint> waves,
	const std::pair<uint8_t, uint8_t> intensity_range = { 0u, 255u })
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
	HWND win_handle = FindWindowA(0, win_name.c_str());
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
	const size_t min, const size_t max, const uint numof_step,
	const uint numof_step_current, const bool is_horizontal)
{
	cv::Mat img = image.getMat();
	const double bright = (max - min) / 2.;
	const double offset = (max + min) / 2.;
	const size_t wave_length = static_cast<size_t>((is_horizontal ? img.rows : img.cols) / numof_wave);

	for (size_t i = 0; i < static_cast<size_t>(is_horizontal ? img.rows : img.cols); ++i)
	{
		const double initial_phase = (2 * std::_Pi / numof_step) * numof_step_current;
		const double angle_frequency = 2 * std::_Pi * i / wave_length;
		const uint8_t value = static_cast<uint8_t>(bright * std::sin(angle_frequency + initial_phase) + offset);

		const int i_ = static_cast<int>(i);
		(is_horizontal ? img.row(i_) : img.col(i_)) = cv::Scalar(value);
	}
	//img.copyTo(image);
	return;
}
*/
