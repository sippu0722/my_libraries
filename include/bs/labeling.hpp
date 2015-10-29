#pragma once

#include <array>

#pragma warning(disable:4819)
#include <opencv2/imgproc.hpp>
#pragma warning(default:4819)


namespace bs
{

/*!
画像のラベリングをするクラス
*/
class Labeling
{
private:
	cv::Mat image_;
	std::vector<std::vector<cv::Point>> connected_pixels_;
	cv::Size image_size_;

	//! Connected Component Analysis Function with 8 neighbors
	template<typename T>
	void ConnectedComponentAnalysisBase_(cv::InputArray src, std::vector<std::vector<cv::Point>>& connected_components)
	{
		cv::Mat src_mat = src.getMat();
		//int x, y;
		int k, m;

		const int kRows = src_mat.rows;
		const int kCols = src_mat.cols;
		const T *kPtrSrcData = (T*)src_mat.data;

		cv::Mat cc_label_map = cv::Mat::zeros(kRows, kCols, CV_32SC1);
		int* ptr_cc_label = (int*)cc_label_map.data;

		std::array<int, 4> neighbor_label;
		const std::array<int, 4> kNeighborPos = { kCols + 1, kCols, kCols - 1, 1 };

		int label;
		int label_prev;
		T val;
		std::vector<int> label_tbl;

		// If the neighbor of the target already has a label, mark the target as the same label
		// If the neighbors have differenct labels, record it to label_tbl
		for (int y = 0; y < kRows; ++y)
		{
			for (int x = 0; x < kCols; ++x)
			{
				const int kOffset = y * kCols + x;

				if (0 < kPtrSrcData[kOffset])
				{
					val = kPtrSrcData[kOffset];
					neighbor_label[0] = (
						0 < y &&
						0 < x &&
						kPtrSrcData[kOffset - kNeighborPos[0]] == val) ?
						ptr_cc_label[kOffset - kNeighborPos[0]] : 0;
					neighbor_label[1] = (
						0 < y &&
						kPtrSrcData[kOffset - kNeighborPos[1]] == val) ?
						ptr_cc_label[kOffset - kNeighborPos[1]] : 0;
					neighbor_label[2] = (
						0 < y &&
						x < kCols - 1 &&
						kPtrSrcData[kOffset - kNeighborPos[2]] == val) ?
						ptr_cc_label[kOffset - kNeighborPos[2]] : 0;
					neighbor_label[3] = (
						0 < x &&
						kPtrSrcData[kOffset - kNeighborPos[3]] == val) ?
						ptr_cc_label[kOffset - kNeighborPos[3]] : 0;

					if (!neighbor_label[0] && !neighbor_label[1] && !neighbor_label[2] && !neighbor_label[3])
					{
						label = (int)label_tbl.size();
						label_tbl.push_back(label);
						++label;
					}
					else
					{
						label = 0;

						for (int i = 0; i < 4; ++i)
						{
							label_prev = neighbor_label[i];

							if (label_prev)
							{
								if (!label)
									label = label_prev;
								else if (label_prev != label)
								{
									k = label - 1;
									m = label_prev - 1;

									while (label_tbl[k] < k)
										k = label_tbl[k];

									while (label_tbl[m] < m)
										m = label_tbl[m];

									if (m < k)
									{
										label_tbl[k] = m;
										label = m + 1;
									}
									else
									{
										label_tbl[m] = k;
										label = k + 1;
									}
								}
							}
						}
					}
					ptr_cc_label[kOffset] = label;
				}
			}
		}

		// integrate labels recorded in label_tbl
		int cc_count = 0;
		std::vector<int> cc_id(label_tbl.size(), 0);

		for (int i = (int)label_tbl.size() - 1; 0 <= i; --i)
		{
			if (label_tbl[i] == i)
				cc_id[i] = ++cc_count;
			else
			{
				m = i;

				while (label_tbl[m] < m)
					m = label_tbl[m];

				label_tbl[i] = m;
			}
		}

		// Store the final result
		ptr_cc_label = (int*)cc_label_map.data;
		connected_components.resize(cc_count);

		for (int y = 0; y < kRows; ++y)
		{
			for (int x = 0; x < kCols; ++x)
			{
				if (0 < *ptr_cc_label)
				{
					int id = cc_id[label_tbl[*ptr_cc_label - 1]] - 1;
					connected_components[id].push_back(cv::Point(x, y));
				}
				ptr_cc_label++;
			}
		}
	}

	//! Connected Component Analysis Function with 8 neighbors
	/*!
	\input[in] src_mat input image which can be CV_8UC1, CV_32SC1, CV_32FC1, CV_64FC1
	\param[out] connected_components all pixel coordinates in connected components
	*/
	void ConnectedComponentAnalysis_(cv::InputArray src, std::vector<std::vector<cv::Point>>& connected_components)
	{
		cv::Mat m = src.getMat();

		switch (m.type())
		{
		case CV_8UC1:
			ConnectedComponentAnalysisBase_<unsigned char>(m, connected_components);
			break;
		case CV_32SC1:
			ConnectedComponentAnalysisBase_<int>(m, connected_components);
			break;
		case CV_32FC1:
			ConnectedComponentAnalysisBase_<float>(m, connected_components);
			break;
		case CV_64FC1:
			ConnectedComponentAnalysisBase_<double>(m, connected_components);
			break;

		default:
			break;
		}
	}

public:

	Labeling(){}

	Labeling(cv::InputArray image)
	{
		image_ = image.getMat();
		ConnectedComponentAnalysis_(image_, connected_pixels_);
		std::sort(connected_pixels_.begin(), connected_pixels_.end(),
			[](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) -> bool{ return a.size() > b.size(); });
		image_size_ = image_.size();
	}

	void compute(cv::InputArray image)
	{
		image_ = image.getMat();
		ConnectedComponentAnalysis_(image_, connected_pixels_);
		std::sort(connected_pixels_.begin(), connected_pixels_.end(),
			[](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) -> bool{ return a.size() > b.size(); });
		image_size_ = image_.size();
		return;
	}

	std::vector<cv::Point> getPixels(const int index)
	{
		return connected_pixels_[index];
	}

	size_t getNumRegions(void)
	{
		return connected_pixels_.size();
	}

	cv::Rect getBoundingBox(const int index)
	{
		std::vector<cv::Point> points = connected_pixels_[index];
		int x_min = std::numeric_limits<int>::max(), y_min = std::numeric_limits<int>::max();
		int x_max = -1, y_max = -1;

		for (cv::Point p : points)
		{
			x_min = x_min < p.x ? x_min : p.x;
			y_min = y_min < p.y ? y_min : p.y;
			x_max = x_max < p.x ? p.x : x_max;
			y_max = y_max < p.y ? p.y : y_max;
		}
		return cv::Rect(cv::Point(x_min, y_min), cv::Point(x_max, y_max));
	}

	void createMask(const int index, cv::OutputArray mask)
	{
		cv::Mat m(image_size_, CV_8U, cv::Scalar::all(0));

		for (const auto& p : connected_pixels_[index])
			m.at<uchar>(p) = 1;
		m.copyTo(mask);
		return;
	}

	void colorize(cv::OutputArray dst, const int index, const cv::Vec3b& color = cv::Vec3b(0, 255, 0))
	{
		cv::Mat im_tmp;
		cv::cvtColor(image_, im_tmp, cv::COLOR_GRAY2RGB);
		cv::cvtColor(im_tmp, im_tmp, cv::COLOR_RGB2HSV);

		for (const auto& p : connected_pixels_[index])
			im_tmp.at<cv::Vec3b>(p) = color;
	}

	void colorize(cv::OutputArray dst, const std::vector<int>& indices)
	{
		cv::Mat im_tmp;
		cv::cvtColor(image_, im_tmp, cv::COLOR_GRAY2RGB);
		cv::cvtColor(im_tmp, im_tmp, cv::COLOR_RGB2HSV);

		const int kStep = 180 / (int)indices.size();

		for (size_t i = 0; i < indices.size(); ++i)
		{
			const cv::Vec3b color(static_cast<uchar>(i * kStep) + 30, 255, 255);

			for (const auto& p : connected_pixels_[i])
				im_tmp.at<cv::Vec3b>(p) = color;
		}
		cv::cvtColor(im_tmp, dst, cv::COLOR_HSV2RGB);
	}

	void colorize(cv::OutputArray dst)
	{
		cv::Mat im_tmp;
		cv::cvtColor(image_, im_tmp, cv::COLOR_GRAY2RGB);
		cv::cvtColor(im_tmp, im_tmp, cv::COLOR_RGB2HSV);

		const size_t kNumRegion = connected_pixels_.size();
		const int kStep = 180 / (int)kNumRegion;

		for (size_t i = 0; i < kNumRegion; ++i)
		{
			const cv::Vec3b color(static_cast<uchar>(i * kStep) + 30, 255, 255);

			for (const auto& p : connected_pixels_[i])
				im_tmp.at<cv::Vec3b>(p) = color;
		}
		cv::cvtColor(im_tmp, dst, cv::COLOR_HSV2RGB);
	}
};		// class Labeling

}	// namespace bs 
