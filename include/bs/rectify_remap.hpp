#pragma once

#include <fstream>
#include <array>
#pragma warning(disable:4819)
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#pragma warning(default:4819)

#include <bs/stereo_util.hpp>

namespace bs
{

class RectifyRemap
{
private:
	cv::Mat mx1_;
	cv::Mat my1_;
	cv::Mat mx2_;
	cv::Mat my2_;

	void saveBinary(const std::string file, const cv::Mat& mat)
	{
		std::ofstream ofs(file, std::ios::binary);

		if (!ofs.is_open())
		{
			return;
		}
		if (mat.empty())
		{
			int s = 0;
			ofs.write((const char*)(&s), sizeof(int));
			return;
		}

		int type = mat.type();
		ofs.write((const char*)(&mat.rows), sizeof(int));
		ofs.write((const char*)(&mat.cols), sizeof(int));
		ofs.write((const char*)(&type), sizeof(int));
		ofs.write((const char*)(mat.data), mat.elemSize() * mat.total());

		return;
	}

	bool loadBinary(const std::string file, cv::Mat& mat)
	{
		std::ifstream ifs(file, std::ios::binary);

		if (!ifs.is_open())		return false;

		int rows, cols, type;
		ifs.read((char*)(&rows), sizeof(int));
		if (rows == 0)			return true;

		ifs.read((char*)(&cols), sizeof(int));
		ifs.read((char*)(&type), sizeof(int));

		mat.release();
		mat.create(rows, cols, type);
		ifs.read((char*)(mat.data), mat.elemSize() * mat.total());

		return true;
	}

public:
	RectifyRemap(){};

	RectifyRemap(const std::string dir, const std::string extention = ".bin")
	{
		load(dir, extention);
	}

	RectifyRemap(const RectifyRemap& o){ *this = o; }

	void save(const std::string dir, const std::string extention = ".bin")
	{
		saveBinary(dir + "mx1" + extention, mx1_);
		saveBinary(dir + "my1" + extention, my1_);
		saveBinary(dir + "mx2" + extention, mx2_);
		saveBinary(dir + "my2" + extention, my2_);

		return;
	}

	bool load(const std::string dir, const std::string extention = ".bin")
	{
		loadBinary(dir + "mx1" + extention, mx1_);
		loadBinary(dir + "my1" + extention, my1_);
		loadBinary(dir + "mx2" + extention, mx2_);
		loadBinary(dir + "my2" + extention, my2_);

		if (isEmpty())
		{
			std::cout << "[Rectify Remap] Fail to load file." << std::endl;
			return false;
		}
		else
		{
			std::cout << "[Rectify Remap] Success load file." << std::endl;
			return true;
		}
	}

	void remap(cv::InputOutputArray image, const int interpolation, const bool is_left)
	{
		if (is_left)
			cv::remap(image, image, mx1_, my1_, interpolation);
		else
			cv::remap(image, image, mx2_, my2_, interpolation);
		return;
	}

	void remap(cv::InputArray im_src, cv::OutputArray im_dst, const int interpolation, const bool is_left)
	{
		if (is_left)
			cv::remap(im_src, im_dst, mx1_, my1_, interpolation);
		else
			cv::remap(im_src, im_dst, mx2_, my2_, interpolation);
		return;
	}

	void st_remap(const Stereo<cv::Mat>& im, const int interpolation)
	{
		cv::remap(im[0], im[0], mx1_, my1_, interpolation);
		cv::remap(im[1], im[1], mx2_, my2_, interpolation);
		return;
	}

	void st_remap(const Stereo<cv::Mat>& im_src, Stereo<cv::Mat>& im_dst, const int interpolation)
	{
		cv::remap(im_src[0], im_dst[0], mx1_, my1_, interpolation);
		cv::remap(im_src[1], im_dst[1], mx2_, my2_, interpolation);
		return;
	}

	bool isEmpty(void)
	{
		bool is_emp =
			mx1_.empty() ||
			my1_.empty() ||
			mx2_.empty() ||
			my2_.empty();

		return is_emp;
	}
};	// RectifyRemap

}	// namespace bs
