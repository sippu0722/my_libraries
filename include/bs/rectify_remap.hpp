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

	static void saveBinary(const std::string file, const cv::Mat& mat)
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

	void save(const std::string dir, const std::string extention = ".bin")
	{
		saveBinary(dir + "mx1" + extention, mx1_);
		saveBinary(dir + "my1" + extention, my1_);
		saveBinary(dir + "mx2" + extention, mx2_);
		saveBinary(dir + "my2" + extention, my2_);
		return;
	}

	static void save(const std::string dir, cv::InputArray mx1, cv::InputArray my1, cv::InputArray mx2, cv::InputArray my2, std::string extention = ".bin")
	{
		saveBinary(dir + "mx1" + extention, mx1.getMat());
		saveBinary(dir + "my1" + extention, my2.getMat());
		saveBinary(dir + "mx2" + extention, mx2.getMat());
		saveBinary(dir + "my2" + extention, my2.getMat());
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

	//void remap(cv::InputOutputArray image, const int interpolation, const bool is_left)
	//{
	//	if (is_left)
	//		cv::remap(image, image, mx1_, my1_, interpolation);
	//	else
	//		cv::remap(image, image, mx2_, my2_, interpolation);
	//	return;
	//}

	//void remap(cv::InputArray im_src, cv::OutputArray im_dst, const int interpolation, const bool is_left)
	//{
	//	if (is_left)
	//		cv::remap(im_src, im_dst, mx1_, my1_, interpolation);
	//	else
	//		cv::remap(im_src, im_dst, mx2_, my2_, interpolation);
	//	return;
	//}

	void st_remap(cv::InputOutputArrayOfArrays im, const int interpolation)
	{
		std::vector<cv::Mat> im_vec;
		im.getMatVector(im_vec);
		assert(im_vec.size() == 2);
		cv::remap(im_vec[0], im_vec[0], mx1_, my1_, interpolation);
		cv::remap(im_vec[1], im_vec[1], mx2_, my2_, interpolation);
		return;
	}

	void st_remap(std::array<cv::Mat, 2>& im, const int interpolation = cv::INTER_LINEAR)
	{
		std::vector<cv::Mat> tmp = { im[L], im[R] };
		st_remap(tmp, interpolation);
		im = { {tmp[L], tmp[R]} };
		return;
	}

	//void st_remap(cv::InputArrayOfArrays im_src, cv::OutputArrayOfArrays im_dst, const int interpolation)
	//{
	//	cv::remap(im_src[0], im_dst[0], mx1_, my1_, interpolation);
	//	cv::remap(im_src[1], im_dst[1], mx2_, my2_, interpolation);
	//	return;
	//}

	void operator()(cv::InputOutputArrayOfArrays im, const int interpolation = cv::INTER_LINEAR)
	{
		st_remap(im, interpolation);
		return;
	}
	
	// å„Ç≈è¡ÇµÇΩÇ¢
	void operator()(std::array<cv::Mat, 2>& im, const int interpolation = cv::INTER_LINEAR)
	{
		st_remap(std::vector<cv::Mat>({ im[0], im[1] }), interpolation);
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
