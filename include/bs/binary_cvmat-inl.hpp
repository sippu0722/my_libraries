#pragma once

#pragma warning(disable:4819)
#include <opencv2/core/core.hpp>
#pragma warning(default:4819)
#include <fstream>


namespace bs
{

//! Save cv::Mat as binary
/*!
\param[in] filename filaname to save
\param[in] output cvmat to save
*/
inline bool saveMatBinary(const std::string& filename, const cv::Mat& output)
{
	std::ofstream ofs(filename, std::ios::binary);

	if (!ofs.is_open())
		return false;

	if (output.empty())
	{
		int s = 0;
		ofs.write((const char*)(&s), sizeof(int));
		return true;
	}
	int type = output.type();
	ofs.write((const char*)(&output.rows), sizeof(int));
	ofs.write((const char*)(&output.cols), sizeof(int));
	ofs.write((const char*)(&type), sizeof(int));
	ofs.write((const char*)(output.data), output.elemSize() * output.total());

	return true;
}


//! Load cv::Mat as binary
/*!
\param[in] filename filaname to load
\param[out] output loaded cv::Mat
*/
inline bool loadMatBinary(const std::string& filename, cv::Mat& output)
{
	std::ifstream ifs(filename, std::ios::binary);

	if (!ifs.is_open())
		return false;

	int rows, cols, type;
	ifs.read((char*)(&rows), sizeof(int));
	if (rows == 0)
		return true;
	ifs.read((char*)(&cols), sizeof(int));
	ifs.read((char*)(&type), sizeof(int));

	output.release();
	output.create(rows, cols, type);
	ifs.read((char*)(output.data), output.elemSize() * output.total());

	return true;
}

}
