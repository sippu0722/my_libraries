#pragma once
#pragma warning(disable:4819)
#include <opencv2/core/cvstd.hpp>
#pragma warning(default:4819)


namespace bs
{

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
			(tmp == CV_64F) ? "CV_64F" : "";
		str += str_cn;
	}
	return str.empty() ? "Unknown type!!!" : str;
}

}
