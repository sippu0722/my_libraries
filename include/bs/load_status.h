#pragma once

#include <iostream>
#include <fstream>

#pragma warning(disable:4819)
#include <opencv2/calib3d.hpp>
#pragma warning(default:4819)

#ifndef _BS_STEREO_
#define _BS_STEREO_
#include<array>
template<class T>
using Stereo = std::array < T, 2 >;
#endif

namespace bs
{
	/**
	@brief Load camera status 'Serial number', 'Size', 'Shutter', 'Gain', 'Frame rate' in stereo.
	*/
	bool loadStereoCameraStatus(
		const std::string file,
		Stereo<unsigned long>& serial,
		cv::Size& sz,
		Stereo<float>& shutter,
		Stereo<float>& gain,
		float& frame_rate);

	/**
	@brief Load camera status 'Serial number', 'Size' in stereo.
	*/
	bool loadStereoCameraStatus(
		const std::string file,
		Stereo<unsigned long>& serial,
		cv::Size& sz);


	/**
	@brief Load stereoBM status 'preFilterSize', 'preFilterCap',
	'SADWindowSize', 'minDisparity', 'textureThreshold', 'textureThreshold'.
	@note 'numberOfDisparities' should be set in yourself.
	*/
	bool loadBmStatus(
		const std::string file,
		cv::Ptr<cv::StereoBM> bm);

	bool loadSgbmStatus(
		const std::string file,
		cv::Ptr<cv::StereoSGBM> sgbm);
}
