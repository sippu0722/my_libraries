#pragma once

#ifndef _BS_STEREO_
#define _BS_STEREO_
#include<array>
template<class T>
using Stereo = std::array < T, 2 >;
#endif

#pragma warning(disable:4819)
#include <opencv2/core/mat.hpp>
#pragma warning(default:4819)

#include <FC1/PGRFlyCapture.h>

// Flycapture wrapper
//
namespace fc
{

using Error		= FlyCaptureError;
using Context	= FlyCaptureContext;
using CamSerial = FlyCaptureCameraSerialNumber;
using PixFmt	= FlyCapturePixelFormat;
using ImFmt		= FlyCaptureImageFileFormat;
using Property	= FlyCaptureProperty;
using Image		= FlyCaptureImage;

inline Error createContext(Context *pContext)
{
	return flycaptureCreateContext(pContext);
}

inline Error initFromSerial(Context context, CamSerial serialNumber)
{
	return flycaptureInitializeFromSerialNumber(context, serialNumber);
}

inline Error startCustomImage(
	Context context,
	unsigned int uiMode,
	unsigned int uiImagePosLeft,
	unsigned int uiImagePosTop,
	unsigned int uiWidth,
	unsigned int uiHeight,
	float fBandwidth,
	PixFmt format)
{
	return flycaptureStartCustomImage(
		context,
		uiMode,
		uiImagePosLeft,
		uiImagePosTop,
		uiWidth,
		uiHeight,
		fBandwidth,
		format);
}

inline Error startCustomImage(
	Context context,
	unsigned int uiWidth,
	unsigned int uiHeight,
	PixFmt format = FLYCAPTURE_MONO8,
	unsigned int uiMode = 0,
	unsigned int uiImagePosLeft = 0,
	unsigned int uiImagePosTop = 0,
	float fBandwidth = 100)
{
	return flycaptureStartCustomImage(
		context,
		uiMode,
		uiImagePosLeft,
		uiImagePosTop,
		uiWidth,
		uiHeight,
		fBandwidth,
		format);
}

inline Error getCamProp(
	Context context,
	Property cameraProperty,
	long *plValueA,
	long *plValueB,
	bool *pbAuto)
{
	return flycaptureGetCameraProperty(
		context,
		cameraProperty,
		plValueA,
		plValueB,
		pbAuto);
}

inline Error setCamProp(
	Context context,
	Property cameraProperty,
	long lValueA,
	long lValueB,
	bool bAuto)
{
	return flycaptureSetCameraProperty(
		context,
		cameraProperty,
		lValueA,
		lValueB,
		bAuto);
}

inline Error getCamAbsProp(
	Context context,
	Property cameraProperty,
	float *pfValue)
{
	return flycaptureGetCameraAbsProperty(context, cameraProperty, pfValue);
}

inline Error setCamAbsProp(
	Context context,
	Property cameraProperty,
	float fValue)
{
	return flycaptureSetCameraAbsProperty(context, cameraProperty, fValue);
}

inline Error stop(Context context)
{
	return flycaptureStop(context);
}

inline Error destroyContext(Context context)
{
	return flycaptureDestroyContext(context);
}

inline Error grabImage2(Context context, Image *pimage)
{
	return flycaptureGrabImage2(context, pimage);
}

inline Error cvtImage(Context context, Image *pimageSrc, Image *pimageDest)
{
	return flycaptureConvertImage(context, pimageSrc, pimageDest);
}

inline Error saveImage(Context context, Image *pImage, const char *pszPath, ImFmt
	format)
{
	return flycaptureSaveImage(context, pImage, pszPath, format);
}

}	// namespace fc


namespace bs
{

fc::Error st_fcCreateContext(Stereo<fc::Context>& fly);

fc::Error st_fcInitFromSerial(
	const Stereo<fc::Context>& fly,
	const Stereo<fc::CamSerial> serial);

fc::Error st_fcStartCustomImage(
	const Stereo<fc::Context>& fly,
	const unsigned int uiMode,
	const unsigned int uiImagePosLeft,
	const unsigned int uiImagePosTop,
	const unsigned int uiWidth,
	const unsigned int uiHeight,
	const float fBandwidth,
	const fc::PixFmt format);

fc::Error st_fcStartCustomImage(
	const Stereo<fc::Context>& fly,
	const unsigned int uiWidth,
	const unsigned int uiHeight,
	const fc::PixFmt format = FLYCAPTURE_MONO8,
	const unsigned int uiMode = 0,
	const unsigned int uiImagePosLeft = 0,
	const unsigned int uiImagePosTop = 0,
	const float fBandwidth = 100);

fc::Error st_fcSetCamProp(
	const Stereo<fc::Context>& fly,
	const Stereo<float> shutter = { { -1.f, -1.f } },
	const Stereo<float> gain = { { -1.f, -1.f } },
	const float frame_rate = -1);

fc::Error st_fcSetCamProp(
	const Stereo<fc::Context>& fly,
	const fc::Property prop,
	const Stereo<float> value);

fc::Error st_fcStop(const Stereo<fc::Context>& fly);

fc::Error st_fcDestroyContext(const Stereo<fc::Context>& fly);

bool capture(
	const fc::Context& fly,
	cv::Mat& dst,
	const fc::PixFmt format = FLYCAPTURE_MONO8);

bool st_capture(
	const Stereo<fc::Context>& fly,
	Stereo<cv::Mat>& dst,
	const fc::PixFmt format = FLYCAPTURE_MONO8);
}
