#pragma once

#include <iostream>
#include <cassert>

#pragma warning(disable:4819)
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <FC1/PGRFlyCapture.h>
#pragma warning(default:4819)


#ifndef _BS_STEREO_
#define _BS_STEREO_
#include<array>
template<class T>
using Stereo = std::array<T, 2>;
#endif


/*!
Flycapture wrapper
*/
namespace fc
{

using Error = FlyCaptureError;
using Context = FlyCaptureContext;
using CamSerial = FlyCaptureCameraSerialNumber;
using PixFmt = FlyCapturePixelFormat;
using Property = FlyCaptureProperty;
using Image = FlyCaptureImage;

inline PixFmt int2PixFmt(const int i)
{
	PixFmt fmt;
	
	switch (i)
	{
	default:
		break;

	case fc::PixFmt::FCPF_FORCE_QUADLET:
		fmt = fc::PixFmt::FCPF_FORCE_QUADLET;
		break;

	case fc::PixFmt::FLYCAPTURE_411YUV8:
		fmt = fc::PixFmt::FLYCAPTURE_411YUV8;
		break;

	case fc::PixFmt::FLYCAPTURE_422YUV8:
		fmt = fc::PixFmt::FLYCAPTURE_422YUV8;
		break;

	case fc::PixFmt::FLYCAPTURE_444YUV8:
		fmt = fc::PixFmt::FLYCAPTURE_444YUV8;
		break;

	case fc::PixFmt::FLYCAPTURE_BGR:
		fmt = fc::PixFmt::FLYCAPTURE_BGR;
		break;

	case fc::PixFmt::FLYCAPTURE_BGRU:
		fmt = fc::PixFmt::FLYCAPTURE_BGRU;
		break;

	case fc::PixFmt::FLYCAPTURE_MONO16:
		fmt = fc::PixFmt::FLYCAPTURE_MONO16;
		break;

	case fc::PixFmt::FLYCAPTURE_MONO8:
		fmt = fc::PixFmt::FLYCAPTURE_MONO8;
		break;

	case fc::PixFmt::FLYCAPTURE_RAW16:
		fmt = fc::PixFmt::FLYCAPTURE_RAW16;
		break;

	case fc::PixFmt::FLYCAPTURE_RAW8:
		fmt = fc::PixFmt::FLYCAPTURE_RAW8;
		break;

	case fc::PixFmt::FLYCAPTURE_RGB16:
		fmt = fc::PixFmt::FLYCAPTURE_RGB16;
		break;

	case fc::PixFmt::FLYCAPTURE_RGB8:
		fmt = fc::PixFmt::FLYCAPTURE_RGB8;
		break;

	case fc::PixFmt::FLYCAPTURE_S_MONO16:
		fmt = fc::PixFmt::FLYCAPTURE_S_MONO16;
		break;

	case fc::PixFmt::FLYCAPTURE_S_RGB16:
		fmt = fc::PixFmt::FLYCAPTURE_S_RGB16;
		break;
	}
	return fmt;
}

inline void fcAssert(const fc::Error error, const std::string name)
{
	if (error != FLYCAPTURE_OK)
	{
		std::cerr << std::endl
			<< "Error in " << name << " -> "
			<< flycaptureErrorToString(error) << std::endl;
		abort();
	}
	return;
}

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

}	// namespace fc

namespace bs
{

struct CameraParams
{
	fc::CamSerial serial;
	fc::PixFmt pixel_format;
	cv::Size size;
	float shutter, gain, fps;

	CameraParams() :
		serial(0), pixel_format(FLYCAPTURE_MONO8), size(cv::Size(0, 0)),
		shutter(-1.f), gain(-1.f), fps(-1.f)
	{}

	CameraParams(
		const fc::CamSerial _serial, const fc::PixFmt _fmt, const cv::Size _sz,
		const float _shutter, const float _gain, const float _fps) :
		serial(_serial), pixel_format(_fmt), size(_sz), shutter(_shutter), gain(_gain), fps(_fps)
	{}

	CameraParams(const fc::CamSerial _serial, const fc::PixFmt _fmt, const cv::Size _sz) :
		serial(_serial), pixel_format(_fmt), size(_sz), shutter(-1.f), gain(-1.f), fps(-1.f)
	{}
};

}

inline void loadCameraParameterBase(cv::FileNode& fn, bs::CameraParams& param)
{
	int tmp_serial, tmp_format;

	fn["serial"]	>> tmp_serial;	param.serial = tmp_serial;
	fn["format"]	>> tmp_format;	param.pixel_format = fc::int2PixFmt(tmp_format);
	fn["size"]		>> param.size;
	fn["shutter"]	>> param.shutter;
	fn["gain"]		>> param.gain;
	fn["fps"]		>> param.fps;
	return;
}

inline void saveCameraParameterBase(cv::FileStorage& fs, const bs::CameraParams& param)
{
	fs << "serial"	<< (int)param.serial;
	fs << "format"	<< (int)param.pixel_format;
	fs << "size"	<< param.size;
	fs << "shutter"	<< param.shutter;
	fs << "gain"	<< param.gain;
	fs << "fps"		<< param.fps;
	return;
}


namespace bs
{

bool loadCameraParameter(const cv::FileStorage& fs, CameraParams& param)
{
	cv::FileNode fn(fs.fs, nullptr);
	loadCameraParameterBase(fn, param);

	return true;
}

bool loadStereoCameraParameter(const cv::FileStorage& fs, CameraParams& param_l, CameraParams& param_r)
{
	loadCameraParameterBase(fs["left"], param_l);
	loadCameraParameterBase(fs["right"], param_r);

	return true;
}

bool loadStereoCameraParameter(const cv::FileStorage& fs, Stereo<CameraParams>& param)
{
	return loadStereoCameraParameter(fs, param[0], param[1]);
}

bool saveCameraParameter(cv::FileStorage& fs, const CameraParams& param)
{
	saveCameraParameterBase(fs, param);
	return true;
}

bool saveStereoCameraParameter(cv::FileStorage& fs, const CameraParams& param_l, const CameraParams& param_r)
{
	fs << "left" << "{";
	saveCameraParameterBase(fs, param_l);
	fs << "}";

	fs << "right" << "{";
	saveCameraParameterBase(fs, param_r);
	fs << "}";

	return true;
}

bool saveStereoCameraParameter(cv::FileStorage& fs, const Stereo<CameraParams>& param)
{
	saveStereoCameraParameter(fs, param[0], param[1]);

	return true;
}


/*!
@brief Flycaptureのカメラを使いやすくするクラス

@code
// Initialization
CameraController cam_cntl(camera_serial_number, camera_size);
cam_cntl.setProp(shutter, gain, frame_rate);

// Get image
cv::Mat image;
cam_cntl >> image;

// Resize image
cv::Size view_size = cam_cntl.setViewSize(640);
cv::resize(image, image, view_size);
@endcode
*/
class CameraController
{
	fc::Context fly_;
	std::string file_param_;
	CameraParams param_;
	bool stereo_;

	void init()
	{
		CV_Assert(0 < param_.size.area());

		fc::Error err;

		err = fc::createContext(&fly_);
		fc::fcAssert(err, "fc::createContext");

		err = fc::initFromSerial(fly_, param_.serial);
		fc::fcAssert(err, "fc::initFromSerial");

		err = fc::startCustomImage(fly_, param_.size.width, param_.size.height, param_.pixel_format);
		fc::fcAssert(err, "fc::startCustomImage");

		setProp(param_.shutter, param_.gain, param_.fps);

		return;
	}


public:
	CameraController() :
		file_param_("./cam_param.xml"), stereo_(false)
	{
		cv::FileStorage fs(file_param_, cv::FileStorage::READ);
		CV_Assert(fs.isOpened() || !"Cannot open file in bs::CameraController constructor.");

		bool complete = loadCameraParameter(fs, param_);
		assert(complete || !"in CameraController constructor from file.");
		init();

		std::cout << "[Camera Controller] Success initialize from file." << std::endl;
	}

	CameraController(const CameraParams& param, const bool stereo = false) :
		param_(param), stereo_(stereo)
	{
		init();

		if (!stereo_)
			std::cout << "[Camera Controller] Success initialize from CameraParams." << std::endl;
	}

	CameraController(const fc::CamSerial serial, const cv::Size size, const fc::PixFmt fmt = FLYCAPTURE_MONO8, const bool stereo = false) :
		param_(CameraParams(serial, fmt, size))
	{
		init();

		if (!stereo_)
			std::cout << "[Camera Controller] Success initialize from variables." << std::endl;
	}

	~CameraController()
	{
		fc::stop(fly_);
		fc::destroyContext(fly_);

		if (!stereo_)
		{
			if (!file_param_.empty())
			{
				cv::FileStorage fs(file_param_, cv::FileStorage::WRITE);
				CV_Assert(fs.isOpened());

				bool complete = saveCameraParameter(fs, param_);
				assert(complete || !"saveCameraParameter");
			}
			std::cout << "[Camera Controller] Bye!" << std::endl;
		}
	}

	void restart(const fc::CamSerial serial = 0, const cv::Size size = cv::Size(), const fc::PixFmt fmt = FCPF_FORCE_QUADLET)
	{
		fc::stop(fly_);
		fc::destroyContext(fly_);

		param_.serial = (serial == 0) ? param_.serial : serial;
		param_.size = (size.area() == 0) ? param_.size : size;
		param_.pixel_format = (fmt == FCPF_FORCE_QUADLET) ? param_.pixel_format : fmt;

		init();
		if (!stereo_)
			std::cout << "[Camera Controller] Success restart." << std::endl;
	}

	void loadProp(cv::FileStorage& fs = cv::FileStorage())
	{
		if (!fs.isOpened())
			fs.open(file_param_, cv::FileStorage::READ);
		CV_Assert(fs.isOpened() || !"in loadProp.");

		if (!loadCameraParameter(fs, param_))
			return;
		setProp(param_.shutter, param_.gain, param_.fps);

		if (!stereo_)
			std::cout << "[Camera Controller] Parameter loaded." << std::endl;
		return;
	}

	CameraParams getParams()
	{
		return param_;
	}

	void setProp(const float shutter, const float gain, const float fps)
	{
		fc::Error err;

		if (0 <= shutter)
			err = fc::setCamAbsProp(fly_, FLYCAPTURE_SHUTTER, shutter);
		else
			err = fc::setCamProp(fly_, FLYCAPTURE_SHUTTER, 0, 0, true);
		fc::fcAssert(err, "fc::setCamProp(shutter)");
		param_.shutter = shutter;

		if (0 <= gain)
			err = fc::setCamAbsProp(fly_, FLYCAPTURE_GAIN, gain);
		else
			err = fc::setCamProp(fly_, FLYCAPTURE_GAIN, 0, 0, true);
		fc::fcAssert(err, "fc::setCamProp(gain)");
		param_.gain = gain;

		if (0 <= fps)
			err = fc::setCamAbsProp(fly_, FLYCAPTURE_FRAME_RATE, fps);
		else
			err = fc::setCamProp(fly_, FLYCAPTURE_FRAME_RATE, 0, 0, true);
		fc::fcAssert(err, "fc::setCamProp(frame rate)");
		param_.fps = fps;

		return;
	}

	void setProp(const fc::Property prop, const float value)
	{
		switch (prop)
		{
		default:
			break;

		case FLYCAPTURE_SHUTTER:
			param_.shutter = value;
			break;
		
		case FLYCAPTURE_GAIN:
			param_.gain = value;
			break;

		case FLYCAPTURE_FRAME_RATE:
			param_.fps = value;
			break;
		}

		fc::Error err;

		if (0 <= value)
			err = fc::setCamAbsProp(fly_, prop, value);
		else
			err = fc::setCamProp(fly_, prop, 0, 0, true);
		fc::fcAssert(err, "fc::setCamProp OR fc::setCamAbsProp");

		return;
	}

	void operator >> (cv::Mat& im)
	{
		fc::Image fc_im, im_conv;
		unsigned char *buffer;

		fc::Error err = fc::grabImage2(fly_, &fc_im);
		fc::fcAssert(err, "fc::grabImage2");

		buffer = new unsigned char[fc_im.iRows * fc_im.iCols];
		im_conv.pData = buffer;
		im_conv.pixelFormat = param_.pixel_format;
		fc::cvtImage(fly_, &fc_im, &im_conv);
		im = cv::Mat(im_conv.iRows, im_conv.iCols, CV_8U);
		memcpy(im.data, im_conv.pData, im_conv.iRows * im_conv.iRowInc);
		delete[]buffer;

		if (im.empty())
		{
			std::cout << "[Camera Controller] Capture failure. Restart." << std::endl;
			restart();
			*this >> im;
		}
		//if (!stereo_)
		//	std::cout << "[Camera Controller] Success capture." << std::endl;
	}

	cv::Size setViewSize(const int width)
	{
		CV_Assert(0 < param_.size.area());

		int height = (param_.size.height * width) / param_.size.width;
		return cv::Size(width, height);
	}

};


/*!
@brief Flycaptureのカメラを2台一度に使いやすくするクラス

@code
// template<class T>
// using Stereo = std::array<T, 2>;

// Initialization
StereoCameraController cam_cntl(camera_serial_number, camera_size);
cam_cntl.setProp(shutter, gain, frame_rate);

// Get image
Stereo<cv::Mat> image;
cam_cntl >> image;

// Resize image
cv::Size view_size = cam_cntl.setViewSize(640);
cv::resize(image[0], image[0], view_size);
@endcode
*/
class StereoCameraController
{
	CameraController cam_left_, cam_right_;
	CameraParams param_l_, param_r_;
	std::string file_param_;

public:
	enum CAM_SELECT
	{
		LEFT,
		RIGHT,
		BOTH
	};

	StereoCameraController() :
		file_param_("./stereo_cam_param.xml")
	{
		cv::FileStorage fs(file_param_, cv::FileStorage::READ);
		CV_Assert(fs.isOpened() || !"Cannot open file in bs::StereoCameraController constructor.");

		loadStereoCameraParameter(fs, param_l_, param_r_);
		cam_left_ = *(new CameraController(param_l_, true));
		cam_right_ = *(new CameraController(param_r_, true));
		std::cout << "[Stereo Camera Controller] Success initialize from file." << std::endl;
	}

	StereoCameraController(const CameraParams& param_l, const CameraParams& param_r) :
		param_l_(param_l), param_r_(param_r)
	{
		cam_left_ = *(new CameraController(param_l, true));
		cam_right_ = *(new CameraController(param_r, true));
		std::cout << "[Stereo Camera Controller] Success initialize from CameraParams." << std::endl;
	}

	StereoCameraController(const Stereo<fc::CamSerial> serial, const cv::Size size, const fc::PixFmt fmt = FLYCAPTURE_MONO8) :
		param_l_(CameraParams(serial[0], fmt, size)), param_r_(CameraParams(serial[1], fmt, size))
	{
		CV_Assert(0 < size.area());

		cam_left_ = *(new CameraController(serial[0], size, fmt, true));
		cam_right_ = *(new CameraController(serial[1], size, fmt, true));
		std::cout << "[Stereo Camera Controller] Success initialize from variables." << std::endl;
	}

	~StereoCameraController()
	{
		if (!file_param_.empty())
		{
			cv::FileStorage fs(file_param_, cv::FileStorage::WRITE);
			CV_Assert(fs.isOpened());

			bool complete = saveStereoCameraParameter(fs, param_l_, param_r_);
			assert(complete || !"saveStereoCameraParameter");
		}
		std::cout << "[Stereo Camera Controller] Bye!" << std::endl;
	}

	void restart(const Stereo<fc::CamSerial> serial, const cv::Size size, const fc::PixFmt fmt = FLYCAPTURE_MONO8)
	{
		cam_left_.restart(serial[0], size, fmt);
		cam_right_.restart(serial[1], size, fmt);

		std::cout << "[Stereo Camera Controller] Success restart." << std::endl;
		return;
	}

	void loadProp(cv::FileStorage fs = cv::FileStorage())
	{
		if (!fs.isOpened())
			fs.open(file_param_, cv::FileStorage::READ);
		CV_Assert(fs.isOpened());

		if (!loadStereoCameraParameter(fs, param_l_, param_r_))
			return;

		Stereo<float> shutter, gain, fps;
		shutter = { { param_l_.shutter, param_r_.shutter } };
		gain = { { param_l_.gain, param_r_.gain } };
		fps = { { param_l_.fps, param_r_.fps } };
		setProp(shutter, gain, fps);
		return;
	}

	CameraParams getParams(const CAM_SELECT select)
	{
		if (select == CAM_SELECT::LEFT)
			return cam_left_.getParams();
		else if (select == CAM_SELECT::RIGHT)
			return cam_right_.getParams();

		std::cerr << "[Stereo Camera Controller] Error: argument must be 'LEFT' or 'RIGHT'." << std::endl;
		return CameraParams();
	}

	void setProp(const Stereo<float> shutter, const Stereo<float> gain, const Stereo<float> fps)
	{
		cam_left_.setProp(shutter[0], gain[0], fps[0]);
		cam_right_.setProp(shutter[1], gain[1], fps[1]);
		return;
	}

	bool setProp(const fc::Property prop, const float value, CAM_SELECT select)
	{
		if (select == CAM_SELECT::LEFT)
		{
			cam_left_.setProp(prop, value);
			return true;
		}
		else if (select == CAM_SELECT::RIGHT)
		{
			cam_right_.setProp(prop, value);
			return true;
		}

		std::cerr << "[Stereo Camera Controller] Error: argument must be 'LEFT' or 'RIGHT'." << std::endl;
		return false;
	}

	void setProp(const fc::Property prop, const Stereo<float> value)
	{
		cam_left_.setProp(prop, value[0]);
		cam_right_.setProp(prop, value[1]);
		return;
	}

	void capture(cv::Mat& im, CAM_SELECT select)
	{
		assert(select == (LEFT | RIGHT) || !"in StereCameraController capture");

		//std::cout << "[Stereo Camera Controller] Capture. ";

		switch (select)
		{
		case StereoCameraController::LEFT:
			//std::cout << "left..";
			cam_left_ >> im;
			break;

		case StereoCameraController::RIGHT:
			//std::cout << "right..";
			cam_right_ >> im;
			break;

		default:
			break;
		}
		//std::cout << "success." << std::endl;
	}

	void operator >> (Stereo<cv::Mat>& im)
	{
		//std::cout << "[Stereo Camera Controller] Capture. left..";
		cam_left_ >> im[0];

		//std::cout << "right..";
		cam_right_ >> im[1];
		//std::cout << "success." << std::endl;
	}

	/*!
	@note Support only left_size == right_size
	*/
	Stereo<cv::Size> setViewSize(const int width)
	{
		CV_Assert(0 < param_l_.size.area() && 0 < param_r_.size.area());
		
		Stereo<cv::Size> output;
		int height = (param_l_.size.height * width) / param_l_.size.width;
		output[0] = cv::Size(width, height);
		height = (param_r_.size.height * width) / param_r_.size.width;
		output[1] = cv::Size(width, height);

		return output;
	}
};

}
