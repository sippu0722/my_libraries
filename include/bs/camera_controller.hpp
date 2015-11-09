#pragma once

#include <iostream>
#include <cassert>

#pragma warning(disable:4819)
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <FC1/PGRFlyCapture.h>
#pragma warning(default:4819)

#include <bs/stereo_util.h>

// Change these values in your environment.
namespace bs
{
	namespace param
	{
		std::string gl_file_cam_param = "__output/cam_param.xml";
		std::string gl_file_stereo_cam_param = "__output/stereo_cam_param.xml";
	}
}

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

inline void Assert(const fc::Error error, const std::string name)
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
	int channels;

	CameraParams() :
		serial(0), pixel_format(FLYCAPTURE_MONO8), size(cv::Size(0, 0)),
		shutter(-1.f), gain(-1.f), fps(-1.f), channels(1)
	{}

	CameraParams(
		const fc::CamSerial _serial, const fc::PixFmt _fmt, const cv::Size _sz,
		const float _shutter, const float _gain, const float _fps) :
		serial(_serial), pixel_format(_fmt), size(_sz), shutter(_shutter), gain(_gain), fps(_fps), channels(1)
	{}

	CameraParams(const fc::CamSerial _serial, const fc::PixFmt _fmt, const cv::Size _sz) :
		serial(_serial), pixel_format(_fmt), size(_sz), shutter(-1.f), gain(-1.f), fps(-1.f), channels(1)
	{}
};

}

namespace
{

inline void loadCameraParameterBase(const cv::FileNode& fn, bs::CameraParams& param)
{
	int tmp_serial, tmp_format;

	fn["serial"] >> tmp_serial;	param.serial = tmp_serial;
	fn["format"] >> tmp_format;	param.pixel_format = static_cast<fc::PixFmt>(tmp_format);
	fn["channels"] >> param.channels;
	fn["size"] >> param.size;
	fn["shutter"] >> param.shutter;
	fn["gain"] >> param.gain;
	fn["fps"] >> param.fps;
	return;
}

inline void saveCameraParameterBase(cv::FileStorage& fs, const bs::CameraParams& param)
{
	fs << "serial" << (int)param.serial;
	fs << "format" << (int)param.pixel_format;
	fs << "channels" << param.channels;
	fs << "size" << param.size;
	fs << "shutter" << param.shutter;
	fs << "gain" << param.gain;
	fs << "fps" << param.fps;
	return;
}

}

namespace bs
{

bool loadCameraParameter(const cv::FileStorage& fs, CameraParams& param)
{
	cv::FileNode fn(fs.fs, nullptr);
	loadCameraParameterBase(fn, param);

	return true;
}

bool loadCameraParameter(const cv::FileNode& fn, CameraParams& param)
{
	loadCameraParameterBase(fn, param);

	return true;
}


bool loadStereoCameraParameter(const cv::FileStorage& fs, Stereo<CameraParams>& param)
{
	loadCameraParameterBase(fs["left"], param[L]);
	loadCameraParameterBase(fs["right"], param[R]);
	return true;
}

//bool loadStereoCameraParameter(const cv::FileStorage& fs, CameraParams& param_l, CameraParams& param_r)
//{
//	return loadStereoCameraParameter(fs, { { param_l, param_r } });
//}

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
	saveStereoCameraParameter(fs, param[L], param[R]);

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
cv::Size view_size = cam_cntl.calcViewSize(640);
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
		fc::Assert(err, "fc::createContext");

		err = fc::initFromSerial(fly_, param_.serial);
		fc::Assert(err, "fc::initFromSerial");

		err = fc::startCustomImage(fly_, param_.size.width, param_.size.height, param_.pixel_format);
		fc::Assert(err, "fc::startCustomImage");

		setProp(param_.shutter, param_.gain, param_.fps);

		switch (param_.pixel_format)
		{
		default:
			param_.channels = -1;
			std::cout
				<< "[Camera Controller] Cannot set image channels." << std::endl
				<< "                    Support only FLYCAPTURE_MONO8, FLYCAPTURE_RGB8 and 3 YUV formats." << std::endl
				<< "                    Use that format or set channels by function CameraController::setImageChannels." << std::endl;
			break;

		case FLYCAPTURE_MONO8:
			param_.channels = 1;
			break;

		case FLYCAPTURE_RGB8:
		case FLYCAPTURE_411YUV8:
		case FLYCAPTURE_422YUV8:
		case FLYCAPTURE_444YUV8:
			param_.channels = 3;
			break;
		}
		return;
	}


public:
	/*!
	@note Change the name of parameter file path written below
	*/
	CameraController() {}

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
			//if (!file_param_.empty())
			//{
			//	cv::FileStorage fs(file_param_, cv::FileStorage::WRITE);
			//	CV_Assert(fs.isOpened());

			//	bool complete = saveCameraParameter(fs, param_);
			//	assert(complete || !"saveCameraParameter");
			//}
			std::cout << "[Camera Controller] Bye!" << std::endl;
		}
	}

	/*!
	@note Change the name of parameter file path written below
	*/
	void initFromFile(const std::string file = param::gl_file_cam_param)
	{
		cv::FileStorage fs(file, cv::FileStorage::READ);
		CV_Assert(fs.isOpened());
		file_param_ = file;

		bool complete = loadCameraParameter(fs, param_);
		assert(complete || !"Cannot load camera parameter from file.");

		init();
		stereo_ = false;

		std::cout << "[Camera Controller] Success initialize from file." << std::endl;
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
		fc::Assert(err, "fc::setCamProp(shutter)");
		param_.shutter = shutter;

		if (0 <= gain)
			err = fc::setCamAbsProp(fly_, FLYCAPTURE_GAIN, gain);
		else
			err = fc::setCamProp(fly_, FLYCAPTURE_GAIN, 0, 0, true);
		fc::Assert(err, "fc::setCamProp(gain)");
		param_.gain = gain;

		if (0 <= fps)
			err = fc::setCamAbsProp(fly_, FLYCAPTURE_FRAME_RATE, fps);
		else
			err = fc::setCamProp(fly_, FLYCAPTURE_FRAME_RATE, 0, 0, true);
		fc::Assert(err, "fc::setCamProp(frame rate)");
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
		fc::Assert(err, "fc::setCamProp OR fc::setCamAbsProp");

		return;
	}

	void setImageChannels(int cn)
	{
		param_.channels = cn;

		if (param_.channels != 1 && param_.channels != 3)
		{
			std::cout << "[Camera Controller] Support only channels = 1(gray scale) or 3(color), sorry." << std::endl;
			abort();
		}

		return;
	}

	void operator >> (cv::Mat& im)
	{
		assert(param_.channels == 1 || param_.channels == 3);
		fc::Image fc_im, fc_im_conv;
		fc::Error err = FlyCaptureError::FLYCAPTURE_FAILED;
		int type = (param_.channels == 1) ? CV_8U : (CV_8UC3);

		for (size_t i = 0; i < 10; i++)
		{
			err = fc::grabImage2(fly_, &fc_im);

			if (err == FLYCAPTURE_OK)
				break;

			std::cout << (stereo_ ? "[Stereo " : "[");
			std::cout << "Camera Controller] fail to capture(fc::grabImage2). Retry!" << std::endl;
			restart();
		}

		if (err != FLYCAPTURE_OK)
			fc::Assert(err, "fc::grabImage2");

		im = cv::Mat(fc_im.iRows, fc_im.iCols, type);
		fc_im_conv.pData = (uchar*)im.data;
		fc_im_conv.pixelFormat = param_.pixel_format;
		err = fc::cvtImage(fly_, &fc_im, &fc_im_conv);
		fc::Assert(err, "fc::cvtImage");

		if (im.empty())
		{
			std::cout << "[Camera Controller] Capture failure. Restart." << std::endl;
			restart();
			*this >> im;
		}
	}

	cv::Size calcViewSize(const int width)
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
cv::Size view_size = cam_cntl.calcViewSize(640);
cv::resize(image[L], image[L], view_size);
@endcode
*/
class StereoCameraController
{
	Stereo<CameraController> cam_;
	std::string file_param_;

public:
	Stereo<CameraParams> param_;

	/*!
	@note Change the name of parameter file path written below
	*/
	StereoCameraController() {}

	StereoCameraController(const CameraParams& param_l, const CameraParams& param_r) :
		param_({ { param_l, param_r } })
	{
		cam_[L] = *(new CameraController(param_l, true));
		cam_[R] = *(new CameraController(param_r, true));
		std::cout << "[Stereo Camera Controller] Success initialize from CameraParams." << std::endl;
	}

	~StereoCameraController()
	{
		//if (!file_param_.empty())
		//{
		//	cv::FileStorage fs(file_param_, cv::FileStorage::WRITE);
		//	CV_Assert(fs.isOpened());

		//	bool complete = saveStereoCameraParameter(fs, param_);
		//	assert(complete || !"saveStereoCameraParameter");
		//}
		std::cout << "[Stereo Camera Controller] Bye!" << std::endl;
	}

	/*!
	@note Change the name of parameter file path written below
	*/
	void initFromFile(const std::string file = param::gl_file_stereo_cam_param)
	{
		cv::FileStorage fs(file, cv::FileStorage::READ);
		CV_Assert(fs.isOpened() || !"Cannot open file in bs::StereoCameraController constructor.");

		bool complete = loadStereoCameraParameter(fs, param_);
		assert(complete || !"Cannot load stereo camera parameters from file.");

		cam_[L] = *(new CameraController(param_[L], true));
		cam_[R] = *(new CameraController(param_[R], true));
		std::cout << "[Stereo Camera Controller] Success initialize from file." << std::endl;
	}

	void restart(const Stereo<fc::CamSerial> serial, const cv::Size size, const fc::PixFmt fmt = FLYCAPTURE_MONO8)
	{
		cam_[L].restart(serial[L], size, fmt);
		cam_[R].restart(serial[R], size, fmt);

		std::cout << "[Stereo Camera Controller] Success restart." << std::endl;
		return;
	}

	void loadProp(cv::FileStorage fs = cv::FileStorage())
	{
		if (!fs.isOpened())
			fs.open(file_param_, cv::FileStorage::READ);
		CV_Assert(fs.isOpened());

		if (!loadStereoCameraParameter(fs, param_))
			return;

		Stereo<float> shutter, gain, fps;
		shutter = { { param_[L].shutter, param_[R].shutter } };
		gain = { { param_[L].gain, param_[R].gain } };
		fps = { { param_[L].fps, param_[R].fps } };
		setProp(shutter, gain, fps);
		return;
	}

	Stereo<CameraParams> getParam()
	{
		return param_;
	}

	CameraParams getParam(const CAM_SELECT select)
	{
		if (select == L)
			return param_[L];
		else if (select == R)
			return param_[R];

		std::cerr << "[Stereo Camera Controller] Error: argument must be 'L' or 'R'." << std::endl;
		return CameraParams();
	}

	void setProp(const Stereo<float> shutter, const Stereo<float> gain, const Stereo<float> fps)
	{
		cam_[L].setProp(shutter[L], gain[L], fps[L]);
		cam_[R].setProp(shutter[R], gain[R], fps[R]);
		return;
	}

	bool setProp(const fc::Property prop, const float value, CAM_SELECT select)
	{
		if (select != (L | R))
		{
			std::cerr << "[Stereo Camera Controller] Error: argument must be 'L' or 'R'." << std::endl;
			return false;
		}

		const bool is_shutter = (prop == FLYCAPTURE_SHUTTER);
		const bool is_gain = (prop == FLYCAPTURE_GAIN);
		const bool is_frame_rate = (prop == FLYCAPTURE_FRAME_RATE);
		float dummy;
		is_shutter ? param_[select].shutter :
			is_gain ? param_[select].gain :
			is_frame_rate ? param_[select].fps : dummy = value;

		cam_[select].setProp(prop, value);

	}

	void setProp(const fc::Property prop, const Stereo<float> value)
	{
		const bool is_shutter = (prop == FLYCAPTURE_SHUTTER);
		const bool is_gain = (prop == FLYCAPTURE_GAIN);
		const bool is_frame_rate = (prop == FLYCAPTURE_FRAME_RATE);
		float dummy;
		is_shutter ? param_[L].shutter : is_gain ? param_[L].gain : is_frame_rate ? param_[L].fps : dummy = value[L];
		is_shutter ? param_[R].shutter : is_gain ? param_[R].gain : is_frame_rate ? param_[R].fps : dummy = value[R];
		cam_[L].setProp(prop, value[L]);
		cam_[R].setProp(prop, value[R]);
		return;
	}

	//void setProp(const fc::Property prop, const Stereo<CameraParams>& param)
	//{
	//	switch (prop)
	//	{
	//	default:
	//		break;

	//	case FLYCAPTURE_SHUTTER:
	//		cam_[L].setProp(prop, param[L].shutter);
	//		param_[L].shutter

	//	}
	//	const bool is_shutter = (prop == FLYCAPTURE_SHUTTER);
	//	const bool is_gain = (prop == FLYCAPTURE_GAIN);
	//	const bool is_frame_rate = (prop == FLYCAPTURE_FRAME_RATE);
	//	float dummy;
	//	is_shutter ? param_[L].shutter : is_gain ? param_[L].gain : is_frame_rate ? param_[L].fps : dummy = value[L];
	//	is_shutter ? param_[R].shutter : is_gain ? param_[R].gain : is_frame_rate ? param_[R].fps : dummy = value[R];
	//	cam_[L].setProp(prop, value[L]);
	//	cam_[R].setProp(prop, value[R]);
	//	return;
	//}

	void capture(cv::Mat& im, CAM_SELECT select)
	{
		assert(select == (L | R) || !"in StereCameraController capture");

		//std::cout << "[Stereo Camera Controller] Capture. ";

		switch (select)
		{
		case L:
			//std::cout << "left..";
			cam_[L] >> im;
			break;

		case R:
			//std::cout << "right..";
			cam_[R] >> im;
			break;

		default:
			break;
		}
		//std::cout << "success." << std::endl;
	}

	void operator >> (Stereo<cv::Mat>& im)
	{
		//std::cout << "[Stereo Camera Controller] Capture. left..";
		cam_[L] >> im[L];

		//std::cout << "right..";
		cam_[R] >> im[R];
		//std::cout << "success." << std::endl;
	}

	/*!
	@note Support only left_size == right_size
	*/
	Stereo<cv::Size> calcViewSize(const int width)
	{
		CV_Assert(0 < param_[L].size.area() && 0 < param_[R].size.area());
		
		Stereo<cv::Size> output;
		int height = (param_[L].size.height * width) / param_[L].size.width;
		output[0] = cv::Size(width, height);
		height = (param_[R].size.height * width) / param_[R].size.width;
		output[1] = cv::Size(width, height);

		return output;
	}
};

}
