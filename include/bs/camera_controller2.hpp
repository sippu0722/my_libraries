#pragma once

#include <iostream>
#include <cassert>

#pragma warning(disable:4819)
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <FlyCapture2/FlyCapture2.h>
#pragma warning(default:4819)

//#include <bs/stereo_util.h>


namespace fc = FlyCapture2;

namespace bs
{

std::string fmt2str(const fc::PixelFormat fmt)
{
	std::string str = "";

	switch (fmt)
	{
	default:							// = 0 /**< Unspecified pixel format. */
		str = "UNSPECIFIED_PIXEL_FORMAT";
		break;

	case fc::PIXEL_FORMAT_MONO8:			// = 0x80000000, /**< 8 bits of mono information. */
		str = "PIXEL_FORMAT_MONO8";
		break;

	case fc::PIXEL_FORMAT_411YUV8:			// = 0x40000000, /**< YUV 4:1:1. */
		str = "PIXEL_FORMAT_411YUV8";
		break;

	case fc::PIXEL_FORMAT_422YUV8:			// = 0x20000000, /**< YUV 4:2:2. */
		str = "PIXEL_FORMAT_422YUV8";
		break;

	case fc::PIXEL_FORMAT_444YUV8:			// = 0x10000000, /**< YUV 4:4:4. */
		str = "PIXEL_FORMAT_444YUV8";
		break;

	case fc::PIXEL_FORMAT_RGB8:				// = 0x08000000, /**< R// = G// = B// = 8 bits. */
		str = "PIXEL_FORMAT_RGB8";
		break;

	case fc::PIXEL_FORMAT_MONO16:			// = 0x04000000, /**< 16 bits of mono information. */
		str = "PIXEL_FORMAT_MONO16";
		break;

	case fc::PIXEL_FORMAT_RGB16:			// = 0x02000000, /**< R// = G// = B// = 16 bits. */
		str = "PIXEL_FORMAT_RGB16";
		break;

	case fc::PIXEL_FORMAT_S_MONO16:			// = 0x01000000, /**< 16 bits of signed mono information. */
		str = "PIXEL_FORMAT_S_MONO16";
		break;

	case fc::PIXEL_FORMAT_S_RGB16:			// = 0x00800000, /**< R// = G// = B// = 16 bits signed. */
		str = "PIXEL_FORMAT_S_RGB16";
		break;

	case fc::PIXEL_FORMAT_RAW8:				// = 0x00400000, /**< 8 bit raw data output of sensor. */
		str = "PIXEL_FORMAT_RAW8";
		break;

	case fc::PIXEL_FORMAT_RAW16:			// = 0x00200000, /**< 16 bit raw data output of sensor. */
		str = "PIXEL_FORMAT_RAW16";
		break;

	case fc::PIXEL_FORMAT_MONO12:			// = 0x00100000, /**< 12 bits of mono information. */
		str = "PIXEL_FORMAT_MONO12";
		break;

	case fc::PIXEL_FORMAT_RAW12:			// = 0x00080000, /**< 12 bit raw data output of sensor. */
		str = "PIXEL_FORMAT_RAW12";
		break;

	case fc::PIXEL_FORMAT_BGR:				// = 0x80000008, /**< 24 bit BGR. */
		str = "PIXEL_FORMAT_BGR";
		break;

	case fc::PIXEL_FORMAT_BGRU:				// = 0x40000008, /**< 32 bit BGRU. */
		str = "PIXEL_FORMAT_BGRU";
		break;

	// PIXEL_FORMAT_RGB = PIXEL_FORMAT_RGB8, /**< 24 bit RGB. */

	case fc::PIXEL_FORMAT_RGBU:				// = 0x40000002, /**< 32 bit RGBU. */
		str = "PIXEL_FORMAT_RGBU";
		break;

	case fc::PIXEL_FORMAT_BGR16:			// = 0x02000001, /**< R// = G// = B// = 16 bits. */
		str = "PIXEL_FORMAT_BGR16";
		break;

	case fc::PIXEL_FORMAT_BGRU16:			// = 0x02000002, /**< 64 bit BGRU. */
		str = "PIXEL_FORMAT_BGRU16";
		break;

	case fc::PIXEL_FORMAT_422YUV8_JPEG:		// = 0x40000001, /**< JPEG compressed stream. */
		str = "PIXEL_FORMAT_422YUV8_JPEG";
		break;

	case fc::NUM_PIXEL_FORMATS:				// = 20, /**< Number of pixel formats. */
		str = "NUM_PIXEL_FORMATS";
		break;
	}
	return str;
}


fc::PixelFormat str2fmt(const std::string str)
{
	if		(str == "PIXEL_FORMAT_MONO8")			return fc::PIXEL_FORMAT_MONO8;
	else if (str == "PIXEL_FORMAT_411YUV8")			return fc::PIXEL_FORMAT_411YUV8;
	else if (str == "PIXEL_FORMAT_422YUV8")			return fc::PIXEL_FORMAT_422YUV8;
	else if (str == "PIXEL_FORMAT_444YUV8")			return fc::PIXEL_FORMAT_444YUV8;
	else if (str == "PIXEL_FORMAT_RGB8")			return fc::PIXEL_FORMAT_RGB8;
	else if (str == "PIXEL_FORMAT_MONO16")			return fc::PIXEL_FORMAT_MONO16;
	else if (str == "PIXEL_FORMAT_RGB16")			return fc::PIXEL_FORMAT_RGB16;
	else if (str == "PIXEL_FORMAT_S_MONO16")		return fc::PIXEL_FORMAT_S_MONO16;
	else if (str == "PIXEL_FORMAT_S_RGB16")			return fc::PIXEL_FORMAT_S_RGB16;
	else if (str == "PIXEL_FORMAT_RAW8")			return fc::PIXEL_FORMAT_RAW8;
	else if (str == "PIXEL_FORMAT_RAW16")			return fc::PIXEL_FORMAT_RAW16;
	else if (str == "PIXEL_FORMAT_MONO12")			return fc::PIXEL_FORMAT_MONO12;
	else if (str == "PIXEL_FORMAT_RAW12")			return fc::PIXEL_FORMAT_RAW12;
	else if (str == "PIXEL_FORMAT_BGR")				return fc::PIXEL_FORMAT_BGR;
	else if (str == "PIXEL_FORMAT_BGRU")			return fc::PIXEL_FORMAT_BGRU;
	else if (str == "PIXEL_FORMAT_RGBU")			return fc::PIXEL_FORMAT_RGBU;
	else if (str == "PIXEL_FORMAT_BGR16")			return fc::PIXEL_FORMAT_BGR16;
	else if (str == "PIXEL_FORMAT_BGRU16")			return fc::PIXEL_FORMAT_BGRU16;
	else if (str == "PIXEL_FORMAT_422YUV8_JPEG")	return fc::PIXEL_FORMAT_422YUV8_JPEG;
	else if (str == "NUM_PIXEL_FORMATS")			return fc::NUM_PIXEL_FORMATS;
	else											return fc::UNSPECIFIED_PIXEL_FORMAT;
}

}


namespace bs
{

// for old version
struct CameraParams
{
	unsigned int serial;
	fc::PixelFormat pixel_format;
	cv::Size size;
	cv::Point tl;
	float shutter, gain, fps;
	int channels;

	CameraParams() :
		tl(cv::Point(0, 0)), shutter(-1.f), gain(-1.f), fps(-1.f), channels(-1)
	{}

	CameraParams(
		const unsigned int _serial, const fc::PixelFormat _fmt, const cv::Point _tl, const cv::Size _sz,
		const float _shutter, const float _gain, const float _fps) :
		serial(_serial), pixel_format(_fmt), tl(_tl), size(_sz), shutter(_shutter), gain(_gain), fps(_fps), channels(-1)
	{}
};

}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

namespace bs
{

void loadCameraParameterBase(
	const cv::FileNode& fn,
	fc::CameraInfo& cam_info,
	fc::Format7ImageSettings& fmt7_imset,
	float& shutter, float& gain, float& fps)
{
	int tmp_int;
	std::string str_fmt;

	fn["serial"] >> tmp_int;
	cam_info.serialNumber = tmp_int;

	fn["mode"] >> tmp_int;
	fmt7_imset.mode = static_cast<fc::Mode>(tmp_int);

	fn["format"] >> str_fmt;
	fmt7_imset.pixelFormat = bs::str2fmt(str_fmt);

	fn["offsetX"] >> tmp_int;
	fmt7_imset.offsetX = tmp_int;

	fn["offsetY"] >> tmp_int;
	fmt7_imset.offsetY = tmp_int;

	fn["width"]	>> tmp_int;
	fmt7_imset.width = tmp_int;

	fn["height"] >> tmp_int;
	fmt7_imset.height = tmp_int;

	fn["shutter"]	>> shutter;
	fn["gain"]		>> gain;
	fn["fps"]		>> fps;
	return;
}


void saveCameraParameterBase(
	cv::FileStorage& fs,
	const fc::CameraInfo& cam_info,
	const fc::Format7ImageSettings& fmt7_imset,
	const float shutter, const float gain, const float fps)
{
	fs << "serial"	<< static_cast<int>(cam_info.serialNumber);
	fs << "mode"	<< static_cast<int>(fmt7_imset.mode);
	fs << "format"	<< bs::fmt2str(fmt7_imset.pixelFormat);
	fs << "offsetX" << static_cast<int>(fmt7_imset.offsetX);
	fs << "offsetY" << static_cast<int>(fmt7_imset.offsetY);
	fs << "width"	<< static_cast<int>(fmt7_imset.width);
	fs << "height"	<< static_cast<int>(fmt7_imset.height);
	fs << "shutter" << shutter;
	fs << "gain"	<< gain;
	fs << "fps"		<< fps;

	return;
}

}	// bs

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

namespace bs
{

void updateParameterFile(const std::string file)
{
	cv::FileStorage fs(file, cv::FileStorage::READ);



}

//bool loadCameraParameter(const cv::FileStorage& fs, CameraParams& param)
//{
//	cv::FileNode fn(fs.fs, nullptr);
//	fc::
//
//
//	loadCameraParameterBase(fn, param);
//
//	return true;
//}


bool loadCameraParameter(
	const cv::FileStorage& fs,
	fc::CameraInfo& cam_info,
	fc::Format7ImageSettings& fmt7_imset,
	float& shutter, float& gain, float& fps)
{
	cv::FileNode fn(fs.fs, nullptr);
	loadCameraParameterBase(fn, cam_info, fmt7_imset, shutter, gain, fps);

	return true;
}


//bool loadCameraParameter(const cv::FileNode& fn, CameraParams& param)
//{
//	loadCameraParameterBase(fn, param);
//
//	return true;
//}


//bool loadStereoCameraParameter(const cv::FileStorage& fs, Stereo<CameraParams>& param)
//{
//	loadCameraParameterBase(fs["left"], param[L]);
//	loadCameraParameterBase(fs["right"], param[R]);
//	return true;
//}
//
////bool loadStereoCameraParameter(const cv::FileStorage& fs, CameraParams& param_l, CameraParams& param_r)
////{
////	return loadStereoCameraParameter(fs, { { param_l, param_r } });
////}

//bool saveCameraParameter(cv::FileStorage& fs, const CameraParams& param)
//{
//	saveCameraParameterBase(fs, param);
//	return true;
//}


bool saveCameraParameter(
	cv::FileStorage& fs,
	const fc::CameraInfo& cam_info,
	const fc::Format7ImageSettings& fmt7_imset,
	const float shutter, const float gain, const float fps)
{
	saveCameraParameterBase(fs, cam_info, fmt7_imset, shutter, gain, fps);
	return true;
}

bool makeCameraParameterFile(
	const std::string file_name,
	const unsigned int serial,
	const fc::Mode mode,
	const fc::PixelFormat pixel_format,
	const unsigned int offset_x,
	const unsigned int offset_y,
	const unsigned int width,
	const unsigned int height,
	const float shutter,
	const float gain,
	const float frame_rate)
{
	cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
	CV_Assert(fs.isOpened());

	fc::CameraInfo ci;
	fc::Format7ImageSettings is;

	ci.serialNumber = serial;
	is.mode = mode;
	is.pixelFormat = pixel_format;
	is.offsetX = offset_x;
	is.offsetY = offset_y;
	is.width = width;
	is.height = height;

	saveCameraParameterBase(fs, ci, is, shutter, gain, frame_rate);
	return true;
}


//bool saveStereoCameraParameter(cv::FileStorage& fs, const CameraParams& param_l, const CameraParams& param_r)
//{
//	fs << "left" << "{";
//	saveCameraParameterBase(fs, param_l);
//	fs << "}";
//
//	fs << "right" << "{";
//	saveCameraParameterBase(fs, param_r);
//	fs << "}";
//
//	return true;
//}
//
//bool saveStereoCameraParameter(cv::FileStorage& fs, const Stereo<CameraParams>& param)
//{
//	saveStereoCameraParameter(fs, param[L], param[R]);
//
//	return true;
//}


class Camera
{
private:
	bool connected_;
	fc::Image im_raw_, im_conv_;

protected:
	fc::Camera cam;

	fc::CameraInfo cam_info;
	fc::Format7ImageSettings fmt7_imset;
	float shutter, gain, fps;

	bool connect();
	bool checkError(fc::Error error);

public:
	Camera() : connected_(false){};
	Camera(const Camera& o) : connected_(false){};
	~Camera()
	{
		if (connected_)
		{
			cam.StopCapture();
			cam.Disconnect();
		}
	}

	void capture(fc::Image& im, const fc::PixelFormat output_format = fc::PIXEL_FORMAT_MONO8)
	{
		fc::Image raw_im;
		cam.RetrieveBuffer(&raw_im);
		raw_im.Convert(output_format, &im);

		return;
	}
};

inline bool Camera::checkError(fc::Error error)
{
	if (error != fc::PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return true;
	}
	return false;
}

inline bool Camera::connect(void)
{
	fc::BusManager mgr;
	fc::PGRGuid guid;

#ifdef _DEBUG
	if (checkError(mgr.GetCameraFromSerialNumber(cam_info.serialNumber, &guid)))
		return false;
	
	if (checkError(cam.Connect(&guid)))
		return false;

	// for high speed
	fc::FC2Config config;
	
	if (checkError(cam.GetConfiguration(&config)))
		return false;
	
	config.highPerformanceRetrieveBuffer = true;
	
	if (checkError(cam.SetConfiguration(&config)))
		return false;

	fc::Format7PacketInfo fmt7packet;
	bool valid = false;

	// DANGER ??
	if (checkError(cam.ValidateFormat7Settings(&fmt7_imset, &valid, &fmt7packet)))
		return false;

	if (!valid)
	{
		std::cerr << std::endl << "Format7 settings are not valid" << std::endl;
		return false;
	}

	if (checkError(cam.SetFormat7Configuration(&fmt7_imset, 100.f)))
		return false;

	if (checkError(cam.StartCapture()))
		return false;
#else
	mgr.GetCameraFromSerialNumber(cam_info.serialNumber, &guid);
	cam.Connect(&guid);

	// for high speed
	fc::FC2Config config;

	cam.GetConfiguration(&config);
	config.highPerformanceRetrieveBuffer = true;
	cam.SetConfiguration(&config);
	cam.SetFormat7Configuration(&fmt7_imset, 100.f);

	if (checkError(cam.StartCapture()))
		return false;
#endif

	connected_ = true;
	return true;
}





/*!
@brief Flycapture2のカメラを使いやすくするクラス

@code
@endcode
*/
class CameraController : Camera
{
	bool stereo_;
	cv::Size view_sz_;

	void message (const std::string str);
	void error	 (const std::string str);

public:
	CameraController () {}
	CameraController (const std::string file_param) : stereo_(false) { initFromFile(file_param); }
	~CameraController() { if (!stereo_) message("Bye!"); }

	void		 initFromFile		 (const std::string file);
	fc::Property getProperty		 (const fc::PropertyType type);
	bool		 setProperty		 (const fc::Property& prop);
	bool		 setProperty		 (const fc::PropertyType type, const float abs_value, const int value_b = 0);
	bool		 setProperty		 (const fc::PropertyType type, const bool is_auto);
	void		 setPropertyFromFile (const cv::FileStorage& fs);
	void		 operator>>			 (cv::OutputArray image);
	cv::Size	 calcViewSize		 (const unsigned int width);
	void		 show				 (cv::OutputArray latest_image = cv::Mat());

	// 前verとの互換性 ---
	CameraParams getParams()
	{
		error("Sorry. The function 'getParams()' is not supported");
		exit(-1);
		return CameraParams();
	}

	void setProp(const float shutter, const float gain, const float fps)
	{
		setProperty(fc::SHUTTER, shutter);
		setProperty(fc::GAIN, gain);
		setProperty(fc::FRAME_RATE, fps);
		return;
	}

	void setProp(const fc::PropertyType type, const float value)
	{
		setProperty(type, value);
		return;
	}

	void setImageChannels(const int cn)
	{
		error("Sorry. The function 'setImageChannels()' is not supported");
		exit(-1);
	}
	// --- 前verとの互換性
};


inline void CameraController::message(const std::string str)
{
	std::cout << "[Camera Controller] " << str << std::endl;
	return;
}


inline void CameraController::error(const std::string str)
{
	std::cerr << std::endl << "[Camera Controller] Error: " << str << std::endl;
	exit(-1);
}


inline void CameraController::initFromFile(const std::string file)
{
	cv::FileStorage fs(file, cv::FileStorage::READ);
	CV_Assert(fs.isOpened());

	bool complete = loadCameraParameter(fs, cam_info, fmt7_imset, shutter, gain, fps);

	if (!complete)
		error("Cannot load camera parameter from file.");

	if (!connect())
		error("Cannot connect to the camera.");

	stereo_ = false;
	message("Success initialize from file.");
	return;
}


inline fc::Property CameraController::getProperty(const fc::PropertyType type)
{
	fc::Property prop;

	prop.type = type;
	checkError(cam.GetProperty(&prop));
	return prop;
}


inline bool CameraController::setProperty(const fc::Property& prop)
{
	return checkError(cam.SetProperty(&prop));
}


inline bool CameraController::setProperty(const fc::PropertyType type, const float abs_value, const int value_b)
{
	auto prop = getProperty(type);

	switch (type)
	{
	default:
		prop.absControl = true;
		prop.absValue = abs_value;
		break;

	case fc::WHITE_BALANCE:
		prop.valueA = static_cast<unsigned int>(abs_value);
		prop.valueB = value_b;
		break;
	}
	return setProperty(prop);
}


inline bool CameraController::setProperty(const fc::PropertyType type, const bool is_auto)
{
	auto prop = getProperty(type);
	fc::PropertyInfo info;

	info.type = type;
	cam.GetPropertyInfo(&info);

	if (!info.autoSupported)
	{
		error("Auto is not supported");
		return false;
	}
	prop.autoManualMode = is_auto;
	return setProperty(prop);
}

inline void CameraController::setPropertyFromFile(const cv::FileStorage& fs)
{
	// Only shutter, gain, frame_rate
	float shutter, gain, fps;

	bs::loadCameraParameter(fs, cam_info, fmt7_imset, shutter, gain, fps);
	setProp(shutter, gain, fps);
	return;
}



inline void CameraController::operator>>(cv::OutputArray image)
{
	fc::Image fc_im;
	cv::Mat im;
	int type;

	switch (fmt7_imset.pixelFormat)
	{
	default:
		type = CV_8U;
		break;

	case fc::PIXEL_FORMAT_RAW8:
	case fc::PIXEL_FORMAT_RGB8:
		type = CV_8UC3;
		break;
	}

	capture(fc_im, fmt7_imset.pixelFormat);
	im.create(fc_im.GetRows(), fc_im.GetCols(), type);
	std::memcpy(im.data, fc_im.GetData(), im.total() * im.channels());
	im.copyTo(image);
}


inline cv::Size CameraController::calcViewSize(const unsigned int width)
{
	unsigned int height = (fmt7_imset.height * width) / fmt7_imset.width;
	view_sz_ = cv::Size((int)width, (int)height);

	return view_sz_;
}


inline void CameraController::show(cv::OutputArray latest_image)
{
	cv::Mat im, view;
	int key = -1;

	if (view_sz_.area() == 0)
		view_sz_ = calcViewSize(640);

	while (key == -1)
	{
		*this >> im;
		cv::resize(im, view, view_sz_);
		cv::imshow("camera", view);
		key = cv::waitKey(2);
	}

	if(latest_image.empty())
		return;
	im.copyTo(latest_image);
}



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
//class StereoCameraController
//{
//	Stereo<CameraController> cam_;
//	std::string file_param;
//	bool need_stop_;
//
//public:
//	Stereo<CameraParams> param;
//
//	/*!
//	@note Change the name of parameter file path written below
//	*/
//	StereoCameraController() : need_stop_(false) {}
//
//	StereoCameraController(const CameraParams& paraml, const CameraParams& paramr) :
//		param({ { paraml, paramr } }), need_stop_(true)
//	{
//		cam_[L] = *(new CameraController(paraml, true));
//		cam_[R] = *(new CameraController(paramr, true));
//		std::cout << "[Stereo Camera Controller] Success initialize from CameraParams." << std::endl;
//	}
//
//	~StereoCameraController()
//	{
//		//if (!file_param.empty())
//		//{
//		//	cv::FileStorage fs(file_param, cv::FileStorage::WRITE);
//		//	CV_Assert(fs.isOpened());
//
//		//	bool complete = saveStereoCameraParameter(fs, param);
//		//	assert(complete || !"saveStereoCameraParameter");
//		//}
//		if (need_stop_)
//			std::cout << "[Stereo Camera Controller] Bye!" << std::endl;
//	}
//
//	/*!
//	@note Change the name of parameter file path written below
//	*/
//	void initFromFile(const std::string file = "scam_param.xml")
//	{
//		cv::FileStorage fs(file, cv::FileStorage::READ);
//		CV_Assert(fs.isOpened() || !"Cannot open file in bs::StereoCameraController constructor.");
//
//		bool complete = loadStereoCameraParameter(fs, param);
//		assert(complete || !"Cannot load stereo camera parameters from file.");
//
//		cam_[L] = *(new CameraController(param[L], true));
//		cam_[R] = *(new CameraController(param[R], true));
//		std::cout << "[Stereo Camera Controller] Success initialize from file." << std::endl;
//	}
//
//	void restart(const Stereo<fc::CamSerial> serial, const cv::Size size, const fc::PixFmt fmt = FLYCAPTURE_MONO8)
//	{
//		cam_[L].restart(serial[L], size, fmt);
//		cam_[R].restart(serial[R], size, fmt);
//
//		std::cout << "[Stereo Camera Controller] Success restart." << std::endl;
//		return;
//	}
//
//	void loadProp(cv::FileStorage fs = cv::FileStorage())
//	{
//		if (!fs.isOpened())
//			fs.open(file_param, cv::FileStorage::READ);
//		CV_Assert(fs.isOpened());
//
//		if (!loadStereoCameraParameter(fs, param))
//			return;
//
//		Stereo<float> shutter, gain, fps;
//		shutter = { { param[L].shutter, param[R].shutter } };
//		gain = { { param[L].gain, param[R].gain } };
//		fps = { { param[L].fps, param[R].fps } };
//		setProp(shutter, gain, fps);
//		return;
//	}
//
//	Stereo<CameraParams> getParam()
//	{
//		return param;
//	}
//
//	CameraParams getParam(const CAM_SELECT select)
//	{
//		if (select == L)
//			return param[L];
//		else if (select == R)
//			return param[R];
//
//		std::cerr << "[Stereo Camera Controller] Error: argument must be 'L' or 'R'." << std::endl;
//		return CameraParams();
//	}
//
//	void setProp(const Stereo<float> shutter, const Stereo<float> gain, const Stereo<float> fps)
//	{
//		cam_[L].setProp(shutter[L], gain[L], fps[L]);
//		cam_[R].setProp(shutter[R], gain[R], fps[R]);
//		param[L] = cam_[L].getParams();
//		param[R] = cam_[R].getParams();
//		return;
//	}
//
//	bool setProp(const fc::Property prop, const float value, CAM_SELECT select)
//	{
//		if (select != (L | R))
//		{
//			std::cerr << "[Stereo Camera Controller] Error: argument must be 'L' or 'R'." << std::endl;
//			return false;
//		}
//
//		const bool is_shutter = (prop == FLYCAPTURE_SHUTTER);
//		const bool is_gain = (prop == FLYCAPTURE_GAIN);
//		const bool is_frame_rate = (prop == FLYCAPTURE_FRAME_RATE);
//		float dummy;
//		is_shutter ? param[select].shutter :
//			is_gain ? param[select].gain :
//			is_frame_rate ? param[select].fps : dummy = value;
//
//		cam_[select].setProp(prop, value);
//		param[L] = cam_[L].getParams();
//		param[R] = cam_[R].getParams();
//
//		return true;
//	}
//
//	void setProp(const fc::Property prop, const Stereo<float> value)
//	{
//		const bool is_shutter = (prop == FLYCAPTURE_SHUTTER);
//		const bool is_gain = (prop == FLYCAPTURE_GAIN);
//		const bool is_frame_rate = (prop == FLYCAPTURE_FRAME_RATE);
//		float dummy;
//		is_shutter ? param[L].shutter : is_gain ? param[L].gain : is_frame_rate ? param[L].fps : dummy = value[L];
//		is_shutter ? param[R].shutter : is_gain ? param[R].gain : is_frame_rate ? param[R].fps : dummy = value[R];
//		cam_[L].setProp(prop, value[L]);
//		cam_[R].setProp(prop, value[R]);
//		param[L] = cam_[L].getParams();
//		param[R] = cam_[R].getParams();
//
//		return;
//	}
//
//	//void setProp(const fc::Property prop, const Stereo<CameraParams>& param)
//	//{
//	//	switch (prop)
//	//	{
//	//	default:
//	//		break;
//
//	//	case FLYCAPTURE_SHUTTER:
//	//		cam_[L].setProp(prop, param[L].shutter);
//	//		param[L].shutter
//
//	//	}
//	//	const bool is_shutter = (prop == FLYCAPTURE_SHUTTER);
//	//	const bool is_gain = (prop == FLYCAPTURE_GAIN);
//	//	const bool is_frame_rate = (prop == FLYCAPTURE_FRAME_RATE);
//	//	float dummy;
//	//	is_shutter ? param[L].shutter : is_gain ? param[L].gain : is_frame_rate ? param[L].fps : dummy = value[L];
//	//	is_shutter ? param[R].shutter : is_gain ? param[R].gain : is_frame_rate ? param[R].fps : dummy = value[R];
//	//	cam_[L].setProp(prop, value[L]);
//	//	cam_[R].setProp(prop, value[R]);
//	//	return;
//	//}
//
//	void capture(cv::Mat& im, CAM_SELECT select)
//	{
//		assert(select == (L | R) || !"in StereCameraController capture");
//
//		//std::cout << "[Stereo Camera Controller] Capture. ";
//
//		switch (select)
//		{
//		case L:
//			//std::cout << "left..";
//			cam_[L] >> im;
//			break;
//
//		case R:
//			//std::cout << "right..";
//			cam_[R] >> im;
//			break;
//
//		default:
//			break;
//		}
//		//std::cout << "success." << std::endl;
//	}
//
//	void operator >> (Stereo<cv::Mat>& im)
//	{
//		//std::cout << "[Stereo Camera Controller] Capture. left..";
//		cam_[L] >> im[L];
//
//		//std::cout << "right..";
//		cam_[R] >> im[R];
//		//std::cout << "success." << std::endl;
//	}
//
//	/*!
//	@note Support only left_size == right_size
//	*/
//	Stereo<cv::Size> calcViewSize(const int width)
//	{
//		CV_Assert(0 < param[L].size.area() && 0 < param[R].size.area());
//		
//		Stereo<cv::Size> output;
//		int height = (param[L].size.height * width) / param[L].size.width;
//		output[0] = cv::Size(width, height);
//		height = (param[R].size.height * width) / param[R].size.width;
//		output[1] = cv::Size(width, height);
//
//		return output;
//	}
//};

}
