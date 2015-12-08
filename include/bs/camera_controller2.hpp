#pragma once

#include <iostream>
#include <cassert>

#pragma warning(disable:4819)
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <FlyCapture2/FlyCapture2.h>
#pragma warning(default:4819)

#include <bs/stereo_util.h>


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


fc::PixelFormat str2fmt(const std::string& str)
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

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

namespace bs
{

void loadCameraParameterBase(
	const cv::FileNode& fn,
	fc::CameraInfo& cam_info,
	fc::Format7ImageSettings& fmt7_imset,
	float& shutter, float& gain, float& frame_rate)
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
	fn["fps"]		>> frame_rate;
	return;
}


void saveCameraParameterBase(
	cv::FileStorage& fs,
	const fc::CameraInfo& cam_info,
	const fc::Format7ImageSettings& fmt7_imset,
	const float shutter, const float gain, const float frame_rate)
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
	fs << "fps"		<< frame_rate;

	return;
}

}	// bs

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

namespace bs
{

bool loadCameraParameter(
	const cv::FileStorage& fs,
	fc::CameraInfo& cam_info,
	fc::Format7ImageSettings& fmt7_imset,
	float& shutter, float& gain, float& frame_rate)
{
	cv::FileNode fn(fs.fs, nullptr);
	loadCameraParameterBase(fn, cam_info, fmt7_imset, shutter, gain, frame_rate);

	return true;
}


bool saveCameraParameter(
	cv::FileStorage& fs,
	const fc::CameraInfo& cam_info,
	const fc::Format7ImageSettings& fmt7_imset,
	const float shutter, const float gain, const float frame_rate)
{
	saveCameraParameterBase(fs, cam_info, fmt7_imset, shutter, gain, frame_rate);
	return true;
}


bool loadStereoCameraParameter(
	const cv::FileStorage& fs,
	Stereo<fc::CameraInfo>& cam_info,
	Stereo<fc::Format7ImageSettings>& fmt7_imset,
	Stereo<float>& shutter, Stereo<float>& gain, Stereo<float>& frame_rate)
{
	loadCameraParameterBase(fs["left"] , cam_info[L], fmt7_imset[L], shutter[L], gain[L], frame_rate[L]);
	loadCameraParameterBase(fs["right"], cam_info[R], fmt7_imset[R], shutter[R], gain[R], frame_rate[R]);
	return true;
}


bool saveStereoCameraParameter(
	cv::FileStorage& fs,
	const Stereo<fc::CameraInfo>& cam_info,
	const Stereo<fc::Format7ImageSettings>& fmt7_imset,
	const Stereo<float>& shutter, const Stereo<float>& gain, const Stereo<float>& frame_rate)
{
	fs << "left" << "{";
	saveCameraParameterBase(fs, cam_info[L], fmt7_imset[L], shutter[L], gain[L], frame_rate[L]);
	fs << "}";

	fs << "right" << "{";
	saveCameraParameterBase(fs, cam_info[R], fmt7_imset[R], shutter[R], gain[R], frame_rate[R]);
	fs << "}";

	return true;
}


bool makeCameraParameterFile(
	const std::string& file_name,
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

	saveCameraParameter(fs, ci, is, shutter, gain, frame_rate);
	return true;
}


bool makeStereoCameraParameterFile(
	const std::string& file_name,
	const Stereo<unsigned int> serial,
	const Stereo<fc::Mode> mode,
	const Stereo<fc::PixelFormat> pixel_format,
	const Stereo<unsigned int> offset_x,
	const Stereo<unsigned int> offset_y,
	const Stereo<unsigned int> width,
	const Stereo<unsigned int> height,
	const Stereo<float> shutter,
	const Stereo<float> gain,
	const Stereo<float> frame_rate)
{
	cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
	CV_Assert(fs.isOpened());

	Stereo<fc::CameraInfo> ci;
	Stereo<fc::Format7ImageSettings> is;

	ci[L].serialNumber = serial[L];
	is[L].mode = mode[L];
	is[L].pixelFormat = pixel_format[L];
	is[L].offsetX = offset_x[L];
	is[L].offsetY = offset_y[L];
	is[L].width = width[L];
	is[L].height = height[L];

	ci[R].serialNumber = serial[R];
	is[R].mode = mode[R];
	is[R].pixelFormat = pixel_format[R];
	is[R].offsetX = offset_x[R];
	is[R].offsetY = offset_y[R];
	is[R].width = width[R];
	is[R].height = height[R];

	saveStereoCameraParameter(fs, ci, is, shutter, gain, frame_rate);
	return true;
}


void updateCameraParameterFile(const std::string& file, const fc::Mode mode)
{
	cv::FileStorage fs(file, cv::FileStorage::READ);

	int serial, tmp;
	float gain, shutter, fps;
	std::string str;
	cv::Point tl;
	cv::Size sz;

	// load
	fs["serial"] >> serial;
	fs["format"] >> tmp;
	str = fmt2str(static_cast<fc::PixelFormat>(tmp));

	fs["tl"] >> tl;
	fs["size"] >> sz;
	fs["shutter"] >> shutter;
	fs["gain"] >> gain;
	fs["fps"] >> fps;
	fs.release();

	// update
	makeCameraParameterFile(
		file, serial, mode, static_cast<fc::PixelFormat>(tmp),
		tl.x, tl.y, sz.width, sz.height, shutter, gain, fps);
	return;
}


void updateStereoCameraParameterFile(const std::string& file, const Stereo<fc::Mode>& mode)
{
	cv::FileStorage fs(file, cv::FileStorage::READ);
	cv::FileNode fn;

	Stereo<int> serial, tmp;
	Stereo<float> gain, shutter, fps;
	Stereo<std::string> str;
	Stereo<cv::Point> tl;
	Stereo<cv::Size> sz;

	// load
	fn = fs["left"];
	fn["serial"] >> serial[L];
	fn["format"] >> tmp[L];
	str[L] = fmt2str(static_cast<fc::PixelFormat>(tmp[L]));

	fn["tl"] >> tl[L];
	fn["size"] >> sz[L];
	fn["shutter"] >> shutter[L];
	fn["gain"] >> gain[L];
	fn["fps"] >> fps[L];

	fn = fs["right"];
	fn["serial"] >> serial[R];
	fn["format"] >> tmp[R];
	str[R] = fmt2str(static_cast<fc::PixelFormat>(tmp[R]));

	fn["tl"] >> tl[R];
	fn["size"] >> sz[R];
	fn["shutter"] >> shutter[R];
	fn["gain"] >> gain[R];
	fn["fps"] >> fps[R];

	fs.release();

	// update
	makeStereoCameraParameterFile(
		file, { { serial[L], serial[R] } }, mode,
		{ { static_cast<fc::PixelFormat>(tmp[L]), static_cast<fc::PixelFormat>(tmp[R]) } },
		{ { tl[L].x, tl[R].x } },
		{ { tl[L].y, tl[R].y } },
		{ { sz[L].width, sz[R].width } },
		{ { sz[L].height, sz[R].height } },
		shutter, gain, fps);
	return;
}


class Camera
{
private:
	bool connected_;

protected:
	fc::Camera cam;

	fc::CameraInfo cam_info;
	fc::Format7ImageSettings fmt7_imset;
	float shutter, gain, fps;

	bool connect();
	bool checkError(fc::Error error);

public:
	Camera		 () : connected_(false){};
	Camera		 (const Camera& o) : connected_(false){};
	~Camera		 ();
	void capture (fc::Image& im, const fc::PixelFormat output_format = fc::PIXEL_FORMAT_MONO8);
};


inline Camera::~Camera()
{
	if (connected_)
	{
		cam.StopCapture();
		cam.Disconnect();
	}
}


inline void Camera::capture(fc::Image& im, const fc::PixelFormat output_format)
{
	fc::Image raw_im;
	cam.RetrieveBuffer(&raw_im);
	raw_im.Convert(output_format, &im);

	return;
}


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
*/
class CameraController : Camera
{
	bool stereo_;
	cv::Size view_sz_;

	void message (const std::string& str);
	void error	 (const std::string& str);

public:
	CameraController() {}
	CameraController (const std::string& file_param) : stereo_(false) { initFromFile(file_param); }
	~CameraController() { if (!stereo_) message("Bye!"); }

	void		 init				 (const fc::CameraInfo& cam_info, const fc::Format7ImageSettings& fmt7imset, const float shutter, const float gain, const float frame_rate, const bool stereo);
	void		 initFromFile		 (const std::string& file);
	fc::Property getProperty		 (const fc::PropertyType type);
	bool		 setProperty		 (const fc::Property& prop);
	bool		 setProperty		 (const fc::PropertyType type, const float abs_value, const int value_b = 0);
	bool		 setProperty		 (const fc::PropertyType type, const bool is_auto);
	bool		 setPropertyFromFile (const std::string& file_param);
	void		 operator>>			 (cv::OutputArray image);
	cv::Size	 calcViewSize		 (const unsigned int width);
	void		 show				 (const std::string win_name = "camera", cv::OutputArray latest_image = cv::Mat());
};


inline void CameraController::message(const std::string& str)
{
	std::cout << "[Camera Controller] " << str << std::endl;
	return;
}


inline void CameraController::error(const std::string& str)
{
	std::cerr << std::endl << "[Camera Controller] Error: " << str << std::endl;
	exit(-1);
}


inline void CameraController::init(
	const fc::CameraInfo& _cam_info,
	const fc::Format7ImageSettings& _fmt7_imset,
	const float _shutter, const float _gain, const float _fps,
	const bool stereo)
{
	cam_info   = _cam_info;
	fmt7_imset = _fmt7_imset;
	shutter    = _shutter;
	gain	   = _gain;
	fps		   = _fps;
	stereo_    = stereo;

	if (!connect())
		error("Cannot connect to the camera.");

	setProperty(fc::SHUTTER, shutter);
	setProperty(fc::GAIN, gain);
	setProperty(fc::FRAME_RATE, fps);

	if (!stereo_)
		message("Success initialize.");
	return;

}


inline void CameraController::initFromFile(const std::string& file)
{
	cv::FileStorage fs(file, cv::FileStorage::READ);
	CV_Assert(fs.isOpened());

	bool complete = loadCameraParameter(fs, cam_info, fmt7_imset, shutter, gain, fps);

	if (!complete)
		error("Cannot load camera parameter from file.");

	if (!connect())
		error("Cannot connect to the camera.");

	setProperty(fc::SHUTTER, shutter);
	setProperty(fc::GAIN, gain);
	setProperty(fc::FRAME_RATE, fps);

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


inline bool CameraController::setPropertyFromFile(const std::string& file_param)
{
	// Only shutter, gain, frame_rate
	fc::CameraInfo dummy_cam_info;
	fc::Format7ImageSettings dummy_fmt7_imset;
	float shutter, gain, fps;

	cv::FileStorage fs(file_param, cv::FileStorage::READ);
	CV_Assert(fs.isOpened());

	if (!bs::loadCameraParameter(fs, dummy_cam_info, dummy_fmt7_imset, shutter, gain, fps))
	{
		message("Cannot load parameter file");
		return false;
	}
	setProperty(fc::SHUTTER, shutter);
	setProperty(fc::GAIN, gain);
	setProperty(fc::FRAME_RATE, fps);

	return true;
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


inline void CameraController::show(const std::string win_name, cv::OutputArray latest_image)
{
	cv::Mat im, view;
	int key = -1;

	if (view_sz_.area() == 0)
		view_sz_ = calcViewSize(640);

	while (key == -1)
	{
		*this >> im;
		cv::resize(im, view, view_sz_);
		cv::imshow(win_name, view);
		key = cv::waitKey(2);
	}

	if(latest_image.empty())
		return;
	im.copyTo(latest_image);
}



/*!
@brief Flycapture2のカメラを2台一度に使いやすくするクラス
*/
class StereoCameraController
{
	Stereo<CameraController> cam;
	Stereo<cv::Size> view_sz_;

	void message(const std::string& str);
	void error(const std::string& str);

public:
	StereoCameraController(){};
	StereoCameraController(const std::string& file_param) { initFromFile(file_param); }
	~StereoCameraController(){};

	void				 init				 (const Stereo<fc::CameraInfo>& cam_info, const Stereo<fc::Format7ImageSettings>& fmt7_imset, const Stereo<float>& shutter, const Stereo<float>& gain, const Stereo<float>& fps);
	void				 initFromFile		 (const std::string& file);
	Stereo<fc::Property> getProperty		 (const fc::PropertyType type);
	bool				 setProperty		 (const Stereo<fc::Property>& prop);
	bool				 setProperty		 (const fc::PropertyType type, const Stereo<float>& abs_value, const Stereo<int>& value_b = { { 0, 0 } });
	bool				 setProperty		 (const fc::PropertyType type, const bool is_auto);
	bool				 setPropertyFromFile (const std::string& file_param);
	void				 operator>>			 (Stereo<cv::Mat>& image);
	Stereo<cv::Size>	 calcViewSize		 (const unsigned int width);
	void				 show				 (const Stereo<std::string>& win_name = { { "left", "right" } });
	void				 show				 (Stereo<cv::Mat>& latest_image, const Stereo<std::string>& win_name = { { "left", "right" } });
};


inline void StereoCameraController::message(const std::string& str)
{
	std::cout << "[Stereo Camera Controller] " << str << std::endl;
	return;
}


inline void StereoCameraController::error(const std::string& str)
{
	std::cerr << std::endl << "[Stereo Camera Controller] Error: " << str << std::endl;
	exit(-1);
}


inline void StereoCameraController::init(
	const Stereo<fc::CameraInfo>& cam_info,
	const Stereo<fc::Format7ImageSettings>& fmt7_imset,
	const Stereo<float>& shutter,
	const Stereo<float>& gain,
	const Stereo<float>& fps)
{
	cam[L].init(cam_info[L], fmt7_imset[L], shutter[L], gain[L], fps[L], true);
	cam[R].init(cam_info[R], fmt7_imset[R], shutter[R], gain[R], fps[R], true);

	message("Success initialize.");
	return;

}


inline void StereoCameraController::initFromFile(const std::string& file)
{
	cv::FileStorage fs(file, cv::FileStorage::READ);
	CV_Assert(fs.isOpened());

	Stereo<fc::CameraInfo> cam_info;
	Stereo<fc::Format7ImageSettings> fmt7_imset;
	Stereo<float> shutter, gain, fps;

	bool complete = loadStereoCameraParameter(fs, cam_info, fmt7_imset, shutter, gain, fps);

	if (!complete)
		error("Cannot load camera parameter from file.");

	init(cam_info, fmt7_imset, shutter, gain, fps);
	message("Success initialize from file.");
	return;
}


inline Stereo<fc::Property> StereoCameraController::getProperty(const fc::PropertyType type)
{
	Stereo<fc::Property> prop;

	prop[L] = cam[L].getProperty(type);
	prop[R] = cam[R].getProperty(type);
	return prop;
}


inline bool StereoCameraController::setProperty(const Stereo<fc::Property>& prop)
{
	bool complete = true;

	if (cam[L].setProperty(prop[L]))
	{
		message("Cannot set property to left camera.");
		complete = false;
	}

	if (cam[R].setProperty(prop[R]))
	{
		message("Cannot set property to right camera.");
		complete = false;
	}
	return complete;
}


inline bool StereoCameraController::setProperty(
	const fc::PropertyType type,
	const Stereo<float>& abs_value, const Stereo<int>& value_b)
{
	bool complete = true;

	if (cam[L].setProperty(type, abs_value[L], value_b[L]))
	{
		message("Cannot set property to left camera.");
		complete = false;
	}

	if (cam[R].setProperty(type, abs_value[R], value_b[R]))
	{
		message("Cannot set property to right camera.");
		complete = false;
	}
	return complete;
}


inline bool StereoCameraController::setProperty(const fc::PropertyType type, const bool is_auto)
{
	bool complete = true;

	if (cam[L].setProperty(type, is_auto))
	{
		message("Cannot set property to left camera.");
		complete = false;
	}

	if (cam[R].setProperty(type, is_auto))
	{
		message("Cannot set property to right camera.");
		complete = false;
	}
	return complete;
}


inline bool StereoCameraController::setPropertyFromFile(const std::string& file_param)
{
	// Only shutter, gain, frame_rate
	Stereo<fc::CameraInfo> dummy_cam_info;
	Stereo<fc::Format7ImageSettings> dummy_fmt7_imset;
	Stereo<float> shutter, gain, fps;

	cv::FileStorage fs(file_param, cv::FileStorage::READ);
	CV_Assert(fs.isOpened());

	if (!bs::loadStereoCameraParameter(fs, dummy_cam_info, dummy_fmt7_imset, shutter, gain, fps))
	{
		message("Cannot load parameter file");
		return false;
	}
	setProperty(fc::SHUTTER, shutter);
	setProperty(fc::GAIN, gain);
	setProperty(fc::FRAME_RATE, fps);

	return true;
}


inline void StereoCameraController::operator>>(Stereo<cv::Mat>& image)
{
	cam[L] >> image[L];
	cam[R] >> image[R];
	return;
}


inline Stereo<cv::Size> StereoCameraController::calcViewSize(const unsigned int width)
{
	view_sz_[L] = cam[L].calcViewSize(width);
	view_sz_[R] = cam[R].calcViewSize(width);
	return view_sz_;
}


inline void StereoCameraController::show(const Stereo<std::string>& win_name)
{
	Stereo<cv::Mat> im, view;
	int key = -1;
	auto sz = calcViewSize(640);

	sz[L] = (view_sz_[L].area() != 0 ? view_sz_[L] : sz[L]);
	sz[R] = (view_sz_[R].area() != 0 ? view_sz_[R] : sz[R]);

	while (key != -1)
	{
		cam[L] >> im[L];
		cv::resize(im[L], view[L], sz[L]);
		cv::imshow(win_name[L], view[L]);

		cam[R] >> im[R];
		cv::resize(im[R], view[R], sz[R]);
		cv::imshow(win_name[R], view[R]);

		key = cv::waitKey(2);
	}
	return;
}


inline void StereoCameraController::show(
	Stereo<cv::Mat>& latest_image,
	const Stereo<std::string>& win_name)
{
	Stereo<cv::Mat> im, view;
	int key = -1;
	auto sz = calcViewSize(640);

	sz[L] = (view_sz_[L].area() != 0 ? view_sz_[L] : sz[L]);
	sz[R] = (view_sz_[R].area() != 0 ? view_sz_[R] : sz[R]);

	while (key != -1)
	{
		cam[L] >> im[L];
		cv::resize(im[L], view[L], sz[L]);
		cv::imshow(win_name[L], view[L]);

		cam[R] >> im[R];
		cv::resize(im[R], view[R], sz[R]);
		cv::imshow(win_name[R], view[R]);

		key = cv::waitKey(2);
	}
	im[L].copyTo(latest_image[L]);
	im[R].copyTo(latest_image[R]);
	return;
}

}
