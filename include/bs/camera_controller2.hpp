#pragma once

#include <iostream>
#include <cassert>

#pragma warning(disable:4819)
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <FlyCapture2/FlyCapture2.h>
#pragma warning(default:4819)

#include <bs/stereo_util.hpp>


namespace fc = FlyCapture2;


void bs__loadCameraParameterBase(const cv::FileNode& fn,
								 fc::CameraInfo& cam_info,
								 fc::Format7ImageSettings& fmt7_imset,
								 float& shutter, float& gain, float& frame_rate);

void bs__saveCameraParameterBase(cv::FileStorage& fs,
								 const fc::CameraInfo& cam_info,
								 const fc::Format7ImageSettings& fmt7_imset,
								 const float shutter, const float gain, const float frame_rate);

namespace bs
{
	std::string fmt2str(const fc::PixelFormat fmt);

	fc::PixelFormat str2fmt(const std::string& str);

	bool loadCameraParameter(const cv::FileNode& fs,
							 fc::CameraInfo& cam_info,
							 fc::Format7ImageSettings& fmt7_imset,
							 float& shutter, float& gain, float& frame_rate);

	bool saveCameraParameter(cv::FileStorage& fs,
							 const fc::CameraInfo& cam_info,
							 const fc::Format7ImageSettings& fmt7_imset,
							 const float shutter, const float gain, const float frame_rate);

	bool loadStereoCameraParameter(const cv::FileStorage& fs,
								   Stereo<fc::CameraInfo>& cam_info,
								   Stereo<fc::Format7ImageSettings>& fmt7_imset,
								   Stereo<float>& shutter, Stereo<float>& gain,
								   Stereo<float>& frame_rate);

	bool saveStereoCameraParameter(cv::FileStorage& fs,
								   const Stereo<fc::CameraInfo>& cam_info,
								   const Stereo<fc::Format7ImageSettings>& fmt7_imset,
								   const Stereo<float>& shutter, const Stereo<float>& gain,
								   const Stereo<float>& frame_rate);

	bool makeCameraParameterFile(const std::string& file_name, const unsigned int serial,
								 const fc::Mode mode, const fc::PixelFormat pixel_format,
								 const cv::Point& offset, const cv::Size& size,
								 const float shutter, const float gain, const float frame_rate);

	bool makeStereoCameraParameterFile(const std::string& file_name, const Stereo<unsigned int> serial,
									   const Stereo<fc::Mode> mode,
									   const Stereo<fc::PixelFormat> pixel_format,
									   const Stereo<cv::Point>& offset,
									   const Stereo<cv::Size>& size,
									   const Stereo<float> shutter, const Stereo<float> gain,
									   const Stereo<float> frame_rate);


	class Camera
	{
	private:
		bool connected_;
		cv::Size size_;

	protected:
		fc::Camera cam;
		fc::CameraInfo cam_info;
		fc::Format7ImageSettings fmt7_imset;

		bool connect();

		bool checkError(fc::Error error) const;

	public:
		Camera() : connected_(false){};

		Camera(const Camera& o) : connected_(false){};

		~Camera();

		cv::Size getSize() const { return size_; };

		void capture(fc::Image& im, const fc::PixelFormat output_format = fc::PIXEL_FORMAT_MONO8);

		Camera& operator=(const Camera& o);
	};


	class CameraController : public Camera
	{
		bool stereo_;
		cv::Size view_sz_;

		void message(const std::string& str);

	public:
		CameraController() {}

		CameraController(const std::string& file_param);

		~CameraController() { if (!stereo_) message("Bye!"); }

		bool init(const fc::CameraInfo& cam_info, const fc::Format7ImageSettings& fmt7imset,
				  const float shutter, const float gain, const float frame_rate, const bool stereo);

		bool initFromFile(const std::string& file);

		/*!
		In case of BRIGHTNESS, SHARPNESS, WRITE_BARALNCE,
		we should get the absolute value(s) from valueA(and valueB).
		*/
		fc::Property getProperty(const fc::PropertyType type);

		bool setProperty(const fc::Property& prop);

		bool setProperty(const fc::PropertyType type, const float abs_value, const int value_b = 0);

		bool setProperty(const fc::PropertyType type, const bool is_auto);

		bool setPropertyFromFile(const std::string& file_param);

		void operator>> (cv::OutputArray image);

		cv::Size calcViewSize(const unsigned int width);

		void printParameter();

		int show(const std::string win_name = "camera");
	};


	class StereoCameraController
	{
		Stereo<CameraController> cam;
		Stereo<cv::Size> view_sz_;

		void message(const std::string& str);

	public:
		StereoCameraController(){};

		StereoCameraController(const std::string& file_param);

		~StereoCameraController(){};

		bool init(const Stereo<fc::CameraInfo>& cam_info,
				  const Stereo<fc::Format7ImageSettings>& fmt7_imset,
				  const Stereo<float>& shutter, const Stereo<float>& gain,
				  const Stereo<float>& frame_rate);

		bool initFromFile(const std::string& file);

		Stereo<cv::Size> getSize() const;

		/*!
		In case of BRIGHTNESS, SHARPNESS, WRITE_BARALNCE,
		we should get the absolute value(s) from valueA(and valueB).
		*/
		Stereo<fc::Property> getProperty(const fc::PropertyType type);

		bool setProperty(const Stereo<fc::Property>& prop);

		bool setProperty(const fc::PropertyType type, const Stereo<float>& abs_value,
						 const Stereo<int>& value_b = make_Stereo(0, 0));

		bool setProperty(const fc::PropertyType type, const bool is_auto);

		bool setPropertyFromFile(const std::string& file_param);

		void operator>>(Stereo<cv::Mat>& image);

		Stereo<cv::Size> calcViewSize(const unsigned int width);

		int show(const Stereo<std::string>& win_name = bs::make_Stereo<std::string>("left", "right"));
	};
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

void bs__loadCameraParameterBase(
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

	fn["width"] >> tmp_int;
	fmt7_imset.width = tmp_int;

	fn["height"] >> tmp_int;
	fmt7_imset.height = tmp_int;

	fn["shutter"] >> shutter;
	fn["gain"] >> gain;
	fn["fps"] >> frame_rate;
	return;
}


void bs__saveCameraParameterBase(
	cv::FileStorage& fs,
	const fc::CameraInfo& cam_info,
	const fc::Format7ImageSettings& fmt7_imset,
	const float shutter, const float gain, const float frame_rate)
{
	fs << "serial" << static_cast<int>(cam_info.serialNumber);
	fs << "mode" << static_cast<int>(fmt7_imset.mode);
	fs << "format" << bs::fmt2str(fmt7_imset.pixelFormat);
	fs << "offsetX" << static_cast<int>(fmt7_imset.offsetX);
	fs << "offsetY" << static_cast<int>(fmt7_imset.offsetY);
	fs << "width" << static_cast<int>(fmt7_imset.width);
	fs << "height" << static_cast<int>(fmt7_imset.height);
	fs << "shutter" << shutter;
	fs << "gain" << gain;
	fs << "fps" << frame_rate;

	return;
}

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
		case 0x00000001:						// FLUCAPTURE_MONO8
			str = "PIXEL_FORMAT_MONO8";
			break;

		case fc::PIXEL_FORMAT_411YUV8:			// = 0x40000000, /**< YUV 4:1:1. */
		case 0x00000002:						// FLYCAPTURE_411YUV8
			str = "PIXEL_FORMAT_411YUV8";
			break;

		case fc::PIXEL_FORMAT_422YUV8:			// = 0x20000000, /**< YUV 4:2:2. */
		case 0x00000004:						// FLYCAPTURE_422YUV8
			str = "PIXEL_FORMAT_422YUV8";
			break;

		case fc::PIXEL_FORMAT_444YUV8:			// = 0x10000000, /**< YUV 4:4:4. */
		case 0x00000008:						// FLYCAPTURE_444YUV8
			str = "PIXEL_FORMAT_444YUV8";
			break;

		case fc::PIXEL_FORMAT_RGB8:				// = 0x08000000, /**< R// = G// = B// = 8 bits. */
		case 0x00000010:						// FLYCAPTURE_RGB8
			str = "PIXEL_FORMAT_RGB8";
			break;

		case fc::PIXEL_FORMAT_MONO16:			// = 0x04000000, /**< 16 bits of mono information. */
		case 0x00000020:						// FLYCAPTURE_MONO16
			str = "PIXEL_FORMAT_MONO16";
			break;

		case fc::PIXEL_FORMAT_RGB16:			// = 0x02000000, /**< R// = G// = B// = 16 bits. */
		case 0x00000040:						// FLYCAPTURE_RGB16
			str = "PIXEL_FORMAT_RGB16";
			break;

		case fc::PIXEL_FORMAT_S_MONO16:			// = 0x01000000, /**< 16 bits of signed mono information. */
		case 0x00000080:						// FLYCAPTURE_S_MONO16
			str = "PIXEL_FORMAT_S_MONO16";
			break;

		case fc::PIXEL_FORMAT_S_RGB16:			// = 0x00800000, /**< R// = G// = B// = 16 bits signed. */
		case 0x00000100:						// FLYCAPTURE_S_RGB16
			str = "PIXEL_FORMAT_S_RGB16";
			break;

		case fc::PIXEL_FORMAT_RAW8:				// = 0x00400000, /**< 8 bit raw data output of sensor. */
		case 0x00000200:						// FLYCAPTURE_RAW8
			str = "PIXEL_FORMAT_RAW8";
			break;

		case fc::PIXEL_FORMAT_RAW16:			// = 0x00200000, /**< 16 bit raw data output of sensor. */
		case 0x00000400:						// FLYCAPTURE_RAW16
			str = "PIXEL_FORMAT_RAW16";
			break;

		case fc::PIXEL_FORMAT_MONO12:			// = 0x00100000, /**< 12 bits of mono information. */
			str = "PIXEL_FORMAT_MONO12";
			break;

		case fc::PIXEL_FORMAT_RAW12:			// = 0x00080000, /**< 12 bit raw data output of sensor. */
			str = "PIXEL_FORMAT_RAW12";
			break;

		case fc::PIXEL_FORMAT_BGR:				// = 0x80000008, /**< 24 bit BGR. */
		case 0x10000001:						// FLYCAPTURE_BGR
			str = "PIXEL_FORMAT_BGR";
			break;

		case fc::PIXEL_FORMAT_BGRU:				// = 0x40000008, /**< 32 bit BGRU. */
		case 0x10000002:						// FLYCAPTURE_BGRU
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

	bool loadCameraParameter(const cv::FileNode& fs, fc::CameraInfo& cam_info, fc::Format7ImageSettings& fmt7_imset, float& shutter, float& gain, float& frame_rate)
	{
		const auto uintmax = std::numeric_limits<unsigned int>::max();
		cv::FileNode fn(fs.fs, nullptr);

		cam_info.serialNumber = uintmax;
		bs__loadCameraParameterBase(fn, cam_info, fmt7_imset, shutter, gain, frame_rate);

		return (cam_info.serialNumber == uintmax ? false : true);
	}

	bool saveCameraParameter(cv::FileStorage& fs, const fc::CameraInfo& cam_info, const fc::Format7ImageSettings& fmt7_imset, const float shutter, const float gain, const float frame_rate)
	{
		bs__saveCameraParameterBase(fs, cam_info, fmt7_imset, shutter, gain, frame_rate);
		return true;
	}

	bool loadStereoCameraParameter(const cv::FileStorage& fs, Stereo<fc::CameraInfo>& cam_info, Stereo<fc::Format7ImageSettings>& fmt7_imset, Stereo<float>& shutter, Stereo<float>& gain, Stereo<float>& frame_rate)
	{
		const auto uintmax = std::numeric_limits<unsigned int>::max();

		cam_info[L].serialNumber = cam_info[R].serialNumber = uintmax;

		bs__loadCameraParameterBase(fs["left"], cam_info[L], fmt7_imset[L], shutter[L], gain[L], frame_rate[L]);
		bs__loadCameraParameterBase(fs["right"], cam_info[R], fmt7_imset[R], shutter[R], gain[R], frame_rate[R]);

		if (cam_info[L].serialNumber == uintmax || cam_info[R].serialNumber == uintmax)
			return false;
		else
			return true;
	}

	bool saveStereoCameraParameter(cv::FileStorage& fs, const Stereo<fc::CameraInfo>& cam_info, const Stereo<fc::Format7ImageSettings>& fmt7_imset, const Stereo<float>& shutter, const Stereo<float>& gain, const Stereo<float>& frame_rate)
	{
		fs << "left" << "{";
		bs__saveCameraParameterBase(fs, cam_info[L], fmt7_imset[L], shutter[L], gain[L], frame_rate[L]);
		fs << "}";

		fs << "right" << "{";
		bs__saveCameraParameterBase(fs, cam_info[R], fmt7_imset[R], shutter[R], gain[R], frame_rate[R]);
		fs << "}";

		return true;
	}

	bool makeCameraParameterFile(const std::string& file_name, const unsigned int serial, const fc::Mode mode, const fc::PixelFormat pixel_format, const cv::Point& offset, const cv::Size& size, const float shutter, const float gain, const float frame_rate)
	{
		cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
		CV_Assert(fs.isOpened());

		fc::CameraInfo ci;
		fc::Format7ImageSettings is;

		ci.serialNumber = serial;
		is.mode = mode;
		is.pixelFormat = pixel_format;
		is.offsetX = static_cast<unsigned int>(offset.x);
		is.offsetY = static_cast<unsigned int>(offset.y);
		is.width = static_cast<unsigned int>(size.width);
		is.height = static_cast<unsigned int>(size.height);

		saveCameraParameter(fs, ci, is, shutter, gain, frame_rate);
		return true;
	}

	bool makeStereoCameraParameterFile(const std::string& file_name, const Stereo<unsigned int> serial, const Stereo<fc::Mode> mode, const Stereo<fc::PixelFormat> pixel_format, const Stereo<cv::Point>& offset, const Stereo<cv::Size>& size, const Stereo<float> shutter, const Stereo<float> gain, const Stereo<float> frame_rate)
	{
		cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
		CV_Assert(fs.isOpened());

		Stereo<fc::CameraInfo> ci;
		Stereo<fc::Format7ImageSettings> is;

		ci[L].serialNumber = serial[L];
		is[L].mode = mode[L];
		is[L].pixelFormat = pixel_format[L];
		is[L].offsetX = static_cast<unsigned int>(offset[L].x);
		is[L].offsetY = static_cast<unsigned int>(offset[L].y);
		is[L].width = static_cast<unsigned int>(size[L].width);
		is[L].height = static_cast<unsigned int>(size[L].height);

		ci[R].serialNumber = serial[R];
		is[R].mode = mode[R];
		is[R].pixelFormat = pixel_format[R];
		is[R].offsetX = static_cast<unsigned int>(offset[R].x);
		is[R].offsetY = static_cast<unsigned int>(offset[R].y);
		is[R].width = static_cast<unsigned int>(size[R].width);
		is[R].height = static_cast<unsigned int>(size[R].height);

		saveStereoCameraParameter(fs, ci, is, shutter, gain, frame_rate);
		return true;
	}


	inline Camera::~Camera()
	{
		if (connected_)
		{
			cam.StopCapture();
			cam.Disconnect();
		}
	}

	inline bool Camera::connect()
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
		size_ = cv::Size(fmt7_imset.width, fmt7_imset.height);
		connected_ = true;
		return true;
	}

	inline bool Camera::checkError(fc::Error error) const
	{
		if (error != fc::PGRERROR_OK)
		{
			error.PrintErrorTrace();
			return true;
		}
		return false;
	}

	inline void Camera::capture(fc::Image& im, const fc::PixelFormat output_format)
	{
		fc::Image raw_im;
		cam.RetrieveBuffer(&raw_im);
		raw_im.Convert(output_format, &im);

		return;
	}

	inline Camera& Camera::operator=(const Camera& o)
	{
		cam_info = o.cam_info;
		fmt7_imset = o.fmt7_imset;
		connect();

		return *this;
	}


	inline CameraController::CameraController(const std::string& file_param) : stereo_(false)
	{
		if (!initFromFile(file_param))
			abort();
	}

	inline void CameraController::message(const std::string& str)
	{
		std::cout << "[Camera Controller] " << str << std::endl;
		return;
	}

	inline bool CameraController::init(const fc::CameraInfo& _cam_info,const fc::Format7ImageSettings& _fmt7_imset, const float shutter, const float gain, const float frame_rate, const bool stereo)
	{
		cam_info = _cam_info;
		fmt7_imset = _fmt7_imset;
		stereo_ = stereo;

		if (!connect())
		{
			message("Error: Cannot connect to the camera.");
			return false;
		}

		setProperty(fc::SHUTTER, shutter);
		setProperty(fc::GAIN, gain);
		setProperty(fc::FRAME_RATE, frame_rate);

		if (!stereo_)
			message("Success initialize.");
		return true;
	}

	inline bool CameraController::initFromFile(const std::string& file)
	{
		float shutter, gain, frame_rate;
		cv::FileStorage fs(file, cv::FileStorage::READ);
		CV_Assert(fs.isOpened());

		cv::FileNode fn(fs.fs, nullptr);

		bool complete = loadCameraParameter(fn, cam_info, fmt7_imset, shutter, gain, frame_rate);

		if (!complete)
		{
			message("Error: Cannot load camera parameter from file.");
			return false;
		}

		if (!connect())
		{
			message("Error: Cannot connect to the camera.");
			return false;
		}

		setProperty(fc::SHUTTER, shutter);
		setProperty(fc::GAIN, gain);
		setProperty(fc::FRAME_RATE, frame_rate);

		message("Success initialize from file.");
		return true;
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

	inline bool CameraController::setProperty(const fc::PropertyType type, const float value, const int value_b)
	{
		auto prop = getProperty(type);

		switch (type)
		{
		default:
			prop.absControl = true;
			prop.absValue = value;
			break;

		case fc::BRIGHTNESS:
		case fc::SHARPNESS:
			prop.valueA = static_cast<unsigned int>(value);
			break;

		case fc::WHITE_BALANCE:
			prop.valueA = static_cast<unsigned int>(value);
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
			message("Error: Auto is not supported");
			return false;
		}
		prop.autoManualMode = is_auto;
		return setProperty(prop);
	}

	inline bool CameraController::setPropertyFromFile(const std::string& file_param)
	{
		// Only shutter, gain, frame_rate
		float shutter, gain, frame_rate;

		cv::FileStorage fs(file_param, cv::FileStorage::READ);
		CV_Assert(fs.isOpened());

		cv::FileNode fn(fs.fs, nullptr);

		if (!bs::loadCameraParameter(fn, cam_info, fmt7_imset, shutter, gain, frame_rate))
		{
			message("Error: Cannot load parameter file");
			return false;
		}
		setProperty(fc::SHUTTER, shutter);
		setProperty(fc::GAIN, gain);
		setProperty(fc::FRAME_RATE, frame_rate);

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

	inline void CameraController::printParameter()
	{
		std::cout << std::endl;
		message("Print parameter");

		std::cout
			<< "Serial number | " << cam_info.serialNumber << std::endl
			<< "Model         | " << cam_info.modelName << std::endl
			<< "Pixel format  | " << bs::fmt2str(fmt7_imset.pixelFormat) << std::endl
			<< "Resolution    | [" << fmt7_imset.width << ", " << fmt7_imset.height << "]" << std::endl
			<< "Shutter       | " << getProperty(fc::SHUTTER).absValue << std::endl
			<< "Gain          | " << getProperty(fc::GAIN).absValue << std::endl
			<< "Frame rate    | " << getProperty(fc::FRAME_RATE).absValue << std::endl;
		return;
	}

	inline int CameraController::show(const std::string win_name)
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
		return key;
	}


	inline StereoCameraController::StereoCameraController(const std::string& file_param)
	{
		if (!initFromFile(file_param))
			abort();
	}

	inline void StereoCameraController::message(const std::string& str)
	{
		std::cout << "[Stereo Camera Controller] " << str << std::endl;
		return;
	}

	inline bool StereoCameraController::init(const Stereo<fc::CameraInfo>& cam_info, const Stereo<fc::Format7ImageSettings>& fmt7_imset, const Stereo<float>& shutter, const Stereo<float>& gain, const Stereo<float>& frame_rate)
	{
		bool complete;

		complete = cam[L].init(cam_info[L], fmt7_imset[L], shutter[L], gain[L], frame_rate[L], true);
		complete = cam[R].init(cam_info[R], fmt7_imset[R], shutter[R], gain[R], frame_rate[R], true) & complete;

		if (!complete)
		{
			message("Error: Initialize failed.");
			return false;
		}
		message("Success initialize.");
		return true;
	}

	inline bool StereoCameraController::initFromFile(const std::string& file)
	{
		cv::FileStorage fs(file, cv::FileStorage::READ);
		CV_Assert(fs.isOpened());

		Stereo<fc::CameraInfo> cam_info;
		Stereo<fc::Format7ImageSettings> fmt7_imset;
		Stereo<float> shutter, gain, frame_rate;

		bool complete = loadStereoCameraParameter(fs, cam_info, fmt7_imset, shutter, gain, frame_rate);

		if (!complete)
		{
			message("Error: Cannot load parameter from file.");
			return false;
		}

		complete = cam[L].init(cam_info[L], fmt7_imset[L], shutter[L], gain[L], frame_rate[L], true);
		complete = cam[R].init(cam_info[R], fmt7_imset[R], shutter[R], gain[R], frame_rate[R], true) & complete;

		if (!complete)
		{
			message("Error: Initialize failed.");
			return false;
		}
		message("Success initialize from file.");
		return true;
	}

	inline Stereo<cv::Size> StereoCameraController::getSize() const
	{
		return bs::make_Stereo<cv::Size>(cam[L].getSize(), cam[R].getSize());
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

	inline bool StereoCameraController::setProperty(const fc::PropertyType type, const Stereo<float>& abs_value, const Stereo<int>& value_b)
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
		Stereo<float> shutter, gain, frame_rate;

		cv::FileStorage fs(file_param, cv::FileStorage::READ);
		CV_Assert(fs.isOpened());

		if (!bs::loadStereoCameraParameter(fs, dummy_cam_info, dummy_fmt7_imset, shutter, gain, frame_rate))
		{
			message("Cannot load parameter file");
			return false;
		}
		setProperty(fc::SHUTTER, shutter);
		setProperty(fc::GAIN, gain);
		setProperty(fc::FRAME_RATE, frame_rate);

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

	inline int StereoCameraController::show(const Stereo<std::string>& win_name)
	{
		Stereo<cv::Mat> im, view;
		int key = -1;
		auto sz = calcViewSize(640);

		sz[L] = (view_sz_[L].area() != 0 ? view_sz_[L] : sz[L]);
		sz[R] = (view_sz_[R].area() != 0 ? view_sz_[R] : sz[R]);

		while (key == -1)
		{
			cam[L] >> im[L];
			cv::resize(im[L], view[L], sz[L]);
			cv::imshow(win_name[L], view[L]);

			cam[R] >> im[R];
			cv::resize(im[R], view[R], sz[R]);
			cv::imshow(win_name[R], view[R]);

			key = cv::waitKey(2);
		}
		return key;
	}
}
