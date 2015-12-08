#pragma warning(disable:4819)
#include <opencv2/opencv.hpp>	// for cv::resize, cv::imshow
#pragma warning(default:4819)

#include <bs/camera_controller.hpp>


int main(int argc, char **argv)
{
///////////////////////////////////////////////////////////////////////////////////////////////////////
// If you don't have parameter file, you should create the file.

/////// Single camera
	bs::CameraParams param;
	
	param.serial = 12345678;
	param.pixel_format = FLYCAPTURE_MONO8;
	param.size = cv::Size(640, 480);

	// Optional
	param.tl = cv::Point(0, 0);
	param.shutter = 30.f;
	param.gain = 1.f;
	param.fps = 30.f;
	param.channels = 1;

	cv::FileStorage fs("cam_param.xml", cv::FileStorage::WRITE);
	CV_Assert(fs.isOpened());
	bs::saveCameraParameter(fs, param);
	fs.release();

/////// Stereo camera
	// Stereo<T> and {L, R} are defined in <bs/stereo_util.h>.

	Stereo<bs::CameraParams> s_param;

	s_param[L].serial = 12345678;
	s_param[L].pixel_format = FLYCAPTURE_MONO8;
	s_param[L].size = cv::Size(640, 480);

	// Optional
	s_param[L].tl = cv::Point(0, 0);
	s_param[L].shutter = 30.f;
	s_param[L].gain = 1.f;
	s_param[L].fps = 30.f;
	s_param[L].channels = 1;

	s_param[R].serial = 87654321;
	s_param[R].pixel_format = FLYCAPTURE_MONO8;
	s_param[R].size = cv::Size(640, 480);

	// Optional
	s_param[R].tl = cv::Point(0, 0);
	s_param[R].shutter = 30.f;
	s_param[R].gain = 1.f;
	s_param[R].fps = 30.f;
	s_param[R].channels = 1;

	fs.open("stereo_cam_param.xml", cv::FileStorage::READ);
	CV_Assert(fs.isOpened());
	bs::saveStereoCameraParameter(fs, s_param);
	fs.release();

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Use camera controller

/////// Single camera
	bs::CameraController cam_cntl;
	cam_cntl.initFromFile("cam_param.xml");
	bs::CameraController cam_cntl_2(param);

	// Set parameter
	cam_cntl.setProp(FLYCAPTURE_SHUTTER, 10.f);
	cam_cntl.setProp(FLYCAPTURE_GAIN, -1.f);		// Setting -1 means 'auto'.

	// Get current parameter
	bs::CameraParams param2 = cam_cntl.getParams();

	// Get image
	cv::Mat image, view;
	cam_cntl >> image;

	// If you want to resize showing image, you can get the size.
	cv::Size view_size = cam_cntl.calcViewSize(640);
	cv::resize(image, view, view_size);
	cv::imshow("image", view);
	cv::waitKey(30);

/////// Stereo Camera
	bs::StereoCameraController scam_cntl;	// Using parameter file
	scam_cntl.initFromFile("stereo_cam_param.xml");

	// Set parameter
	Stereo<float> sshutter = { 10.f, 2.f }, sgain = { 2.f, 1.f }, sfps = { 20.f, 30.f };
	scam_cntl.setProp(sshutter, sgain, sfps);			// Set shutter, gain and fps together.
	scam_cntl.setProp(FLYCAPTURE_SHUTTER, sshutter);	// Set one parameter to two cameras.
	scam_cntl.setProp(FLYCAPTURE_GAIN, 1.5f, L);		// Set one parameter to one camera.

	// Get image
	Stereo<cv::Mat> simage;
	cv::Mat image_left;

	scam_cntl >> simage;				// Get stereo image.
	scam_cntl.capture(image_left, L);	// Get one image.

	Stereo<cv::Size> sview_size = scam_cntl.calcViewSize(640);
	cv::resize(simage[L], view, sview_size[L]);
	cv::imshow("left", view);
	cv::resize(simage[R], view, sview_size[R]);
	cv::imshow("right", view);
	cv::waitKey();

	return 0;
}
