#include <opencv2/opencv.hpp>	// for cv::resize, cv::imshow
#include <bs/camera_controller.hpp>


int main(int argc, char **argv)
{
///////////////////////////////////////////////////////////////////////////////////////////////////////
// If you don't have parameter file, you should create the file.

/////// Single camera
	bs::CameraParams param(12345678, FLYCAPTURE_MONO8, cv::Size(640, 480));

	/* Or, set all values in constructor.
	bs::CameraParams param(12345678, FLYCAPTURE_MONO8, cv::Size(640, 480), 33.f, 5.f, 30.f);	*/

	/*  Or, set each value
	param.serial = 12345678;
	param.pixel_format = FLYCAPTURE_MONO8;
	param.size = cv::Size(640, 480);
	param.shutter = 3.f;
	param.gain = 5.f;
	param.fps = 30.f;	*/

	cv::FileStorage fs("./cam_param.xml", cv::FileStorage::WRITE);
	CV_Assert(fs.isOpened());
	bs::saveCameraParameter(fs, param);
	fs.release();

/////// Stereo camera
	// In camera_controller.hpp, 
	//	#ifndef _BS_STEREO_
	//	#define _BS_STEREO_
	//	#include<array>
	//		template<class T>
	//		using Stereo = std::array<T, 2>;
	//	#endif
	Stereo<bs::CameraParams> s_param;
	s_param[0] = *(new bs::CameraParams(12345678, FLYCAPTURE_MONO8, cv::Size(640, 480), 30.f, 5.f, 30.f));
	s_param[1] = *(new bs::CameraParams(87654321, FLYCAPTURE_RGB8, cv::Size(1280, 1080), 15.f, 3.f, 60.f));

	fs.open("./stereo_cam_param.xml", cv::FileStorage::READ);
	CV_Assert(fs.isOpened());
	bs::saveStereoCameraParameter(fs, s_param);
	fs.release();

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Use camera controller

/////// Single camera
	bs::CameraController cam_cntl_1;								// Using parameter file
	bs::CameraController cam_cntl_2(param);							// Using bs::CameraParams
	bs::CameraController cam_cntl_3(12345678, cv::Size(640, 480));	// Set values directly

	// Set parameter
	cam_cntl_1.setProp(FLYCAPTURE_SHUTTER, 10.f);
	cam_cntl_1.setProp(FLYCAPTURE_GAIN, -1.f);		// Setting -1 means 'auto'.

	// Get current parameter
	bs::CameraParams param2 = cam_cntl_1.getParams();

	// Get image
	cv::Mat image, view;
	cam_cntl_1 >> image;

	// If you want to resize showing image, you can get the size.
	cv::Size view_size = cam_cntl_1.setViewSize(640);
	cv::resize(image, view, view_size);
	cv::imshow("image", view);
	cv::waitKey(30);

	// Note: If you call the constructor of using parameter file,
	//       current parameters are automatically saved that file.

/////// Stereo Camera
	bs::StereoCameraController scam_cntl;	// Using parameter file

	// Set parameter
	Stereo<float> sshutter = { 10.f, 2.f }, sgain = { 2.f, 1.f }, sfps = { 20.f, 30.f };
	scam_cntl.setProp(sshutter, sgain, sfps);												// Set shutter, gain and fps together.
	scam_cntl.setProp(FLYCAPTURE_SHUTTER, sshutter);										// Set one parameter to two cameras.
	scam_cntl.setProp(FLYCAPTURE_GAIN, 1.5f, bs::StereoCameraController::CAM_SELECT::LEFT);	// Set one parameter to one camera.

	// Get image
	Stereo<cv::Mat> simage;
	cv::Mat image_left;

	scam_cntl >> simage;															// Get stereo image.
	scam_cntl.capture(image_left, bs::StereoCameraController::CAM_SELECT::LEFT);	// Get one image.

	Stereo<cv::Size> sview_size = scam_cntl.setViewSize(640);
	cv::resize(simage[0], view, sview_size[0]);
	cv::imshow("left", view);
	cv::resize(simage[1], view, sview_size[1]);
	cv::imshow("right", view);
	cv::waitKey();

	return 0;
}
