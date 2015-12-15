#pragma once
#include <iostream>
#include <omp.h>

#include <bs/path.h>
#include <bs/my_opencv.hpp>
#include <bs/camera_controller2.hpp>
#include <bs/projection_image.hpp>

using Points2d = std::vector<cv::Point2d>;

class OpticalFlowApp
{
private:
	// Prameter for cv::goodFeaturesToTrack()
	int max_corners_;
	double quality_level_, min_distance_;

	bs::StereoCameraController cam;
	std::vector<cv::Mat> prj_ims_;
	Stereo<std::vector<Points2d>> points_;

public:
	unsigned int view_width;
	unsigned int draw_circle_radius;
	unsigned int draw_line_width;
	int delay;
	cv::Size view_sz;
	bool sw_save_all_image;

	OpticalFlowApp(){};

	OpticalFlowApp(const unsigned int numof_shift);

	~OpticalFlowApp(){}
	
	void setParameter(const int max_corners = 100, 
					  const double quality_level = 0.05,
					  const double min_distance = 3.);

	void resizePoints(const unsigned int numof_shift);
	
	bool startCamera();

	int showCameraImage();

	void detectFeatures(Stereo<cv::Mat>& draw_image);

	void setProjectionImages(cv::InputArrayOfArrays images);

	bool compute();

	Stereo<std::vector<Points2d>> getPoints() const { return points_; };
};


inline OpticalFlowApp::OpticalFlowApp(const unsigned int numof_shift) :
	draw_circle_radius(20), draw_line_width(5),
	view_width(640), delay(300),
	sw_save_all_image(true)
{
	resizePoints(numof_shift);
}

inline void OpticalFlowApp::setParameter(const int max_corners, const double quality_level, const double min_distance)
{
	max_corners_ = max_corners;
	quality_level_ = quality_level;
	min_distance_ = min_distance;
	return;
}

inline void OpticalFlowApp::resizePoints(const unsigned int numof_shift)
{
	points_[L].resize(numof_shift);
	points_[R].resize(numof_shift);
	return;
}

inline bool OpticalFlowApp::startCamera()
{
	if (!cam.initFromFile(kFileCamParam))
		return false;

	view_sz = cam.calcViewSize(view_width)[L];
	return true;
}

inline int OpticalFlowApp::showCameraImage()
{
	return cam.show();
}

inline void OpticalFlowApp::detectFeatures(Stereo<cv::Mat>& draw_image)
{
	assert(!points_[L].empty() && !points_[R].empty());

	Stereo<cv::Mat> im;
	Stereo<std::vector<cv::Point2d>> corners;

	cam >> im;

#pragma omp parallel sections
	{
#pragma omp section
	{
		cv::goodFeaturesToTrack(im[L], corners[L], max_corners_, quality_level_, min_distance_);
		cv::cvtColor(im[L], draw_image[L], cv::COLOR_GRAY2BGR);

		for (const auto& p : corners[L])
			cv::circle(draw_image[L], p, draw_circle_radius, cv::Scalar(0, 255, 0),
					   cv::FILLED, cv::LINE_AA);
		points_[L][0] = corners[L];
	}
#pragma omp section
	{
		cv::goodFeaturesToTrack(im[R], corners[R], max_corners_, quality_level_, min_distance_);
		cv::cvtColor(im[R], draw_image[R], cv::COLOR_GRAY2BGR);

		for (const auto& p : corners[R])
			cv::circle(draw_image[R], p, draw_circle_radius, cv::Scalar(0, 255, 0),
					   cv::FILLED, cv::LINE_AA);
		points_[R][0] = corners[R];
	}
	}
	assert(points_[L][0].size() == points_[R][0].size());
	return;
}

inline void OpticalFlowApp::setProjectionImages(cv::InputArrayOfArrays images)
{
	images.getMatVector(prj_ims_);

	if (points_[L].empty() || points_[R].empty())
		resizePoints(prj_ims_.size());
	return;
}

inline bool OpticalFlowApp::compute()
{
	assert(0 < prj_ims_.size());
	assert(points_[L][0].size() == points_[R][0].size());

	const std::string dir = kDirTop + "BM_data/OpticalFlowMeasurement/images/";

	Stereo<cv::Mat> prev_im, next_im, flow_view_im;
	std::vector<uchar> status;
	std::vector<float> errors;

	bs::showWindowNoframe(prj_ims_[0]);

	if (cv::waitKey(delay) == 'q')
		return false;

	cam >> prev_im;
	cv::cvtColor(prev_im[L], flow_view_im[L], cv::COLOR_GRAY2BGR);
	cv::cvtColor(prev_im[R], flow_view_im[R], cv::COLOR_GRAY2BGR);

	for (size_t i = 0; i < points_[L][0].size(); ++i)
	{
		cv::circle(flow_view_im[L], points_[L][0][i], draw_circle_radius,
				   cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);
		cv::circle(flow_view_im[R], points_[R][0][i], draw_circle_radius,
				   cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);
	}
	bs::imshow("left: flow", flow_view_im[L], view_sz);
	bs::imshow("right: flow", flow_view_im[R], view_sz);
	cv::waitKey(2);

	if (sw_save_all_image)
	{
		bs::imwrite(dir + "cam_l0.png", prev_im[L], view_sz * 2);
		bs::imwrite(dir + "cam_r0.png", prev_im[R], view_sz * 2);
	}

	for (size_t i = 1; i < prj_ims_.size(); ++i)
	{

		bs::showWindowNoframe(prj_ims_[i]);

		if (cv::waitKey(delay) == 'q')
			return false;
		cam >> next_im;

		if (sw_save_all_image)
		{
			bs::imwrite(dir + "cam_l" + std::to_string(i) + ".png", next_im[L], view_sz * 2);
			bs::imwrite(dir + "cam_r" + std::to_string(i) + ".png", next_im[R], view_sz * 2);
		}

		cv::calcOpticalFlowPyrLK(prev_im[L], next_im[L],
								 points_[L][i - 1], points_[L][i],
								 status, errors);
		cv::calcOpticalFlowPyrLK(prev_im[R], next_im[R],
								 points_[R][i - 1], points_[R][i],
								 status, errors);
		assert(points_[L][i].size() == points_[R][i].size());

		cv::cvtColor(next_im[L], flow_view_im[L], cv::COLOR_GRAY2BGR);
		cv::cvtColor(next_im[R], flow_view_im[R], cv::COLOR_GRAY2BGR);

		for (size_t j = 0; j <= i; ++j)
		{
			for (size_t k = 0; k < points_[L][j].size(); ++k)
			{
				cv::circle(flow_view_im[L], points_[L][j][k], draw_circle_radius,
						   cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);
				cv::circle(flow_view_im[R], points_[R][j][k], draw_circle_radius,
						   cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);
			}
		}
		bs::imshow("left: flow", flow_view_im[L], view_sz);
		bs::imshow("right: flow", flow_view_im[R], view_sz);
		cv::waitKey(2);

		next_im[L].copyTo(prev_im[L]);
		next_im[R].copyTo(prev_im[R]);
	}
	return true;
}
