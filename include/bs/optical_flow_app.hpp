#pragma once
#include <iostream>
#include <omp.h>

#include <boost/filesystem.hpp>

#include <bs/path.h>
#include <bs/my_opencv.hpp>
#include <bs/camera_controller2.hpp>
#include <bs/projection_image.hpp>
#include <bs/rectify_remap.hpp>


bool bs__isGoodFlow(const std::vector<cv::Point2f>& flow, const double ang_th = 5);


namespace bs
{
	struct OpticalFlow
	{
		cv::Point2f point;
		double magnitude, angle;
	};

	class OpticalFlowApp
	{
	private:
		std::string out_dir_;

		// Prameter for cv::goodFeaturesToTrack()
		int max_corners_;
		double quality_level_, min_distance_;

		bs::RectifyRemap map_;
		std::vector<cv::Mat> prj_ims_;
		Stereo<std::vector<std::vector<cv::Point2f>>> points_;
		Stereo<size_t> numof_pixels_;
		size_t numof_computings_;

	public:
		bs::StereoCameraController cam;
		unsigned int view_width;
		unsigned int draw_circle_radius;
		unsigned int draw_line_width;
		int delay;
		cv::Size cam_size, view_sz;
		bool sw_save_all_images;

		OpticalFlowApp();

		OpticalFlowApp(const size_t numof_shift);

		~OpticalFlowApp(){}

		void setFeatureParameter(const int max_corners = 100,
			const double quality_level = 0.05,
			const double min_distance = 3.);

		void initOutputDirectory(const std::string& dir_name);

		void resizePoints(const size_t numof_shift);

		bool startCamera();

		int showCameraImage();

		void captureRectifiedStereoCameraImage(Stereo<cv::Mat>& image);

		void detectFeatures(Stereo<cv::Mat>& draw_image,
							const Stereo<cv::Mat>& mask = bs::make_Stereo(cv::Mat(), cv::Mat()));
		void detectFeatures(Stereo<cv::Mat>& draw_image, const Stereo<cv::Rect>& roi);

		void setProjectionImages(cv::InputArrayOfArrays images);

		bool compute();

		Stereo<std::vector<std::vector<cv::Point2f>>> OpticalFlowApp::getPoints() const;
			
		Stereo<std::vector<cv::Point2f>> getPointsAtOnePixel(const size_t index) const;
		
		Stereo<std::vector<cv::Point2f>> getPointsAtOneComputing(const size_t index) const;
	};

	void rmBadFlow(const Stereo<std::vector<std::vector<cv::Point2f>>>& src,
				   Stereo<std::vector<std::vector<cv::Point2f>>>& dst);

	void integrateFlow(const Stereo<std::vector<std::vector<cv::Point2f>>>& pixel_set,
					   Stereo<std::vector<OpticalFlow>>& dst_flow);

	std::vector<OpticalFlow> searchEpipolar(const OpticalFlow& src_flow,
									   const std::vector<OpticalFlow>& target_flow_vec);

	std::ostream& operator<< (std::ostream& os, const OpticalFlow& flow)
	{
		os << "[mag, ang] = [" << flow.magnitude << ", " << flow.angle << "] from " << flow.point;
		return os;
	}

	inline OpticalFlowApp::OpticalFlowApp() :
		draw_circle_radius(20), draw_line_width(5),
		view_width(640), delay(300),
		sw_save_all_images(true)
	{}

	inline OpticalFlowApp::OpticalFlowApp(const size_t numof_shift) :
		draw_circle_radius(20), draw_line_width(5),
		view_width(640), delay(300),
		sw_save_all_images(true)
	{
		resizePoints(numof_shift);
	}

	inline void OpticalFlowApp::setFeatureParameter(const int max_corners, const double quality_level, const double min_distance)
	{
		max_corners_ = max_corners;
		quality_level_ = quality_level;
		min_distance_ = min_distance;
		return;
	}

	inline void OpticalFlowApp::initOutputDirectory(const std::string& dir_name)
	{
		namespace bf = boost::filesystem;
		out_dir_ = *dir_name.crbegin() == '/' ? dir_name : dir_name + "/";
		bf::create_directories(out_dir_ + "camera/raw");
		bf::create_directory(out_dir_ + "projection");
		bf::create_directory(out_dir_ + "flow_data");
		return;
	}

	inline void OpticalFlowApp::resizePoints(const size_t numof_shift)
	{
		points_[L].resize(numof_shift);
		points_[R].resize(numof_shift);
		numof_computings_ = numof_shift;
		return;
	}

	inline bool OpticalFlowApp::startCamera()
	{
		if (!map_.load(kDirCalib))
			return false;

		if (!cam.initFromFile(kFileCamParam))
			return false;

		view_sz = cam.calcViewSize(view_width)[L];
		cam_size = cam.getSize()[L];
		return true;
	}

	inline int OpticalFlowApp::showCameraImage()
	{
		return cam.show(bs::make_Stereo<std::string>("cam left", "cam right"));
	}

	inline void OpticalFlowApp::captureRectifiedStereoCameraImage(Stereo<cv::Mat>& image)
	{
		cam >> image;
		map_(image);
		return;
	}

	inline void OpticalFlowApp::setProjectionImages(cv::InputArrayOfArrays images)
	{
		images.getMatVector(prj_ims_);

		if (sw_save_all_images && !out_dir_.empty())
		{
#pragma omp parallel for
			for (int i = 0; i < (int)prj_ims_.size(); ++i)		// dst_size = [640, 640 * kPrjSize.height / kPrjSize.width] = [640, 640 * 800 / 1280] = [640, 400]
				bs::imwrite(out_dir_ + "projection/prj_" + std::to_string(i) + ".png", prj_ims_[i], cv::Size(640, 400));
		}

		if (points_[L].empty() || points_[R].empty())
			resizePoints(prj_ims_.size());
		bs::showWindowNoframe(prj_ims_[0], cv::Point(2560, -prj_ims_[0].rows));
		cv::waitKey(2);
		return;
	}

	inline void OpticalFlowApp::detectFeatures(Stereo<cv::Mat>& draw_image, const Stereo<cv::Mat>& mask)
	{
		assert(0 < prj_ims_.size());
		assert(!points_[L].empty() && !points_[R].empty());

		const int w = cam.getSize()[L].width, h = cam.getSize()[L].height;
		Stereo<cv::Mat> im, msk;

		bs::showWindowNoframe(prj_ims_[0], cv::Point(2560, -prj_ims_[0].rows));
		cv::waitKey(delay * 100);

		cam >> im;
		map_(im, cv::INTER_LINEAR);

		cv::Mat flow_direc_mask = cv::Mat::zeros(im[L].size(), CV_8U);
		flow_direc_mask(cv::Rect(1000, 0, im[L].cols - 1000, im[L].rows)) = 1;

		msk[L] = mask[L].empty() ? cv::Mat::ones(im[L].size(), CV_8U) : mask[L];
		cv::bitwise_and(msk[L], flow_direc_mask, msk[L]);

		// create right mask
		msk[R] = cv::Mat::zeros(msk[L].size(), CV_8U);

		for (int row = 0; row < msk[R].rows; ++row)
		{
			const auto *l_ptr = msk[L].ptr<uchar>(row);
			auto *r_ptr = msk[R].ptr<uchar>(row);

			for (int col = 0; col < msk[R].cols; ++col)
			{
				const auto val = l_ptr[col];

				if(val == 1)
					for (int c = col; c < std::min(col + 512, msk[R].cols); ++c)	r_ptr[c] = 1;
			}
		}

#pragma omp parallel sections
		{
#pragma omp section
		{
			cv::goodFeaturesToTrack(im[L], points_[L][0],
									max_corners_, quality_level_, min_distance_, msk[L]);
			cv::cvtColor(im[L], draw_image[L], cv::COLOR_GRAY2BGR);

			for (const auto& p : points_[L][0])
				cv::circle(draw_image[L], p, draw_circle_radius, cv::Scalar(0, 255, 0),
				cv::FILLED, cv::LINE_AA);
		}
#pragma omp section
		{
			cv::goodFeaturesToTrack(im[R], points_[R][0],
									max_corners_, quality_level_ / 2.0, min_distance_, msk[R]);
			cv::cvtColor(im[R], draw_image[R], cv::COLOR_GRAY2BGR);

			for (const auto& p : points_[R][0])
				cv::circle(draw_image[R], p, draw_circle_radius, cv::Scalar(0, 255, 0),
				cv::FILLED, cv::LINE_AA);
		}
		}
		if (points_[L][0].size() == 0 || points_[R][0].size() == 0)
		{
			std::cerr << "No feature points." << std::endl;
			std::exit(-1);
		}
		numof_pixels_ = bs::make_Stereo(points_[L][0].size(), points_[R][0].size());
		return;
	}

	inline void OpticalFlowApp::detectFeatures(Stereo<cv::Mat>& draw_image, const Stereo<cv::Rect>& roi)
	{
		Stereo<cv::Mat> mask = { cv::Mat::zeros(cam_size, CV_8U), cv::Mat::zeros(cam_size, CV_8U) };
		mask[L](roi[L]) = 1;
		mask[R](roi[R]) = 1;

		detectFeatures(draw_image, mask);
		return;
	}

	inline bool OpticalFlowApp::compute()
	{
		assert(0 < prj_ims_.size());

		Stereo<cv::Mat> prev_im, next_im, flow_view_im;
		std::vector<uchar> status;
		std::vector<float> errors;

		bs::showWindowNoframe(prj_ims_[0], cv::Point(2560, -prj_ims_[0].rows));

		if (cv::waitKey(delay) == 'q')
			return false;

		cam >> prev_im;
		map_(prev_im, cv::INTER_LINEAR);
		cv::cvtColor(prev_im[L], flow_view_im[L], cv::COLOR_GRAY2BGR);
		cv::cvtColor(prev_im[R], flow_view_im[R], cv::COLOR_GRAY2BGR);

		for (size_t i = 0; i < points_[L][0].size(); ++i)
			cv::circle(flow_view_im[L], points_[L][0][i], draw_circle_radius, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);

		for (size_t i = 0; i < points_[R][0].size(); ++i)
			cv::circle(flow_view_im[R], points_[R][0][i], draw_circle_radius, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);

		bs::imshow("left: flow", flow_view_im[L], view_sz);
		bs::imshow("right: flow", flow_view_im[R], view_sz);
		cv::waitKey(2);

		if (sw_save_all_images && !out_dir_.empty())
		{
			bs::imwrite(out_dir_ + "camera/raw/cam_l0.png", prev_im[L]);
			bs::imwrite(out_dir_ + "camera/raw/cam_r0.png", prev_im[R]);
			bs::imwrite(out_dir_ + "camera/cam_flow_l0.png", flow_view_im[L], view_sz * 2);
			bs::imwrite(out_dir_ + "camera/cam_flow_r0.png", flow_view_im[R], view_sz * 2);
		}

		for (size_t i = 1; i < prj_ims_.size(); ++i)
		{

			bs::showWindowNoframe(prj_ims_[i], cv::Point(2560, -prj_ims_[i].rows));

			if (cv::waitKey(delay) == 'q')
				return false;
			cam >> next_im;
			map_(next_im, cv::INTER_LINEAR);

			cv::calcOpticalFlowPyrLK(prev_im[L], next_im[L],
				points_[L][i - 1], points_[L][i],
				status, errors);
			cv::calcOpticalFlowPyrLK(prev_im[R], next_im[R],
				points_[R][i - 1], points_[R][i],
				status, errors);
			assert(points_[L][i - 1].size() == points_[L][i].size());
			assert(points_[R][i - 1].size() == points_[R][i].size());

			cv::cvtColor(next_im[L], flow_view_im[L], cv::COLOR_GRAY2BGR);
			cv::cvtColor(next_im[R], flow_view_im[R], cv::COLOR_GRAY2BGR);

			// Draw points
			/// previous
			for (size_t j = 0; j < points_[L][i - 1].size(); ++j)
				cv::circle(flow_view_im[L], points_[L][i - 1][j], draw_circle_radius, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);
			for (size_t j = 0; j < points_[R][i - 1].size(); ++j)
				cv::circle(flow_view_im[R], points_[R][i - 1][j], draw_circle_radius, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);
			/// current
			for (size_t j = 0; j < points_[L][i].size(); ++j)
				cv::circle(flow_view_im[L], points_[L][i][j], draw_circle_radius, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);
			for (size_t j = 0; j < points_[R][i].size(); ++j)
				cv::circle(flow_view_im[R], points_[R][i][j], draw_circle_radius, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);

			//for (size_t j = 0; j <= i; ++j)
			//{
			//	for (size_t k = 0; k < points_[L][j].size(); ++k)
			//	{
			//		cv::circle(flow_view_im[L], points_[L][j][k], draw_circle_radius,
			//			cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);

			//		if(k < points_.size())
			//			cv::circle(flow_view_im[R], points_[R][j][k], draw_circle_radius,
			//			cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);
			//	}
			//}
			bs::imshow("left: flow", flow_view_im[L], view_sz);
			bs::imshow("right: flow", flow_view_im[R], view_sz);
			cv::waitKey(2);

			if (sw_save_all_images && !out_dir_.empty())
			{
				bs::imwrite(out_dir_ + "camera/raw/cam_l" + std::to_string(i) + ".png", prev_im[L]);
				bs::imwrite(out_dir_ + "camera/raw/cam_r" + std::to_string(i) + ".png", prev_im[R]);
				bs::imwrite(out_dir_ + "camera/cam_flow_l" + std::to_string(i) + ".png", flow_view_im[L], view_sz * 2);
				bs::imwrite(out_dir_ + "camera/cam_flow_r" + std::to_string(i) + ".png", flow_view_im[R], view_sz * 2);
			}
			next_im[L].copyTo(prev_im[L]);
			next_im[R].copyTo(prev_im[R]);
		}
		return true;
	}

	inline Stereo<std::vector<std::vector<cv::Point2f>>> OpticalFlowApp::getPoints() const
	{
		return points_;
	}

	inline Stereo<std::vector<cv::Point2f>> OpticalFlowApp::getPointsAtOnePixel(const size_t index) const
	{
		Stereo<std::vector<cv::Point2f>> points, output;

		points[L].resize(numof_computings_);
		points[R].resize(numof_computings_);

		for (size_t i = 0; i < numof_computings_; ++i)
		{
			points[L][i] = points_[L][i][index];
			points[R][i] = points_[R][i][index];
		}
		return points;
	}

	inline Stereo<std::vector<cv::Point2f>> OpticalFlowApp::getPointsAtOneComputing(const size_t index) const
	{
		return bs::make_Stereo(points_[L][index], points_[R][index]);
	}

	void rmBadFlow(const Stereo<std::vector<std::vector<cv::Point2f>>>& src, Stereo<std::vector<std::vector<cv::Point2f>>>& dst)
	{
		const size_t numof_computings = src[L].size();
		const Stereo<size_t> numof_pixels = { src[L][0].size(), src[R][0].size() };
		size_t numof_good_flow = 0;

		for (size_t lr = 0; lr < 2; ++lr)
		{
			std::vector<size_t> lut;

			for (size_t j = 0; j < numof_pixels[lr]; ++j)
			{
				std::vector<cv::Point2f> flow(numof_computings);

				for (size_t k = 0; k < numof_computings; ++k)
					flow[k] = src[lr][k][j];

				if (bs__isGoodFlow(flow, 30.0))
					lut.push_back(j);
			}
			dst[lr].resize(numof_computings);

			for (auto& d : dst[lr])
				d.resize(lut.size());

			for (size_t j = 0; j < numof_computings; ++j)
			{
				for (size_t k = 0; k < lut.size(); ++k)
					dst[lr][j][k] = src[lr][j][lut[k]];
			}
		}
		return;
	}

	void integrateFlow(const Stereo<std::vector<std::vector<cv::Point2f>>>& pixel_set, Stereo<std::vector<OpticalFlow>>& dst_flow)
	{
		assert(pixel_set[L].size() == pixel_set[R].size());

		const Stereo<size_t> numof_pixels = { pixel_set[L][0].size(), pixel_set[R][0].size() };
		const size_t numof_computings = pixel_set[L].size();
		Stereo<std::vector<std::pair<cv::Point2f, cv::Point2f>>> each_points;

		for (size_t lr = 0; lr < 2; ++lr)
		{
			dst_flow[lr].resize(numof_pixels[lr]);

			for (size_t i = 0; i < numof_pixels[lr]; ++i)
			{
				cv::Point2f from, to;
				double x, y, mag, ang;

				from = pixel_set[lr][0][i];
				to = pixel_set[lr][numof_computings - 1][i];
				x = to.x - from.x;
				y = to.y - from.y;
				mag = std::sqrt(x * x + y * y);
				ang = std::atan2(y, x);
				ang *= (180.0 / std::_Pi);	// rad -> deg

				dst_flow[lr][i].point = from;
				dst_flow[lr][i].magnitude = mag;
				dst_flow[lr][i].angle = ang;
			}
		}
		return;
	}

	std::vector<OpticalFlow> searchEpipolar(const OpticalFlow& src_flow, const std::vector<OpticalFlow>& target_flow_vec)
	{
		const cv::Point p0 = src_flow.point;
		std::vector<OpticalFlow> epipolar_flows;
		epipolar_flows.reserve(target_flow_vec.size());

		for (const auto& flow : target_flow_vec)
		{
			cv::Point p = cv::Point(flow.point);
			if (p0.y == p.y)
				epipolar_flows.push_back(flow);
		}
		return epipolar_flows;
	}

}


bool bs__isGoodFlow(const std::vector<cv::Point2f>& flow, const double ang_th)
{
	double prev_ang = 400.;

	for (auto it = flow.cbegin() + 1; it != flow.cend(); ++it)
	{
		const double x = it->x - (it - 1)->x;
		const double y = it->y - (it - 1)->y;

		double ang = std::atan2(y, x);
		ang *= (180.0 / std::_Pi);	// radian to degree

		double tmp = ang - prev_ang;
		double ang_diff = 180 < tmp ? tmp - 360.0 : tmp < -180 ? tmp + 360.0 : tmp;

		if (prev_ang <= 360 && ang_th < ang_diff)
		{
			std::cout << std::endl << "Bad angles (" << prev_ang << " -> " << ang << ")" << std::endl;
			return false;
		}
		prev_ang = ang;
	}
	return true;
}
