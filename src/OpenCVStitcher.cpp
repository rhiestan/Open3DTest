
#include "OpenCVStitcher.h"

#include <iostream>
#include <sstream>
#include <cstdint>

#include <Eigen/LU>

#include <opencv2/imgcodecs.hpp>

OpenCVStitcher::OpenCVStitcher()
{
}

OpenCVStitcher::~OpenCVStitcher()
{

}

void OpenCVStitcher::setup()
{
	cv::setUseOptimized(true);

	cv::Ptr<cv::kinfu::Params> params = cv::kinfu::Params::defaultParams();
	params->volumeDims(0) = 128;
	params->volumeDims(1) = 128;
	params->volumeDims(2) = 128;
	params->depthFactor = 1000.0;
	depthFactor_ = params->depthFactor;

	dynfu_ = cv::dynafu::DynaFu::create(params);
	//dynfu_ = cv::kinfu::KinFu::create(params);
}

void OpenCVStitcher::addNewImage(const open3d::geometry::Image& colorImg, const open3d::geometry::Image& depthImg)
{
	std::unique_lock<std::mutex> lock(mutex_);

	cv::Mat depth, depthF;
	depth.create(depthImg.height_, depthImg.width_, CV_16UC1);
	std::int_fast16_t* cvPtr = depth.ptr<std::int_fast16_t>(0);
	unsigned char* dataPtr = depthImg.PointerAt<unsigned char>(0, 0, 0);
	memcpy(cvPtr, &(depthImg.data_[0]), 2 * depthImg.height_ * depthImg.width_);

	depth.convertTo(depthF, CV_32F);


	cv::UMat cvt8;
	convertScaleAbs(depthF, cvt8, 0.25 * 256. / depthFactor_);

	bool isOk = dynfu_->update(cvt8);

	if (isOk)
		std::cout << "Registering successful" << std::endl;
	else
		std::cout << "Registering not successful" << std::endl;
}

void OpenCVStitcher::saveVolume()
{
	std::unique_lock<std::mutex> lock(mutex_);

	//cv::Mat3d points, normals;
	cv::Mat points, normals;

	dynfu_->getCloud(points, normals);

	cv::Mat img;
	img.create(512, 512, CV_8UC3);
	dynfu_->render(img);
	cv::imwrite("render.png", img);
}

void OpenCVStitcher::reset()
{
	std::unique_lock<std::mutex> lock(mutex_);

}

open3d::geometry::Image OpenCVStitcher::convertCVToO3D(const cv::Mat& cvimg)
{
	open3d::geometry::Image img;
	if (cvimg.channels() == 3)
	{
		// Color
		img.Prepare(cvimg.cols, cvimg.rows, 3, 1);
		unsigned char* dataPtr = img.PointerAt<unsigned char>(0, 0, 0);
		memcpy(dataPtr, cvimg.ptr(), 3 * cvimg.cols * cvimg.rows);
	}
	else
	{
		// Depth
		//double minVal, maxVal;
		//cv::minMaxLoc(cvimg, &minVal, &maxVal);
		//std::cout << "Min: " << minVal << ", max: " << maxVal << std::endl;

		img.Prepare(cvimg.cols, cvimg.rows, 1, 2);
		unsigned char* dataPtr = img.PointerAt<unsigned char>(0, 0, 0);
		memcpy(dataPtr, cvimg.ptr(), 2 * cvimg.cols * cvimg.rows);
	}

	return img;
}
