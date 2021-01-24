
#include "SaveStreamToDisk.h"

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <sstream>

SaveStreamToDisk::SaveStreamToDisk()
{

}
SaveStreamToDisk::~SaveStreamToDisk()
{

}

void SaveStreamToDisk::setup()
{
	index_ = 0;
}

void SaveStreamToDisk::addNewImage(const open3d::geometry::Image& colorImg, const open3d::geometry::Image& depthImg)
{
	cv::Mat colorI, colorO;
	colorI.create(colorImg.height_, colorImg.width_, CV_8UC3);
	unsigned char* cvPtr = colorI.ptr<unsigned char>(0);
	memcpy(cvPtr, &(colorImg.data_[0]), 3 * colorImg.height_ * colorImg.width_);
	cv::cvtColor(colorI, colorO, cv::COLOR_RGB2BGR);

	cv::Mat depth;
	depth.create(depthImg.height_, depthImg.width_, CV_16UC1);
	unsigned char* cvDPtr = depth.ptr<unsigned char>(0);
	memcpy(cvDPtr, &(depthImg.data_[0]), 2 * depthImg.height_ * depthImg.width_);

	std::ostringstream ostrC, ostrD;
	ostrC << "color_" << index_ << ".png";
	ostrD << "depth_" << index_ << ".png";
	std::string colorFN = ostrC.str();
	std::string depthFN = ostrD.str();
	cv::imwrite(colorFN, colorO);
	cv::imwrite(depthFN, depth);

	index_++;
}

void SaveStreamToDisk::saveVolume()
{

}

void SaveStreamToDisk::reset()
{
	index_ = 0;
}

void SaveStreamToDisk::setDepthScale(double depthScale)
{

}
