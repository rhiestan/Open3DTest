
#include "StitcherI.h"

#include "open3d/Open3D.h"

#include <vector>
#include <mutex>

#include <Eigen/StdVector>

#include <opencv2/rgbd.hpp>

/*
 *
 */
class OpenCVStitcher : public StitcherI
{
public:
	OpenCVStitcher();
	virtual ~OpenCVStitcher();

	virtual void setup();

	virtual void addNewImage(const open3d::geometry::Image& colorImg, const open3d::geometry::Image& depthImg);

	virtual void saveVolume();

	virtual void reset();

	virtual void setDepthScale(double depthScale) { depthScale_ = depthScale; }

	static open3d::geometry::Image convertCVToO3D(const cv::Mat& cvimg);

private:
	std::mutex mutex_;

	double depthScale_{ 1000.0 };

	open3d::geometry::RGBDImage oldRGBDImage_;

	Eigen::Matrix4d pos_;

	cv::Ptr<cv::dynafu::DynaFu> dynfu_;
	//cv::Ptr<cv::kinfu::KinFu> dynfu_;
	float depthFactor_{ 1.0f };

	std::vector<open3d::geometry::RGBDImage> images_;
	std::vector<Eigen::Matrix4d> posvec_, transvec_;
	std::vector<Eigen::Matrix6d> infovec_;
};
