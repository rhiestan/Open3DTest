
#include "StitcherI.h"

#include "open3d/Open3D.h"

#include <vector>
#include <mutex>

#include <Eigen/StdVector>

#include <opencv2/core.hpp>

/*
 *
 */
class OpenCVCameraCalibrator : public StitcherI
{
public:
	OpenCVCameraCalibrator();
	virtual ~OpenCVCameraCalibrator();

	virtual void setup();

	virtual void addNewImage(const open3d::geometry::Image& colorImg, const open3d::geometry::Image& depthImg);

	virtual void saveVolume();

	virtual void reset();

	virtual void setDepthScale(double depthScale) { };

private:
	std::mutex mutex_;

	std::vector<std::vector<cv::Point2f> > imagePoints_;
	cv::Size imageSize_;
};
