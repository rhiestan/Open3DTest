#ifndef OPEN3DICPVISUALIZER_H
#define OPEN3DICPVISUALIZER_H

#include "StitcherI.h"

#include "open3d/Open3D.h"

#include <vector>
#include <mutex>

#include <Eigen/StdVector>

/*
 *
 */
class Open3DICPVisualizer : public StitcherI
{
public:
	Open3DICPVisualizer();
	virtual ~Open3DICPVisualizer();

	virtual void setup();

	virtual void addNewImage(const open3d::geometry::Image& colorImg, const open3d::geometry::Image& depthImg);

	virtual void saveVolume();

	virtual void reset();

	virtual void setDepthScale(double depthScale) { depthScale_ = depthScale; }

private:
	std::mutex mutex_;

	double depthScale_{ 1000.0 };

	open3d::geometry::RGBDImage oldRGBDImage_;

	Eigen::Matrix4d pos_;

	std::unique_ptr<open3d::pipelines::integration::ScalableTSDFVolume> volume_;
};

#endif
