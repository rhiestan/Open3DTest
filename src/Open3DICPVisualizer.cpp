
#include "Open3DICPVisualizer.h"

#include <iostream>
#include <sstream>
#include <limits>

#include <Eigen/LU>
#include <Eigen/Geometry>

Open3DICPVisualizer::Open3DICPVisualizer()
{
}

Open3DICPVisualizer::~Open3DICPVisualizer()
{

}

void Open3DICPVisualizer::setup()
{
	volume_ = std::make_unique<open3d::pipelines::integration::ScalableTSDFVolume>(4.0 / 512, 0.04, open3d::pipelines::integration::TSDFVolumeColorType::RGB8);
}

void Open3DICPVisualizer::addNewImage(const open3d::geometry::Image& colorImg, const open3d::geometry::Image& depthImg)
{
	std::unique_lock<std::mutex> lock(mutex_);

	open3d::camera::PinholeCameraIntrinsic intrinsic = open3d::camera::PinholeCameraIntrinsic(
		open3d::camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);
	//intrinsic.SetIntrinsics(640, 480, 524.0, 524.0, 316.7, 238.5);	// from https://www.researchgate.net/figure/ntrinsic-parameters-of-Kinect-RGB-camera_tbl2_305108995
	//intrinsic.SetIntrinsics(640, 480, 517.3, 516.5, 318.6, 255.3);		// from Freiburg test data set
	//intrinsic.SetIntrinsics(640, 480, 537.408, 537.40877, 321.897, 236.29);		// from Calibration of my own camera
	//intrinsic.SetIntrinsics(640, 480, 533.82, 533.82, 320.55, 232.35);		// from Calibration of my own camera
	intrinsic.SetIntrinsics(640, 480, 542.7693, 544.396, 318.79, 239.99);		// from Calibration of my own camera

	Eigen::Matrix4d odo_init = Eigen::Matrix4d::Identity();
	auto depthFlt = depthImg.ConvertDepthToFloatImage(depthScale_, std::numeric_limits<float>::max());

	/*{
		float *ptr = depthFlt->PointerAt<float>(0, 0);
		int size = depthImg.width_ * depthImg.height_;
		float maxVal{ std::numeric_limits<float>::lowest() }, minVal{ std::numeric_limits<float>::max() };
		for (int i = 0; i < size; i++)
		{
			const float& val = *ptr;
			maxVal = std::max(maxVal, val);
			minVal = std::min(minVal, val);
			ptr++;
		}

		std::cout << "Min: " << minVal << ", max: " << maxVal << std::endl;
	}*/

	//open3d::geometry::RGBDImage source(colorImg, *depthFlt);

	auto source = open3d::geometry::RGBDImage::CreateFromColorAndDepth(colorImg, depthImg, depthScale_, 4.0,
		false);

	if(false)
	{
		open3d::visualization::DrawGeometries({ source }, "RGBD", depthFlt->width_ * 2,
			depthFlt->height_);
	}

	if (!oldRGBDImage_.IsEmpty())
	{
		std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d> rgbd_odo =
			open3d::pipelines::odometry::ComputeRGBDOdometry(
				oldRGBDImage_, *source, intrinsic, odo_init,
				open3d::pipelines::odometry::RGBDOdometryJacobianFromHybridTerm(),
				open3d::pipelines::odometry::OdometryOption());

		auto ptCloudOld = open3d::geometry::PointCloud::CreateFromDepthImage(oldRGBDImage_.depth_,
			intrinsic, Eigen::Matrix4d::Identity(), depthScale_);
		auto ptCloudNew = open3d::geometry::PointCloud::CreateFromDepthImage(source->depth_,
			intrinsic, Eigen::Matrix4d::Identity(), depthScale_);
		ptCloudOld->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30), true);
		ptCloudNew->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30), true);

		double maxCorrDistance = 0.2;
		std::shared_ptr<open3d::pipelines::registration::RobustKernel> kernel = std::make_shared<open3d::pipelines::registration::TukeyLoss>(0.1);
		open3d::pipelines::registration::TransformationEstimationPointToPlane p2pl(kernel);
		auto icpResult = open3d::pipelines::registration::RegistrationICP(*ptCloudOld, *ptCloudNew,
			maxCorrDistance, Eigen::Matrix4d::Identity(), p2pl);

		{
			auto srcCopy = std::make_shared< open3d::geometry::PointCloud>(*ptCloudOld);
			auto dstCopy = std::make_shared< open3d::geometry::PointCloud>(*ptCloudNew);
			srcCopy->PaintUniformColor({ 1, 0.706, 0 });
			dstCopy->PaintUniformColor({ 0, 0.651, 0.929 });

			Eigen::Matrix4d transInv = icpResult.transformation_.inverse();

			dstCopy->Transform(transInv);
			open3d::visualization::DrawGeometries({ srcCopy, dstCopy }, "ICP Result", 1600, 900);
		}

		std::cout << "Matching ";
		if (std::get<0>(rgbd_odo))
			std::cout << "successful";
		else
			std::cout << "unsuccessful";
		std::cout << std::endl;
		//std::cout << " " << std::get<1>(rgbd_odo) << std::endl;

		if (std::get<0>(rgbd_odo))
		{
			//Eigen::Matrix4d extrinsic = std::get<1>(rgbd_odo);
			Eigen::Matrix4d extrinsic = icpResult.transformation_;

			pos_ = extrinsic * pos_;
			Eigen::Matrix4d posInv = pos_.inverse();

			Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();

			volume_->Integrate(*source, intrinsic, pos_);

			auto mesh_ptr = volume_->ExtractTriangleMesh();
			open3d::visualization::DrawGeometries({ mesh_ptr }, "Mesh", 1600, 900);
		}

	}
	else
	{
		Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
		pos_ = identity;
		volume_->Integrate(*source, intrinsic, identity);
		auto mesh_ptr = volume_->ExtractTriangleMesh();
		open3d::visualization::DrawGeometries({ mesh_ptr }, "Mesh", 1600, 900);

	}

	oldRGBDImage_ = *source;
}

void Open3DICPVisualizer::saveVolume()
{
}

void Open3DICPVisualizer::reset()
{
	std::unique_lock<std::mutex> lock(mutex_);

	/*{
		std::ostringstream ostr;
		ostr << "mesh_online_" << variant_ << ".ply";
		auto mesh = volume_->ExtractTriangleMesh();
		open3d::io::WriteTriangleMesh(ostr.str(),
			*mesh);
	}*/

	std::cout << "Reset" << std::endl;


	oldRGBDImage_.Clear();
	pos_ = Eigen::Matrix4d::Identity();

	setup();
}