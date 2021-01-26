// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>

#include <boost/filesystem.hpp>

#include "open3d/Open3D.h"

#include "ONIDevice.h"
#include "ONI3DConverter.h"
#include "Stitcher.h"
#include "OpenCVStitcher.h"
#include "OpenCVCameraCalibrator.h"
#include "SaveStreamToDisk.h"
#include "Open3DICPVisualizer.h"

#include <opencv2/imgcodecs.hpp>

std::vector<std::string> rgbFilenames
{
	"1305031790.645155.png",
	"1305031790.681208.png",
	"1305031790.713097.png",
	"1305031790.745223.png",
	"1305031790.781258.png",
	"1305031790.813207.png",
	"1305031790.845151.png",
	"1305031790.882094.png",
	"1305031790.913129.png",
	"1305031790.945171.png",
	"1305031790.981159.png",
	"1305031791.013075.png",
	"1305031791.045137.png",
	"1305031791.081293.png",
	"1305031791.113136.png",
	"1305031791.145096.png",
	"1305031791.181287.png",
	"1305031791.213098.png",
	"1305031791.245087.png",
	"1305031791.281340.png",
	"1305031791.313088.png",
	"1305031791.345123.png",
	"1305031791.382161.png",
	"1305031791.413087.png",
	"1305031791.445165.png",
	"1305031791.481375.png",
	"1305031791.513084.png",
	"1305031791.545065.png",
	"1305031791.581127.png",
	"1305031791.613194.png",
	"1305031791.645074.png",
};

std::vector<std::string> depthFilenames
{
	"1305031790.640468.png",
	"1305031790.672866.png",
	"1305031790.709421.png",
	"1305031790.739530.png",
	"1305031790.773548.png",
	"1305031790.809521.png",
	"1305031790.839363.png",
	"1305031790.871606.png",
	"1305031790.909436.png",
	"1305031790.941495.png",
	"1305031790.973696.png",
	"1305031791.009486.png",
	"1305031791.041197.png",
	"1305031791.073525.png",
	"1305031791.109452.png",
	"1305031791.141330.png",
	"1305031791.173227.png",
	"1305031791.209506.png",
	"1305031791.241091.png",
	"1305031791.273558.png",
	"1305031791.309764.png",
	"1305031791.341415.png",
	"1305031791.373525.png",
	"1305031791.409609.png",
	"1305031791.441538.png",
	"1305031791.473230.png",
	"1305031791.509595.png",
	"1305031791.541450.png",
	"1305031791.574034.png",
	"1305031791.609401.png",
	"1305031791.641229.png",
	"1305031791.677621.png",
	"1305031791.709275.png",
	"1305031791.741364.png",
	"1305031791.776668.png",
	"1305031791.808429.png",
};



void scanFolder()
{
	//Open3DICPVisualizer stitcher;
	Stitcher stitcher;
	stitcher.setup();
	stitcher.setDepthScale(1000.0);
	//stitcher.setCameraMatrix();

	cv::Mat color, depth;
	open3d::geometry::Image colorImg, depthImg;

	boost::filesystem::path colorPath("F:/Projects/libs/Open3D/testdata/rgbd_dataset_freiburg1_360/rgb");
	boost::filesystem::path depthPath("F:/Projects/libs/Open3D/testdata/rgbd_dataset_freiburg1_360/depth/");
	//boost::filesystem::path colorPath("F:/Projects/libs/Open3D/testdata/meins_2pic/rgb");
	//boost::filesystem::path depthPath("F:/Projects/libs/Open3D/testdata/meins_2pic/depth");

	boost::system::error_code ec;
	boost::filesystem::directory_iterator colorIter(colorPath, ec);
	if (ec)
	{
		std::cout << "Could not read directory " + colorPath.string() << std::endl;
		return;
	}
	boost::filesystem::directory_iterator depthIter(depthPath, ec);
	if (ec)
	{
		std::cout << "Could not read directory " + depthPath.string() << std::endl;
		return;
	}

	// Scan all files from the directories
	while (colorIter != boost::filesystem::directory_iterator()
		&& depthIter != boost::filesystem::directory_iterator())
	{
		auto colorEntry = *colorIter;
		auto depthEntry = *depthIter;

		// On Windows, this only works with ANSI paths
		color = cv::imread(colorEntry.path().string());
		depth = cv::imread(depthEntry.path().string(), cv::IMREAD_UNCHANGED);
		colorImg = OpenCVStitcher::convertCVToO3D(color);
		depthImg = OpenCVStitcher::convertCVToO3D(depth);
		stitcher.addNewImage(colorImg, depthImg);

		colorIter++;
		depthIter++;
	}
	/*for (int i = 0; i < 30; i++)
	{
		std::string filenameRGB = "F:/Projects/libs/Open3D/testdata/rgbd_dataset_freiburg1_360/rgb/" + rgbFilenames[i];
		std::string filenameDepth = "F:/Projects/libs/Open3D/testdata/rgbd_dataset_freiburg1_360/depth/" + depthFilenames[i];
		color = cv::imread(filenameRGB);
		depth = cv::imread(filenameDepth, cv::IMREAD_UNCHANGED);
		colorImg = OpenCVStitcher::convertCVToO3D(color);
		depthImg = OpenCVStitcher::convertCVToO3D(depth);
		stitcher.addNewImage(colorImg, depthImg);
	}*/

	stitcher.saveVolume();
}

void scanOpenNI()
{
	ONIDevice dev;

	bool isOk = dev.initializeOpenNI();
	if (!isOk)
		std::cout << dev.getLastErrorString() << std::endl;

	Stitcher stitcher;
	//OpenCVStitcher stitcher;
	//OpenCVCameraCalibrator stitcher;
	//SaveStreamToDisk stitcher;
	stitcher.setup();
	stitcher.setDepthScale(1000.0);

	if (dev.connectDevice())
	{
		ONI3DConverter conv;
		conv.setup(&stitcher);
		dev.setConverter(&conv);

		//for (int i = 0; i < 16; i++)
		{
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(10s);

			//stitcher.reset();
		}

		dev.pause();
		dev.disconnectDevice();
		conv.cleanup();

		stitcher.saveVolume();
	}
	dev.shutdownOpenNI();

}


// A simplified version of examples/cpp/Visualizer.cpp to demonstrate linking
// an external project to Open3D.
int main(int argc, char* argv[])
{
	//scanFolder();
	scanOpenNI();

/*	using namespace open3d;
	camera::PinholeCameraIntrinsic intrinsic = camera::PinholeCameraIntrinsic(
		camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);

*/
/*	color = cv::imread("F:/Projects/libs/Open3D/testdata/rgbd_dataset_freiburg1_360/rgb/1305031790.645155.png");
	depth = cv::imread("F:/Projects/libs/Open3D/testdata/rgbd_dataset_freiburg1_360/depth/1305031790.640468.png", cv::IMREAD_UNCHANGED);
	std::cout << color.cols << ", " << color.rows << ", " << color.depth() << ", " << color.channels() << std::endl;
	std::cout << depth.cols << ", " << depth.rows << ", " << depth.depth() << ", " << depth.channels() << std::endl;
	//std::cout << depth.cols << ", " << depth.rows << ", " << depth.depth() << ", " << depth.channels() << std::endl;
	colorImg = OpenCVStitcher::convertCVToO3D(color);
	depthImg = OpenCVStitcher::convertCVToO3D(depth);
	stitcher.addNewImage(colorImg, depthImg);
	color = cv::imread("F:/Projects/libs/Open3D/testdata/rgbd_dataset_freiburg1_360/rgb/1305031790.681208.png");
	depth = cv::imread("F:/Projects/libs/Open3D/testdata/rgbd_dataset_freiburg1_360/depth/1305031790.672866.png", cv::IMREAD_UNCHANGED);
	colorImg = OpenCVStitcher::convertCVToO3D(color);
	depthImg = OpenCVStitcher::convertCVToO3D(depth);
	stitcher.addNewImage(colorImg, depthImg);
	stitcher.saveVolume();*/

/*	utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
	if (argc < 3) {
		utility::LogInfo("Open3D {}\n", OPEN3D_VERSION);
		utility::LogInfo("\n");
		utility::LogInfo("Usage:\n");
		utility::LogInfo("    > TestVisualizer [mesh|pointcloud] [filename]\n");
		// CI will execute this file without input files, return 0 to pass
		return 0;
	}

	std::string option(argv[1]);
	if (option == "mesh") {
		auto mesh_ptr = std::make_shared<geometry::TriangleMesh>();
		if (io::ReadTriangleMesh(argv[2], *mesh_ptr)) {
			utility::LogInfo("Successfully read {}\n", argv[2]);
		}
		else {
			utility::LogWarning("Failed to read {}\n\n", argv[2]);
			return 1;
		}
		mesh_ptr->ComputeVertexNormals();
		visualization::DrawGeometries({ mesh_ptr }, "Mesh", 1600, 900);
	}
	else if (option == "pointcloud") {
		auto cloud_ptr = std::make_shared<geometry::PointCloud>();
		if (io::ReadPointCloud(argv[2], *cloud_ptr)) {
			utility::LogInfo("Successfully read {}\n", argv[2]);
		}
		else {
			utility::LogWarning("Failed to read {}\n\n", argv[2]);
			return 1;
		}
		cloud_ptr->NormalizeNormals();
		visualization::DrawGeometries({ cloud_ptr }, "PointCloud", 1600, 900);
	}
	else {
		utility::LogWarning("Unrecognized option: {}\n", option);
		return 1;
	}
	utility::LogInfo("End of the test.\n");*/


	return 0;
}
