#include "ONIListener.h"

#undef min
#undef max

#include "ONI3DConverter.h"


ONIListener::ONIListener()
	: pConverter_(NULL)
{
}

ONIListener::~ONIListener()
{
}

void ONIListener::onNewFrame( openni::VideoStream &vs )
{
	openni::VideoFrameRef frame;
	openni::Status rs = vs.readFrame(&frame);
	int sz = frame.getDataSize();
	int frameIndex = frame.getFrameIndex();
	int height = frame.getHeight();
	int width = frame.getWidth();
	openni::SensorType sensorType = frame.getSensorType();


	int stride = frame.getStrideInBytes();
	uint64_t timeStamp = frame.getTimestamp();
	const openni::VideoMode &videoMode = frame.getVideoMode();
	openni::PixelFormat pixelFormat = videoMode.getPixelFormat();

	int fps = videoMode.getFps();
	int resX = videoMode.getResolutionX();
	int resY = videoMode.getResolutionY();
/*
	char str[1024];
	sprintf_s(str, 1024, "%s: frame = %d  timestamp = %ld\n",
		(sensorType == openni::SENSOR_COLOR ? "Color" : "Depth"),
		frameIndex,
		timeStamp);
	OutputDebugStringA(str);*/
	
	if(pConverter_ != NULL)
	{
		if(sensorType == openni::SENSOR_COLOR)
			pConverter_->newColorFrame(frameIndex, width, height, stride, sz, frame.getData(), &vs);
		else if(sensorType == openni::SENSOR_DEPTH)
			pConverter_->newDepthFrame(frameIndex, width, height, stride, sz, frame.getData(), &vs);
	}
}

void ONIListener::setConverter(ONI3DConverter *pConverter)
{
	pConverter_ = pConverter;
}

