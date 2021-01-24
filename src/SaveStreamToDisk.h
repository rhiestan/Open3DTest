
#ifndef SAVESTREAMTODISK_H
#define SAVESTREAMTODISK_H

#include "open3d/Open3D.h"

#include "StitcherI.h"

/*
 *
 */
class SaveStreamToDisk: public StitcherI
{
public:
	SaveStreamToDisk();
	virtual ~SaveStreamToDisk();

	virtual void setup();

	virtual void addNewImage(const open3d::geometry::Image& colorImg, const open3d::geometry::Image& depthImg);

	virtual void saveVolume();

	virtual void reset();

	virtual void setDepthScale(double depthScale);

private:
	int index_{ 0 };
};

#endif
