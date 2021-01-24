#ifndef ONILISTENER_H
#define ONILISTENER_H

class ONI3DConverter;

// Workaround for MinGW
#if defined(_WIN32) && !defined(_MSC_VER)
#	define _MSC_VER 1300
#endif
#include <OpenNI.h>

class ONIListener: public openni::VideoStream::NewFrameListener
{
public:
	ONIListener();
	virtual ~ONIListener();

	virtual void onNewFrame( openni::VideoStream &vs );

	void setConverter(ONI3DConverter *pConverter);

private:
	ONI3DConverter *pConverter_;
};

#endif
