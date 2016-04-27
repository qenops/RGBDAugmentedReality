#ifndef _COLORVIDEOSTREAMWRAPPER_H_
#define _COLORVIDEOSTREAMWRAPPER_H_

#include <OpenNI.h>
#include <boost/thread/mutex.hpp>
#include <opencv2/opencv.hpp>

class ColorVideoStreamWrapper {
public:
	ColorVideoStreamWrapper();
	~ColorVideoStreamWrapper();
	bool create(openni::Device& device);
	bool hasNewImage() const;
	void copyNextImageTo(cv::Mat& outImage);

private:
	class FrameListener;
	FrameListener* _listener;
	openni::VideoStream _stream;
	openni::RGB888Pixel* _rawPixels;
	bool _isNew;
	bool _initialized;
	boost::mutex _lock;

	friend class FrameListener;

	static const int FRAME_WIDTH = 640;
	static const int FRAME_HEIGHT = 480;
};

#endif