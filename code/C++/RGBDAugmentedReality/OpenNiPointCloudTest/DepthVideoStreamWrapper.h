#ifndef _DEPTHVIDEOSTREAMWRAPPER_H_
#define _DEPTHVIDEOSTREAMWRAPPER_H_

#include <OpenNI.h>
#include <boost/thread/mutex.hpp>
#include <opencv2/opencv.hpp>

class DepthVideoStreamWrapper {
public:
	DepthVideoStreamWrapper();
	~DepthVideoStreamWrapper();
	bool create(openni::Device& device);
	bool hasNewImage() const;
	void copyNextImageTo(cv::Mat& outImage);
	int getMinDepth() const;
	int getMaxDepth() const;

private:
	class FrameListener;
	FrameListener* _listener;
	openni::VideoStream _stream;
	openni::DepthPixel* _rawPixels;
	bool _isNew;
	bool _initialized;
	int _minDepth;
	int _maxDepth;
	boost::mutex _lock;

	friend class FrameListener;

	static const int FRAME_WIDTH = 640;
	static const int FRAME_HEIGHT = 480;
};

#endif