#ifndef _IRVIDEOSTREAMWRAPPER_H_
#define _IRVIDEOSTREAMWRAPPER_H_

#include <opencv2/opencv.hpp>
#include <OpenNI.h>

class IRVideoStreamWrapper {
public:
	IRVideoStreamWrapper();
	~IRVideoStreamWrapper();
	bool create(openni::Device& device);
	bool hasNewImage() const;
	void copyNextImageTo(cv::Mat& m);

private:
	class FrameListener;
	FrameListener* _listener;
	openni::VideoStream _stream;
	cv::Mat _frame;
	bool _isNew;
	bool _initialized;

	friend class FrameListener;
};

#endif