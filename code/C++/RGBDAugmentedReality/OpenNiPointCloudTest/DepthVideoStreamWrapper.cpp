#include "DepthVideoStreamWrapper.h"
#include <iostream>
using namespace std;

class DepthVideoStreamWrapper::FrameListener:
	public openni::VideoStream::NewFrameListener {
private:
	DepthVideoStreamWrapper* _wrapper;
	
public:
	FrameListener(DepthVideoStreamWrapper* wrapper) {
		_wrapper = wrapper;
	}

	void onNewFrame(openni::VideoStream& stream) {
		openni::VideoFrameRef newFrame;
		stream.readFrame(&newFrame);
		auto depthBuffer = (const openni::DepthPixel*)(newFrame.getData());
		int w = newFrame.getWidth();
		int h = newFrame.getHeight();
		//_wrapper->_lock.lock();
		memcpy(_wrapper->_rawPixels, depthBuffer, w*h*sizeof(openni::DepthPixel));
		_wrapper->_isNew = true;
		//_wrapper->_lock.unlock();
	}
};

DepthVideoStreamWrapper::DepthVideoStreamWrapper() {
	_initialized = false;
	_isNew = false;
}

DepthVideoStreamWrapper::~DepthVideoStreamWrapper() {
	if (_initialized) {
		_stream.removeNewFrameListener(_listener);
		_stream.stop();
		_stream.destroy();
		delete _listener;
		delete _rawPixels;
	}
}

bool DepthVideoStreamWrapper::create(openni::Device& device) {
	if (_stream.create(device, openni::SENSOR_DEPTH) != 0) {
		return false;
	}
	auto videoMode = _stream.getVideoMode();
	videoMode.setFps(30);
	videoMode.setResolution(FRAME_WIDTH, FRAME_HEIGHT);
	videoMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
	
	_stream.setVideoMode(videoMode);
	if (_stream.start() != 0) {
		return false;
	}

	_listener = new FrameListener(this);
	_stream.addNewFrameListener(_listener);
	_minDepth = _stream.getMinPixelValue();
	_maxDepth = _stream.getMaxPixelValue();
	cout << "Min Depth: " << _minDepth << endl;
	cout << "Max Depth: " << _maxDepth << endl;
	_rawPixels = new openni::DepthPixel[640*480];
	_initialized = true;
	return true;
}

bool DepthVideoStreamWrapper::hasNewImage() const {
	return _isNew;
}

void DepthVideoStreamWrapper::copyNextImageTo(cv::Mat& outImage) {
	if (!_isNew) return;
	const int numPixels = FRAME_WIDTH * FRAME_HEIGHT;

	//_lock.lock();
	//outImage.create(FRAME_HEIGHT, FRAME_WIDTH, CV_32FC1);
	int k = 0;
	for (int i = 0; i < FRAME_HEIGHT; i++) {
		for (int j = 0; j < FRAME_WIDTH; j++) {
			outImage.at<float>(i,j) = _rawPixels[k++] * 1.f;
		}
	}
	_isNew = false;
	//_lock.unlock();
}

int DepthVideoStreamWrapper::getMinDepth() const {
	return _minDepth;
}

int DepthVideoStreamWrapper::getMaxDepth() const {
	return _maxDepth;
}
