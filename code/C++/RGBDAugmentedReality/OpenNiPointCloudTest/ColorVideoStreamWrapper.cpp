#include "ColorVideoStreamWrapper.h"
#include <iostream>
using namespace std;

class ColorVideoStreamWrapper::FrameListener:
	public openni::VideoStream::NewFrameListener {
private:
	ColorVideoStreamWrapper* _wrapper;
	
public:
	FrameListener(ColorVideoStreamWrapper* wrapper) {
		_wrapper = wrapper;
	}

	void onNewFrame(openni::VideoStream& stream) {
		openni::VideoFrameRef newFrame;
		stream.readFrame(&newFrame);
		auto colorBuffer = (const openni::RGB888Pixel*)(newFrame.getData());
		int w = newFrame.getWidth();
		int h = newFrame.getHeight();
		//_wrapper->_lock.lock();
		memcpy(_wrapper->_rawPixels, colorBuffer, w*h*sizeof(openni::RGB888Pixel));
		_wrapper->_isNew = true;
		//_wrapper->_lock.unlock();
	}
};

ColorVideoStreamWrapper::ColorVideoStreamWrapper() {
	_initialized = false;
	_isNew = false;
}

ColorVideoStreamWrapper::~ColorVideoStreamWrapper() {
	if (_initialized) {
		_stream.removeNewFrameListener(_listener);
		_stream.stop();
		_stream.destroy();
		delete _listener;
	}
}

bool ColorVideoStreamWrapper::create(openni::Device& device) {
	if (_stream.create(device, openni::SENSOR_COLOR) != 0) {
		return false;
	}
	auto videoMode = _stream.getVideoMode();
	videoMode.setFps(30);
	videoMode.setResolution(FRAME_WIDTH, FRAME_HEIGHT);
	videoMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
	_stream.setVideoMode(videoMode);

	if (_stream.start() != 0) {
		return false;
	}

	_listener = new FrameListener(this);
	_stream.addNewFrameListener(_listener);
	_initialized = true;
	_rawPixels = new openni::RGB888Pixel[FRAME_WIDTH * FRAME_HEIGHT];
	return true;
}

bool ColorVideoStreamWrapper::hasNewImage() const {
	return _isNew;
}

void ColorVideoStreamWrapper::copyNextImageTo(cv::Mat& outImage) {
	if (!_isNew) return;

	const int numPixels = FRAME_WIDTH * FRAME_HEIGHT;

	//_lock.lock();
	//outImage.create(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3);
	memcpy(outImage.data, _rawPixels, 3*numPixels*sizeof(uint8_t));
	_isNew = false;
	//_lock.unlock();
}

