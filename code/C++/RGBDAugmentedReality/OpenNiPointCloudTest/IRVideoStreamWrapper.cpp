#include "IRVideoStreamWrapper.h"
#include <iostream>
using namespace std;

class IRVideoStreamWrapper::FrameListener:
	public openni::VideoStream::NewFrameListener {
private:
	IRVideoStreamWrapper* _wrapper;
	
public:
	FrameListener(IRVideoStreamWrapper* wrapper) {
		_wrapper = wrapper;
	}

	void onNewFrame(openni::VideoStream& stream) {
		openni::VideoFrameRef newFrame;
		stream.readFrame(&newFrame);
		auto irBuffer = (const openni::Grayscale16Pixel*)(newFrame.getData());
		int w = newFrame.getWidth();
		int h = newFrame.getHeight();

		_wrapper->_frame.create(h, w, CV_8UC1);
		int k = 0;
		for (int i = 0; i < h; ++i) {
			for (int j = 0; j < w; ++j) {
				_wrapper->_frame.at<uint8_t>(i, j) = (uint8_t)(irBuffer[k++]);
			}
		}
		_wrapper->_isNew = true;
	}
};

IRVideoStreamWrapper::IRVideoStreamWrapper() {
	_initialized = false;
	_isNew = false;
}

IRVideoStreamWrapper::~IRVideoStreamWrapper() {
	if (_initialized) {
		_stream.removeNewFrameListener(_listener);
		_stream.stop();
		_stream.destroy();
		delete _listener;
	}
}

bool IRVideoStreamWrapper::create(openni::Device& device) {
	if (_stream.create(device, openni::SENSOR_IR) != 0) {
		return false;
	}
	auto videoMode = _stream.getVideoMode();
	videoMode.setFps(30);
	videoMode.setResolution(640, 480);
	videoMode.setPixelFormat(openni::PIXEL_FORMAT_GRAY16);
	_stream.setVideoMode(videoMode);

	if (_stream.start() != 0) {
		return false;
	}

	_listener = new FrameListener(this);
	_stream.addNewFrameListener(_listener);
	_initialized = true;
	return true;
}

bool IRVideoStreamWrapper::hasNewImage() const {
	return _isNew;
}

void IRVideoStreamWrapper::copyNextImageTo(cv::Mat& m) {
	if (_isNew) {
		_frame.copyTo(m);
	}
	/*
	openni::VideoFrameRef newFrame;
	_stream.readFrame(&newFrame);
	auto colorBuffer = (const openni::RGB888Pixel*)(newFrame.getData());
	int w = newFrame.getWidth();
	int h = newFrame.getHeight();

	_frame.create(h, w, CV_8UC3);
	memcpy(_frame.data, colorBuffer, 3*w*h*sizeof(uint8_t));
	cv::cvtColor(_frame, m, CV_BGR2RGB);
	*/
}

