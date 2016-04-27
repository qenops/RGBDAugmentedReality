#ifndef _MAINAPP_H_
#define _MAINAPP_H_

#include <GLHelper.h>
#include <OpenNI.h>
#include "ColorVideoStreamWrapper.h"
#include "DepthVideoStreamWrapper.h"
#include <opencv2/opencv.hpp>

class MainApp {

public:
	MainApp();
	~MainApp();
	void runApp(int argc, char** argv);

private:
	void timer(int t);
	void display();
	void reshape(int w, int h);
	void acquirePoints();
	void loadParamsFromFile(const char* filename);
	void initGL();
	void initBuffer();
	void initCamera();

	// GL variables
	GLuint _program;
	algebra3::mat4 _modelview[2];
	algebra3::mat4 _projection;
	GLfloat* _pointData[2];
	GLuint _pointDataLen;
	GLuint _numPoints;

	// OpenNI variables
	openni::Device _device[2];
	ColorVideoStreamWrapper _colorStream[2];
	DepthVideoStreamWrapper _depthStream[2];

	// OpenCV variables
	cv::Mat _distMap[2][2];
	cv::Mat _K[2], _D[2], _RT;

	friend void timerGL(int t);
	friend void displayGL();
	friend void reshapeGL(int w, int h);
	friend void idleGL();
};

#endif