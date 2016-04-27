#include "MainApp.h"
#include <GL/glut.h>
using namespace algebra3;

MainApp::MainApp() {
}

MainApp::~MainApp() {
}

void MainApp::loadParamsFromFile(const char* filename) {
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened()) {
		cout << "The params file could not be opened." << endl;
		exit(-1);
	}
	
	fs["K1"] >> _K[0];
	fs["K2"] >> _K[1];
	fs["D1"] >> _D[0];
	fs["D2"] >> _D[1];
	fs["RT"] >> _RT;

	fs.release();

	cv::Mat I = cv::Mat::eye(3, 3, CV_32FC1);
	cv::Size winSize(640, 480);
	cv::initUndistortRectifyMap(_K[0], _D[0], I, _K[0], winSize, CV_32FC1, _distMap[0][0], _distMap[0][1]);
	//cv::initUndistortRectifyMap(_K[1], _D[1], I, _K[1], winSize, CV_32FC1, _distMap[1][0], _distMap[1][1]);

	fs.release();
}

void MainApp::initCamera() {
	if (openni::OpenNI::initialize() != 0) {
		cout << "OpenNI could not be initialized." << endl;
		exit(-1);
	}

	openni::Array<openni::DeviceInfo> devInfoList;
	openni::OpenNI::enumerateDevices(&devInfoList);

	if (devInfoList.getSize() < 1) {
		cout << "Could not find two cameras. Application will now exit." << endl;
		exit(-1);
	}

	for (int i = 0; i < 1; i++) {
		if (_device[i].open(devInfoList[i].getUri()) != 0) {
			cout << "The camera could not be opened." << endl;
			exit(-1);
		}

		if (!_colorStream[i].create(_device[i])) {
			cout << "The color stream could not be opened." << endl;
			exit(-1);
		}

		if (!_depthStream[i].create(_device[i])) {
			cout << "The depth stream could not be opened." << endl;
			exit(-1);
		}

		if (_device[i].setDepthColorSyncEnabled(true) != 0) {
			cout << "The color/depth sync could not be enabled." << endl;
			exit(-1);
		}

		if (_device[i].setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR) != 0) {
			cout << "The image registration mode could not be set." << endl;
			exit(-1);
		}
	}
}

void MainApp::initBuffer() {
	_numPoints = 640 * 480;
	_pointDataLen = 6 * _numPoints;
	_pointData[0] = new GLfloat[_pointDataLen];
	//_pointData[1] = new GLfloat[_pointDataLen];

	/*
	int k = 0;
	for (int i = 0; i < 480; i++) {
		for (int j = 0; j < 640; j++) {
			_pointData[0][k]	= j + 0.5f;
			_pointData[0][k+1]  = i + 0.5f;
			k += 6;
		}
	}

	k = 0;
	for (int i = 0; i < 480; i++) {
		for (int j = 0; j < 640; j++) {
			_pointData[1][k]	= j + 0.5f;
			_pointData[1][k+1]  = i + 0.5f;
			k += 6;
		}
	}
	*/
}

void MainApp::initGL() {
	if (glewInit() != 0) {
		cout << "GLEW could not be initialized." << endl;
		exit(-1);
	}
	_program = createShaderProgram(
		"../../Shaders/point_cloud.vert",
		"../../Shaders/point_cloud.frag");

	_modelview[0] = scaling3D(vec3(1.0, -1.0, -1.0));
	//_modelview[1] = scaling3D(vec3(1.0, -1.0, -1.0)) *
	//	mat4(  vec4(_RT.at<double>(0,0), _RT.at<double>(0,1), _RT.at<double>(0,2), _RT.at<double>(0,3)),
	//		   vec4(_RT.at<double>(1,0), _RT.at<double>(1,1), _RT.at<double>(1,2), _RT.at<double>(1,3)),
	//		   vec4(_RT.at<double>(2,0), _RT.at<double>(2,1), _RT.at<double>(2,2), _RT.at<double>(2,3)),
	//		   vec4(_RT.at<double>(3,0), _RT.at<double>(3,1), _RT.at<double>(3,2), _RT.at<double>(3,3)));
	
	_projection = getPerspectiveMatrix(60.0, 1.0, 300.0, 10000.0) * translation3D(vec3(0, -70, -100));
	
	this->initBuffer();

	setUniformMat4(_program, "projection", _projection);
	setUniformFloat(_program, "depthScale", 1.f);

	glClearColor(0, 0, 0, 1);
	glDisable(GL_CULL_FACE);
	glEnable(GL_ALPHA_TEST);
}

void MainApp::timer(int t) {
	this->acquirePoints();
	glutTimerFunc(30, timerGL, t+1);
	glutPostRedisplay();
}

void MainApp::display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	GLuint buff;
	glGenBuffers(1, &buff);
	glBindBuffer(GL_ARRAY_BUFFER, buff);

	int positionIndex = glGetAttribLocation(_program, "vPosition");
	dieOnInvalidIndex(positionIndex, "vPosition");

	int colorIndex = glGetAttribLocation(_program, "vColor");
	dieOnInvalidIndex(colorIndex, "vColor");

	glEnableVertexAttribArray(positionIndex);
	glVertexAttribPointer(positionIndex, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), BUFFER_OFFSET(0));
	
	glEnableVertexAttribArray(colorIndex);
	glVertexAttribPointer(colorIndex, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), BUFFER_OFFSET(3*sizeof(GLfloat)));

	glBufferData(GL_ARRAY_BUFFER, _pointDataLen*sizeof(GLfloat), _pointData[0], GL_STATIC_DRAW);
	setUniformMat4(_program, "modelview", _modelview[0]);
	setUniformVec4(_program, "K", vec4(_K[0].at<double>(0,0), _K[0].at<double>(1,1), _K[0].at<double>(0,2), _K[0].at<double>(1,2)));
	glDrawArrays(GL_POINTS, 0, _numPoints);

	/*glBufferData(GL_ARRAY_BUFFER, _pointDataLen*sizeof(GLfloat), _pointData[1], GL_STATIC_DRAW);
	setUniformMat4(_program, "modelview", _modelview[1]);
	setUniformVec4(_program, "K", vec4(_K[1].at<double>(0,0), _K[1].at<double>(1,1), _K[1].at<double>(0,2), _K[1].at<double>(1,2)));
	glDrawArrays(GL_POINTS, 0, _numPoints);*/

	glDisableVertexAttribArray(positionIndex);
	glDisableVertexAttribArray(colorIndex);
	glDeleteBuffers(1, &buff);

	glFlush();
	glutSwapBuffers();
}

void MainApp::reshape(int w, int h) {
	glViewport(0, 0, w, h);
	_projection = getPerspectiveMatrix(60.0, (1.0*w)/h, 300.0, 10000.0);
	glutPostRedisplay();
}

void MainApp::acquirePoints() {
	cv::Mat colorImage[2], depthImage[2], undistortedColor[2], undistortedDepth[2];
	for (int i = 0; i < 1; i++) {
		colorImage[i].create(480, 640, CV_8UC3);
		_colorStream[i].copyNextImageTo(colorImage[i]);
		depthImage[i].create(480, 640, CV_32FC1);
		_depthStream[i].copyNextImageTo(depthImage[i]);
	}

	for (int i = 0; i < 1; i++) {
		cv::remap(colorImage[i], undistortedColor[i], _distMap[i][0], _distMap[i][1], cv::INTER_LINEAR);
		cv::remap(depthImage[i], undistortedDepth[i], _distMap[i][0], _distMap[i][1], cv::INTER_NEAREST);
	}

	int k = 0;
	for (int i = 0; i < 480; i++) {
		for (int j = 0; j < 640; j++) {
			_pointData[0][k] = j + 0.5f;
			_pointData[0][k+1] = i + 0.5f;
			_pointData[0][k+2] = undistortedDepth[0].at<float>(i,j);
			auto c = undistortedColor[0].at<cv::Vec3b>(i,j);
			_pointData[0][k+3] = c[0];
			_pointData[0][k+4] = c[1];
			_pointData[0][k+5] = c[2];
			k += 6;
		}
	}

	k = 0;
	for (int i = 0; i < 480; i++) {
		for (int j = 0; j < 640; j++) {
			_pointData[1][k] = j + 0.5f;
			_pointData[1][k+1] = i + 0.5f;
			_pointData[1][k+2] = undistortedDepth[1].at<float>(i,j);
			auto c = undistortedColor[1].at<cv::Vec3b>(i,j);
			_pointData[1][k+3] = c[0];
			_pointData[1][k+4] = c[1];
			_pointData[1][k+5] = c[2];
			k += 6;
		}
	}

	glutPostRedisplay();
}

void MainApp::runApp(int argc, char** argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glutInitWindowPosition(50, 50);
	glutInitWindowSize(800, 800);
	glutCreateWindow("Point Cloud Visualizer");
	this->loadParamsFromFile("../../Data/asus_rig_params.xml");
	this->initGL();
	this->initCamera();
	glutDisplayFunc(displayGL);
	glutReshapeFunc(reshapeGL);
	//glutIdleFunc(idleGL);
	glutTimerFunc(2000, timerGL, 0);
	glutMainLoop();
	return;
}