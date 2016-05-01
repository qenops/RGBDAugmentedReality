#include "OpenGLApp.h"
using namespace algebra3;
OpenGLApp::OpenGLApp()
{
}

OpenGLApp::~OpenGLApp()
{
}

bool OpenGLApp::Run(int argc, char * argv[])
{
	return false;
}

bool OpenGLApp::initGL(float l, float r,float t,float b,float n ,float f )
{
	int err = glewInit();
	if ((err != 0))
	{
		cout << "GLEW could not be initialized!" << endl;
		cout << "Error code is: " << err << std::endl;
		return false;
	}
	glDisable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_MULTISAMPLE_ARB);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glClearColor(0, 0, 0, 1); // Black*/
	m_projection = getPerspectiveMatrix( l,  r,  t,  b,  n,  f);
	m_view = identity3D();
	return true;
}

bool OpenGLApp::initGLUT(int argc, char* argv[], string name, int windowWidth, int windowHeight)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(windowWidth, windowHeight);
	glutCreateWindow(name.c_str());
	glutDisplayFunc(displayGL);
	glutKeyboardFunc(keyGL);
	return true;

}

void OpenGLApp::SaveImageFromGL(string name)
{
}

void OpenGLApp::keyFunc(unsigned char key, int x, int y)
{
}

void OpenGLApp::display()
{
}

bool OpenGLApp::loadShaders()
{
	return false;
}

