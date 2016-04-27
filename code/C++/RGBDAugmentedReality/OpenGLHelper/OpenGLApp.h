#pragma once
#include <string>
#include<vector>
#include "GLHelper.h"
#include <GL/glut.h>
using namespace std;
class OpenGLApp
{
public:
	OpenGLApp();
	~OpenGLApp();
	
	virtual bool Run(int argc, char* argv[]);
protected:
	bool initGL(float l, float r, float t, float b, float n, float f);
	bool initGLUT(int argc, char* argv[],string name,int windowWidth,int windowHeight);
	void SaveImageFromGL(string name);
	virtual void keyFunc(unsigned char key, int x, int y);
	virtual void display();
	void friend displayGL();
	void friend keyGL(unsigned char key, int x, int y);
	virtual bool loadShaders();
	struct RenderModelInfo
	{
		RenderObject* mesh;
		algebra3::mat4 modelTranform;
		vector<float> vertices;
		vector<float> normals;
		vector<float> colors;
	};
	
	algebra3::mat4 m_view;
	algebra3::mat4 m_projection;

};
