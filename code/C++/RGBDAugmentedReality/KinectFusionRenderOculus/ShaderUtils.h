#ifndef SHADERUTILS_H
#define SHADERUTILS_H

// Include GLFW
#include <glfw3.h>

class ShaderUtils
{
public:
	static GLuint loadShaders(const char * vertex_file_path, const char * fragment_file_path);
};
#endif

