#ifndef TEXTURELOADER_H
#define TEXTURELOADER_H

// Include GLEW
#include "GL/CAPI_GLE.h"

// Include GLFW
#include "glfw3.h"

//Include Image lib
#include "FreeImage.h"

// Include GLM
#include <glm.hpp>
#include <gtc\\type_ptr.hpp>
#include <gtc\\type_precision.hpp>

#include <algorithm>
#include <vector>

using namespace std;
using namespace glm;

struct BitmapData
{
	BitmapData() : width(0), height(0) {}

	unsigned int width;
	unsigned int height;
	float* pixelData;
	float* pixelDataToneMapped;
};

class TextureLoader
{
public:
	static GLuint loadTexture(const char* fileName);
	static void saveBitmap(const char* filename, BitmapData bitmapDesc);
};
#endif
