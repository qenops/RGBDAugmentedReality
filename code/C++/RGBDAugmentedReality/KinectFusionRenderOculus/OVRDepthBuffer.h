#ifndef OVR_DEPTH_BUFFER
#define OVR_DEPTH_BUFFER

// Include GLE
#include "GL/CAPI_GLE.h"

#include "OVR_CAPI_GL.h"


#include "OVR_Math.h"



//using namespace OVR;

class OVRDepthBuffer
{
	public:
		OVRDepthBuffer() {}
		OVRDepthBuffer(OVR::Sizei size, int sampleCount);
		~OVRDepthBuffer();
		GLuint getTextureId() { return texId;}

	private:
		GLuint        texId;
};
#endif

