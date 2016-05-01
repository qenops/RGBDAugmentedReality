#ifndef OVR_TEXTURE
#define OVR_TEXTURE

// Include GLEW
#include "GL/CAPI_GLE.h"

#include "Kernel/OVR_System.h"

// Include the Oculus SDK
#include "OVR_CAPI_GL.h"

#include "OVR_Math.h"



#include "OVRDepthBuffer.h"

//using namespace OVR;

class OVRTextureBuffer
{
	public:
		OVRTextureBuffer() {}
		OVRTextureBuffer(ovrHmd hmd, bool rendertarget, bool displayableOnHmd, OVR::Sizei size, int mipLevels, unsigned char * data, int sampleCount);
		~OVRTextureBuffer();

		void SetAndClearRenderSurface(OVRDepthBuffer* dbuffer);

		void UnsetRenderSurface();

		OVR::Sizei GetSize() const
		{
			return texSize;
		}

		ovrSwapTextureSet* getTextureSet() { return TextureSet; }

	private:
		ovrHmd              hmd;
		ovrSwapTextureSet*  TextureSet;
		GLuint              texId;
		GLuint              fboId;
		OVR::Sizei               texSize;
};

#endif