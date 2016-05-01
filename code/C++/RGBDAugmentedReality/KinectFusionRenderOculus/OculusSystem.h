#ifndef OCULUS_SYSTEM
#define OCULUS_SYSTEM
#include "GL/CAPI_GLE.h"
#include "Kernel/OVR_System.h"
// Include the Oculus SDK
#include "OVR_CAPI_GL.h"
#include "OVRTextureBuffer.h"
#include "OVRDepthBuffer.h"
#include "GLHelper.h"
class OculusSystem
{
	public:
	OculusSystem();
	~OculusSystem();
	void initialize();
	void shutdown();
	void destroy();
	bool loadBuffers();
	void Render();
	void Render(Texture& left,Texture& right);
	bool GetIdealRenderSize(OVR::Sizei& leftTextureSize, OVR::Sizei& rightTextureSize);
	bool GetEyePositionsFromHMD(OVR::Vector3f& leftEyeTrans, OVR::Vector3f& rightEyeTrans);
	bool GetEyeFOVFromHMD(ovrFovPort& leftEyeFov, ovrFovPort& rightEyeFov);
	GLuint fboId;
	OVR::GLEContext gleContext;

	ovrHmd HMD;
	ovrHmdDesc hmdDesc;
	bool m_isVisible;
	OVRTextureBuffer* eyeRenderTexture[2];
	OVRDepthBuffer* eyeDepthBuffer[2];
	ovrEyeRenderDesc EyeRenderDesc[2];

	ovrGLTexture* mirrorTexture;
	GLuint        mirrorFBO;

	int m_winWidth;
	int m_winHeight;
};
#endif