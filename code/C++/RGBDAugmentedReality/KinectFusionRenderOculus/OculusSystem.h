#ifndef OCULUS_SYSTEM
#define OCULUS_SYSTEM
// Include GLM

#include <glm.hpp>
#include "gtc\\matrix_transform.hpp"
#include "Scene.h"
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
	void Render(Texture & leftTex, Texture & rightTex, OVR::Matrix4f leftPose, OVR::Matrix4f rightPose, float deltaTime);
	bool GetIdealRenderSize(OVR::Sizei& leftTextureSize, OVR::Sizei& rightTextureSize);
	bool GetEyePositionsFromHMD(OVR::Vector3f& leftEyeTrans, OVR::Vector3f& rightEyeTrans);
	bool GetEyeFOVFromHMD(ovrFovPort& leftEyeFov, ovrFovPort& rightEyeFov);
	void setDynamicMeshesDataPath(const char* path) { dynamicMeshesDataPath = path; }
	void compileShaders();
	void InitScene();
	void setShaderPath(const char* path) { shaderPath = path; }

private:
	void moveMesh(OVR::Matrix4f & MeshMat);
	//bool initializeGL(const int width, const int height);
	Scene* m_scene;
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

	//For Rendering Dragon
	GLuint objProgramId;
	GLuint plyProgramId;
	glm::mat4 prespectiveMatrix;
	glm::mat4 viewMatrix;
	glm::mat4 modelMatrix;
	glm::mat4 mvpMatrix;
	const char* dynamicMeshesDataPath;
	const char* dynamicMeshesSecondaryDataPath;
	const char* dynamicMeshesThirdDataPath;
	const char* staticMeshesDataPath;
	const char* shaderPath;
	OVR::Matrix4f m_meshTrasformMat;
	int m_winWidth;
	int m_winHeight;
};
#endif