#include "OculusSystem.h"


OculusSystem::OculusSystem() :
	m_winHeight(1024),
	m_winWidth(1024)
{}
OculusSystem::~OculusSystem()
{
	// Deleteing Mirror FBO and Texture
	/*if (mirrorFBO)
		glDeleteFramebuffers(1, &mirrorFBO);
	if (mirrorTexture)
		ovr_DestroyMirrorTexture(HMD, reinterpret_cast<ovrTexture*>(mirrorTexture));*/

	 //Releasing Render Buffers
	for (int eye = 0; eye < 2; ++eye)
	{
		delete eyeRenderTexture[eye];
		delete eyeDepthBuffer[eye];
	}

	// Releasing Device
	if (fboId)
	{
		glDeleteFramebuffers(1, &fboId);
		fboId = 0;
	}
	gleContext.Shutdown();


	
	ovr_Destroy(HMD);

	shutdown();
	destroy();
}

void OculusSystem::Render()
{
		glEnable(GL_DEPTH_TEST);
		ovrVector3f   ViewOffset[2] = { EyeRenderDesc[0].HmdToEyeViewOffset,
		EyeRenderDesc[1].HmdToEyeViewOffset };
		ovrPosef                  EyeRenderPose[2];

		double           ftiming = ovr_GetPredictedDisplayTime(HMD, 0);
		// Keeping sensorSampleTime as close to ovr_GetTrackingState as possible - fed into the layer
		double           sensorSampleTime = ovr_GetTimeInSeconds();
		ovrTrackingState hmdState = ovr_GetTrackingState(HMD, ftiming, ovrTrue);
		ovr_CalcEyePoses(hmdState.HeadPose.ThePose, ViewOffset, EyeRenderPose);
		if (m_isVisible)
		{
			for (int eye = 0; eye < 2; ++eye)
			{
				// Increment to use next texture, just before writing
				eyeRenderTexture[eye]->getTextureSet()->CurrentIndex = (eyeRenderTexture[eye]->getTextureSet()->CurrentIndex + 1) % eyeRenderTexture[eye]->getTextureSet()->TextureCount;

				// Switch to eye render target
				eyeRenderTexture[eye]->SetAndClearRenderSurface(eyeDepthBuffer[eye]);

				OVR::Sizei sz = eyeRenderTexture[eye]->GetSize();
				glMatrixMode(GL_PROJECTION);
				glLoadIdentity();
				glFrustum(-0.1, 0.1, -0.1, 0.1, 0.1, 1000);
				glViewport(0, 0, sz.w,sz.h);
				glMatrixMode(GL_MODELVIEW);
				glLoadIdentity();
				glTranslatef(0.0, 0, -0.1);
				glBegin(GL_QUADS);
				glColor3f(1.0, 0.0, 0.0);
				glVertex3f(-0.1, -0.1, 0.0);
				glVertex3f(0.1, -0.1, 0.0);
				glVertex3f(0.1, 0.1, 0.0);
				glVertex3f(-0.1, 0.1, 0.0);
				glEnd();


				// Avoids an error when calling SetAndClearRenderSurface during next iteration.
				// Without this, during the next while loop iteration SetAndClearRenderSurface
				// would bind a framebuffer with an invalid COLOR_ATTACHMENT0 because the texture ID
				// associated with COLOR_ATTACHMENT0 had been unlocked by calling wglDXUnlockObjectsNV.
				eyeRenderTexture[eye]->UnsetRenderSurface();
			}
		}
		// Do distortion rendering, Present and flush/sync

		// Set up positional data.
		ovrViewScaleDesc viewScaleDesc;
		viewScaleDesc.HmdSpaceToWorldScaleInMeters = 1.0f;
		viewScaleDesc.HmdToEyeViewOffset[0] = ViewOffset[0];
		viewScaleDesc.HmdToEyeViewOffset[1] = ViewOffset[1];

		ovrLayerEyeFov ld;
		ld.Header.Type = ovrLayerType_EyeFov;
		ld.Header.Flags = ovrLayerFlag_TextureOriginAtBottomLeft;   // Because OpenGL.

		for (int eye = 0; eye < 2; ++eye)
		{
			ld.ColorTexture[eye] = eyeRenderTexture[eye]->getTextureSet();
			ld.Viewport[eye] = OVR::Recti(eyeRenderTexture[eye]->GetSize());
			ld.Fov[eye] = hmdDesc.DefaultEyeFov[eye];
			ld.RenderPose[eye] = EyeRenderPose[eye];
			ld.SensorSampleTime = sensorSampleTime;
		}

		ovrLayerHeader* layers = &ld.Header;
		ovrResult result = ovr_SubmitFrame(HMD, 0, &viewScaleDesc, &layers, 1);
		// exit the rendering loop if submit returns an error
	

		m_isVisible = (result == ovrSuccess);

		// Blit mirror texture to back buffer
		glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBO);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
		GLint w = mirrorTexture->OGL.Header.TextureSize.w;
		GLint h = mirrorTexture->OGL.Header.TextureSize.h;
		glBlitFramebuffer(0, h, w, 0,
			0, 0, w, h,
			GL_COLOR_BUFFER_BIT, GL_NEAREST);
		glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);


	 // Check if the ESC key was pressed or the window was closed


	glDisable(GL_DEPTH_TEST);


}
void OculusSystem::Render(Texture & leftTex, Texture & rightTex)
{
	glEnable(GL_DEPTH_TEST);
	ovrVector3f   ViewOffset[2] = { EyeRenderDesc[0].HmdToEyeViewOffset,
		EyeRenderDesc[1].HmdToEyeViewOffset };
	ovrPosef                  EyeRenderPose[2];

	double           ftiming = ovr_GetPredictedDisplayTime(HMD, 0);
	// Keeping sensorSampleTime as close to ovr_GetTrackingState as possible - fed into the layer
	double           sensorSampleTime = ovr_GetTimeInSeconds();
	ovrTrackingState hmdState = ovr_GetTrackingState(HMD, ftiming, ovrTrue);
	ovr_CalcEyePoses(hmdState.HeadPose.ThePose, ViewOffset, EyeRenderPose);
	if (m_isVisible)
	{
		for (int eye = 0; eye < 2; ++eye)
		{
			// Increment to use next texture, just before writing
			eyeRenderTexture[eye]->getTextureSet()->CurrentIndex = (eyeRenderTexture[eye]->getTextureSet()->CurrentIndex + 1) % eyeRenderTexture[eye]->getTextureSet()->TextureCount;

			// Switch to eye render target
			eyeRenderTexture[eye]->SetAndClearRenderSurface(eyeDepthBuffer[eye]);

			OVR::Sizei sz = eyeRenderTexture[eye]->GetSize();
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glFrustum(-0.1, 0.1, -0.1, 0.1, 0.1, 1000);
			glViewport(0, 0, sz.w, sz.h);
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			glTranslatef(0.0, 0, -0.1);
			glEnable(GL_TEXTURE_2D);
			if (eye == 0)
			{
				glBindTexture(GL_TEXTURE_2D, leftTex.GetTexture());
			}
			else 
			{
				glBindTexture(GL_TEXTURE_2D, rightTex.GetTexture());
			}
			glActiveTexture(GL_TEXTURE0);
			glBegin(GL_QUADS);
			//glColor3f(1.0, 0.0, 0.0);
			glVertex3f(-0.1, -0.1, 0.0);
			glTexCoord2f(1.0,1.0);
			glVertex3f(0.1, -0.1, 0.0);
			glTexCoord2f(1.0, 0.0);
			glVertex3f(0.1, 0.1, 0.0);
			glTexCoord2f(0.0, 0.0);
			glVertex3f(-0.1, 0.1, 0.0);
			glTexCoord2f(0.0, 1.0);
			glEnd();
			glBindTexture(GL_TEXTURE_2D, 0);

			// Avoids an error when calling SetAndClearRenderSurface during next iteration.
			// Without this, during the next while loop iteration SetAndClearRenderSurface
			// would bind a framebuffer with an invalid COLOR_ATTACHMENT0 because the texture ID
			// associated with COLOR_ATTACHMENT0 had been unlocked by calling wglDXUnlockObjectsNV.
			eyeRenderTexture[eye]->UnsetRenderSurface();
		}
	}
	// Do distortion rendering, Present and flush/sync

	// Set up positional data.
	ovrViewScaleDesc viewScaleDesc;
	viewScaleDesc.HmdSpaceToWorldScaleInMeters = 1.0f;
	viewScaleDesc.HmdToEyeViewOffset[0] = ViewOffset[0];
	viewScaleDesc.HmdToEyeViewOffset[1] = ViewOffset[1];

	ovrLayerEyeFov ld;
	ld.Header.Type = ovrLayerType_EyeFov;
	ld.Header.Flags = ovrLayerFlag_TextureOriginAtBottomLeft;   // Because OpenGL.

	for (int eye = 0; eye < 2; ++eye)
	{
		ld.ColorTexture[eye] = eyeRenderTexture[eye]->getTextureSet();
		ld.Viewport[eye] = OVR::Recti(eyeRenderTexture[eye]->GetSize());
		ld.Fov[eye] = hmdDesc.DefaultEyeFov[eye];
		ld.RenderPose[eye] = EyeRenderPose[eye];
		ld.SensorSampleTime = sensorSampleTime;
	}

	ovrLayerHeader* layers = &ld.Header;
	ovrResult result = ovr_SubmitFrame(HMD, 0, &viewScaleDesc, &layers, 1);
	// exit the rendering loop if submit returns an error


	m_isVisible = (result == ovrSuccess);

	// Blit mirror texture to back buffer
	glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBO);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
	GLint w = mirrorTexture->OGL.Header.TextureSize.w;
	GLint h = mirrorTexture->OGL.Header.TextureSize.h;
	glBlitFramebuffer(0, h, w, 0,
		0, 0, w, h,
		GL_COLOR_BUFFER_BIT, GL_NEAREST);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);


	// Check if the ESC key was pressed or the window was closed


	glDisable(GL_DEPTH_TEST);

}
bool OculusSystem::GetIdealRenderSize(OVR::Sizei& leftTextureSize, OVR::Sizei& rightTextureSize)
{
	leftTextureSize = ovr_GetFovTextureSize(HMD, ovrEyeType(0), hmdDesc.DefaultEyeFov[0], 1);
	rightTextureSize = ovr_GetFovTextureSize(HMD, ovrEyeType(1), hmdDesc.DefaultEyeFov[1], 1);
	return true;
}

bool OculusSystem::GetEyePositionsFromHMD(OVR::Vector3f & leftEyeTrans, OVR::Vector3f & rightEyeTrans)
{
	leftEyeTrans = EyeRenderDesc[0].HmdToEyeViewOffset;
	rightEyeTrans = EyeRenderDesc[1].HmdToEyeViewOffset;
	return true;
}

bool OculusSystem::GetEyeFOVFromHMD(ovrFovPort& leftEyeFov, ovrFovPort& rightEyeFov)
{
	leftEyeFov = EyeRenderDesc[0].Fov;
	rightEyeFov = EyeRenderDesc[1].Fov;
	return true;
}


bool OculusSystem::loadBuffers()
{
	for (int eye = 0; eye < 2; ++eye)
	{
		eyeRenderTexture[eye] = nullptr;
		eyeDepthBuffer[eye] = nullptr;
	}
	mirrorTexture = nullptr;

	// Make eye render buffers
	for (int eye = 0; eye < 2; ++eye)
	{
		ovrSizei idealTextureSize = ovr_GetFovTextureSize(HMD, ovrEyeType(eye), hmdDesc.DefaultEyeFov[eye], 1);
		eyeRenderTexture[eye] = new OVRTextureBuffer(HMD, true, true, idealTextureSize, 1, NULL, 1);
		eyeDepthBuffer[eye] = new OVRDepthBuffer(eyeRenderTexture[eye]->GetSize(), 0);

		if (!eyeRenderTexture[eye]->getTextureSet())
		{
			printf("Failed to create texture.\n");
			return false;
		}
	}

	// Create mirror texture and an FBO used to copy mirror texture to back buffer
	ovrResult result = ovr_CreateMirrorTextureGL(HMD, GL_SRGB8_ALPHA8, m_winWidth, m_winHeight, reinterpret_cast<ovrTexture**>(&mirrorTexture));
	if (!OVR_SUCCESS(result))
	{
		printf("Failed to create mirror texture\n");
		return false;
	}

	// Configure the mirror read buffer
	glGenFramebuffers(1, &mirrorFBO);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBO);
	glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mirrorTexture->OGL.TexId, 0);
	glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

	EyeRenderDesc[0] = ovr_GetRenderDesc(HMD, ovrEye_Left, hmdDesc.DefaultEyeFov[0]);
	EyeRenderDesc[1] = ovr_GetRenderDesc(HMD, ovrEye_Right, hmdDesc.DefaultEyeFov[1]);

	return true;

}
void OculusSystem::initialize()
{
	OVR::System::Init();

	// Initializes LibOVR, and the Rift
	ovrResult result = ovr_Initialize(nullptr);
	if (!OVR_SUCCESS(result))
		printf("Sorry Failed to initialize libOVR.\n");
	else
		printf("OVR initialized\n");


	ovrGraphicsLuid luid;
	result = ovr_Create(&HMD, &luid);
	if (!OVR_SUCCESS(result))
	{
		printf("Sorry Device creation failed\n");
	}
	else
		printf("OVR Device created\n");

	hmdDesc = ovr_GetHmdDesc(HMD);

	// Setup Window and Graphics
	// Note: the mirror window can be any size, for this sample we use 1/2 the HMD resolution
	ovrSizei windowSize = { hmdDesc.Resolution.w / 2, hmdDesc.Resolution.h / 2 };

	OVR::GLEContext::SetCurrentContext(&gleContext);
	gleContext.Init();
	
}

void OculusSystem::shutdown()
{
	ovr_Shutdown();
}

void OculusSystem::destroy()
{
	OVR::System::Destroy();
}
