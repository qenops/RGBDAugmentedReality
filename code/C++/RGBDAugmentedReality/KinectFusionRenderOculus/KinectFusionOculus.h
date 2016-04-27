#pragma once
#include "stdafx.h"
#include "GLHelper.h"
#include "OpenGLApp.h"
#include <NuiApi.h>
#include <NuiKinectFusionApi.h>
#include "Timer.h"
#include <opencv2\opencv.hpp>
using namespace std;
using namespace cv;
class KinectFusionRenderOculus:OpenGLApp
{

	static const int cBytesPerPixel = 4; // for depth float and int-per-pixel raycast images
public:
	KinectFusionRenderOculus();
	~KinectFusionRenderOculus();
	virtual bool Run(int argc, char* argv[]);

private:
	bool init();
	bool initKinect();
	void display();
	void keyFunc(unsigned char key, int x, int y);
	bool loadShaders();
	void friend displayGL();
	void friend keyGL(unsigned char key, int x, int y);

	//Render to Texture
	bool GenerateTextureToRender();

	//Acquire Depth
	bool AquireDepth();

	//Kinect Fusion Functions
	
	/// Main processing function
    void Update();

    // Initialize KinFu
    HRESULT InitializeKinectFusion();

    //Copy Depth Image
    HRESULT CopyExtendedDepth(NUI_IMAGE_FRAME &imageFrame);

     /// Handle new depth data
    void ProcessDepth();

	void KinectDepthFloatImageToOpenCV(NUI_FUSION_IMAGE_FRAME * depthImgFrame);

    //Reset Reconstruction and clear camera Pose
    HRESULT ResetReconstruction();

	//Set matrix to identity
	void SetIdentityMatrix(Matrix4 & mat);



	//Current Kinect
	INuiSensor* m_pNuiSensor;
	NUI_IMAGE_RESOLUTION        m_depthImageResolution; //set to 640 x 480
	int                         m_cDepthWidth; //width of depth image
	int                         m_cDepthHeight;	//height of depth image
	int                         m_cDepthImagePixels; // number of pixels in depth image

	HANDLE                      m_pDepthStreamHandle;
	HANDLE                      m_hNextDepthFrameEvent;

	LARGE_INTEGER               m_cLastDepthFrameTimeStamp;  //Last depthmap timestamp

	cv::Mat						m_floatDepthMapOpenCV;
	//Depth in RGBX format
	BYTE* m_pDepthRGBX;
	//KinFusion Variables
	//KinFusion Mesh
	 //The Kinect Fusion Reconstruction Volume
    INuiFusionReconstruction*   m_pVolume;

    // The Kinect Fusion Volume Parameters
    NUI_FUSION_RECONSTRUCTION_PARAMETERS m_reconstructionParams;

    // The Kinect Fusion Camera Transform
    Matrix4                     m_worldToCameraTransform;

    // The default Kinect Fusion World to Volume Transform
    Matrix4                     m_defaultWorldToVolumeTransform;

    /// Frames from the depth input
    NUI_DEPTH_IMAGE_PIXEL*      m_pDepthImagePixelBuffer;
    NUI_FUSION_IMAGE_FRAME*     m_pDepthFloatImage;

    /// Frames generated from ray-casting the Reconstruction Volume
    NUI_FUSION_IMAGE_FRAME*     m_pPrevPointCloud;

    /// Images for display
	NUI_FUSION_IMAGE_FRAME*     m_pPointCloud;
    NUI_FUSION_IMAGE_FRAME*     m_pShadedSurface;
	cv::Mat						m_RenderTarget;
    /// Camera Tracking parameters
    int                         m_cLostFrameCounter;
    bool                        m_bTrackingFailed;

    /// Parameter to turn automatic reset of the reconstruction when camera tracking is lost on or off.
    /// Set to true in the constructor to enable auto reset on cResetOnNumberOfLostFrames lost frames,
    /// or set false to never automatically reset.
    bool                        m_bAutoResetReconstructionWhenLost;

    /// Parameter to enable automatic reset of the reconstruction when there is a large
    /// difference in timestamp between subsequent frames. This should usually be set true as 
    /// default to enable recorded .xed files to generate a reconstruction reset on looping of
    /// the playback or scrubbing, however, for debug purposes, it can be set false to prevent
    /// automatic reset on timeouts.
    bool                        m_bAutoResetReconstructionOnTimeout;

    /// Processing parameters
    int                         m_deviceIndex;
    NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE m_processorType;
    bool                        m_bInitializeError;
    float                       m_fMinDepthThreshold;
    float                       m_fMaxDepthThreshold;
    bool                        m_bMirrorDepthFrame;
    unsigned short              m_cMaxIntegrationWeight;
    int                         m_cFrameCounter;
    double                      m_fStartTime;
    Timing::Timer               m_timer;


};