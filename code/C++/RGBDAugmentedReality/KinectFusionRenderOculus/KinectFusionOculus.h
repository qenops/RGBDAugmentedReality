#pragma once
#include "stdafx.h"
#include "OculusSystem.h"
#include "GL/CAPI_GLE.h"
#include "OVR_CAPI_GL.h"
#include"algebra3.h"
#include "glfw3.h"
#include <GL/glut.h>
#include <NuiApi.h>
#include <NuiKinectFusionApi.h>
#include "Timer.h"
#include <opencv2\opencv.hpp>
#include "KinectFusionHelper.h"
#include <ctime>
using namespace std;
using namespace cv;
class KinectFusionRenderOculus
{

	static const int			cBytesPerPixel = 4; // for depth float and int-per-pixel raycast images
	static const int			cMinTimestampDifferenceForFrameReSync = 17;
	static const int            cTimeDisplayInterval = 10;
	static const int            cResetOnTimeStampSkippedMillisecondsGPU = 2000;
	static const int            cResetOnNumberOfLostFrames = 1;
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
	bool AcquireColor();
	void friend displayGL();
	void friend keyGL(unsigned char key, int x, int y);
	bool initGLUT(int argc, char* argv[], string name, int windowWidth, int windowHeight);


	//Render to Texture
	bool GenerateTextureToRender();

	//Acquire Depth
	bool AquireDepth();

	//Kinect Fusion Functions
	
	//Perform Camera Tracking without integrating data
	HRESULT CameraTrackingOnly();

	// Main processing function
    void Update();

    // Initialize KinFu
    HRESULT InitializeKinectFusion();

    //Copy Depth Image
    HRESULT CopyExtendedDepth(NUI_IMAGE_FRAME &imageFrame);

	// Copy the color data out of a Kinect image frame
	HRESULT CopyColor(NUI_IMAGE_FRAME &imageFrame);

	// Adjust color to the same space as depth
	HRESULT                     MapColorToDepth();

    // Handle new depth data
    void ProcessDepth();

	// Handle new depth data
	void ProcessDepthAdvanced();

	void KinectColorFloatImageToOpenCV(NUI_FUSION_IMAGE_FRAME* colorImgFrame, string winName);
	//void Matrix4toOVRMatrix4f(Matrix4& kinMat, Matrix4& ocMat);
	void KinectDepthFloatImageToOpenCV(NUI_FUSION_IMAGE_FRAME * depthImgFrame);

	bool GenerateTextureToRender(NUI_FUSION_IMAGE_FRAME* pShadedSurface, NUI_FUSION_IMAGE_FRAME* pCloud, Matrix4 poseWorldtoCamera, Texture& tex);
    //Reset Reconstruction and clear camera Pose
    HRESULT ResetReconstruction();

	//Set matrix to identity
	void SetIdentityMatrix(Matrix4 & mat);
	void printMat(Matrix4 mat);
	//Oculus related functions
	bool RenderForOculus();
	bool RenderForOculusAdvanced();
	bool CreateRenderObjectsForOculus();
	float ComputeFocalLengthFromAngleTan(float tanAngle,float width);
	NUI_FUSION_CAMERA_PARAMETERS ComputeCamParams(ovrFovPort fov, OVR::Sizei texSize, float correction);
	HRESULT InitializeFrame(int cImageSize);
	void FreeBuffers();
	//Oculus system
	clock_t m_lastTime;
	double m_deltaTime;

	OculusSystem* m_osystem;
	OVR::Vector3f m_rightEyeTrans;
	OVR::Vector3f m_leftEyeTrans;
	OVR::Sizei m_leftTextureSize;
	OVR::Sizei m_rightTextureSize;
	ovrFovPort m_leftEyeFOV;
	ovrFovPort m_rightEyeFOV;
	Texture* leftEyeTexture;
	Texture* rightEyeTexture;
	NUI_FUSION_CAMERA_PARAMETERS m_camParamsLeft;
	NUI_FUSION_CAMERA_PARAMETERS m_camParamsRight;
	NUI_FUSION_IMAGE_FRAME* m_pShadedSurfaceLeft;
	NUI_FUSION_IMAGE_FRAME* m_pShadedSurfaceRight;
	NUI_FUSION_IMAGE_FRAME* m_pPointCloudOculusLeft;
	NUI_FUSION_IMAGE_FRAME* m_pPointCloudOculusRight;


	//Current Kinect
	INuiSensor* m_pNuiSensor;
	NUI_IMAGE_RESOLUTION        m_depthImageResolution; //set to 640 x 480
	NUI_IMAGE_RESOLUTION		m_colorImageResolution;
	int                         m_cDepthWidth; //width of depth image
	int                         m_cDepthHeight;	//height of depth image
	int                         m_cDepthImagePixels; // number of pixels in depth image

	int                         m_cColorWidth;
	int                         m_cColorHeight;
	int                         m_cColorImagePixels;

	HANDLE                      m_pDepthStreamHandle;
	HANDLE                      m_hNextDepthFrameEvent;

	HANDLE                      m_pColorStreamHandle;
	HANDLE                      m_hNextColorFrameEvent;

	LONGLONG					m_cLastDepthFrameTimeStamp;
	LONGLONG					m_cLastColorFrameTimeStamp;

	LONGLONG					m_currentColorFrameTime;
	LONGLONG					m_currentDepthFrameTime;

	cv::Mat						m_floatDepthMapOpenCV;
	cv::Mat						m_floatColorOpenCV;
	//Depth in RGBX format

	BYTE* m_pDepthRGBX;
	BYTE* m_pReconstructionRGBX;
	BYTE* m_pTrackingDataRGBX;
	//Advacnced Process Depth Functions
	bool IsCameraPoseFinderAvailable();
	HRESULT FindCameraPoseAlignPointClouds();
	bool GetKinectFrames(bool &colorSynchronized);
	HRESULT ProcessColorForCameraPoseFinder(bool &resampled);
	HRESULT SetReferenceFrame(const Matrix4 &worldToCamera);
	void SetTrackingSucceeded();
	void SetTrackingFailed();
	void ResetTracking();
	HRESULT StoreImageToFrameBuffer(const NUI_FUSION_IMAGE_FRAME* imageFrame,
		BYTE* buffer);
	HRESULT TrackCameraAlignDepthFloatToReconstruction(Matrix4 &calculatedCameraPose, FLOAT &alignmentEnergy);
	HRESULT TrackCameraAlignPointClouds(Matrix4 &calculatedCameraPose, FLOAT &alignmentEnergy);
	HRESULT UpdateCameraPoseFinder();
	HRESULT InitializeKinectFusionAdvanced();
	HRESULT KinectFusionRenderOculus::CreateFrame(
		NUI_FUSION_IMAGE_TYPE frameType,
		unsigned int imageWidth,
		unsigned int imageHeight,
		NUI_FUSION_IMAGE_FRAME** ppImageFrame);
	//Advanced Process Depth Variables
	bool m_bAutoFindCameraPoseWhenLost;

	//Camera Pose Finder
	INuiFusionCameraPoseFinder* m_pCameraPoseFinder;
	NUI_FUSION_IMAGE_FRAME*     m_pResampledColorImage;
	NUI_FUSION_IMAGE_FRAME*     m_pDepthPointCloud;
	NUI_FUSION_IMAGE_FRAME*     m_pSmoothDepthFloatImage;

	float						m_fMaxAlignPointCloudsEnergyForSuccess;;
	unsigned int                m_cCameraPoseFinderProcessFrameCalculationInterval;
	unsigned int                m_cMaxCameraPoseFinderPoseHistory;
	unsigned int                m_cCameraPoseFinderFeatureSampleLocationsPerFrame;
	float                       m_fMaxCameraPoseFinderDepthThreshold;
	float                       m_fCameraPoseFinderDistanceThresholdReject;
	float                       m_fCameraPoseFinderDistanceThresholdAccept;
	unsigned int                m_cMaxCameraPoseFinderPoseTests;
	unsigned int                m_cMinSuccessfulTrackingFramesForCameraPoseFinder;
	unsigned int                m_cMinSuccessfulTrackingFramesForCameraPoseFinderAfterFailure;

	//AlignPointCLouds
	NUI_FUSION_IMAGE_FRAME*     m_pFloatDeltaFromReference;
	NUI_FUSION_IMAGE_FRAME*     m_pDownsampledDepthFloatImage;
	NUI_FUSION_IMAGE_FRAME*     m_pDownsampledSmoothDepthFloatImage;
	NUI_FUSION_IMAGE_FRAME*     m_pDownsampledDepthPointCloud;

	unsigned int                m_cAlignPointCloudsImageDownsampleFactor;
	unsigned int                m_cSmoothingKernelWidth;
	float                       m_fSmoothingDistanceThreshold;
	float                       m_fMaxTranslationDelta;
	float                       m_fMaxRotationDelta;

	//Frames generated from the depth input for AlignPointClouds

	unsigned                    m_cSuccessfulFrameCounter;
	bool                        m_bTrackingHasFailedPreviously;
	bool                        m_bCalculateDeltaFrame; //should keep disabled
	bool						m_bIntegrationResumed;
	unsigned int                m_cDeltaFromReferenceFrameCalculationInterval;
	float						m_fMinAlignPointCloudsEnergyForSuccess;
	float                       m_fMaxAlignToReconstructionEnergyForSuccess;
	float                       m_fMinAlignToReconstructionEnergyForSuccess;
	//Depth in RGBX format

	bool m_bColorCaptured;
	float m_fFramesPerSecond;
	unsigned int m_deviceMemory;
	int m_cbImageSize;
	/// Frames generated from ray-casting the Reconstruction Volume.
	/// </summary>
	Matrix4                     m_worldToBGRTransform;
	NUI_FUSION_IMAGE_FRAME*     m_pRaycastPointCloud;
	NUI_FUSION_IMAGE_FRAME*     m_pRaycastDepthFloatImage;
	NUI_FUSION_IMAGE_FRAME*     m_pDownsampledRaycastPointCloud;

	//Display Frames not used
	NUI_FUSION_IMAGE_FRAME*   m_pDownsampledShadedDeltaFromReference;
	NUI_FUSION_IMAGE_FRAME*   m_pShadedDeltaFromReference;

	//KinFusion Variables

	//The Kinect Fusion Reconstruction Volume
	INuiFusionColorReconstruction*   m_pVolume;

    // The Kinect Fusion Volume Parameters
    NUI_FUSION_RECONSTRUCTION_PARAMETERS m_reconstructionParams;

    // The Kinect Fusion Camera Transform
    Matrix4                     m_worldToCameraTransform;

    // The default Kinect Fusion World to Volume Transform
    Matrix4                     m_defaultWorldToVolumeTransform;

    /// Frames from the depth input
    NUI_DEPTH_IMAGE_PIXEL*      m_pDepthImagePixelBuffer;
    NUI_FUSION_IMAGE_FRAME*     m_pDepthFloatImage;
	int                         m_cPixelBufferLength;

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

	bool						m_bTranslateResetPoseByMinDepthThreshold;
	bool						m_bTranslateToCenterOfVolume;
	/// <summary>
	/// For mapping depth to color
	/// </summary>
	NUI_FUSION_IMAGE_FRAME*     m_pColorImage;
	NUI_FUSION_IMAGE_FRAME*     m_pResampledColorImageDepthAligned;
	NUI_COLOR_IMAGE_POINT*      m_pColorCoordinates;
	int							m_cColorCoordinateBufferLength;
	float                       m_colorToDepthDivisor;
	float                       m_oneOverDepthDivisor;
	INuiCoordinateMapper*       m_pMapper;
	bool                        m_bCaptureColor;
	unsigned int                m_cColorIntegrationInterval;
	bool						m_integrateColor;
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

	bool						m_bNearMode;


};