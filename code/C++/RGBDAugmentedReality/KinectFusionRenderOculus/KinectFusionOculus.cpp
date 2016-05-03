#include "KinectFusionOculus.h"
#pragma warning(push)
#pragma warning(disable:6255)
#pragma warning(disable:6263)
#pragma warning(disable:4995)
#include "ppl.h"
#pragma warning(pop)


KinectFusionRenderOculus::KinectFusionRenderOculus():
	m_pNuiSensor(nullptr),
	m_depthImageResolution(NUI_IMAGE_RESOLUTION_640x480),
	m_colorImageResolution(NUI_IMAGE_RESOLUTION_640x480),
	m_pVolume(nullptr),
	m_cDepthImagePixels(0),
	m_cColorImagePixels(0),
	m_pDepthStreamHandle(INVALID_HANDLE_VALUE),
	m_hNextDepthFrameEvent(INVALID_HANDLE_VALUE),
	m_bTranslateResetPoseByMinDepthThreshold(true),
	m_bTranslateToCenterOfVolume(false),
	m_bAutoResetReconstructionWhenLost(true),
	m_bAutoResetReconstructionOnTimeout(false),
	m_cLostFrameCounter(0),
	m_bTrackingFailed(false),
	m_cFrameCounter(0),
	m_fStartTime(0),
	m_pDepthImagePixelBuffer(nullptr),
	m_pColorCoordinates(nullptr),
	m_pMapper(nullptr),
	m_pDepthFloatImage(nullptr),
	m_pColorImage(nullptr),
	m_pResampledColorImageDepthAligned(nullptr),
	m_pPointCloud(nullptr),
	m_pPrevPointCloud(nullptr),
	m_pShadedSurface(nullptr),
	m_bInitializeError(false),
	m_bCaptureColor(true),
	m_bMirrorDepthFrame(false),
	m_cColorIntegrationInterval(2),
	m_bNearMode(true), //should we really

	//For Advanced Process Depth
	m_bAutoFindCameraPoseWhenLost(true),
	m_fMaxAlignPointCloudsEnergyForSuccess(0.006f),
	m_cMaxCameraPoseFinderPoseTests(5),
	m_cCameraPoseFinderProcessFrameCalculationInterval(5),
	m_cMaxCameraPoseFinderPoseHistory(NUI_FUSION_CAMERA_POSE_FINDER_DEFAULT_POSE_HISTORY_COUNT),
	m_cCameraPoseFinderFeatureSampleLocationsPerFrame(NUI_FUSION_CAMERA_POSE_FINDER_DEFAULT_FEATURE_LOCATIONS_PER_FRAME_COUNT),
	m_fMaxCameraPoseFinderDepthThreshold(NUI_FUSION_CAMERA_POSE_FINDER_DEFAULT_MAX_DEPTH_THRESHOLD),
	m_fCameraPoseFinderDistanceThresholdReject(1.0f), // a value of 1.0 means no rejection
	m_fCameraPoseFinderDistanceThresholdAccept(0.1f),
	m_cMinSuccessfulTrackingFramesForCameraPoseFinder(45), // only update the camera pose finder initially after 45 successful frames (1.5s)
	m_cMinSuccessfulTrackingFramesForCameraPoseFinderAfterFailure(200), // resume integration following 200 successful frames after tracking failure (~7s)
	m_cAlignPointCloudsImageDownsampleFactor(2),
	m_cSmoothingKernelWidth(1),                 // 0=just copy, 1=3x3, 2=5x5, 3=7x7, here we create a 3x3 kernel
	m_fSmoothingDistanceThreshold(0.04f),       // 4cm, could use up to around 0.1f
	m_fMaxTranslationDelta(0.3f),				// 0.15 - 0.3m per frame typical
	m_fMaxRotationDelta(20.0f),          // 10-20 degrees per frame typical
	m_bTrackingHasFailedPreviously(false),
	m_bCalculateDeltaFrame(false),
	m_bIntegrationResumed(false),
	m_cDeltaFromReferenceFrameCalculationInterval(2),
	m_fMinAlignPointCloudsEnergyForSuccess(0.0f),
	m_fMaxAlignToReconstructionEnergyForSuccess(0.15f),
	m_fMinAlignToReconstructionEnergyForSuccess(0.005f),
	m_cPixelBufferLength(0),
	m_pCameraPoseFinder(nullptr),
	m_pResampledColorImage(nullptr),
	m_pDepthPointCloud(nullptr),
	m_pSmoothDepthFloatImage(nullptr),
	m_pFloatDeltaFromReference(nullptr),
	m_pDownsampledDepthFloatImage(nullptr),
	m_pDownsampledSmoothDepthFloatImage(nullptr),
	m_pDownsampledDepthPointCloud(nullptr),
	m_cSuccessfulFrameCounter(0),
	m_pDepthRGBX(nullptr),
	m_pTrackingDataRGBX(nullptr),
	m_cbImageSize(0),
	m_fFramesPerSecond(0),
	m_bColorCaptured(false),
	m_deviceMemory(0)


{
	m_osystem = new OculusSystem();
	//Initialize Depth Image Size
	DWORD width = 0, height = 0;
    NuiImageResolutionToSize(m_depthImageResolution, width, height);
    m_cDepthWidth = width;
    m_cDepthHeight = height;
    m_cDepthImagePixels = m_cDepthWidth*m_cDepthHeight;

	NuiImageResolutionToSize(m_colorImageResolution, width, height);
	m_cColorWidth = width;
	m_cColorHeight = height;
	m_cColorImagePixels = m_cColorWidth*m_cColorHeight;

    // create heap storage for depth pixel data in RGBX format
    m_pDepthRGBX = new BYTE[m_cDepthImagePixels*cBytesPerPixel];

	//Initialize OpenCV matrices
	m_floatDepthMapOpenCV = cv::Mat(m_cDepthHeight, m_cDepthWidth, CV_32F);
	m_RenderTarget = cv::Mat(m_cDepthHeight, m_cDepthWidth, CV_8UC4);
     // Define a cubic Kinect Fusion reconstruction volume,
    // with the Kinect at the center of the front face and the volume directly in front of Kinect.
    m_reconstructionParams.voxelsPerMeter = 128;// 1000mm / 256vpm = ~3.9mm/voxel    
    m_reconstructionParams.voxelCountX = 512;   // 512 / 256vpm = 2m wide reconstruction
    m_reconstructionParams.voxelCountY = 512;   // Memory = 512*384*512 * 4bytes per voxel
    m_reconstructionParams.voxelCountZ = 512;   // This will require a GPU with at least 512MB

    // These parameters are for optionally clipping the input depth image 
    m_fMinDepthThreshold = NUI_FUSION_DEFAULT_MINIMUM_DEPTH;   // min depth in meters
    m_fMaxDepthThreshold = NUI_FUSION_DEFAULT_MAXIMUM_DEPTH;    // max depth in meters

     // This parameter is the temporal averaging parameter for depth integration into the reconstruction
    //m_cMaxIntegrationWeight = NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT;	// Reasonable for static scenes
	m_cMaxIntegrationWeight = 2;
    // This parameter sets whether GPU or CPU processing is used. Note that the CPU will likely be 
    // too slow for real-time processing.
    m_processorType = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP;

     m_deviceIndex = -1;    // automatically choose device index for processing

    SetIdentityMatrix(m_worldToCameraTransform);
    SetIdentityMatrix(m_defaultWorldToVolumeTransform);

	m_cLastDepthFrameTimeStamp = 0;
	m_cLastColorFrameTimeStamp = 0;

	m_hNextDepthFrameEvent = CreateEvent(
		nullptr,
		TRUE, /* bManualReset - KinectSDK will reset this internally */
		FALSE, /* bInitialState */
		nullptr);
	m_hNextColorFrameEvent = CreateEvent(
		nullptr,
		TRUE, /* bManualReset - KinectSDK will reset this internally */
		FALSE, /* bInitialState */
		nullptr);
	
}
	void KinectFusionRenderOculus::FreeBuffers()
	{
		SAFE_DELETE_ARRAY(m_pReconstructionRGBX);
		SAFE_DELETE_ARRAY(m_pDepthRGBX);
		SAFE_DELETE_ARRAY(m_pTrackingDataRGBX);

		m_cbImageSize = 0;
	}
	HRESULT KinectFusionRenderOculus::InitializeFrame(int cImageSize)
	{
		HRESULT hr = S_OK;


		ULONG cbImageSize = cImageSize *cBytesPerPixel;

		if (m_cbImageSize != cbImageSize)
		{
			FreeBuffers();

			m_cbImageSize = cbImageSize;
			m_pReconstructionRGBX = new(std::nothrow) BYTE[m_cbImageSize];
			m_pDepthRGBX = new(std::nothrow) BYTE[m_cbImageSize];
			m_pTrackingDataRGBX = new(std::nothrow) BYTE[m_cbImageSize];

			if (nullptr != m_pReconstructionRGBX ||
				nullptr != m_pDepthRGBX ||
				nullptr != m_pTrackingDataRGBX)
			{
				ZeroMemory(m_pReconstructionRGBX, m_cbImageSize);
				ZeroMemory(m_pDepthRGBX, m_cbImageSize);
				ZeroMemory(m_pTrackingDataRGBX, m_cbImageSize);
			}
			else
			{
				FreeBuffers();
				hr = E_OUTOFMEMORY;
			}
		}

		return hr;
	}
bool KinectFusionRenderOculus::initGLUT(int argc, char* argv[], string name, int windowWidth, int windowHeight)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(windowWidth, windowHeight);
	glutCreateWindow(name.c_str());
	glutDisplayFunc(displayGL);
	glutKeyboardFunc(keyGL);
	return true;

}
KinectFusionRenderOculus::~KinectFusionRenderOculus()
{
	SAFE_DELETE(m_osystem);
	// Clean up Kinect Fusion
    SafeRelease(m_pVolume);
	SafeRelease(m_pMapper);

    SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pShadedSurface);
    SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pPointCloud);
    SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pDepthFloatImage);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pColorImage);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pResampledColorImageDepthAligned);
    // Clean up Kinect
    if (m_pNuiSensor)
    {
        m_pNuiSensor->NuiShutdown();
        m_pNuiSensor->Release();
    }

    if (m_hNextDepthFrameEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_hNextDepthFrameEvent);
    }

	if (m_hNextColorFrameEvent != INVALID_HANDLE_VALUE)
	{
		CloseHandle(m_hNextColorFrameEvent);
	}
    // clean up the depth pixel array
    SAFE_DELETE_ARRAY(m_pDepthImagePixelBuffer);

	// Clean up the color pixel arrays
	SAFE_DELETE_ARRAY(m_pColorCoordinates);

    // done with depth pixel data
    SAFE_DELETE_ARRAY(m_pDepthRGBX);
}

bool KinectFusionRenderOculus::Run(int argc, char * argv[])
{
	if (!initGLUT(argc, argv, "Cam Render", 1024, 1024))
	{
		std::cout << "Init GLUT Failed" << std::endl;
		return false;
	}

	m_osystem->initialize();
	if (!m_osystem->loadBuffers())
		printf("Sorry loading of the required buffers failed \n");
	else
		printf("All Buffers loaded \n");
	m_osystem->GetEyePositionsFromHMD(m_leftEyeTrans, m_rightEyeTrans);
	std::cout << "EyePoseLeft" << m_leftEyeTrans.x << "  " << m_leftEyeTrans.y << m_leftEyeTrans.z << endl;
	m_osystem->GetIdealRenderSize(m_leftTextureSize, m_rightTextureSize);
	m_osystem->GetEyeFOVFromHMD(m_leftEyeFOV,m_rightEyeFOV);
	if (!CreateRenderObjectsForOculus())
	{
		cout << "couldn't generate render object for oculus" << endl;
	}
	else
	{
		cout << "Render objects for oculus created" << endl;
	}
	/*if (!initGL(-0.1, 0.1, -0.1, 0.1, 0.1, 1000))
	{
		std::cout << "Init GL Failed" << std::endl;
		return false;
	}*/
	
	if(!initKinect())
	{
		std::cout<<"Init Kinect Failed"<<std::endl;
		return false;
	}
	else
	{
		std::cout << "Kinect initialised Properly" << endl;
	}
	if (FAILED(InitializeKinectFusion()))
	{
		std::cout << "KinectFusion could not be  initialised Properly" << endl;
	}

	if (FAILED(InitializeKinectFusionAdvanced()))
	{
		std::cout << "KinectFusionAdvanced could not be  initialised Properly" << endl;
	}

	glutMainLoop();
	return true;

}

bool KinectFusionRenderOculus::init()
{
	return false;
}

HRESULT KinectFusionRenderOculus::CreateFrame(
	NUI_FUSION_IMAGE_TYPE frameType,
	unsigned int imageWidth,
	unsigned int imageHeight,
	NUI_FUSION_IMAGE_FRAME** ppImageFrame)
{
	HRESULT hr = S_OK;

	if (nullptr != *ppImageFrame)
	{
		// If image size or type has changed, release the old one.
		if ((*ppImageFrame)->width != imageWidth ||
			(*ppImageFrame)->height != imageHeight ||
			(*ppImageFrame)->imageType != frameType)
		{
			static_cast<void>(NuiFusionReleaseImageFrame(*ppImageFrame));
			*ppImageFrame = nullptr;
		}
	}

	// Create a new frame as needed.
	if (nullptr == *ppImageFrame)
	{
		hr = NuiFusionCreateImageFrame(
			frameType,
			imageWidth,
			imageHeight,
			nullptr,
			ppImageFrame);

		if (FAILED(hr))
		{
			cout << "Failed to initialize Kinect Fusion image." << endl;;
		}
	}
	return hr;
}

HRESULT KinectFusionRenderOculus::InitializeKinectFusionAdvanced()
{
	HRESULT hr;
	unsigned int width = static_cast<UINT>(m_cDepthWidth);
	unsigned int height = static_cast<UINT>(m_cDepthHeight);

	unsigned int colorWidth = static_cast<UINT>(m_cColorWidth);
	unsigned int colorHeight = static_cast<UINT>(m_cColorHeight);

	// Calculate the down sampled image sizes, which are used for the AlignPointClouds calculation frames
	unsigned int downsampledWidth = width / m_cAlignPointCloudsImageDownsampleFactor;
	unsigned int downsampledHeight = height / m_cAlignPointCloudsImageDownsampleFactor;
	InitializeFrame(m_cDepthImagePixels);                           
	// Frames generated from the depth input
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, downsampledWidth, downsampledHeight, &m_pDownsampledDepthFloatImage)))
	{
		return hr;
	}

	// Frame generated from the raw color input of Kinect
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_COLOR, colorWidth, colorHeight, &m_pColorImage)))
	{
		return hr;
	}

	// Frame generated from the raw color input of Kinect for use in the camera pose finder.
	// Note color will be down-sampled to the depth size if depth and color capture resolutions differ.
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_COLOR, width, height, &m_pResampledColorImage)))
	{
		return hr;
	}

	// Frame re-sampled from the color input of Kinect, aligned to depth - this will be the same size as the depth.
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_COLOR, width, height, &m_pResampledColorImageDepthAligned)))
	{
		return hr;
	}

	// Point Cloud generated from ray-casting the volume
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, width, height, &m_pRaycastPointCloud)))
	{
		return hr;
	}

	// Point Cloud generated from ray-casting the volume
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, downsampledWidth, downsampledHeight, &m_pDownsampledRaycastPointCloud)))
	{
		return hr;
	}


	// Depth frame generated from ray-casting the volume
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, width, height, &m_pRaycastDepthFloatImage)))
	{
		return hr;
	}

	// Image of the raycast Volume to display
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_COLOR, width, height, &m_pShadedSurface)))
	{
		return hr;
	}

	/*// Image of the raycast Volume with surface normals to display
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_COLOR, width, height, &m_pShadedSurfaceNormals)))
	{
		return hr;
	}

	// Image of the raycast Volume with the captured color to display
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_COLOR, width, height, &m_pCapturedSurfaceColor)))
	{
		return hr;
	}*/

	// Image of the camera tracking deltas to display
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, width, height, &m_pFloatDeltaFromReference)))
	{
		return hr;
	}

	// Image of the camera tracking deltas to display
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_COLOR, width, height, &m_pShadedDeltaFromReference)))
	{
		return hr;
	}

	// Image of the camera tracking deltas to display
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_COLOR, downsampledWidth, downsampledHeight, &m_pDownsampledShadedDeltaFromReference)))
	{
		return hr;
	}

	// Image from input depth for use with AlignPointClouds call
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, width, height, &m_pSmoothDepthFloatImage)))
	{
		return hr;
	}

	// Frames generated from smoothing the depth input
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, downsampledWidth, downsampledHeight, &m_pDownsampledSmoothDepthFloatImage)))
	{
		return hr;
	}

	// Image used in post pose finding success check AlignPointClouds call
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, width, height, &m_pDepthPointCloud)))
	{
		return hr;
	}

	// Point Cloud generated from depth input, in local camera coordinate system
	if (FAILED(hr = CreateFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, downsampledWidth, downsampledHeight, &m_pDownsampledDepthPointCloud)))
	{
		return hr;
	}

	if (nullptr != m_pDepthImagePixelBuffer)
	{
		// If buffer length has changed, delete the old one.
		if (m_cDepthImagePixels != m_cPixelBufferLength)
		{
			SAFE_DELETE_ARRAY(m_pDepthImagePixelBuffer);
		}
	}

	if (nullptr == m_pDepthImagePixelBuffer)
	{
		// Depth pixel array to capture data from Kinect sensor
		m_pDepthImagePixelBuffer =
			new(std::nothrow) NUI_DEPTH_IMAGE_PIXEL[m_cDepthImagePixels];

		if (nullptr == m_pDepthImagePixelBuffer)
		{
			cout<<"Failed to initialize Kinect Fusion depth image pixel buffer."<<endl;
			return hr;
		}

		m_cPixelBufferLength = m_cDepthImagePixels;
	}

	if (nullptr != m_pColorCoordinates)
	{
		// If buffer length has changed, delete the old one.
		if (m_cDepthImagePixels != m_cColorCoordinateBufferLength)
		{
			SAFE_DELETE_ARRAY(m_pColorCoordinates);
		}
	}

	if (nullptr == m_pColorCoordinates)
	{
		// Color coordinate array to capture data from Kinect sensor and for color to depth mapping
		// Note: this must be the same size as the depth
		m_pColorCoordinates =
			new(std::nothrow) NUI_COLOR_IMAGE_POINT[m_cDepthImagePixels];

		if (nullptr == m_pColorCoordinates)
		{
			cout << "Failed to initialize Kinect Fusion color image coordinate buffers." << endl;
			return hr;
		}

		m_cColorCoordinateBufferLength = m_cDepthImagePixels;
	}

	if (nullptr != m_pCameraPoseFinder)
	{
		SafeRelease(m_pCameraPoseFinder);
	}

	// Create the camera pose finder if necessary
	if (nullptr == m_pCameraPoseFinder)
	{
		NUI_FUSION_CAMERA_POSE_FINDER_PARAMETERS cameraPoseFinderParameters;

		cameraPoseFinderParameters.featureSampleLocationsPerFrameCount = m_cCameraPoseFinderFeatureSampleLocationsPerFrame;
		cameraPoseFinderParameters.maxPoseHistoryCount = m_cMaxCameraPoseFinderPoseHistory;
		cameraPoseFinderParameters.maxDepthThreshold = m_fMaxCameraPoseFinderDepthThreshold;

		if (FAILED(hr = NuiFusionCreateCameraPoseFinder(
			&cameraPoseFinderParameters,
			nullptr,
			&m_pCameraPoseFinder)))
		{
			return hr;
		}
	} 
	cout << "Kinect Fusion Advanced Initialised" << endl;
	return hr;
}
bool KinectFusionRenderOculus::initKinect()
{
		INuiSensor * pNuiSensor;
		HRESULT hr;
		int iSensorCount = 0;
		hr = NuiGetSensorCount(&iSensorCount);
		if (FAILED(hr))
		{
			std::cout << "No ready Kinect found!" << std::endl;
			return false;
		}

		// Look at each Kinect sensor
		for (int i = 0; i < iSensorCount; ++i)
		{
			// Create the sensor so we can check status, if we can't create it, move on to the next
			hr = NuiCreateSensorByIndex(i, &pNuiSensor);
			if (FAILED(hr))
			{
				continue;
			}

			// Get the status of the sensor, and if connected, then we can initialize it
			hr = pNuiSensor->NuiStatus();
			if (S_OK == hr)
			{
				m_pNuiSensor = pNuiSensor;
				break;
			}

			// This sensor wasn't OK, so release it since we're not using it
			pNuiSensor->Release();
		}

		if (nullptr != m_pNuiSensor)
		{
			// Initialize the Kinect and specify that we'll be using depth
			hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH| NUI_INITIALIZE_FLAG_USES_COLOR);
			if (SUCCEEDED(hr))
			{
				// Open a depth image stream to receive depth frames
				hr = m_pNuiSensor->NuiImageStreamOpen(
					NUI_IMAGE_TYPE_DEPTH,
					m_depthImageResolution,
					0,
					2,
					m_hNextDepthFrameEvent,
					&m_pDepthStreamHandle);
				if (SUCCEEDED(hr))
				{
					// Open a color image stream to receive color frames
					hr = m_pNuiSensor->NuiImageStreamOpen(
						NUI_IMAGE_TYPE_COLOR,
						m_colorImageResolution,
						0,
						2,
						m_hNextColorFrameEvent,
						&m_pColorStreamHandle);
				}
				if (SUCCEEDED(hr))
				{
					// Create the coordinate mapper for converting color to depth space
					hr = m_pNuiSensor->NuiGetCoordinateMapper(&m_pMapper);
				}
			}
		}

		if (m_bNearMode)
		{
			// Set near mode based on our internal state
			HRESULT nearHr = m_pNuiSensor->NuiImageStreamSetImageFrameFlags(m_pDepthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);
		}
		if (nullptr == m_pNuiSensor || FAILED(hr))
		{
			std::cout<<"No ready Kinect found!"<<endl;
			return false;
		}
		return (S_OK == hr);

}

void KinectFusionRenderOculus::display()
{
	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT);
	glClearDepth(1.0f);
	glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	//m_osystem->Render();
	if (!RenderForOculus())
	{
	cout << "Couldn't render frame to Oculus" << endl;
	}
	Update();
	GenerateTextureToRender();
	
	glutPostRedisplay();
	glutSwapBuffers();
}

void KinectFusionRenderOculus::keyFunc(unsigned char key, int x, int y)
{
}

bool KinectFusionRenderOculus::loadShaders()
{
	return false;
}

bool KinectFusionRenderOculus::AcquireColor()
{
	////////////////////////////////////////////////////////
	// Get a color frame from Kinect
	NUI_IMAGE_FRAME imageFrame;
	m_currentColorFrameTime = m_cLastColorFrameTimeStamp;

	HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pColorStreamHandle, 0, &imageFrame);
	if (FAILED(hr))
	{
		m_integrateColor = false;
	}
	else
	{
		hr = CopyColor(imageFrame);

		m_currentColorFrameTime = imageFrame.liTimeStamp.QuadPart;

		// Release the Kinect camera frame
		m_pNuiSensor->NuiImageStreamReleaseFrame(m_pColorStreamHandle, &imageFrame);

		if (FAILED(hr))
		{
			return false;
		}
	}
	return true;
}

void KinectFusionRenderOculus::SetTrackingSucceeded()
{
	m_cLostFrameCounter = 0;
	m_cSuccessfulFrameCounter++;
	m_bTrackingFailed = false;
}

HRESULT KinectFusionRenderOculus::SetReferenceFrame(const Matrix4 &worldToCamera)
{
	HRESULT hr = S_OK;

	// Raycast to get the predicted previous frame to align against in the next frame
	hr = m_pVolume->CalculatePointCloudAndDepth(
		m_pRaycastPointCloud,
		m_pRaycastDepthFloatImage,
		nullptr,
		&worldToCamera);

	if (FAILED(hr))
	{
		cout<<"Kinect Fusion CalculatePointCloud call failed."<<endl;
		return hr;
	}

	// Set this frame as a reference for AlignDepthFloatToReconstruction
	hr = m_pVolume->SetAlignDepthFloatToReconstructionReferenceFrame(m_pRaycastDepthFloatImage);

	if (FAILED(hr))
	{
		cout<<"Kinect Fusion SetAlignDepthFloatToReconstructionReferenceFrame call failed."<<endl;
		return hr;
	}
	return hr;
}

void KinectFusionRenderOculus::SetTrackingFailed()
{
	m_cLostFrameCounter++;
	m_cSuccessfulFrameCounter = 0;
	m_bTrackingFailed = true;
	m_bTrackingHasFailedPreviously = true;

	m_bIntegrationResumed = false;
}
bool KinectFusionRenderOculus::GetKinectFrames(bool &colorSynchronized)
{
	m_currentDepthFrameTime = 0;
	m_currentColorFrameTime = 0;
	colorSynchronized = true;   // assume we are synchronized to start with

								////////////////////////////////////////////////////////
								// Get an extended depth frame from Kinect

	if (!AquireDepth())
	{
		return false;
	}
	else
	{
		
	}


	////////////////////////////////////////////////////////
	// Get a color frame from Kinect
	if (!AcquireColor())
	{
		return false;
	}
	else
	{

	}

	// Check color and depth frame timestamps to ensure they were captured at the same time
	// If not, we attempt to re-synchronize by getting a new frame from the stream that is behind.
	int timestampDiff = static_cast<int>(abs(m_currentColorFrameTime - m_currentDepthFrameTime));

	if (timestampDiff >= cMinTimestampDifferenceForFrameReSync && m_cSuccessfulFrameCounter > 0 && (m_bAutoFindCameraPoseWhenLost || m_bCaptureColor))
	{
		bool colorSucceeded = false;
		// Get another frame to try and re-sync
		if (m_currentColorFrameTime - m_currentDepthFrameTime >= cMinTimestampDifferenceForFrameReSync)
		{
			// Perform camera tracking only from this current depth frame
			if (nullptr != m_pVolume)
			{
				// Convert the pixels describing extended depth as unsigned short type in millimeters to depth
				// as floating point type in meters.
				HRESULT hr = m_pVolume->DepthToDepthFloatFrame(
					m_pDepthImagePixelBuffer,
					m_cDepthImagePixels * sizeof(NUI_DEPTH_IMAGE_PIXEL),
					m_pDepthFloatImage,
					m_fMinDepthThreshold,
					m_fMaxDepthThreshold,
					m_bMirrorDepthFrame);

				if (FAILED(hr))
				{
					cout<<"Kinect Fusion NuiFusionDepthToDepthFloatFrame call failed."<<endl;
					return false;
				}

				Matrix4 calculatedCameraPose = m_worldToCameraTransform;
				FLOAT alignmentEnergy = 1.0f;

				hr = TrackCameraAlignPointClouds(calculatedCameraPose, alignmentEnergy);

				if (SUCCEEDED(hr))
				{
					m_worldToCameraTransform = calculatedCameraPose;

					// Raycast and set reference frame for tracking with AlignDepthFloatToReconstruction
					hr = SetReferenceFrame(m_worldToCameraTransform);
					SetTrackingSucceeded();
				}
				else
				{
					SetTrackingFailed();
				}
			}

			// Get another depth frame to try and re-sync as color ahead of depth
			if (!AquireDepth())
			{
				cout << "Kinect Depth stream NuiImageStreamReleaseFrame call failed." << endl;
				return false;
			}
		}
		else if (m_currentDepthFrameTime - m_currentColorFrameTime >= cMinTimestampDifferenceForFrameReSync && WaitForSingleObject(m_hNextColorFrameEvent, 0) != WAIT_TIMEOUT)
		{
			// Get another color frame to try and re-sync as depth ahead of color and there is another color frame waiting
			bool colorSucceeded = AcquireColor();
			if (!colorSucceeded)
			{
				colorSynchronized = false;
			}

		}

		timestampDiff = static_cast<int>(abs(m_currentColorFrameTime - m_currentDepthFrameTime));

		// If the difference is still too large, we do not want to integrate color
		if (timestampDiff > cMinTimestampDifferenceForFrameReSync || !(colorSucceeded))
		{
			colorSynchronized = false;
		}
		else
		{
			colorSynchronized = true;
		}
	}

	////////////////////////////////////////////////////////
	// To enable playback of a .xed file through Kinect Studio and reset of the reconstruction
	// if the .xed loops, we test for when the frame timestamp has skipped a large number. 
	// Note: this will potentially continually reset live reconstructions on slow machines which
	// cannot process a live frame in less time than the reset threshold. Increase the number of
	// milliseconds in cResetOnTimeStampSkippedMilliseconds if this is a problem.

	int cResetOnTimeStampSkippedMilliseconds = cResetOnTimeStampSkippedMillisecondsGPU;

	/*	
	if (m_bAutoResetReconstructionOnTimeout && m_cFrameCounter != 0 && nullptr != m_pVolume
		&& abs(m_currentDepthFrameTime - m_cLastDepthFrameTimeStamp) > cResetOnTimeStampSkippedMilliseconds)
	{
		HRESULT hr = ResetReconstruction();

		if (SUCCEEDED(hr))
		{
			cout<<"Reconstruction has been reset."<<endl;
		}
		else
		{
			cout << "Failed to reset reconstruction." << endl;
		}
	}
	*/
	m_cLastDepthFrameTimeStamp = m_currentDepthFrameTime;
	m_cLastColorFrameTimeStamp = m_currentColorFrameTime;

	return true;
}

void KinectFusionRenderOculus::ProcessDepth()
{
	if (m_bInitializeError)
	{
		return;
	}

	HRESULT hr = S_OK;
	m_integrateColor = m_bCaptureColor && m_cFrameCounter % m_cColorIntegrationInterval == 0;
	
	if (!AquireDepth())
	{
		return;
	}
	else
	{
		KinectDepthFloatImageToOpenCV(m_pDepthFloatImage);
	}

	////////////////////////////////////////////////////////
	// Get a color frame from Kinect
	if (!AcquireColor()) 
	{
		m_integrateColor = false;
	}
	else 
	{
		KinectColorFloatImageToOpenCV(m_pColorImage,"Color Image");
	}
	// Check color and depth frame timestamps to ensure they were captured at the same time
	// If not, we attempt to re-synchronize by getting a new frame from the stream that is behind.
	int timestampDiff = static_cast<int>(abs(m_currentColorFrameTime - m_currentDepthFrameTime));
	
	if (m_integrateColor && timestampDiff >= cMinTimestampDifferenceForFrameReSync)
	{
		// Get another frame to try and re-sync
		if (m_currentColorFrameTime- m_currentDepthFrameTime >= cMinTimestampDifferenceForFrameReSync)
		{
			// Perform camera tracking only from this current depth frame
			if (m_cFrameCounter > 0)
			{
				CameraTrackingOnly();
			}

			// Get another depth frame to try and re-sync as color ahead of depth
			if (!AquireDepth())
			{
				cout<<"Kinect Depth stream NuiImageStreamReleaseFrame call failed."<<endl;
			}
		}
		else if (m_currentDepthFrameTime - m_currentColorFrameTime >= cMinTimestampDifferenceForFrameReSync && WaitForSingleObject(m_hNextColorFrameEvent, 0) != WAIT_TIMEOUT)
		{
			// Get another color frame to try and re-sync as depth ahead of color
			if (!AcquireColor()) 
			{
				m_integrateColor = false;
			}

			timestampDiff = static_cast<int>(abs(m_currentColorFrameTime - m_currentDepthFrameTime));

			// If the difference is still too large, we do not want to integrate color
			if (timestampDiff > cMinTimestampDifferenceForFrameReSync)
			{
				m_integrateColor = false;
			}
		}
	}

	////////////////////////////////////////////////////////
	// To enable playback of a .xed file through Kinect Studio and reset of the reconstruction
	// if the .xed loops, we test for when the frame timestamp has skipped a large number. 
	// Note: this will potentially continually reset live reconstructions on slow machines which
	// cannot process a live frame in less time than the reset threshold. Increase the number of
	// milliseconds in cResetOnTimeStampSkippedMilliseconds if this is a problem.
	if (m_bAutoResetReconstructionOnTimeout &&  m_cFrameCounter != 0
		&& abs(m_currentDepthFrameTime - m_cLastDepthFrameTimeStamp) > cResetOnTimeStampSkippedMillisecondsGPU)
	{
		ResetReconstruction();

		if (FAILED(hr))
		{
			return;
		}
	}

	m_cLastDepthFrameTimeStamp = m_currentDepthFrameTime;
	m_cLastColorFrameTimeStamp = m_currentColorFrameTime;

	// Return if the volume is not initialized
	if (nullptr == m_pVolume)
	{
		cout << "Kinect Fusion reconstruction volume not initialized. Please try reducing volume size or restarting." << endl;
		return;
	}

	////////////////////////////////////////////////////////
	// Depth to DepthFloat

	// Convert the pixels describing extended depth as unsigned short type in millimeters to depth
	// as floating point type in meters.
	hr = m_pVolume->DepthToDepthFloatFrame(m_pDepthImagePixelBuffer, m_cDepthImagePixels * sizeof(NUI_DEPTH_IMAGE_PIXEL), m_pDepthFloatImage, m_fMinDepthThreshold, m_fMaxDepthThreshold, m_bMirrorDepthFrame);

	if (FAILED(hr))
	{
		cout<<"Kinect Fusion NuiFusionDepthToDepthFloatFrame call failed."<<endl;
		return;
	}

	////////////////////////////////////////////////////////
	// ProcessFrame

	if (m_integrateColor)
	{
		// Map the color frame to the depth
		MapColorToDepth();
	}

	// Perform the camera tracking and update the Kinect Fusion Volume
	// This will create memory on the GPU, upload the image, run camera tracking and integrate the
	// data into the Reconstruction Volume if successful. Note that passing nullptr as the final 
	// parameter will use and update the internal camera pose.
	hr = m_pVolume->ProcessFrame(
		m_pDepthFloatImage,
		m_integrateColor ? m_pResampledColorImageDepthAligned : nullptr,
		NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
		m_cMaxIntegrationWeight,
		NUI_FUSION_DEFAULT_COLOR_INTEGRATION_OF_ALL_ANGLES,
		&m_worldToCameraTransform);

	// Test to see if camera tracking failed. 
	// If it did fail, no data integration or raycast for reference points and normals will have taken 
	//  place, and the internal camera pose will be unchanged.
	if (FAILED(hr))
	{
		if (hr == E_NUI_FUSION_TRACKING_ERROR)
		{
			//Add code to perform alignment if tracking fails
			m_cLostFrameCounter++;
			m_bTrackingFailed = true;
			std::cout<<"ProcFrame: Kinect Fusion camera tracking failed! Align the camera to the last tracked position. "<<endl;
		}
		else
		{
			std::cout<<"Kinect Fusion ProcessFrame call failed!"<<endl;
			return;
		}
	}
	else
	{
		Matrix4 calculatedCameraPose;
		hr = m_pVolume->GetCurrentWorldToCameraTransform(&calculatedCameraPose);

		if (SUCCEEDED(hr))
		{
			// Set the pose
			m_worldToCameraTransform = calculatedCameraPose;
			m_cLostFrameCounter = 0;
			m_bTrackingFailed = false;
		}
	}
	GenerateTextureToRender();
	if (m_bAutoResetReconstructionWhenLost && m_bTrackingFailed && m_cLostFrameCounter >= cResetOnNumberOfLostFrames)
	{
		// Automatically clear volume and reset tracking if tracking fails
		hr = ResetReconstruction();

		if (FAILED(hr))
		{
			return;
		}

		// Set bad tracking message
		cout<<L"Kinect Fusion camera tracking failed, automatically reset volume."<<endl;
	}

	////////////////////////////////////////////////////////
	// CalculatePointCloud

	// Raycast all the time, even if we camera tracking failed, to enable us to visualize what is happening with the system
	

	////////////////////////////////////////////////////////
	// Render

	//// Draw the shaded raycast volume image
	//INuiFrameTexture * pShadedImageTexture = m_pShadedSurface->pFrameTexture;
	//NUI_LOCKED_RECT ShadedLockedRect;

	//// Lock the frame data so the Kinect knows not to modify it while we're reading it
	//hr = pShadedImageTexture->LockRect(0, &ShadedLockedRect, nullptr, 0);
	//if (FAILED(hr))
	//{
	//	return;
	//}

	//// Make sure we've received valid data
	//if (ShadedLockedRect.Pitch != 0)
	//{
	//	BYTE * pBuffer = (BYTE *)ShadedLockedRect.pBits;

	//	// Draw the data with Direct2D
	//	m_pDrawDepth->Draw(pBuffer, m_cDepthWidth * m_cDepthHeight * cBytesPerPixel);
	//}

	//// We're done with the texture so unlock it
	//pShadedImageTexture->UnlockRect(0);

	////////////////////////////////////////////////////////
	// Periodically Display Fps

	// Update frame counter
	m_cFrameCounter++;

	// Display fps count approximately every cTimeDisplayInterval seconds
	double elapsed = m_timer.AbsoluteTime() - m_fStartTime;
	if ((int)elapsed >= cTimeDisplayInterval)
	{
		double fps = (double)m_cFrameCounter / elapsed;

		// Update status display
		if (!m_bTrackingFailed)
		{
			WCHAR str[MAX_PATH];
			swprintf_s(str, ARRAYSIZE(str), L"Fps: %5.2f", fps);
			//SetStatusMessage(str);
		}

		m_cFrameCounter = 0;
		m_fStartTime = m_timer.AbsoluteTime();
	}
}

HRESULT KinectFusionRenderOculus::CameraTrackingOnly()
{
	// Convert the pixels describing extended depth as unsigned short type in millimeters to depth
	// as floating point type in meters.
	HRESULT hr = m_pVolume->DepthToDepthFloatFrame(m_pDepthImagePixelBuffer, m_cDepthImagePixels * sizeof(NUI_DEPTH_IMAGE_PIXEL), m_pDepthFloatImage, m_fMinDepthThreshold, m_fMaxDepthThreshold, m_bMirrorDepthFrame);

	if (FAILED(hr))
	{
		cout<<"Kinect Fusion NuiFusionDepthToDepthFloatFrame call failed."<<endl;
		return hr;
	}

	HRESULT tracking = m_pVolume->AlignDepthFloatToReconstruction(
		m_pDepthFloatImage,
		NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
		nullptr,
		nullptr,
		nullptr);

	if (FAILED(tracking))
	{
		m_cLostFrameCounter++;
		m_bTrackingFailed = true;

		if (tracking == E_NUI_FUSION_TRACKING_ERROR)
		{
			
				cout<<"CAMtrack:Kinect Fusion camera tracking failed! Align the camera to the last tracked position."<<endl;
		}
		else
		{
			cout<<"Kinect Fusion AlignDepthFloatToReconstruction call failed!"<<endl;
			hr = tracking;
		}
	}
	else
	{
		m_pVolume->GetCurrentWorldToCameraTransform(&m_worldToCameraTransform);
		m_cLostFrameCounter = 0;
		m_bTrackingFailed = false;
	}

	return hr;
}
bool KinectFusionRenderOculus::GenerateTextureToRender()
{
	if (m_pVolume == nullptr)
	{
		return false;
	}
	//Camera Matrix to render from
	Matrix4 worldToCamera = m_worldToCameraTransform;

	//HRESULT hr = m_pVolume->CalculatePointCloud(m_pPointCloud, ((m_bCaptureColor == true) ? m_pShadedSurface : nullptr), &worldToCamera);
	HRESULT hr = m_pVolume->CalculatePointCloud(m_pPointCloud, m_pShadedSurface , &worldToCamera);
	if (FAILED(hr))
	{
		std::cout << "Kinect Fusion CalculatePointCloud call failed." << endl;
		return false;
	}

	if (!m_bCaptureColor)
	{
		hr = NuiFusionShadePointCloud(m_pPointCloud, &worldToCamera, nullptr, m_pShadedSurface, nullptr);

		if (FAILED(hr))
		{
			std::cout << "Kinect Fusion NuiFusionShadePointCloud call failed." << endl;
			return false;
		}
	}
	

	// Draw the shaded raycast volume image
	INuiFrameTexture * pShadedImageTexture = m_pShadedSurface->pFrameTexture;
	NUI_LOCKED_RECT ShadedLockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	hr = pShadedImageTexture->LockRect(0, &ShadedLockedRect, nullptr, 0);
	if (FAILED(hr))
	{
		return false;
	}

	// Make sure we've received valid data
	if (ShadedLockedRect.Pitch != 0)
	{
		BYTE * pBuffer = (BYTE *)ShadedLockedRect.pBits;
		// Draw the data with Direct2D
		//m_pDrawDepth->Draw(pBuffer, m_cDepthWidth * m_cDepthHeight * cBytesPerPixel);
		m_RenderTarget = cv::Mat(m_pShadedSurface->height, m_pShadedSurface->width,CV_8UC4,pBuffer);
	}
	imshow("Render Target", m_RenderTarget);
	// We're done with the texture so unlock it
	pShadedImageTexture->UnlockRect(0);
	return true;
}
bool KinectFusionRenderOculus::GenerateTextureToRender(NUI_FUSION_IMAGE_FRAME* pShadedSurface, NUI_FUSION_IMAGE_FRAME* pCloud, Matrix4 poseWorldtoCamera,Texture& tex)
{
	cv::Mat renderImg;
	if (m_pVolume == nullptr)
	{
		return false;
	}
	//Camera Matrix to render from
	Matrix4 worldToCamera = poseWorldtoCamera;
	//HRESULT hr = m_pVolume->CalculatePointCloud(m_pPointCloud, ((m_bCaptureColor == true) ? m_pShadedSurface : nullptr), &worldToCamera);
	HRESULT hr = m_pVolume->CalculatePointCloud(pCloud, pShadedSurface, &worldToCamera);
	if (FAILED(hr))
	{
		std::cout << "Kinect Fusion CalculatePointCloud call failed." << endl;
		return false;
	}
	if (!m_bCaptureColor)
	{
		hr = NuiFusionShadePointCloud(pCloud, &worldToCamera, nullptr, pShadedSurface, nullptr);
	}
	//
	// Draw the shaded raycast volume image
	INuiFrameTexture * pShadedImageTexture = pShadedSurface->pFrameTexture;
	NUI_LOCKED_RECT ShadedLockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	hr = pShadedImageTexture->LockRect(0, &ShadedLockedRect, nullptr, 0);
	if (FAILED(hr))
	{
		return false;
	}

	// Make sure we've received valid data
	if (ShadedLockedRect.Pitch != 0)
	{
		BYTE * pBuffer = (BYTE *)ShadedLockedRect.pBits;
		// Draw the data with Direct2D
		//m_pDrawDepth->Draw(pBuffer, m_cDepthWidth * m_cDepthHeight * cBytesPerPixel);
		renderImg = cv::Mat(pShadedSurface->height, pShadedSurface->width, CV_8UC4, pBuffer);
	}
	// We're done with the texture so unlock it
	pShadedImageTexture->UnlockRect(0);
	tex.UpdateTexture(renderImg);
	return true;
}


bool KinectFusionRenderOculus::AquireDepth()
{
	NUI_IMAGE_FRAME imageFrame;
	HRESULT hr;
	////////////////////////////////////////////////////////
	// Get an extended depth frame from Kinect

	hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &imageFrame);
	if (FAILED(hr))
	{
		cout<<"Kinect NuiImageStreamGetNextFrame call failed."<<endl;
		return false;
	}

	hr = CopyExtendedDepth(imageFrame);

	m_currentDepthFrameTime = imageFrame.liTimeStamp.QuadPart;

	// Release the Kinect camera frame
	m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);

	if (FAILED(hr))
	{
		cout << "Kinect Depth stream NuiImageStreamReleaseFrame call failed." << endl;
		return false;
	}
	//m_cLastDepthFrameTimeStamp = currentDepthFrameTime;
	return true;
	// To enable playback of a .xed file through Kinect Studio and reset of the reconstruction
	// if the .xed loops, we test for when the frame timestamp has skipped a large number. 
	// Note: this will potentially continually reset live reconstructions on slow machines which
	// cannot process a live frame in less time than the reset threshold. Increase the number of
	// milliseconds in cResetOnTimeStampSkippedMilliseconds if this is a problem.
	
	//Reset Reconstruction on too large gap
	/*if (m_bAutoResetReconstructionOnTimeout && m_cFrameCounter != 0
		&& abs(currentDepthFrameTime.QuadPart - m_cLastDepthFrameTimeStamp.QuadPart) > cResetOnTimeStampSkippedMilliseconds)
	{
		ResetReconstruction();

		if (FAILED(hr))
		{
			 false;
		}
	}*/
}

void KinectFusionRenderOculus::Update()
{
	if (nullptr == m_pNuiSensor)
	{
		cout << "Sensor is null" << endl;
		return;
	}
	//Should color be here???
	if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0))
	{
		//GetKinectFrames(colorSynchronized);
		ProcessDepth();
		//ProcessDepthAdvanced();
	}
}

HRESULT KinectFusionRenderOculus::InitializeKinectFusion()
{
	HRESULT hr = S_OK;

	// Check to ensure suitable DirectX11 compatible hardware exists before initializing Kinect Fusion
	WCHAR description[MAX_PATH];
	WCHAR instancePath[MAX_PATH];
	UINT memorySize = 0;

	if (FAILED(hr = NuiFusionGetDeviceInfo(
		m_processorType,
		m_deviceIndex,
		&description[0],
		ARRAYSIZE(description),
		&instancePath[0],
		ARRAYSIZE(instancePath),
		&memorySize)))
	{
		if (hr == E_NUI_BADINDEX)
		{
			// This error code is returned either when the device index is out of range for the processor 
			// type or there is no DirectX11 capable device installed. As we set -1 (auto-select default) 
			// for the device index in the parameters, this indicates that there is no DirectX11 capable 
			// device. The options for users in this case are to either install a DirectX11 capable device
			// (see documentation for recommended GPUs) or to switch to non-real-time CPU based 
			// reconstruction by changing the processor type to NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU.
			cout<<"No DirectX11 device detected, or invalid device index - Kinect Fusion requires a DirectX11 device for GPU-based reconstruction."<<endl;
		}
		else
		{
			cout << "Failed in call to NuiFusionGetDeviceInfo." << endl;
		}
		return hr;
	}

	// Create the Kinect Fusion Reconstruction Volume
	hr = NuiFusionCreateColorReconstruction(
		&m_reconstructionParams,
		m_processorType, m_deviceIndex,
		&m_worldToCameraTransform,
		&m_pVolume);

	if (FAILED(hr))
	{
		if (E_NUI_GPU_FAIL == hr)
		{
			WCHAR buf[MAX_PATH];
			swprintf_s(buf, ARRAYSIZE(buf), L"Device %d not able to run Kinect Fusion, or error initializing.", m_deviceIndex);
			cout<<buf<<endl;
		}
		else if (E_NUI_GPU_OUTOFMEMORY == hr)
		{
			WCHAR buf[MAX_PATH];
			swprintf_s(buf, ARRAYSIZE(buf), L"Device %d out of memory error initializing reconstruction - try a smaller reconstruction volume.", m_deviceIndex);
			cout << buf << endl;
		}
		else if (NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU != m_processorType)
		{
			WCHAR buf[MAX_PATH];
			swprintf_s(buf, ARRAYSIZE(buf), L"Failed to initialize Kinect Fusion reconstruction volume on device %d.", m_deviceIndex);
			cout << buf << endl;
		}
		else
		{
			cout << "Failed to initialize Kinect Fusion reconstruction volume on CPU." << endl;
		}

		return hr;
	}

	// Save the default world to volume transformation to be optionally used in ResetReconstruction
	hr = m_pVolume->GetCurrentWorldToVolumeTransform(&m_defaultWorldToVolumeTransform);
	if (FAILED(hr))
	{
		cout << "Failed in call to GetCurrentWorldToVolumeTransform." << endl;
		return hr;
	}

	if (m_bTranslateToCenterOfVolume)
	{
		// This call will set the world-volume transformation
		hr = ResetReconstruction();
		if (FAILED(hr))
		{
			return hr;
		}
	}

	// Frames generated from the depth input
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, m_cDepthWidth, m_cDepthHeight, nullptr, &m_pDepthFloatImage);
	if (FAILED(hr))
	{
		cout<<"Failed to initialize Kinect Fusion image."<<endl;
		return hr;
	}
	
	// Frames generated from the color input
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, m_cColorWidth, m_cColorHeight, nullptr, &m_pColorImage);
	if (FAILED(hr))
	{
		cout<<"Failed to initialize Kinect Fusion image."<<endl;
		return hr;
	}

	// Create images to raycast the Reconstruction Volume
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, m_cDepthWidth, m_cDepthHeight, nullptr, &m_pPointCloud);
	if (FAILED(hr))
	{
		cout << "Failed to initialize Kinect Fusion image." << endl;
		return hr;
	}

	// Create images to raycast the Reconstruction Volume
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, m_cDepthWidth, m_cDepthHeight, nullptr, &m_pPrevPointCloud);
	if (FAILED(hr))
	{
		cout<<"Failed to initialize Kinect Fusion image."<<endl;
		return hr;
	}

	// Create images to raycast the Reconstruction Volume
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, m_cDepthWidth, m_cDepthHeight, nullptr, &m_pShadedSurface);
	if (FAILED(hr))
	{
		cout<<"Failed to initialize Kinect Fusion image."<<endl;
		return hr;
	}
	
	// Frames generated from the color input aligned to depth - same size as depth
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, m_cDepthWidth, m_cDepthHeight, nullptr, &m_pResampledColorImageDepthAligned);
	if (FAILED(hr))
	{
		cout<<"Failed to initialize Kinect Fusion image."<<endl;
		return hr;
	}
	//Buffer to store DepthImage
	m_pDepthImagePixelBuffer = new(std::nothrow) NUI_DEPTH_IMAGE_PIXEL[m_cDepthImagePixels];
	if (nullptr == m_pDepthImagePixelBuffer)
	{
		cout << "Failed to initialize Kinect Fusion depth image pixel buffer." << endl;
		return hr;
	}

	// Setup color coordinate image for depth to color mapping - this must be the same size as the depth
	m_pColorCoordinates = new(std::nothrow) NUI_COLOR_IMAGE_POINT[m_cDepthImagePixels];
	if (nullptr == m_pColorCoordinates)
	{
		cout<<"Failed to initialize Kinect Fusion color image pixel buffer."<<endl;
		return hr;
	}

	m_fStartTime = m_timer.AbsoluteTime();
	cout << "Kinect Fusion Initialised" << endl;

	return hr;
}
/// <summary>
/// Reset the tracking flags
/// </summary>
void KinectFusionRenderOculus::ResetTracking()
{
	m_bTrackingFailed = false;
	m_bTrackingHasFailedPreviously = false;

	m_cLostFrameCounter = 0;
	m_cSuccessfulFrameCounter = 0;

	m_bIntegrationResumed = true;
	// Reset pause and signal that the integration resumed
	//remove any attempt to pause integration

	

	if (nullptr != m_pCameraPoseFinder)
	{
		m_pCameraPoseFinder->ResetCameraPoseFinder();
	}
}
HRESULT KinectFusionRenderOculus::ResetReconstruction()
{
	if (nullptr == m_pVolume)
	{
		return E_FAIL;
	}

	HRESULT hr = S_OK;

	SetIdentityMatrix(m_worldToCameraTransform);
	
	//Translate to center of voulme as that ismore appropriate for our task
	if (m_bTranslateToCenterOfVolume)
	{
		Matrix4 worldToVolumeTransform = m_defaultWorldToVolumeTransform;
		worldToVolumeTransform.M41 = m_reconstructionParams.voxelCountX / 2;
		worldToVolumeTransform.M42 = m_reconstructionParams.voxelCountY / 2;
		worldToVolumeTransform.M43 = m_reconstructionParams.voxelCountZ / 2;
		hr = m_pVolume->ResetReconstruction(&m_worldToCameraTransform, &worldToVolumeTransform);
	}
	// Translate the reconstruction volume location away from the world origin by an amount equal
	// to the minimum depth threshold. This ensures that some depth signal falls inside the volume.
	// If set false, the default world origin is set to the center of the front face of the 
	// volume, which has the effect of locating the volume directly in front of the initial camera
	// position with the +Z axis into the volume along the initial camera direction of view.
	
	else if (m_bTranslateResetPoseByMinDepthThreshold)
	{
		Matrix4 worldToVolumeTransform = m_defaultWorldToVolumeTransform;

		// Translate the volume in the Z axis by the minDepthThreshold distance
		float minDist = (m_fMinDepthThreshold < m_fMaxDepthThreshold) ? m_fMinDepthThreshold : m_fMaxDepthThreshold;
		worldToVolumeTransform.M43 -= (minDist * m_reconstructionParams.voxelsPerMeter);

		hr = m_pVolume->ResetReconstruction(&m_worldToCameraTransform, &worldToVolumeTransform);
	}
	else
	{
		hr = m_pVolume->ResetReconstruction(&m_worldToCameraTransform, nullptr);
	}

	m_cLostFrameCounter = 0;
	m_cFrameCounter = 0;
	m_fStartTime = m_timer.AbsoluteTime();

	if (SUCCEEDED(hr))
	{
		m_bTrackingFailed = false;

		cout << "Reconstruction has been reset." << endl;;
	}
	else
	{
		cout << "Failed to reset reconstruction." << endl;
	}

	if (SUCCEEDED(hr))
	{
		ResetTracking();
	}

	return hr;
}

HRESULT KinectFusionRenderOculus::CopyExtendedDepth(NUI_IMAGE_FRAME & imageFrame)
{
	HRESULT hr = S_OK;

	if (nullptr == m_pDepthImagePixelBuffer)
	{
		cout<<"Error depth image pixel buffer is nullptr."<<endl;
		return E_FAIL;
	}
	INuiFrameTexture *extendedDepthTex = nullptr;

	// Extract the extended depth in NUI_DEPTH_IMAGE_PIXEL format from the frame
	BOOL nearModeOperational = FALSE;
	hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(m_pDepthStreamHandle, &imageFrame, &nearModeOperational, &extendedDepthTex);
	if (FAILED(hr))
	{
		cout<<"Error getting extended depth texture."<<endl;
		return hr;
	}

	NUI_LOCKED_RECT extendedDepthLockedRect;

	// Lock the frame data to access the un-clamped NUI_DEPTH_IMAGE_PIXELs
	hr = extendedDepthTex->LockRect(0, &extendedDepthLockedRect, nullptr, 0);

	if (FAILED(hr) || extendedDepthLockedRect.Pitch == 0)
	{
		cout<<"Error getting extended depth texture pixels."<<endl;
		return hr;
	}

	// Copy the depth pixels so we can return the image frame
	errno_t err = memcpy_s(m_pDepthImagePixelBuffer, m_cDepthImagePixels * sizeof(NUI_DEPTH_IMAGE_PIXEL), extendedDepthLockedRect.pBits, extendedDepthTex->BufferLen());
	extendedDepthTex->UnlockRect(0);

	if (0 != err)
	{
		cout<<"Error copying extended depth texture pixels."<<endl;
		return hr;
	}
	return hr;
}

//
//void KinectFusionRenderOculus::ProcessDepth()
//{
//	HRESULT hr;
//	if (!AquireDepth())
//	{
//		return;
//	}
//
//	// Return if the volume is not initialized
//	if (nullptr == m_pVolume)
//	{
//		cout<<"Kinect Fusion reconstruction volume not initialized. Please try reducing volume size or restarting.";
//		return;
//	}
//
//	////////////////////////////////////////////////////////
//	// Depth to DepthFloat
//
//	// Convert the pixels describing extended depth as unsigned short type in millimeters to depth
//	// as floating point type in meters.
//	hr = m_pVolume->DepthToDepthFloatFrame(m_pDepthImagePixelBuffer, m_cDepthImagePixels * sizeof(NUI_DEPTH_IMAGE_PIXEL), m_pDepthFloatImage, m_fMinDepthThreshold, m_fMaxDepthThreshold, m_bMirrorDepthFrame);
//
//	if (FAILED(hr))
//	{
//		cout<<"Kinect Fusion NuiFusionDepthToDepthFloatFrame call failed."<<endl;
//		return;
//	}
//	else
//	{
//		KinectDepthFloatImageToOpenCV(m_pDepthFloatImage);
//	}
//
//	////////////////////////////////////////////////////////
//	// ProcessFrame
//
//	// Perform the camera tracking and update the Kinect Fusion Volume
//	// This will create memory on the GPU, upload the image, run camera tracking and integrate the
//	// data into the Reconstruction Volume if successful. Note that passing nullptr as the final 
//	// parameter will use and update the internal camera pose.
//	hr = m_pVolume->ProcessFrame(m_pDepthFloatImage, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT, m_cMaxIntegrationWeight, &m_worldToCameraTransform);
//
//	// Test to see if camera tracking failed. 
//	// If it did fail, no data integration or raycast for reference points and normals will have taken 
//	//  place, and the internal camera pose will be unchanged.
//	if (FAILED(hr))
//	{
//		if (hr == E_NUI_FUSION_TRACKING_ERROR)
//		{
//			//Add code to perform relative tracking on poses
//			m_cLostFrameCounter++;
//			m_bTrackingFailed = true;
//			cout<<"Kinect Fusion camera tracking failed! Align the camera to the last tracked position. "<<endl;
//		}
//		else
//		{
//			cout<<"Kinect Fusion ProcessFrame call failed!"<<endl;
//			return;
//		}
//	}
//	else
//	{
//		Matrix4 calculatedCameraPose;
//		hr = m_pVolume->GetCurrentWorldToCameraTransform(&calculatedCameraPose);
//
//		if (SUCCEEDED(hr))
//		{
//			// Set the pose
//			m_worldToCameraTransform = calculatedCameraPose;
//			m_cLostFrameCounter = 0;
//			m_bTrackingFailed = false;
//		}
//	}
//
//	hr = m_pVolume->CalculatePointCloud(m_pPrevPointCloud, &m_worldToCameraTransform);
//
//	if (FAILED(hr))
//	{
//		cout<<"Kinect Fusion CalculatePointCloud call failed."<<endl;
//		return;
//	}
//
//	
//}

HRESULT KinectFusionRenderOculus::CopyColor(NUI_IMAGE_FRAME &imageFrame)
{
	HRESULT hr = S_OK;

	if (nullptr == m_pColorImage)
	{
		cout<<"Error copying color texture pixels."<<endl;
		return E_FAIL;
	}

	INuiFrameTexture *srcColorTex = imageFrame.pFrameTexture;
	INuiFrameTexture *destColorTex = m_pColorImage->pFrameTexture;

	if (nullptr == srcColorTex || nullptr == destColorTex)
	{
		return E_NOINTERFACE;
	}

	// Lock the frame data to access the color pixels
	NUI_LOCKED_RECT srcLockedRect;

	hr = srcColorTex->LockRect(0, &srcLockedRect, nullptr, 0);

	if (FAILED(hr) || srcLockedRect.Pitch == 0)
	{
		cout<<"Error getting color texture pixels."<<endl;
		return E_NOINTERFACE;
	}

	// Lock the frame data to access the color pixels
	NUI_LOCKED_RECT destLockedRect;

	hr = destColorTex->LockRect(0, &destLockedRect, nullptr, 0);

	if (FAILED(hr) || destLockedRect.Pitch == 0)
	{
		srcColorTex->UnlockRect(0);
		cout<<"Error copying color texture pixels."<<endl;
		return E_NOINTERFACE;
	}

	// Copy the color pixels so we can return the image frame
	errno_t err = memcpy_s(
		destLockedRect.pBits,
		m_cColorImagePixels * cBytesPerPixel,
		srcLockedRect.pBits,
		srcLockedRect.size);

	srcColorTex->UnlockRect(0);
	destColorTex->UnlockRect(0);

	if (0 != err)
	{
		cout<<"Error copying color texture pixels."<<endl;
		hr = E_FAIL;
	}

	return hr;
}

bool KinectFusionRenderOculus::IsCameraPoseFinderAvailable()
{
	return m_bAutoFindCameraPoseWhenLost
		&& (nullptr != m_pCameraPoseFinder)
		&& m_pCameraPoseFinder->GetStoredPoseCount() > 0;
}

HRESULT KinectFusionRenderOculus::StoreImageToFrameBuffer(
	const NUI_FUSION_IMAGE_FRAME* imageFrame,
	BYTE* buffer)
{

	HRESULT hr = S_OK;

	if (nullptr == imageFrame || nullptr == imageFrame->pFrameTexture || nullptr == buffer)
	{
		return E_INVALIDARG;
	}

	if (NUI_FUSION_IMAGE_TYPE_COLOR != imageFrame->imageType &&
		NUI_FUSION_IMAGE_TYPE_FLOAT != imageFrame->imageType)
	{
		return E_INVALIDARG;
	}

	if (0 == imageFrame->width || 0 == imageFrame->height)
	{
		return E_NOINTERFACE;
	}

	INuiFrameTexture *imageFrameTexture = imageFrame->pFrameTexture;
	NUI_LOCKED_RECT LockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	imageFrameTexture->LockRect(0, &LockedRect, nullptr, 0);

	// Make sure we've received valid data
	if (LockedRect.Pitch != 0)
	{
		// Convert from floating point depth if required
		if (NUI_FUSION_IMAGE_TYPE_FLOAT == imageFrame->imageType)
		{
			// Depth ranges set here for better visualization, and map to black at 0 and white at 4m
			const FLOAT range = 4.0f;
			const FLOAT oneOverRange = (1.0f / range) * 256.0f;
			const FLOAT minRange = 0.0f;

			const float *pFloatBuffer = reinterpret_cast<float *>(LockedRect.pBits);

			Concurrency::parallel_for(0u, imageFrame->height, [&](unsigned int y)
			{
				unsigned int* pColorRow = reinterpret_cast<unsigned int*>(reinterpret_cast<unsigned char*>(buffer) + (y * LockedRect.Pitch));
				const float* pFloatRow = reinterpret_cast<const float*>(reinterpret_cast<const unsigned char*>(pFloatBuffer) + (y * LockedRect.Pitch));

				for (unsigned int x = 0; x < imageFrame->width; ++x)
				{
					float depth = pFloatRow[x];

					// Note: Using conditionals in this loop could degrade performance.
					// Consider using a lookup table instead when writing production code.
					BYTE intensity = (depth >= minRange) ?
						static_cast<BYTE>((int)((depth - minRange) * oneOverRange) % 256) :
						0; // % 256 to enable it to wrap around after the max range

					pColorRow[x] = (255 << 24) | (intensity << 16) | (intensity << 8) | intensity;
				}
			});
		}
		else	// already in 4 bytes per int (RGBA/BGRA) format
		{
			const size_t destPixelCount =
				m_cDepthWidth * m_cDepthHeight;

			BYTE * pBuffer = (BYTE *)LockedRect.pBits;

			// Draw the data with Direct2D
			memcpy_s(
				buffer,
				destPixelCount * cBytesPerPixel,
				pBuffer,
				imageFrame->width * imageFrame->height * cBytesPerPixel);
		}
	}
	else
	{
		return E_NOINTERFACE;
	}

	// We're done with the texture so unlock it
	imageFrameTexture->UnlockRect(0);

	return hr;
}
/// <summary>
/// Perform camera tracking using AlignDepthFloatToReconstruction
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectFusionRenderOculus::TrackCameraAlignDepthFloatToReconstruction(Matrix4 &calculatedCameraPose, FLOAT &alignmentEnergy)
{
	HRESULT hr = S_OK;

	// Only calculate the residual delta from reference frame every m_cDeltaFromReferenceFrameCalculationInterval
	// frames to reduce computation time
	HRESULT tracking = S_OK;

	if (m_bCalculateDeltaFrame)
	{
		tracking = m_pVolume->AlignDepthFloatToReconstruction(
			m_pDepthFloatImage,
			NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
			m_pFloatDeltaFromReference,
			&alignmentEnergy,
			&calculatedCameraPose);
	}
	else
	{
		tracking = m_pVolume->AlignDepthFloatToReconstruction(
			m_pDepthFloatImage,
			NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
			nullptr,
			&alignmentEnergy,
			&calculatedCameraPose);
	}

	bool trackingSuccess = !(FAILED(tracking) || alignmentEnergy > m_fMaxAlignToReconstructionEnergyForSuccess || (alignmentEnergy == 0.0f && m_cSuccessfulFrameCounter > 1));

	if (trackingSuccess)
	{
		// Get the camera pose
		m_pVolume->GetCurrentWorldToCameraTransform(&calculatedCameraPose);
	}
	else
	{
		if (FAILED(tracking))
		{
			hr = tracking;
		}
		else
		{
			// We failed in the energy check
			hr = E_NUI_FUSION_TRACKING_ERROR;
		}
	}

	return hr;
}

/// <summary>
/// Perform camera tracking using AlignPointClouds
/// </summary>
HRESULT KinectFusionRenderOculus::TrackCameraAlignPointClouds(Matrix4 &calculatedCameraPose, FLOAT &alignmentEnergy)
{

	////////////////////////////////////////////////////////
	// Down sample the depth image

	HRESULT hr = DownsampleFrameNearestNeighbor(
		m_pDepthFloatImage,
		m_pDownsampledDepthFloatImage,
		m_cAlignPointCloudsImageDownsampleFactor);

	if (FAILED(hr))
	{
		cout<<"Kinect Fusion DownsampleFrameNearestNeighbor call failed."<<endl;
		return hr;
	}

	////////////////////////////////////////////////////////
	// Smooth depth image

	hr = m_pVolume->SmoothDepthFloatFrame(
		m_pDownsampledDepthFloatImage,
		m_pDownsampledSmoothDepthFloatImage,
		m_cSmoothingKernelWidth,
		m_fSmoothingDistanceThreshold);

	if (FAILED(hr))
	{
		cout<<"Kinect Fusion SmoothDepth call failed."<<endl;
		return hr;
	}

	////////////////////////////////////////////////////////
	// Calculate Point Cloud from smoothed input Depth Image

	hr = NuiFusionDepthFloatFrameToPointCloud(
		m_pDownsampledSmoothDepthFloatImage,
		m_pDownsampledDepthPointCloud);

	if (FAILED(hr))
	{
		cout<<"Kinect Fusion NuiFusionDepthFloatFrameToPointCloud call failed."<<endl;
		return hr;
	}

	////////////////////////////////////////////////////////
	// CalculatePointCloud

	// Raycast even if camera tracking failed, to enable us to visualize what is 
	// happening with the system
	hr = m_pVolume->CalculatePointCloud(
		m_pDownsampledRaycastPointCloud,
		nullptr,
		&calculatedCameraPose);

	if (FAILED(hr))
	{
		cout<<L"Kinect Fusion CalculatePointCloud call failed."<<endl;
		return hr;
	}

	////////////////////////////////////////////////////////
	// Call AlignPointClouds

	HRESULT tracking = S_OK;

	// Only calculate the residual delta from reference frame every m_cDeltaFromReferenceFrameCalculationInterval
	// frames to reduce computation time
	if (m_bCalculateDeltaFrame)
	{
		tracking = NuiFusionAlignPointClouds(
			m_pDownsampledRaycastPointCloud,
			m_pDownsampledDepthPointCloud,
			NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
			m_pDownsampledShadedDeltaFromReference,
			&calculatedCameraPose);

		// Up sample the delta from reference image to display as the original resolution
		hr = UpsampleFrameNearestNeighbor(
			m_pDownsampledShadedDeltaFromReference,
			m_pShadedDeltaFromReference,
			m_cAlignPointCloudsImageDownsampleFactor);

		if (FAILED(hr))
		{
			cout<<"Kinect Fusion UpsampleFrameNearestNeighbor call failed."<<endl;
			return hr;
		}
	}
	else
	{
		tracking = NuiFusionAlignPointClouds(
			m_pDownsampledRaycastPointCloud,
			m_pDownsampledDepthPointCloud,
			NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
			nullptr,
			&calculatedCameraPose);
	}

	if (!FAILED(tracking))
	{
		// Perform additional transform magnitude check
		// Camera Tracking has converged but did we get a sensible pose estimate?
		// see if relative rotation and translation exceed thresholds 
		if (CameraTransformFailed(
			m_worldToCameraTransform,
			calculatedCameraPose,
			m_fMaxTranslationDelta,
			m_fMaxRotationDelta))
		{
			// We calculated too large a move for this to be a sensible estimate,
			// quite possibly the camera tracking drifted. Force camera pose finding.
			hr = E_NUI_FUSION_TRACKING_ERROR;

			cout<<"Kinect Fusion AlignPointClouds camera tracking failed "<<"in transform magnitude check!"<<endl;
		}
	}
	else
	{
		hr = tracking;
	}

	return hr;
}

/// Process the color image for the camera pose finder.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectFusionRenderOculus::ProcessColorForCameraPoseFinder(bool &resampled)
{
	HRESULT hr = S_OK;

	// If color and depth are different resolutions we first we re-sample the color frame using nearest neighbor
	// before passing to the CameraPoseFinder
	if (m_cDepthImagePixels != m_cColorImagePixels)
	{
		if (m_pColorImage->width > m_pResampledColorImage->width)
		{
			// Down-sample
			unsigned int factor = m_pColorImage->width / m_pResampledColorImage->width;
			hr = DownsampleFrameNearestNeighbor(m_pColorImage, m_pResampledColorImage, factor);

			if (FAILED(hr))
			{
				cout<<"Kinect Fusion DownsampleFrameNearestNeighbor call failed."<<endl;
				return hr;
			}
		}
		else
		{
			// Up-sample
			unsigned int factor = m_pResampledColorImage->width / m_pColorImage->width;
			hr = UpsampleFrameNearestNeighbor(m_pColorImage, m_pResampledColorImage, factor);

			if (FAILED(hr))
			{
				cout<<"Kinect Fusion UpsampleFrameNearestNeighbor call failed."<<endl;
				return hr;
			}
		}

		resampled = true;
	}
	else
	{
		resampled = false;
	}

	return hr;
}

/// Perform camera pose finding when tracking is lost using AlignPointClouds.
/// This is typically more successful than FindCameraPoseAlignDepthFloatToReconstruction.
/// </summary>
HRESULT KinectFusionRenderOculus::FindCameraPoseAlignPointClouds()
{
	HRESULT hr = S_OK;

	if (!IsCameraPoseFinderAvailable())
	{
		return E_FAIL;
	}

	bool resampled = false;

	hr = ProcessColorForCameraPoseFinder(resampled);

	if (FAILED(hr))
	{
		return hr;
	}

	// Start  kNN (k nearest neighbors) camera pose finding
	INuiFusionMatchCandidates *pMatchCandidates = nullptr;

	// Test the camera pose finder to see how similar the input images are to previously captured images.
	// This will return an error code if there are no matched frames in the camera pose finder database.
	hr = m_pCameraPoseFinder->FindCameraPose(
		m_pDepthFloatImage,
		resampled ? m_pResampledColorImage : m_pColorImage,
		&pMatchCandidates);

	if (FAILED(hr) || nullptr == pMatchCandidates)
	{
		goto FinishFrame;
	}

	unsigned int cPoses = pMatchCandidates->MatchPoseCount();

	float minDistance = 1.0f;   // initialize to the maximum normalized distance
	hr = pMatchCandidates->CalculateMinimumDistance(&minDistance);

	if (FAILED(hr) || 0 == cPoses)
	{
		goto FinishFrame;
	}

	// Check the closest frame is similar enough to our database to re-localize
	// For frames that have a larger minimum distance, standard tracking will run
	// and if this fails, tracking will be considered lost.
	if (minDistance >= m_fCameraPoseFinderDistanceThresholdReject)
	{
		cout<<"FindCameraPose exited early as not good enough pose matches."<<endl;
		hr = E_NUI_NO_MATCH;
		goto FinishFrame;
	}

	// Get the actual matched poses
	const Matrix4 *pNeighbors = nullptr;
	hr = pMatchCandidates->GetMatchPoses(&pNeighbors);

	if (FAILED(hr))
	{
		goto FinishFrame;
	}

	////////////////////////////////////////////////////////
	// Smooth depth image

	hr = m_pVolume->SmoothDepthFloatFrame(
		m_pDepthFloatImage,
		m_pSmoothDepthFloatImage,
		m_cSmoothingKernelWidth,
		m_fSmoothingDistanceThreshold); // ON GPU

	if (FAILED(hr))
	{
		cout << "Kinect Fusion SmoothDepth call failed." << endl;
		goto FinishFrame;
	}

	////////////////////////////////////////////////////////
	// Calculate Point Cloud from smoothed input Depth Image

	hr = NuiFusionDepthFloatFrameToPointCloud(
		m_pSmoothDepthFloatImage,
		m_pDepthPointCloud);

	if (FAILED(hr))
	{
		cout<<"Kinect Fusion NuiFusionDepthFloatFrameToPointCloud call failed."<<endl;
		goto FinishFrame;
	}

	HRESULT tracking = S_OK;
	FLOAT alignmentEnergy = 0;

	unsigned short relocIterationCount = NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT;

	double smallestEnergy = DBL_MAX;
	int smallestEnergyNeighborIndex = -1;

	int bestNeighborIndex = -1;
	Matrix4 bestNeighborCameraPose;
	SetIdentityMatrix(bestNeighborCameraPose);
	// Exclude very tiny alignment energy case which is unlikely to happen in reality - this is more likely a tracking error
	double bestNeighborAlignmentEnergy = m_fMaxAlignPointCloudsEnergyForSuccess;

	// Run alignment with best matched poses (i.e. k nearest neighbors (kNN))
	unsigned int maxTests = min(m_cMaxCameraPoseFinderPoseTests, cPoses);

	for (unsigned int n = 0; n < maxTests; n++)
	{
		////////////////////////////////////////////////////////
		// Call AlignPointClouds

		Matrix4 poseProposal = pNeighbors[n];

		// Get the saved pose view by raycasting the volume
		hr = m_pVolume->CalculatePointCloud(m_pRaycastPointCloud, nullptr, &poseProposal);

		tracking = m_pVolume->AlignPointClouds(
			m_pRaycastPointCloud,
			m_pDepthPointCloud,
			relocIterationCount,
			nullptr,
			&alignmentEnergy,
			&poseProposal);


		if (SUCCEEDED(tracking) && alignmentEnergy < bestNeighborAlignmentEnergy  && alignmentEnergy > m_fMinAlignPointCloudsEnergyForSuccess)
		{
			bestNeighborAlignmentEnergy = alignmentEnergy;
			bestNeighborIndex = n;

			// This is after tracking succeeds, so should be a more accurate pose to store...
			bestNeighborCameraPose = poseProposal;
		}

		// Find smallest energy neighbor independent of tracking success
		if (alignmentEnergy < smallestEnergy)
		{
			smallestEnergy = alignmentEnergy;
			smallestEnergyNeighborIndex = n;
		}
	}

	// Use the neighbor with the smallest residual alignment energy
	// At the cost of additional processing we could also use kNN+Mean camera pose finding here
	// by calculating the mean pose of the best n matched poses and also testing this to see if the 
	// residual alignment energy is less than with kNN.
	if (bestNeighborIndex > -1)
	{
		m_worldToCameraTransform = bestNeighborCameraPose;

		// Get the saved pose view by raycasting the volume
		hr = m_pVolume->CalculatePointCloud(m_pRaycastPointCloud, nullptr, &m_worldToCameraTransform);

		if (FAILED(hr))
		{
			goto FinishFrame;
		}

		// Tracking succeeded!
		hr = S_OK;

		SetTrackingSucceeded();

		// Run a single iteration of AlignPointClouds to get the deltas frame
		hr = m_pVolume->AlignPointClouds(
			m_pRaycastPointCloud,
			m_pDepthPointCloud,
			1,
			m_pShadedDeltaFromReference,
			&alignmentEnergy,
			&bestNeighborCameraPose);

		if (SUCCEEDED(hr))
		{
			StoreImageToFrameBuffer(m_pShadedDeltaFromReference, m_pTrackingDataRGBX);
		}

		// Stop the residual image being displayed as we have stored our own
		m_bCalculateDeltaFrame = false;

		WCHAR str[MAX_PATH];
		swprintf_s(str, ARRAYSIZE(str), L"Camera Pose Finder SUCCESS! Residual energy=%f, %d frames stored, minimum distance=%f, best match index=%d", bestNeighborAlignmentEnergy, cPoses, minDistance, bestNeighborIndex);
		cout<<str<<endl;
	}
	else
	{
		m_worldToCameraTransform = pNeighbors[smallestEnergyNeighborIndex];

		// Get the smallest energy view by raycasting the volume
		hr = m_pVolume->CalculatePointCloud(m_pRaycastPointCloud, nullptr, &m_worldToCameraTransform);

		if (FAILED(hr))
		{
			goto FinishFrame;
		}

		// Camera pose finding failed - return the tracking failed error code
		hr = E_NUI_FUSION_TRACKING_ERROR;

		// Tracking Failed will be set again on the next iteration in ProcessDepth
		WCHAR str[MAX_PATH];
		swprintf_s(str, ARRAYSIZE(str), L"Camera Pose Finder FAILED! Residual energy=%f, %d frames stored, minimum distance=%f, best match index=%d", smallestEnergy, cPoses, minDistance, smallestEnergyNeighborIndex);
		cout<<(str);
	}

FinishFrame:

	SafeRelease(pMatchCandidates);

	return hr;
}

/// Update the camera pose finder data.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectFusionRenderOculus::UpdateCameraPoseFinder()
{
	//cout << "Update Camera Pose" << endl;
	HRESULT hr = S_OK;

	if (nullptr == m_pDepthFloatImage || nullptr == m_pColorImage
		|| nullptr == m_pResampledColorImage || nullptr == m_pCameraPoseFinder)
	{
		return E_FAIL;
	}

	bool resampled = false;

	hr = ProcessColorForCameraPoseFinder(resampled);

	if (FAILED(hr))
	{
		return hr;
	}

	BOOL poseHistoryTrimmed = FALSE;
	BOOL addedPose = FALSE;

	// This function will add the pose to the camera pose finding database when the input frame's minimum
	// distance to the existing database is equal to or above m_fDistanceThresholdAccept (i.e. indicating 
	// that the input has become dis-similar to the existing database and a new frame should be captured).
	// Note that the color and depth frames must be the same size, however, the horizontal mirroring
	// setting does not have to be consistent between depth and color. It does have to be consistent
	// between camera pose finder database creation and calling FindCameraPose though, hence we always
	// reset both the reconstruction and database when changing the mirror depth setting.
	hr = m_pCameraPoseFinder->ProcessFrame(
		m_pDepthFloatImage,
		resampled ? m_pResampledColorImage : m_pColorImage,
		&m_worldToCameraTransform,
		m_fCameraPoseFinderDistanceThresholdAccept,
		&addedPose,
		&poseHistoryTrimmed);

	if (TRUE == addedPose)
	{
		WCHAR str[MAX_PATH];
		swprintf_s(str, ARRAYSIZE(str), L"Camera Pose Finder Added Frame! %d frames stored, minimum distance>=%f\n", m_pCameraPoseFinder->GetStoredPoseCount(), m_fCameraPoseFinderDistanceThresholdAccept);
		cout << str << endl;;
	}

	if (TRUE == poseHistoryTrimmed)
	{
		cout<<"Kinect Fusion Camera Pose Finder pose history is full, overwritten oldest pose to store current pose."<<endl;
	}

	if (FAILED(hr))
	{
		cout<<"Kinect Fusion Camera Pose Finder Process Frame call failed."<<endl;
	}

	return hr;
}
void KinectFusionRenderOculus::printMat(Matrix4 mat)
{
	cout << mat.M11 << mat.M12 << mat.M13  << mat.M14 << endl;
	cout<<mat.M21 << mat.M22 << mat.M23 << mat.M24 << endl;
	cout << mat.M31 << mat.M32 << mat.M33<< mat.M34 << endl;
	cout << mat.M41 << mat.M42 <<mat.M43 << mat.M44 << endl;
}
void KinectFusionRenderOculus::ProcessDepthAdvanced()
{

	HRESULT hr = S_OK;
	bool depthAvailable = false;
	bool raycastFrame = false;
	bool cameraPoseFinderAvailable = IsCameraPoseFinderAvailable();
	bool integrateColor = m_bCaptureColor && m_cFrameCounter % m_cColorIntegrationInterval == 0;
	bool colorSynchronized = false;
	FLOAT alignmentEnergy = 1.0f;
	Matrix4 calculatedCameraPose = m_worldToCameraTransform;
	m_bCalculateDeltaFrame = (m_cFrameCounter % m_cDeltaFromReferenceFrameCalculationInterval == 0)
		|| (m_bTrackingHasFailedPreviously && m_cSuccessfulFrameCounter <= 2);

	if (!GetKinectFrames(colorSynchronized))
	{
		goto FinishFrame;
	}
	//std::cout << colorSynchronized << endl;
	// Only integrate when color is synchronized with depth
	integrateColor = integrateColor && colorSynchronized;

	////////////////////////////////////////////////////////
	// Depth to Depth Float

	// Convert the pixels describing extended depth as unsigned short type in millimeters to depth
	// as floating point type in meters.
	if (nullptr == m_pVolume)
	{
		hr = NuiFusionDepthToDepthFloatFrame(
			m_pDepthImagePixelBuffer,
			m_cDepthWidth,
			m_cDepthHeight,
			m_pDepthFloatImage,
			m_fMinDepthThreshold,
			m_fMaxDepthThreshold,
			m_bMirrorDepthFrame);
	}
	else
	{
		hr = m_pVolume->DepthToDepthFloatFrame(
			m_pDepthImagePixelBuffer,
			m_cDepthImagePixels * sizeof(NUI_DEPTH_IMAGE_PIXEL),
			m_pDepthFloatImage,
			m_fMinDepthThreshold,
			m_fMaxDepthThreshold,
			m_bMirrorDepthFrame);
	}
	if (FAILED(hr))
	{
		cout << "Kinect Fusion NuiFusionDepthToDepthFloatFrame call failed." << endl;
		goto FinishFrame;
	}
	else
	{
		KinectDepthFloatImageToOpenCV(m_pDepthFloatImage);
	}
	depthAvailable = true;
	if (colorSynchronized && depthAvailable)
	{
		KinectColorFloatImageToOpenCV(m_pColorImage, "Color Image");
	}

	// Return if the volume is not initialized, just drawing the depth image
	if (nullptr == m_pVolume)
	{
		cout<<"Kinect Fusion reconstruction volume not initialized. "<<	L"Please try reducing volume size or restarting.";
		goto FinishFrame;
	}

	////////////////////////////////////////////////////////
	// Perform Camera Tracking

	HRESULT tracking = E_NUI_FUSION_TRACKING_ERROR;

	if (!m_bTrackingFailed && 0 != m_cFrameCounter)
	{
		// Here we can either call or TrackCameraAlignDepthFloatToReconstruction or TrackCameraAlignPointClouds
		// The TrackCameraAlignPointClouds function typically has higher performance with the camera pose finder 
		// due to its wider basin of convergence, enabling it to more robustly regain tracking from nearby poses
		// suggested by the camera pose finder after tracking is lost.
		if (false)
		{
			tracking = TrackCameraAlignPointClouds(calculatedCameraPose, alignmentEnergy);
		}
		else
		{
			// If the camera pose finder is not turned on, we use AlignDepthFloatToReconstruction
			tracking = TrackCameraAlignDepthFloatToReconstruction(calculatedCameraPose, alignmentEnergy);
		}
	}


	if (FAILED(tracking) && 0 != m_cFrameCounter)   // frame 0 always succeeds
	{
		hr = ResetReconstruction();

		if (SUCCEEDED(hr))
		{
			// Set bad tracking message
			cout << "Kinect Fusion camera tracking failed, " << "automatically reset volume." << endl;
		}
		else
		{
			cout << "Kinect Fusion Reset Reconstruction call failed." << endl;
			goto FinishFrame;
		}
		/*
		SetTrackingFailed();

		if (!cameraPoseFinderAvailable)
		{
			if (tracking == E_NUI_FUSION_TRACKING_ERROR)
			{
				WCHAR str[MAX_PATH];
				swprintf_s(str, ARRAYSIZE(str), L"Kinect Fusion camera tracking FAILED! Align the camera to the last tracked position.");
				cout<<str<<endl;
			}
			else
			{
				cout<<"Kinect Fusion camera tracking call failed!"<<endl;
				goto FinishFrame;
			}
		}
		else
		{
			// Here we try to find the correct camera pose, to re-localize camera tracking.
			// We can call either the version using AlignDepthFloatToReconstruction or the version 
			// using AlignPointClouds, which typically has a higher success rate with the camera pose finder.
			//tracking = FindCameraPoseAlignDepthFloatToReconstruction();
			tracking = FindCameraPoseAlignPointClouds();
			cout << "Using cam Pose finder" << endl;
			if (FAILED(tracking) && tracking != E_NUI_FUSION_TRACKING_ERROR)
			{
				cout<<"Kinect Fusion FindCameraPose call failed."<<endl;
				goto FinishFrame;
			}
		}*/
	}
	/*if (FAILED(tracking) && 0 != m_cFrameCounter)
	{
		SetTrackingFailed();
	}*/
	else
	{
		if (m_bTrackingHasFailedPreviously)
		{
			WCHAR str[MAX_PATH];
			if (!m_bAutoFindCameraPoseWhenLost)
			{
				swprintf_s(str, ARRAYSIZE(str), L"Kinect Fusion camera tracking RECOVERED! Residual energy=%f", alignmentEnergy);
			}
			else
			{
				swprintf_s(str, ARRAYSIZE(str), L"Kinect Fusion camera tracking RECOVERED!");
			}
			cout<<str<<endl;
		}

		m_worldToCameraTransform = calculatedCameraPose;
		//printMat(m_worldToCameraTransform);
		SetTrackingSucceeded();
	}

	//If tracking has been lost for some number of frames reset the reconstruction volume
/*	if (m_bAutoResetReconstructionWhenLost &&
		m_bTrackingFailed &&
		m_cLostFrameCounter >= cResetOnNumberOfLostFrames)
	{
		// Automatically Clear Volume and reset tracking if tracking fails
		hr = ResetReconstruction();

		if (SUCCEEDED(hr))
		{
			// Set bad tracking message
			cout<<"Kinect Fusion camera tracking failed, "<<"automatically reset volume."<<endl;
		}
		else
		{
			cout<<"Kinect Fusion Reset Reconstruction call failed."<<endl;
			goto FinishFrame;
		}
	}*/

	////////////////////////////////////////////////////////
	// Integrate Depth Data into volume

	// Don't integrate depth data into the volume if:
	// 1) tracking failed
	// 2) camera pose finder is off and we have paused capture
	// 3) camera pose finder is on and we are still under the m_cMinSuccessfulTrackingFramesForCameraPoseFinderAfterFailure
	//    number of successful frames count.
	bool integrateData = !m_bTrackingFailed || (cameraPoseFinderAvailable && !(m_bTrackingHasFailedPreviously && m_cSuccessfulFrameCounter < m_cMinSuccessfulTrackingFramesForCameraPoseFinderAfterFailure));
	if (integrateData)
	{
		if (cameraPoseFinderAvailable)
		{
			// If integration resumed, this will un-check the pause integration check box back on in the UI automatically
			m_bIntegrationResumed = true;
		}

		// Reset this flag as we are now integrating data again
		m_bTrackingHasFailedPreviously = false;

		if (integrateColor)
		{
			// Map the color frame to the depth - this fills m_pResampledColorImageDepthAligned
			MapColorToDepth();

			// Integrate the depth and color data into the volume from the calculated camera pose
			hr = m_pVolume->IntegrateFrame(
				m_pDepthFloatImage,
				m_pResampledColorImageDepthAligned,
				m_cMaxIntegrationWeight,
				NUI_FUSION_DEFAULT_COLOR_INTEGRATION_OF_ALL_ANGLES,
				&m_worldToCameraTransform);

			   m_bColorCaptured = true;
		}
		else
		{
			// Integrate just the depth data into the volume from the calculated camera pose
			hr = m_pVolume->IntegrateFrame(
				m_pDepthFloatImage,
				nullptr,
				m_cMaxIntegrationWeight,
				NUI_FUSION_DEFAULT_COLOR_INTEGRATION_OF_ALL_ANGLES,
				&m_worldToCameraTransform);
		}

		if (FAILED(hr))
		{
			cout<<"Kinect Fusion IntegrateFrame call failed."<<endl;
			goto FinishFrame;
		}
	}


	////////////////////////////////////////////////////////
	// Update camera pose finder, adding key frames to the database

	if (m_bAutoFindCameraPoseWhenLost && !m_bTrackingHasFailedPreviously
		&& m_cSuccessfulFrameCounter > m_cMinSuccessfulTrackingFramesForCameraPoseFinder
		&& m_cFrameCounter % m_cCameraPoseFinderProcessFrameCalculationInterval == 0
		&& colorSynchronized)
	{
		hr = UpdateCameraPoseFinder();

		if (FAILED(hr))
		{
			cout << "Kinect Fusion UpdateCameraPoseFinder call failed." << endl;
			goto FinishFrame;
		}
	}

FinishFrame:


	if (cameraPoseFinderAvailable)
	{
		// Do not set false, as camera pose finder will toggle automatically depending on whether it has
		// regained tracking (re-localized) and is integrating again.
		//m_bIntegrationResumed = m_bIntegrationResumed;
	}
	else
	{
		//m_bIntegrationResumed = m_bIntegrationResumed;
		m_bIntegrationResumed = false;
	}

	////////////////////////////////////////////////////////
	// Copy the images to their frame buffers
	
	if (depthAvailable)
	{
		StoreImageToFrameBuffer(m_pDepthFloatImage, m_pDepthRGBX);
	}
	
	if (raycastFrame)
	{
	/*	if (m_bCaptureColor)
		{
			StoreImageToFrameBuffer(m_pCapturedSurfaceColor, m_pReconstructionRGBX);
		}
		else if (m_bDisplaySurfaceNormals)
		{
			StoreImageToFrameBuffer(m_pShadedSurfaceNormals, m_pReconstructionRGBX);
		}
		else
		{
			StoreImageToFrameBuffer(m_pShadedSurface, m_pReconstructionRGBX);
		}*/
	}
	
	// Display raycast depth image when in pose finding mode
	if (m_bTrackingFailed && cameraPoseFinderAvailable)
	{
		StoreImageToFrameBuffer(m_pRaycastDepthFloatImage, m_pTrackingDataRGBX);
	}
	else
	{
		// Don't calculate the residual delta from reference frame every frame to reduce computation time
		if (m_bCalculateDeltaFrame)
		{
			if (!m_bAutoFindCameraPoseWhenLost)
			{
				// Color the float residuals from the AlignDepthFloatToReconstruction
				hr = ColorResiduals(m_pFloatDeltaFromReference, m_pShadedDeltaFromReference);
			}

			if (SUCCEEDED(hr))
			{
				StoreImageToFrameBuffer(m_pShadedDeltaFromReference, m_pTrackingDataRGBX);
			}
		}
	}
	m_cFrameCounter++;
	double elapsed = m_timer.AbsoluteTime() - m_fStartTime;
	if ((int)elapsed >= cTimeDisplayInterval)
	{
		double fps = (double)m_cFrameCounter / elapsed;

		// Update status display
		if (!m_bTrackingFailed)
		{
			WCHAR str[MAX_PATH];
			swprintf_s(str, ARRAYSIZE(str), L"Fps: %5.2f", fps);
			//SetStatusMessage(str);
		}

		m_cFrameCounter = 0;
		m_fStartTime = m_timer.AbsoluteTime();
	}



}

void KinectFusionRenderOculus::KinectColorFloatImageToOpenCV(NUI_FUSION_IMAGE_FRAME* colorImgFrame,string winName)
{
	//Extract pointer data from image
	INuiFrameTexture *pColorImageTexture = colorImgFrame->pFrameTexture;
	NUI_LOCKED_RECT colorLockedRect;
	HRESULT hr = pColorImageTexture->LockRect(0, &colorLockedRect, nullptr, 0);
	if (colorLockedRect.Pitch != 0)
	{
		BYTE* pBuffer = (BYTE *)colorLockedRect.pBits;
		m_floatColorOpenCV = cv::Mat(colorImgFrame->height, colorImgFrame->width, CV_8UC4, pBuffer);
	}
	pColorImageTexture->UnlockRect(0);
	imshow(winName, m_floatColorOpenCV);
}


void KinectFusionRenderOculus::KinectDepthFloatImageToOpenCV(NUI_FUSION_IMAGE_FRAME* depthImgFrame)
{
	//Extract pointer data from image
	INuiFrameTexture *pDepthImageTexture = depthImgFrame->pFrameTexture;
	NUI_LOCKED_RECT depthLockedRect;
	HRESULT hr = pDepthImageTexture->LockRect(0, &depthLockedRect, nullptr, 0);
	if (depthLockedRect.Pitch != 0)
	{
		BYTE* pBuffer = (BYTE *)depthLockedRect.pBits;
		m_floatDepthMapOpenCV = cv::Mat(m_cDepthHeight, m_cDepthWidth, CV_32F, pBuffer);
	}
	pDepthImageTexture->UnlockRect(0);
	imshow("Depth Image", m_floatDepthMapOpenCV);

}
HRESULT KinectFusionRenderOculus::MapColorToDepth()
{
	
	HRESULT hr;

	if (nullptr == m_pColorImage || nullptr == m_pResampledColorImageDepthAligned
		|| nullptr == m_pDepthImagePixelBuffer || nullptr == m_pColorCoordinates)
	{
		cout << "Mapping color to depth failing" << endl;
		return E_FAIL;
	}

	INuiFrameTexture *srcColorTex = m_pColorImage->pFrameTexture;
	INuiFrameTexture *destColorTex = m_pResampledColorImageDepthAligned->pFrameTexture;

	if (nullptr == srcColorTex || nullptr == destColorTex)
	{
		cout<<"Error accessing color textures."<<endl;
		return E_NOINTERFACE;
	}

	// Lock the source color frame
	NUI_LOCKED_RECT srcLockedRect;

	// Lock the frame data to access the color pixels
	hr = srcColorTex->LockRect(0, &srcLockedRect, nullptr, 0);

	if (FAILED(hr) || srcLockedRect.Pitch == 0)
	{
		cout<<"Error accessing color texture pixels."<<endl;
		return  E_FAIL;
	}

	// Lock the destination color frame
	NUI_LOCKED_RECT destLockedRect;

	// Lock the frame data to access the color pixels
	hr = destColorTex->LockRect(0, &destLockedRect, nullptr, 0);

	if (FAILED(hr) || destLockedRect.Pitch == 0)
	{
		srcColorTex->UnlockRect(0);
		cout<<"Error accessing color texture pixels."<<endl;
		return  E_FAIL;
	}

	int *rawColorData = reinterpret_cast<int*>(srcLockedRect.pBits);
	int *colorDataInDepthFrame = reinterpret_cast<int*>(destLockedRect.pBits);

	// Get the coordinates to convert color to depth space
	hr = m_pMapper->MapDepthFrameToColorFrame(
		m_depthImageResolution,
		m_cDepthImagePixels,
		m_pDepthImagePixelBuffer,
		NUI_IMAGE_TYPE_COLOR,
		m_colorImageResolution,
		m_cDepthImagePixels,   // the color coordinates that get set are the same array size as the depth image
		m_pColorCoordinates);

	if (FAILED(hr))
	{
		srcColorTex->UnlockRect(0);
		destColorTex->UnlockRect(0);
		return hr;
	}

	// Loop over each row and column of the destination color image and copy from the source image
	// Note that we could also do this the other way, and convert the depth pixels into the color space, 
	// avoiding black areas in the converted color image and repeated color images in the background.
	// However, then the depth would have radial and tangential distortion like the color camera image,
	// which is not ideal for Kinect Fusion reconstruction.
	Concurrency::parallel_for(0, static_cast<int>(m_cDepthHeight), [&](int y)
	{
		// Horizontal flip the color image as the standard depth image is flipped internally in Kinect Fusion
		// to give a viewpoint as though from behind the Kinect looking forward by default.
		unsigned int destIndex = y * m_cDepthWidth;
		unsigned int flippedDestIndex = destIndex + (m_cDepthWidth - 1);

		for (int x = 0; x < m_cDepthWidth; ++x, ++destIndex, --flippedDestIndex)
		{
			// Calculate index into depth array
			int colorInDepthX = m_pColorCoordinates[destIndex].x;
			int colorInDepthY = m_pColorCoordinates[destIndex].y;

			// Make sure the depth pixel maps to a valid point in color space
			// Depth and color images are the same size in this sample, so we use the depth image size here.
			// For a more flexible version, see the KinectFusionExplorer-D2D sample.
			if (colorInDepthX >= 0 && colorInDepthX < m_cColorWidth
				&& colorInDepthY >= 0 && colorInDepthY < m_cColorHeight
				&& m_pDepthImagePixelBuffer[destIndex].depth != 0)
			{
				// Calculate index into color array- this will perform a horizontal flip as well
				unsigned int sourceColorIndex = colorInDepthX + (colorInDepthY * m_cColorWidth);

				// Copy color pixel
				colorDataInDepthFrame[flippedDestIndex] = rawColorData[sourceColorIndex];
			}
			else
			{
				colorDataInDepthFrame[flippedDestIndex] = 0;
			}
		}
	});
	srcColorTex->UnlockRect(0);
	destColorTex->UnlockRect(0);
	// Code for displaying aligned color image
	KinectColorFloatImageToOpenCV(m_pResampledColorImageDepthAligned, "Color Image Aligned");
	return hr;
}

void KinectFusionRenderOculus::SetIdentityMatrix(Matrix4 &mat)
{
	mat.M11 = 1; mat.M12 = 0; mat.M13 = 0; mat.M14 = 0;
	mat.M21 = 0; mat.M22 = 1; mat.M23 = 0; mat.M24 = 0;
	mat.M31 = 0; mat.M32 = 0; mat.M33 = 1; mat.M34 = 0;
	mat.M41 = 0; mat.M42 = 0; mat.M43 = 0; mat.M44 = 1;
}

bool KinectFusionRenderOculus::RenderForOculus()
{
	Matrix4 leftEye = m_worldToCameraTransform;
	leftEye.M41 -= m_leftEyeTrans.x;
	leftEye.M42 += 0.13;
	if (!GenerateTextureToRender(m_pShadedSurfaceLeft,m_pPointCloudOculusLeft, leftEye, *leftEyeTexture))
	{
		cout << "couldn't generate Eye texture" << endl;
		return false;
	}
	Matrix4 rightEye = m_worldToCameraTransform;
	rightEye.M41 -= m_rightEyeTrans.x;
	rightEye.M42 += 0.13;
	if (!GenerateTextureToRender(m_pShadedSurfaceRight, m_pPointCloudOculusRight, rightEye, *rightEyeTexture))
	{
		cout << "couldn't generate Eye texture" << endl;
		return false;
	}
	m_osystem->Render(*leftEyeTexture,*rightEyeTexture);
	return true;
}

bool KinectFusionRenderOculus::CreateRenderObjectsForOculus()
{

	// Frames generated from the depth input

	m_camParamsLeft = ComputeCamParams(m_leftEyeFOV, m_leftTextureSize,1.15);
	m_camParamsRight = ComputeCamParams(m_rightEyeFOV, m_rightTextureSize,1.15);

	//std::cout << "Left FOV: " << m_leftEyeFOV.LeftTan << "  " << m_leftEyeFOV.UpTan << endl;
	//std::cout << "Right FOV: " << m_rightEyeFOV.LeftTan << "  " << m_rightEyeFOV.UpTan << endl;
	//std::cout << "Left Size: " << m_leftTextureSize.w << "  " << m_leftTextureSize.w << endl;
	//std::cout << "Right Size: " << m_leftTextureSize.h << "  " << m_leftTextureSize.h << endl;
	HRESULT hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, m_leftTextureSize.w, m_leftTextureSize.h, nullptr, &m_pShadedSurfaceLeft);
	if (FAILED(hr))
	{
		cout << "Failed to initialize left eye textures." << endl;
		return false;
	}
	else
	{
		m_pShadedSurfaceLeft->pCameraParameters = &m_camParamsLeft;
		//NUI_FUSION_CAMERA_PARAMETERS* params = m_pShadedSurfaceLeft->pCameraParameters;
		//cout << "Default focal" << params->focalLengthX << " " << params->focalLengthY << " " << params->principalPointX << " " << params->principalPointY << endl;
	}
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, m_rightTextureSize.w, m_rightTextureSize.h, nullptr, &m_pShadedSurfaceRight);
	if (FAILED(hr))
	{
		cout << "Failed to initialize right eye textures." << endl;
		return false;
	}
	else
	{
		m_pShadedSurfaceRight->pCameraParameters = &m_camParamsRight;
	}
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, m_leftTextureSize.w, m_leftTextureSize.h, nullptr, &m_pPointCloudOculusLeft);
	if (FAILED(hr))
	{
		cout << "Failed to initialize left eye pc." << endl;
		return false;
	}
	else
	{
		m_pPointCloudOculusLeft->pCameraParameters = &m_camParamsLeft;
	}
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, m_rightTextureSize.w, m_rightTextureSize.h, nullptr, &m_pPointCloudOculusRight);
	if (FAILED(hr))
	{
		cout << "Failed to initialize right eye pc." << endl;
		return false;
	}
	else
	{
		m_pPointCloudOculusRight->pCameraParameters = &m_camParamsRight;
	}
	cv::Mat zeroImg = cv::Mat(m_leftTextureSize.h, m_leftTextureSize.w,CV_8UC4);
	leftEyeTexture = new Texture(zeroImg, GL_LINEAR, GL_CLAMP_TO_EDGE, GL_RGBA);
	zeroImg = cv::Mat(m_rightTextureSize.h, m_rightTextureSize.w, CV_8UC4);
	rightEyeTexture = new Texture(zeroImg, GL_LINEAR, GL_CLAMP_TO_EDGE, GL_RGBA);
	return true;
}

float KinectFusionRenderOculus::ComputeFocalLengthFromAngleTan(float tanAngle,float width)
{
	return width/(2 * tanAngle);
}

NUI_FUSION_CAMERA_PARAMETERS KinectFusionRenderOculus::ComputeCamParams(ovrFovPort fov, OVR::Sizei texSize, float correction)
{
	NUI_FUSION_CAMERA_PARAMETERS params;
	params.focalLengthX = correction*ComputeFocalLengthFromAngleTan(fov.LeftTan, texSize.w)/texSize.w;
	params.focalLengthY = correction*ComputeFocalLengthFromAngleTan(fov.UpTan, texSize.h)/texSize.h;
	params.principalPointX = 0.5;
	params.principalPointY = 0.5;
	return params;
}


KinectFusionRenderOculus* mainApp;
void displayGL()
{
	mainApp->display();
}
void keyGL(unsigned char key, int x, int y)
{
	mainApp->keyFunc(key, x, y);
}
int main(int argc, char* argv[])
{
	mainApp = new KinectFusionRenderOculus;
	mainApp->Run(argc, argv);
	delete mainApp;
	int x;
	cin >> x;
	return 0;
}