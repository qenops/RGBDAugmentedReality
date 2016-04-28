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
	m_bAutoResetReconstructionWhenLost(false),
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
	m_cColorIntegrationInterval(2)
{
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
    m_reconstructionParams.voxelsPerMeter = 256;// 1000mm / 256vpm = ~3.9mm/voxel    
    m_reconstructionParams.voxelCountX = 512;   // 512 / 256vpm = 2m wide reconstruction
    m_reconstructionParams.voxelCountY = 384;   // Memory = 512*384*512 * 4bytes per voxel
    m_reconstructionParams.voxelCountZ = 512;   // This will require a GPU with at least 512MB

    // These parameters are for optionally clipping the input depth image 
    m_fMinDepthThreshold = NUI_FUSION_DEFAULT_MINIMUM_DEPTH;   // min depth in meters
    m_fMaxDepthThreshold = NUI_FUSION_DEFAULT_MAXIMUM_DEPTH;    // max depth in meters

     // This parameter is the temporal averaging parameter for depth integration into the reconstruction
    m_cMaxIntegrationWeight = NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT;	// Reasonable for static scenes

    // This parameter sets whether GPU or CPU processing is used. Note that the CPU will likely be 
    // too slow for real-time processing.
    m_processorType = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP;

     m_deviceIndex = -1;    // automatically choose device index for processing

    SetIdentityMatrix(m_worldToCameraTransform);
    SetIdentityMatrix(m_defaultWorldToVolumeTransform);

	m_cLastDepthFrameTimeStamp.QuadPart = 0;
	m_cLastColorFrameTimeStamp.QuadPart = 0;

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

KinectFusionRenderOculus::~KinectFusionRenderOculus()
{
	// Clean up Kinect Fusion
    SafeRelease(m_pVolume);

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


	if (!initGL(-0.1, 0.1, -0.1, 0.1, 0.1, 1000))
	{
		std::cout << "Init GL Failed" << std::endl;
		return false;
	}
	
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

	glutMainLoop();
	return true;

}

bool KinectFusionRenderOculus::init()
{
	return false;
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
			hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH);
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
	LARGE_INTEGER currentColorFrameTime = m_cLastColorFrameTimeStamp;

	HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pColorStreamHandle, 0, &imageFrame);
	if (FAILED(hr))
	{
		m_integrateColor = false;
	}
	else
	{
		hr = CopyColor(imageFrame);

		currentColorFrameTime = imageFrame.liTimeStamp;

		// Release the Kinect camera frame
		m_pNuiSensor->NuiImageStreamReleaseFrame(m_pColorStreamHandle, &imageFrame);

		if (FAILED(hr))
		{
			return false;
		}
	}
	return true;
}

void KinectFusionRenderOculus::ProcessDepth()
{
	if (m_bInitializeError)
	{
		return;
	}

	HRESULT hr = S_OK;
	NUI_IMAGE_FRAME imageFrame;
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
		KinectDepthFloatImageToOpenCV(m_pColorImage);
	}
	// Check color and depth frame timestamps to ensure they were captured at the same time
	// If not, we attempt to re-synchronize by getting a new frame from the stream that is behind.
	int timestampDiff = static_cast<int>(abs(m_currentColorFrameTime.QuadPart - m_currentDepthFrameTime.QuadPart));
	
	if (m_integrateColor && timestampDiff >= cMinTimestampDifferenceForFrameReSync)
	{
		// Get another frame to try and re-sync
		if (m_currentColorFrameTime.QuadPart - m_currentDepthFrameTime.QuadPart >= cMinTimestampDifferenceForFrameReSync)
		{
			// Perform camera tracking only from this current depth frame
			if (m_cFrameCounter > 0)
			{
				CameraTrackingOnly();
			}

			// Get another depth frame to try and re-sync as color ahead of depth
			AquireDepth();
		}
		else if (m_currentDepthFrameTime.QuadPart - m_currentColorFrameTime.QuadPart >= cMinTimestampDifferenceForFrameReSync && WaitForSingleObject(m_hNextColorFrameEvent, 0) != WAIT_TIMEOUT)
		{
			// Get another color frame to try and re-sync as depth ahead of color
			AcquireColor();

			timestampDiff = static_cast<int>(abs(m_currentColorFrameTime.QuadPart - m_currentDepthFrameTime.QuadPart));

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
/*	if (m_bAutoResetReconstructionOnTimeout &&  m_cFrameCounter != 0
		&& abs(currentDepthFrameTime - m_cLastDepthFrameTimeStamp) > cResetOnTimeStampSkippedMilliseconds)
	{
		ResetReconstruction();

		if (FAILED(hr))
		{
			return;
		}
	}*/

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
			std::cout<<"Kinect Fusion camera tracking failed! Align the camera to the last tracked position. "<<endl;
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
/*	if (m_bAutoResetReconstructionWhenLost && m_bTrackingFailed && m_cLostFrameCounter >= cResetOnNumberOfLostFrames)
	{
		// Automatically clear volume and reset tracking if tracking fails
		hr = ResetReconstruction();

		if (FAILED(hr))
		{
			return;
		}

		// Set bad tracking message
		SetStatusMessage(L"Kinect Fusion camera tracking failed, automatically reset volume.");
	}*/

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

	//// Display fps count approximately every cTimeDisplayInterval seconds
	//double elapsed = m_timer.AbsoluteTime() - m_fStartTime;
	//if ((int)elapsed >= cTimeDisplayInterval)
	{
	//	double fps = (double)m_cFrameCounter / elapsed;

	//	// Update status display
	//	if (!m_bTrackingFailed)
	//	{
	//		WCHAR str[MAX_PATH];
	//		swprintf_s(str, ARRAYSIZE(str), L"Fps: %5.2f", fps);
	//		SetStatusMessage(str);
	//	}

	//	m_cFrameCounter = 0;
	//	m_fStartTime = m_timer.AbsoluteTime();
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
			
				cout<<"Kinect Fusion camera tracking failed! Align the camera to the last tracked position."<<endl;
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

	HRESULT hr = m_pVolume->CalculatePointCloud(m_pPointCloud, ((m_bCaptureColor == true) ? m_pShadedSurface : nullptr), &worldToCamera);

	if (FAILED(hr))
	{
		std::cout << "Kinect Fusion CalculatePointCloud call failed." << endl;
		return false;
	}

	////////////////////////////////////////////////////////
	// ShadePointCloud

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
		m_RenderTarget = cv::Mat(m_pShadedSurface->height,m_pShadedSurface->width,CV_8UC4,pBuffer);
	}
	imshow("Render Target", m_RenderTarget);
	// We're done with the texture so unlock it
	pShadedImageTexture->UnlockRect(0);
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

	LARGE_INTEGER currentDepthFrameTime = imageFrame.liTimeStamp;

	// Release the Kinect camera frame
	m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);

	if (FAILED(hr))
	{
		return false;
	}
	m_cLastDepthFrameTimeStamp = currentDepthFrameTime;
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
		ProcessDepth();
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

	/*if (m_bTranslateResetPoseByMinDepthThreshold)
	{
		// This call will set the world-volume transformation
		hr = ResetReconstruction();
		if (FAILED(hr))
		{
			return hr;
		}
	}*/

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


void KinectFusionRenderOculus::KinectColorFloatImageToOpenCV(NUI_FUSION_IMAGE_FRAME* colorImgFrame)
{
	//Extract pointer data from image
	INuiFrameTexture *pColorImageTexture = colorImgFrame->pFrameTexture;
	NUI_LOCKED_RECT colorLockedRect;
	HRESULT hr = pColorImageTexture->LockRect(0, &colorLockedRect, nullptr, 0);
	if (colorLockedRect.Pitch != 0)
	{
		BYTE* pBuffer = (BYTE *)colorLockedRect.pBits;
		m_floatColorOpenCV = cv::Mat(m_cDepthHeight, m_cDepthWidth, CV_8UC4, pBuffer);
	}
	pColorImageTexture->UnlockRect(0);
	imshow("Color Image", m_floatColorOpenCV);

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

	return hr;
}


HRESULT KinectFusionRenderOculus::ResetReconstruction()
{
	return E_NOTIMPL;
}

void KinectFusionRenderOculus::SetIdentityMatrix(Matrix4 &mat)
{
	mat.M11 = 1; mat.M12 = 0; mat.M13 = 0; mat.M14 = 0;
	mat.M21 = 0; mat.M22 = 1; mat.M23 = 0; mat.M24 = 0;
	mat.M31 = 0; mat.M32 = 0; mat.M33 = 1; mat.M34 = 0;
	mat.M41 = 0; mat.M42 = 0; mat.M43 = 0; mat.M44 = 1;
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