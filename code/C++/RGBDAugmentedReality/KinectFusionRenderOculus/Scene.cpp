// Include standard headers
#include <stdio.h>
#include <stdlib.h>

#include "Scene.h"
#include "TextureLoader.h"
#include "ObjLoader.h"
#include "PlyLoader.h"

#define NORMAL_SPEED 1.0/20.0
#define FAST_SPEED NORMAL_SPEED/2.0
#define SLOW_SPEED NORMAL_SPEED*2.0
#define STOP_TIME 0.0
#define TIME_INCREMENT 0.001

Scene::Scene()
{
	numStaticMeshes = 0;
	numDynamicMeshes = 0;
	renderDynamicMeshes = true;
	renderStaticMeshes = true;
	renderExtraMeshes = true;
	dynamicMeshes = NULL;
	staticMeshes = NULL;
	playBackState = PLAY_NORMAL;
	secondaryDynamicMeshes = false;
	thirdDynamicMeshes = false;
	dynamicMeshType = PRIMARY;
	playBackSense = FORWARDS;
	time = 0.0;
	speed = NORMAL_SPEED;
	rType = Mesh;
	StepF = false;
	StepB = false;
}

Scene::~Scene()
{
	if(secondaryDynamicMeshes)
		delete[] dynamicMeshesSecondary;
	delete[] dynamicMeshes;
	delete[] staticMeshes;
}

void Scene::loadDynamicMeshes(const int startIndex, const int lastIndex, const int stepSize, const MeshType& meshType, char* prefix)
{
	int numDynamicMeshes = (lastIndex - startIndex + 1) / stepSize;
	this->numDynamicMeshes = numDynamicMeshes;
	dynamicMeshes = new SceneMesh[numDynamicMeshes];
	for (int i = 0; i < numDynamicMeshes; i++)
	{
		dynamicMeshes[i].initialize(meshType);
		char fileName[256];
		if (meshType == OBJ)
		{
			if(prefix !=NULL)
				sprintf(fileName, "%s%04d.obj", prefix, i * stepSize + startIndex);
			else
				sprintf(fileName, "%04d.obj", prefix, i * stepSize + startIndex);
			dynamicMeshes[i].setDataPath(dynamicMeshesDataPath);
			bool objLoadResult = ObjLoader::loadOBJ(dynamicMeshesDataPath, fileName, dynamicMeshes[i].getMesh());

			if (objLoadResult)
				printf("Dynamic Mesh %s loaded\n", fileName);
			else
				printf("Dynamic Mesh %s not loaded\n", fileName);
		}
		else if (meshType == PLY)
		{
			if (prefix != NULL)
				sprintf(fileName, "%s%d.ply", prefix, i * stepSize + startIndex);
			else
				sprintf(fileName, "%d.ply", prefix, i * stepSize + startIndex);
			dynamicMeshes[i].setDataPath(dynamicMeshesDataPath);
			bool plyLoadResult = PlyLoader::loadPly(dynamicMeshesDataPath, fileName, dynamicMeshes[i].getPlyMesh());

			if (plyLoadResult)
				printf("Dynamic Mesh %s loaded\n", fileName);
			else
				printf("Dynamic Mesh %s not loaded\n", fileName);
		}
	}
}

void Scene::loadDynamicMeshesSecondary(const int startIndex, const int lastIndex, const int stepSize, const MeshType& meshType, char* prefix)
{
	int numDynamicMeshes = (lastIndex - startIndex + 1) / stepSize;
	this->numDynamicMeshes = numDynamicMeshes;
	dynamicMeshesSecondary = new SceneMesh[numDynamicMeshes];
	for (int i = 0; i < numDynamicMeshes; i++)
	{
		dynamicMeshesSecondary[i].initialize(meshType);
		char fileName[256];
		if (meshType == OBJ)
		{
			if (prefix != NULL)
				sprintf(fileName, "%s%d.obj", prefix, i * stepSize + startIndex);
			else
				sprintf(fileName, "%d.obj", prefix, i * stepSize + startIndex);
			dynamicMeshesSecondary[i].setDataPath(dynamicMeshesSecondaryDataPath);
			bool objLoadResult = ObjLoader::loadOBJ(dynamicMeshesSecondaryDataPath, fileName, dynamicMeshesSecondary[i].getMesh());

			if (objLoadResult)
				printf("Dynamic Mesh Secondary %s loaded\n", fileName);
			else
				printf("Dynamic Mesh Secondary %s not loaded\n", fileName);
		}
		else if (meshType == PLY)
		{
			if (prefix != NULL)
				sprintf(fileName, "%s%d.ply", prefix, i * stepSize + startIndex);
			else
				sprintf(fileName, "%d.ply", prefix, i * stepSize + startIndex);
			dynamicMeshesSecondary[i].setDataPath(dynamicMeshesSecondaryDataPath);
			bool plyLoadResult = PlyLoader::loadPly(dynamicMeshesSecondaryDataPath, fileName, dynamicMeshesSecondary[i].getPlyMesh());

			if (plyLoadResult)
				printf("Dynamic Mesh Secondary %s loaded\n", fileName);
			else
				printf("Dynamic Mesh Secondary %s not loaded\n", fileName);
		}
	}
}

void Scene::loadDynamicMeshesThird(const int startIndex, const int lastIndex, const int stepSize, const MeshType & meshType, char * prefix)
{
	int numDynamicMeshes = (lastIndex - startIndex + 1) / stepSize;
	this->numDynamicMeshes = numDynamicMeshes;
	dynamicMeshesThird = new SceneMesh[numDynamicMeshes];
	for (int i = 0; i < numDynamicMeshes; i++)
	{
		dynamicMeshesThird[i].initialize(meshType);
		char fileName[256];
		if (meshType == OBJ)
		{
			if (prefix != NULL)
				sprintf(fileName, "%s%d.obj", prefix, i * stepSize + startIndex);
			else
				sprintf(fileName, "%d.obj", prefix, i * stepSize + startIndex);
			dynamicMeshesThird[i].setDataPath(dynamicMeshesThirdDataPath);
			bool objLoadResult = ObjLoader::loadOBJ(dynamicMeshesThirdDataPath, fileName, dynamicMeshesThird[i].getMesh());

			if (objLoadResult)
				printf("Dynamic Mesh Third %s loaded\n", fileName);
			else
				printf("Dynamic Mesh Third %s not loaded\n", fileName);
		}
		else if (meshType == PLY)
		{
			if (prefix != NULL)
				sprintf(fileName, "%s%d.ply", prefix, i * stepSize + startIndex);
			else
				sprintf(fileName, "%d.ply", prefix, i * stepSize + startIndex);
			dynamicMeshesThird[i].setDataPath(dynamicMeshesThirdDataPath);
			bool plyLoadResult = PlyLoader::loadPly(dynamicMeshesThirdDataPath, fileName, dynamicMeshesThird[i].getPlyMesh());

			if (plyLoadResult)
				printf("Dynamic Mesh Third %s loaded\n", fileName);
			else
				printf("Dynamic Mesh Third %s not loaded\n", fileName);
		}
	}
}

void Scene::loadStaticMesh(const char * modelFileName, const int staticMeshID, const MeshType& meshType)
{
	numStaticMeshes = staticMeshID + 1;
	staticMeshes[staticMeshID].initialize(meshType);
	staticMeshes[staticMeshID].setDataPath(staticMeshesDataPath);
	if (meshType == OBJ)
	{
		bool objLoadResult = ObjLoader::loadOBJ(staticMeshesDataPath, modelFileName, staticMeshes[staticMeshID].getMesh());
		if (objLoadResult)
			printf("Static Mesh %d loaded\n", staticMeshID);
		else
			printf("Static Mesh %d not loaded\n", staticMeshID);
	}
	else if (meshType == PLY)
	{
		bool plyLoadResult = PlyLoader::loadPly(staticMeshesDataPath, modelFileName, staticMeshes[staticMeshID].getPlyMesh());

		if (plyLoadResult)
			printf("Static Mesh %d loaded\n", staticMeshID);
		else
			printf("Static Mesh %d not loaded\n", staticMeshID);
	}
	
}

void Scene::initialize()
{
	mvpMatrixId = glGetUniformLocation(objProgramId, "MVP");
	mvpMatrixPlyId = glGetUniformLocation(plyProgramId, "MVP");
	samplerId = glGetUniformLocation(objProgramId, "textureSampler");
	
	for (int i = 0; i < numDynamicMeshes; i++)
	{
		dynamicMeshes[i].loadBuffers();
		if(dynamicMeshes[i].getMeshType() == OBJ)
			dynamicMeshes[i].setMVPMatrixId(mvpMatrixId);
		else if (dynamicMeshes[i].getMeshType() == PLY)
			dynamicMeshes[i].setMVPMatrixId(mvpMatrixPlyId);
		if (secondaryDynamicMeshes)
		{
			dynamicMeshesSecondary[i].loadBuffers();
			if (dynamicMeshesSecondary[i].getMeshType() == OBJ)
				dynamicMeshesSecondary[i].setMVPMatrixId(mvpMatrixId);
			else if (dynamicMeshesSecondary[i].getMeshType() == PLY)
				dynamicMeshesSecondary[i].setMVPMatrixId(mvpMatrixPlyId);
		}
		if (thirdDynamicMeshes)
		{
			dynamicMeshesThird[i].loadBuffers();
			if (dynamicMeshesThird[i].getMeshType() == OBJ)
				dynamicMeshesThird[i].setMVPMatrixId(mvpMatrixId);
			else if (dynamicMeshesThird[i].getMeshType() == PLY)
				dynamicMeshesThird[i].setMVPMatrixId(mvpMatrixPlyId);
		}
	}

	for (int i = 0; i < numStaticMeshes; i++)
	{
		staticMeshes[i].loadBuffers();
		if (staticMeshes[i].getMeshType() == OBJ)
			staticMeshes[i].setMVPMatrixId(mvpMatrixId);
		else if (staticMeshes[i].getMeshType() == PLY)
			staticMeshes[i].setMVPMatrixId(mvpMatrixPlyId);
	}

	currentMeshId = 0;
	previousTime = GetTickCount64();
	elapsedTime = 0.0;
}




void Scene::render(const OVR::Matrix4f& projectionMat, const OVR::Matrix4f& viewMat, const OVR::Matrix4f& transformMat)
{
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// Bind our texture in Texture Unit 0
	
	glPushMatrix();
	
	OVR::Matrix4f scaleMat = OVR::Matrix4f::Scaling(0.01f, 0.01f, 0.01f);
	OVR::Matrix4f rotateMat = OVR::Matrix4f::RotationY(0.0f);
	OVR::Matrix4f transMat = OVR::Matrix4f::Translation(0.0f, 14.0f, 0.0f);

	OVR::Matrix4f modelMat =  transformMat;

	OVR::Matrix4f mvp = projectionMat * viewMat * modelMat;

	//if (playBackState != REST)
	{

		if (renderDynamicMeshes)
		{
			if (dynamicMeshType == PRIMARY)
			{
				if (dynamicMeshes[currentMeshId].getMeshType() == OBJ)
				{
					glActiveTexture(GL_TEXTURE0);
					// Use our shader
					glUseProgram(objProgramId);
					glUniform1i(samplerId, 0);
				}
				else if (dynamicMeshes[currentMeshId].getMeshType() == PLY)
				{
					// Use our shader
					glUseProgram(plyProgramId);
				}
				dynamicMeshes[currentMeshId].render(mvp, rType);
			}
			else if (dynamicMeshType == SECONDARY)
			{
				if (dynamicMeshesSecondary[currentMeshId].getMeshType() == OBJ)
				{
					glActiveTexture(GL_TEXTURE0);
					// Use our shader
					glUseProgram(objProgramId);
					glUniform1i(samplerId, 0);
				}
				else if (dynamicMeshesSecondary[currentMeshId].getMeshType() == PLY)
				{
					// Use our shader
					glUseProgram(plyProgramId);
				}
				dynamicMeshesSecondary[currentMeshId].render(mvp, rType);
			}
			else if (dynamicMeshType == THIRD)
			{
				if (dynamicMeshesThird[currentMeshId].getMeshType() == OBJ)
				{
					glActiveTexture(GL_TEXTURE0);
					// Use our shader
					glUseProgram(objProgramId);
					glUniform1i(samplerId, 0);
				}
				else if (dynamicMeshesThird[currentMeshId].getMeshType() == PLY)
				{
					// Use our shader
					glUseProgram(plyProgramId);
				}
				dynamicMeshesThird[currentMeshId].render(mvp, rType);
			}

		}

		/*rotateMat = OVR::Matrix4f::RotationZ(3.142f);

		modelMat = scaleMat  *  transformMat;

		mvp = projectionMat * viewMat * modelMat;*/


		for (int i = 0; i < numStaticMeshes; i++)
		{
			if (staticMeshes[i].getMeshType() == OBJ)
			{
				glActiveTexture(GL_TEXTURE0);
				// Use our shader
				glUseProgram(objProgramId);
				glUniform1i(samplerId, 0);
			}
			else if (staticMeshes[i].getMeshType() == PLY)
			{
				// Use our shader
				glUseProgram(plyProgramId);
			}

			staticMeshes[i].render(mvp);
		}
		printf("%s Dynamic Mesh Frame Id %d\r", dynamicMeshType == PRIMARY ? "PRIMARY" : "SECONDARY", currentMeshId + 1);
	}
	
	glUseProgram(0);
	glPopMatrix();
	

	/*elapsedTime += GetTickCount64() - previousTime;
	previousTime = GetTickCount64();

	double speed = 0.0f;

	if (playBackState == PLAY_NORMAL)
		speed = NORMAL_SPEED;
	else if (playBackState == PLAY_FAST)
		speed = FAST_SPEED;
	else if (playBackState == PLAY_SLOW)
		speed = SLOW_SPEED;
	else if (playBackState == REST)
	{
		if (elapsedTime >= STOP_TIME)
		{
			playBackState = PLAY_NORMAL;
			elapsedTime = 0.0;
		}
		return;
	}
	else
	{
		elapsedTime = 0.0;
		return;
	}

	if (elapsedTime >= speed)
	{
		currentMeshId++;
		elapsedTime = 0.0;
	}

	if (currentMeshId == numDynamicMeshes)
	{
		playBackState = REST;
		currentMeshId = 0;
	}*/


}

void Scene::playBack(GLFWwindow * window, const double deltaTime)
{
	if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
	{
		playBackState = PLAY_NORMAL;
		speed = NORMAL_SPEED;
	}
	if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS)
	{
		playBackState = PLAY_FAST;
		speed = FAST_SPEED;
	}
	if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
	{
		//playBackState = PLAY_SLOW;
		//speed = SLOW_SPEED;
		time = 0.0f;
	}
	if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS)
	{
		playBackState = STOP;
	}
	if (glfwGetKey(window, GLFW_KEY_KP_SUBTRACT) == GLFW_PRESS)
	{
		speed += TIME_INCREMENT;
	}
	if (glfwGetKey(window, GLFW_KEY_KP_ADD) == GLFW_PRESS)
	{
		speed -= TIME_INCREMENT;
	}
	if (glfwGetKey(window, GLFW_KEY_KP_4) == GLFW_PRESS)
	{
		playBackSense = BACKWARDS;
	}
	if (glfwGetKey(window, GLFW_KEY_KP_6) == GLFW_PRESS)
	{
		playBackSense = FORWARDS;
	}
	if (glfwGetKey(window, GLFW_KEY_KP_8) == GLFW_PRESS)
	{
		StepF = true;
	}
	if (glfwGetKey(window, GLFW_KEY_KP_2) == GLFW_PRESS)
	{
		StepB = true;
	}
	if (glfwGetKey(window, GLFW_KEY_KP_8) == GLFW_RELEASE)
	{
		if (StepF)
		{
			playBackState = STEP_FORWARD;
			StepF = false;
		}
	}
	if (glfwGetKey(window, GLFW_KEY_KP_2) == GLFW_RELEASE)
	{
		if (StepB)
		{
			playBackState = STEP_BACKWARD;
			StepB = false;
		}
	}
	if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS)
	{
		dynamicMeshType = PRIMARY;
	}
	if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS && secondaryDynamicMeshes)
	{
		dynamicMeshType = SECONDARY;
	}
	if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS && thirdDynamicMeshes)
	{
		dynamicMeshType = THIRD;
	}
	if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS)
	{
		rType = PointCloud;
	}
	if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
	{
		rType = Mesh;
	}

	time += deltaTime;
	if (playBackState == REST)
	{
		
		if (time >= STOP_TIME)
		{
			printf("\nSequence Started\n");
			playBackState = PLAY_NORMAL;
			time = 0.0;
		}
		return;
	}
	else if (playBackState == STEP_FORWARD)
	{
		currentMeshId++;
		time = 0.0;
		playBackState = STOP;
	}
	else if (playBackState == STEP_BACKWARD)
	{
		currentMeshId--;
		time = 0.0;
		playBackState = STOP;
	}
	else if (playBackState == STOP)
	{
		time = 0.0;
		return;
	}

	if (time >= speed)
	{
		if (playBackSense == FORWARDS)
			currentMeshId++;
		else
			currentMeshId--;
		time = 0.0;
	}

	if (currentMeshId == numDynamicMeshes)
	{
		printf("\nSequence Finished");
		if(playBackState != STEP_FORWARD && playBackState != STEP_BACKWARD)
			playBackState = STOP;
		currentMeshId = 0;
	}
	else if (currentMeshId == -1)
	{
		printf("\nSequence Finished");
		if (playBackState != STEP_FORWARD && playBackState != STEP_BACKWARD)
			playBackState = STOP;
		currentMeshId = numDynamicMeshes - 1;
	}
	
}

void Scene::playBack( const double deltaTime)
{

	playBackState = PLAY_NORMAL;
	speed = NORMAL_SPEED;
	time += deltaTime;

	if (playBackState == REST)
	{

		if (time >= STOP_TIME)
		{
			printf("\nSequence Started\n");
			playBackState = PLAY_NORMAL;
			time = 0.0;
		}
		return;
	}
	else if (playBackState == STEP_FORWARD)
	{
		currentMeshId++;
		time = 0.0;
		playBackState = STOP;
	}
	else if (playBackState == STEP_BACKWARD)
	{
		currentMeshId--;
		time = 0.0;
		playBackState = STOP;
	}
	else if (playBackState == STOP)
	{
		time = 0.0;
		return;
	}

	if (time >= speed)
	{
		if (playBackSense == FORWARDS)
			currentMeshId++;
		else
			currentMeshId--;
		time = 0.0;
	}

	if (currentMeshId == numDynamicMeshes)
	{
		printf("\nSequence Finished");
		if (playBackState != STEP_FORWARD && playBackState != STEP_BACKWARD)
			playBackState = STOP;
		currentMeshId = 0;
	}
	else if (currentMeshId == -1)
	{
		printf("\nSequence Finished");
		if (playBackState != STEP_FORWARD && playBackState != STEP_BACKWARD)
			playBackState = STOP;
		currentMeshId = numDynamicMeshes - 1;
	}

}