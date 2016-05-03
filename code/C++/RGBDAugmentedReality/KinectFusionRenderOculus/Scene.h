
#ifndef SCENE
#define SCENE

// Include GLEW
#include "GL/CAPI_GLE.h"

#include "glfw3.h"

// Include GLM
#include <glm.hpp>
#include "gtc\\matrix_transform.hpp"

// Include Mesh Data
#include "MeshData.h"

#include "OVR_Math.h"

#include "SceneMesh.h"
#include <ctime>

using namespace glm;

enum PlayBackState
{
	PLAY_NORMAL,
	PLAY_SLOW,
	PLAY_FAST,
	REST,
	STEP_FORWARD,
	STEP_BACKWARD,
	STOP
};

enum PlayBackSense
{
	FORWARDS,
	BACKWARDS
};

enum DynamicMeshType
{
	PRIMARY,
	SECONDARY,
	THIRD
};

class Scene
{
	public:
		Scene();
		~Scene();

		void loadDynamicMeshes(const int startIndex, const int lastIndex, const int stepSize, const MeshType& meshType, char* prefix);
		void loadDynamicMeshesSecondary(const int startIndex, const int lastIndex, const int stepSize, const MeshType& meshType, char* prefix);
		void loadDynamicMeshesThird(const int startIndex, const int lastIndex, const int stepSize, const MeshType& meshType, char* prefix);
		void loadStaticMesh(const char * modelFileName, const int staticMeshID, const MeshType& meshType);
		void setPrograms(const GLuint& objProgramId, const GLuint& plyProgramId) { this->objProgramId = objProgramId; this->plyProgramId = plyProgramId;}

		void initialize();

		void render(const OVR::Matrix4f& projectionMat, const OVR::Matrix4f& viewMat , const OVR::Matrix4f& transformMat);


		void setDynamicMeshesDataPath(const char* path) { dynamicMeshesDataPath = path; }
		void setDynamicMeshesSecondaryDataPath(const char* path) {
			dynamicMeshesSecondaryDataPath = path; secondaryDynamicMeshes = true;
		}
		void setDynamicMeshesThirdDataPath(const char* path) {
			dynamicMeshesThirdDataPath = path; thirdDynamicMeshes = true;
		}
		void setStaticMeshesDataPath(const char* path) { staticMeshesDataPath = path; }

		void setNumStaticMeshes(const int numMeshes) { numStaticMeshes = numMeshes; staticMeshes = new SceneMesh[numStaticMeshes]; }

		void toggleDynamicMeshesRenderFlag() { renderDynamicMeshes = !renderDynamicMeshes; }

		void toggleStaticMeshesRenderFlag() { renderStaticMeshes = !renderStaticMeshes; }

		void toggleExtraMeshesRenderFlag() { renderExtraMeshes = !renderExtraMeshes; }

		void playBack(GLFWwindow* window, const double deltaTime);
		void playBack(const double deltaTime);
	private:

		bool renderDynamicMeshes;
		bool renderStaticMeshes;
		bool renderExtraMeshes;
		bool secondaryDynamicMeshes;
		bool thirdDynamicMeshes;

		bool StepF;
		bool StepB;

		RenderType rType;

		const char* dynamicMeshesDataPath;
		const char* dynamicMeshesSecondaryDataPath;
		const char* dynamicMeshesThirdDataPath;
		const char* staticMeshesDataPath;

		OVR::Matrix4f mvpMatrix;

		GLuint objProgramId;
		GLuint plyProgramId;

		GLuint mvpMatrixId;
		GLuint samplerId;
		GLuint mvpMatrixPlyId;

		SceneMesh* dynamicMeshes;
		SceneMesh* dynamicMeshesSecondary;
		SceneMesh* dynamicMeshesThird;
		SceneMesh* staticMeshes;

		PlayBackState playBackState;

		DynamicMeshType dynamicMeshType;

		int currentMeshId;


		int numDynamicMeshes;
		int numStaticMeshes;

		double elapsedTime;
		double currentTime;
		double previousTime;
		double time;
		double speed;
		PlayBackSense playBackSense;
};

#endif