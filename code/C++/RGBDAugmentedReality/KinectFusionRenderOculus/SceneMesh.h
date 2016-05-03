#ifndef SCENE_MESH_H
#define SCENE_MESH_H


// Include GLEW
#include "GL/CAPI_GLE.h"



// Include GLM
#include <glm.hpp>
#include "gtc\\matrix_transform.hpp"

// Include Mesh
#include "MeshData.h"

#include "OVR_Math.h"

enum MeshType
{
	OBJ,
	PLY
};

enum RenderType
{
	Mesh,
	PointCloud
};

using namespace glm;
class SceneMesh
{
	public:
		SceneMesh() { rType = Mesh; }
		~SceneMesh();

		void initialize(const MeshType& mType)
		{
			type = mType;
			if (type == OBJ)
			{
				mesh = new MeshData();
			}
			else if (type == PLY)
			{
				plyMesh = new PlyMesh();
			}
		}

		void setRenderType(RenderType type) { rType = type; }

		MeshData* getMesh() { return mesh; }
		PlyMesh* getPlyMesh() { return plyMesh;}

		MeshType getMeshType() { return type; }
		
		void loadBuffers();
		void render(OVR::Matrix4f& mvp, RenderType rtype = Mesh);
		void setDataPath(const char* path) { dataPath = path; }
		void setMVPMatrixId(const GLuint& mvpMatrixId) { this->mvpMatrixId = mvpMatrixId; }
		void setSamplerId(const GLuint& sId) { samplerId = sId; }
	
	private:

		MeshType type;

		RenderType rType;

		//void loadBuffers();
		void unloadBuffers();
		void renderMeshGroup(const int index);
		void renderPlyMesh();

		GLuint mvpMatrixId;
		GLuint* vertexBuffers;
		GLuint* normalBuffers;
		GLuint* uvBuffers;
		GLuint* indexBuffers;
		GLuint vertexArrayId;
		GLuint samplerId;
		
		GLuint* textureIds;

		GLuint plyVertexBuffer;
		GLuint plyNormalBuffer;
		GLuint plyColorBuffer;
		GLuint plyIndexBuffer;

		unsigned int numPlyMeshIndices;

		MeshData* mesh;
		PlyMesh* plyMesh;

		const char* dataPath;
		
		vector<int> textureIdList;
		vector<int> numIndicesList;

		int numMeshGroups;
		int numMeshMaterials;
};
#endif