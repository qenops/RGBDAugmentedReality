//#include "Util\Util_Direct3D.h"
#include "SceneMesh.h"
#include "TextureLoader.h"

SceneMesh::~SceneMesh()
{
	unloadBuffers();
	if (type == OBJ)
	{
		if (vertexBuffers)
			delete vertexBuffers;
		if (normalBuffers)
			delete normalBuffers;
		if (indexBuffers)
			delete indexBuffers;
		if (uvBuffers)
			delete uvBuffers;
		if (textureIds)
			delete textureIds;
	}
	//delete mesh;
}



void SceneMesh::render(OVR::Matrix4f& mvp, RenderType rtype)
{
	glUniformMatrix4fv(mvpMatrixId, 1, GL_TRUE, (FLOAT*)&mvp);
	//const int numGroups = mesh->numGroups;

	if (type == OBJ)
	{
		for (int i = 0; i < numMeshGroups; i++)
			renderMeshGroup(i);
	}
	else if (type == PLY)
	{
		renderPlyMesh();
	}
	rType = rtype;
}

void SceneMesh::loadBuffers()
{
	if (type == OBJ)
	{
		const int numGroups = mesh->numGroups;
		const int numMaterials = mesh->numMaterials;

		vertexBuffers = new GLuint[numGroups];
		normalBuffers = new GLuint[numGroups];
		uvBuffers = new GLuint[numGroups];
		indexBuffers = new GLuint[numGroups];
		//vertexArrayIds = new GLuint[numGroups];
		textureIds = new GLuint[numMaterials];
		glGenVertexArrays(1, &vertexArrayId);
		glBindVertexArray(vertexArrayId);

		//glBindFramebuffer(GL_FRAMEBUFFER, 0);

		for (int i = 0; i < numGroups; i++)
		{
			const Group g = mesh->groups[i];

			const int numVertex = g.vertices.size();
			const int numNormal = g.normals.size();
			const int numUvs = g.uvs.size();
			const int numIndices = g.indices.size();



			glGenBuffers(1, &vertexBuffers[i]);
			glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[i]);
			glBufferData(GL_ARRAY_BUFFER, sizeof(float) * numVertex * 3, &g.vertices[0], GL_STATIC_DRAW);

			glGenBuffers(1, &normalBuffers[i]);
			glBindBuffer(GL_ARRAY_BUFFER, normalBuffers[i]);
			glBufferData(GL_ARRAY_BUFFER, sizeof(float) * numNormal * 3, &g.normals[0], GL_STATIC_DRAW);

			glGenBuffers(1, &uvBuffers[i]);
			glBindBuffer(GL_ARRAY_BUFFER, uvBuffers[i]);
			glBufferData(GL_ARRAY_BUFFER, sizeof(float) * numUvs * 2, &g.uvs[0], GL_STATIC_DRAW);

			glGenBuffers(1, &indexBuffers[i]);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffers[i]);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, numIndices * sizeof(int), &g.indices[0], GL_STATIC_DRAW);

		}

		for (int i = 0; i < numMaterials; i++)
		{
			char textureFilePath[256];
			sprintf_s(textureFilePath, "%s%s", dataPath, mesh->materials[i].map_Kd);
			textureIds[i] = TextureLoader::loadTexture(textureFilePath);
		}

		numMeshGroups = mesh->numGroups;
		numMeshMaterials = mesh->numMaterials;
		for (int groupId = 0; groupId < numMeshGroups; groupId++)
		{
			int i = 0;
			for (i = 0; i < numMaterials; i++)
			{
				if (strcmp(mesh->groups[groupId].mat.tag, mesh->materials[i].tag) == 0)
				{
					textureIdList.push_back(i);
					break;
				}
			}
			if (i == numMaterials)
				textureIdList.push_back(-1);
			numIndicesList.push_back(mesh->groups[groupId].indices.size());
		}

		// Deleting mesh data from Host's heap. The copy of mesh data is transferred to the GPU.
		delete mesh;
	}
	else if (type == PLY)
	{

		glGenVertexArrays(1, &vertexArrayId);
		glBindVertexArray(vertexArrayId);

		const int numVertex = plyMesh->vertices.size();
		const int numNormal = plyMesh->normals.size();
		const int numColors = plyMesh->colors.size();
		const int numIndices = plyMesh->indices.size();

		glGenBuffers(1, &plyVertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, plyVertexBuffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * numVertex * 3, &plyMesh->vertices[0], GL_STATIC_DRAW);

		glGenBuffers(1, &plyNormalBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, plyNormalBuffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * numNormal * 3, &plyMesh->normals[0], GL_STATIC_DRAW);

		glGenBuffers(1, &plyColorBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, plyColorBuffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * numColors * 3, &plyMesh->colors[0], GL_STATIC_DRAW);

		glGenBuffers(1, &plyIndexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, plyIndexBuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, numIndices * sizeof(unsigned int), &plyMesh->indices[0], GL_STATIC_DRAW);

		numPlyMeshIndices = plyMesh->indices.size();
		// Deleting mesh data from Host's heap. The copy of mesh data is transferred to the GPU.
		delete plyMesh;
	}
}

void SceneMesh::unloadBuffers()
{
	if (type == OBJ)
	{
		const int numGroups = numMeshGroups;
		const int numMaterials = numMeshMaterials;

		glDeleteBuffers(numGroups, vertexBuffers);
		glDeleteBuffers(numGroups, normalBuffers);
		glDeleteBuffers(numGroups, uvBuffers);
		glDeleteBuffers(numGroups, indexBuffers);
		//glDeleteVertexArrays(1, vertexArrayId);
		glDeleteTextures(numMaterials, textureIds);
	}
	else if (type == PLY)
	{
		glDeleteBuffers(1, &plyVertexBuffer);
		glDeleteBuffers(1, &plyNormalBuffer);
		glDeleteBuffers(1, &plyColorBuffer);
		glDeleteBuffers(1, &plyIndexBuffer);
	}
}

void SceneMesh::renderMeshGroup(const int groupId)
{
	// Bind our texture in Texture Unit 0
	glActiveTexture(GL_TEXTURE0);

	int textureId = textureIdList[groupId];
	
	glBindTexture(GL_TEXTURE_2D, textureIds[textureId]);

	glBindVertexArray(vertexArrayId);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[groupId]);
	glVertexAttribPointer(
		0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
		);


	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, uvBuffers[groupId]);
	glVertexAttribPointer(
		1,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
		2,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
		);

	glEnableVertexAttribArray(2);
	glBindBuffer(GL_ARRAY_BUFFER, normalBuffers[groupId]);
	glVertexAttribPointer(
		2,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
		);

	// Index buffer
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffers[groupId]);

	glDrawElements(
		GL_TRIANGLES,      // mode
		numIndicesList[groupId],    // count
		GL_UNSIGNED_INT,   // type
		(void*)0           // element array buffer offset
		);
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
}

void SceneMesh::renderPlyMesh()
{
	
	glBindVertexArray(vertexArrayId);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, plyVertexBuffer);
	glVertexAttribPointer(
		0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
		);


	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, plyColorBuffer);
	glVertexAttribPointer(
		1,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
		);

	glEnableVertexAttribArray(2);
	glBindBuffer(GL_ARRAY_BUFFER, plyNormalBuffer);
	glVertexAttribPointer(
		2,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
		);

	//if (rType == Mesh)
	{
		// Index buffer
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, plyIndexBuffer);

		glDrawElements(
			GL_TRIANGLES,      // mode
			numPlyMeshIndices,    // count
			GL_UNSIGNED_INT,   // type
			(void*)0           // element array buffer offset
			);
	}
	/*else
	{
		glEnable(GL_PROGRAM_POINT_SIZE);
		glPointSize(5);
		glDrawArrays(GL_POINTS, 0, numPlyMeshIndices);
		glDisable(GL_PROGRAM_POINT_SIZE);
	}*/
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
}

