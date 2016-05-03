#include "ObjLoader.h"

#pragma warning (disable : 4996)
bool ObjLoader::loadOBJ(const char* dataPath, const char * objFileName, MeshData* mesh)
{
	printf("Loading OBJ file %s...\n", objFileName);

	std::vector<std::vector<unsigned int>> vertexIndices, uvIndices, normalIndices;
	std::vector<glm::vec3> temp_vertices;
	std::vector<glm::vec2> temp_uvs;
	std::vector<glm::vec3> temp_normals;
	std::vector<Material> temp_materials;

	char objPath[256];
	sprintf(objPath, "%s%s", dataPath, objFileName);

	FILE * file = fopen(objPath, "r");
	if (file == NULL)
	{
		printf("Impossible to open the obj file!\n");
		getchar();
		return false;
	}

	char mtlPath[256], mtlFileName[256];
	fscanf(file, "mtllib %s", mtlFileName);

	sprintf(mtlPath, "%s%s", dataPath, mtlFileName);

	FILE * fileMtl = fopen(mtlPath, "r");
	if (file == NULL)
	{
		printf("Impossible to open the material file!\n");
		getchar();
		return false;
	}

	while (1)
	{
		char lineHeader[128];
		// read the first word of the line
		int res = fscanf(fileMtl, "%s", lineHeader);
		if (res == EOF)
			break;
		if (strcmp(lineHeader, "newmtl") == 0)
		{
			char tag[256], map[256];
			fscanf(fileMtl, "%s", tag);
			fscanf(fileMtl, "%s", lineHeader);
			if (strcmp(lineHeader, "map_Kd") == 0)
				fscanf(fileMtl, "%s", map);
			Material m;
			strcpy(m.tag, tag);
			strcpy(m.map_Kd, map);

			temp_materials.push_back(m);
		}
	}

	fclose(fileMtl);

	int groupId = -1;
	while (1)
	{
		char lineHeader[128];
		// read the first word of the line
		int res = fscanf(file, "%s", lineHeader);
		if (res == EOF)
			break; // EOF = End Of File. Quit the loop.

				   // else : parse lineHeader

		if (strcmp(lineHeader, "v") == 0)
		{
			glm::vec3 vertex;
			fscanf(file, "%f %f %f\n", &vertex.x, &vertex.y, &vertex.z);
			temp_vertices.push_back(vertex);
		}
		else if (strcmp(lineHeader, "vt") == 0)
		{
			glm::vec2 uv;
			fscanf(file, "%f %f\n", &uv.x, &uv.y);
			uv.y = uv.y; // Invert V coordinate since we will only use DDS texture, which are inverted. Remove if you want to use TGA or BMP loaders.
			temp_uvs.push_back(uv);
		}
		else if (strcmp(lineHeader, "vn") == 0)
		{
			glm::vec3 normal;
			fscanf(file, "%f %f %f\n", &normal.x, &normal.y, &normal.z);
			temp_normals.push_back(normal);
		}
		else if (strcmp(lineHeader, "g") == 0)
		{
			char tag[256], matTag[256];
			fscanf(file, "%s", tag);

			Group g;
			strcpy(g.tag, tag);

			fscanf(file, "%s", lineHeader);
			if (strcmp(lineHeader, "usemtl") == 0)
				fscanf(file, "%s", matTag);

			for (int i = 0; i < temp_materials.size(); i++)
			{
				if (strcmp(temp_materials[i].tag, matTag) == 0)
				{
					g.mat = temp_materials[i];
					break;
				}
			}
			groupId = mesh->groups.size();
			mesh->groups.push_back(g);

			vector<unsigned int> g_vertexIndices, g_uvIndices, g_normalIndices;
			vertexIndices.push_back(g_vertexIndices);
			uvIndices.push_back(g_uvIndices);
			normalIndices.push_back(g_normalIndices);

		}
		else if (strcmp(lineHeader, "f") == 0)
		{
			std::string vertex1, vertex2, vertex3;
			unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
			int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0], &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2], &normalIndex[2]);
			if (matches != 9)
			{
				printf("File can't be read by our simple parser :-( Try exporting with other options\n");
				return false;
			}
			vertexIndices[groupId].push_back(vertexIndex[0]);
			vertexIndices[groupId].push_back(vertexIndex[1]);
			vertexIndices[groupId].push_back(vertexIndex[2]);
			uvIndices[groupId].push_back(uvIndex[0]);
			uvIndices[groupId].push_back(uvIndex[1]);
			uvIndices[groupId].push_back(uvIndex[2]);
			normalIndices[groupId].push_back(normalIndex[0]);
			normalIndices[groupId].push_back(normalIndex[1]);
			normalIndices[groupId].push_back(normalIndex[2]);
		}
		else
		{
			// Probably a comment, eat up the rest of the line
			char stupidBuffer[1000];
			fgets(stupidBuffer, 1000, file);
		}

	}
	fclose(file);

	mesh->numGroups = mesh->groups.size();
	mesh->numMaterials = temp_materials.size();
	mesh->materials = temp_materials;

	// For each group
	for (unsigned int i = 0; i< mesh->numGroups; i++)
	{
		Group *g = &mesh->groups[i];
		// For each triangle
		for (unsigned int j = 0; j < vertexIndices[i].size(); j++)
		{
			// Get the indices of its attributes
			unsigned int vertexIndex = vertexIndices[i][j];
			unsigned int uvIndex = uvIndices[i][j];
			unsigned int normalIndex = normalIndices[i][j];

			// Get the attributes thanks to the index
			glm::vec3 vertex = temp_vertices[vertexIndex - 1];
			glm::vec2 uv = glm::vec2(0, 0);
			if (uvIndex)
				uv = temp_uvs[uvIndex - 1];
			glm::vec3 normal = temp_normals[normalIndex - 1];

			// Put the attributes in buffers
			g->vertices.push_back(vertex);
			g->uvs.push_back(uv);
			g->normals.push_back(normal);
			g->indices.push_back(j);
		}
	}

	return true;
}
