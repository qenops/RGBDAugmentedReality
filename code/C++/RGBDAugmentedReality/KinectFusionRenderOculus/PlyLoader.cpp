#include "PlyLoader.h"

bool PlyLoader::loadPly(const char * dataPath, const char * plyFileName, PlyMesh * mesh)
{
	printf("Loading Ply file %s...\n", plyFileName);

	char plyPath[256];
	sprintf(plyPath, "%s%s", dataPath, plyFileName);

	FILE * file = fopen(plyPath, "r");
	if (file == NULL)
	{
		printf("Impossible to open the ply file!\n");
		getchar();
		return false;
	}
	unsigned int numVertices = 0;
	unsigned int numFaces = 0;
	bool alpha = false;
	while (1)
	{
		char lineHeader[128];
		// read the first word of the line
		int res = fscanf(file, "%s", lineHeader);
		if (res == EOF)
			break; // EOF = End Of File. Quit the loop.

				   // else : parse lineHeader
		if (strcmp(lineHeader, "element") == 0)
		{
			fscanf(file, "%s", lineHeader);
			if (strcmp(lineHeader, "vertex") == 0)
			{
				fscanf(file, "%u", &numVertices);
			}
			else if (strcmp(lineHeader, "face") == 0)
			{
				fscanf(file, "%u", &numFaces);
			}
		}
		if (strcmp(lineHeader, "alpha") == 0)
			alpha = true;
		if (strcmp(lineHeader, "end_header") == 0)
		{
			for (unsigned int i = 0; i < numVertices; i++)
			{
				vec3 vertex,normal,color;
				if (!alpha)
				{
					int matches = fscanf(file, "%f %f %f %f %f %f %f %f %f\n", &vertex.x, &vertex.y, &vertex.z, &normal.x, &normal.y, &normal.z, &color.x, &color.y, &color.z);


					if (matches != 9)
					{
						printf("Matches loaded %d File can't be read by our simple parser :-( Try exporting with other options\n", matches);
						return false;
					}
				}
				else
				{
					float alphaC;
					int matches = fscanf(file, "%f %f %f %f %f %f %f %f %f %f\n", &vertex.x, &vertex.y, &vertex.z, &normal.x, &normal.y, &normal.z, &color.x, &color.y, &color.z,&alphaC);


					if (matches != 10)
					{
						printf("Matches loaded %d File can't be read by our simple parser :-( Try exporting with other options\n", matches);
						return false;
					}
				}
				color = color / 255.0f;
				mesh->vertices.push_back(vertex);
				mesh->normals.push_back(normal);
				mesh->colors.push_back(color);
			}
			for (unsigned int i = 0; i < numFaces; i++)
			{
				//printf("Reading Faces\n");
				unsigned int indices[3];
				int numSides;
				int matches = fscanf(file, "%d %u %u %u\n", &numSides, &indices[0], &indices[1], &indices[2]);
				if (numSides != 3)
				{
					printf("File can't be read by our simple parser as we support only triangular faces\n");
					return false;
				}
				else if (numSides != matches-1)
				{
					printf("File corrupted as one or more faces have incorrect number of edges\n");
					return false;
				}
				for (int j = 0; j < 3; j++)
				{
					mesh->indices.push_back(indices[j]);
				}
			}
			break;
		}

	}
	fclose(file);
	return true;
}
