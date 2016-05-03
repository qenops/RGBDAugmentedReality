#ifndef OBJLOADER_H
#define OBJLOADER_H

#include <vector>
#include <glm.hpp>

#include "MeshData.h"

class ObjLoader
{
	public:
		static bool loadOBJ(const char* dataPath, const char * objFileName, MeshData* mesh);

};
#endif