
#ifndef MESH_DATA_H
#define MESH_DATA_H

// Include GLM
#include <glm.hpp>

#include <vector>

using namespace glm;
using namespace std;

struct Material
{
	float Ks;
	float Ka;
	float Kd;
	char map_Kd[256];
	char tag[256];
};


struct Group
{
	char tag[256];
	Material mat;
	vector<vec3> vertices;
	vector<vec3> normals;
	vector<vec2> uvs;
	vector<unsigned int> indices;
};

struct MeshData
{
	vector<Group> groups;
	int numGroups;
	int numMaterials;
	vector<Material> materials;
};

struct PlyMesh
{
	vector<vec3> vertices;
	vector<vec3> normals;
	vector<vec3> colors;
	vector<unsigned int> indices;
};

#endif

