#include "GLHelper.h"
#include <stdexcept>
#include <fstream>
#include <iostream>
using namespace std;

inline void getGLMat4(const algebra3::mat4& M, GLfloat arr[16]) {
	const int R[] = {0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3};
	const int C[] = {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3};
	for (int i = 0; i < 16; ++i) {
		arr[i] = M[R[i]][C[i]];
	}
}

inline void getGLMat3(const algebra3::mat3& M, GLfloat arr[9]) {
	const int R[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
	const int C[] = {0, 0, 0, 1, 1, 1, 2, 2, 2};
	for (int i = 0; i < 9; ++i) {
		arr[i] = M[R[i]][C[i]];
	}
}

void die() {
	system("pause");
	exit(1);
}

void dieOnInvalidIndex(int k, const string& err) {
	if (k < 0) {
		std::cout << "Index " << k << " invalid on " << err << std::endl;
		die();
	}
}

void dieOnGLError(const string& errMessage) {
	GLenum errID = GL_NO_ERROR;
	errID = glGetError();
	if (errID != GL_NO_ERROR) {
		char errorMessage[1024];
		cout << "GL Error " << errID << ": " << errMessage << endl;
		die();
	}
}

void setUniformInt(GLuint program, const string& uniformName, int v) {
	int loc = glGetUniformLocation(program, uniformName.c_str());
	dieOnInvalidIndex(loc, uniformName);
	glUniform1i(loc, v);
	dieOnGLError("glUniform1i on " + uniformName);
}

void setUniformFloat(GLuint program, const string& uniformName, float v) {
	int loc = glGetUniformLocation(program, uniformName.c_str());
	dieOnInvalidIndex(loc, uniformName);
	glUniform1f(loc, v);
	dieOnGLError("glUniform1f on " + uniformName);
}

void setUniformVec2(GLuint program, const string& uniformName, const algebra3::vec2& v) {
	int loc = glGetUniformLocation(program, uniformName.c_str());
	dieOnInvalidIndex(loc, uniformName);
	glUniform2f(loc, v[0], v[1]);
	dieOnGLError("glUniform2f on " + uniformName);
}

void setUniformVec3(GLuint program, const string& uniformName, const algebra3::vec3& v) {
	int loc = glGetUniformLocation(program, uniformName.c_str());
	dieOnInvalidIndex(loc, uniformName);
	glUniform3f(loc, v[0], v[1], v[2]);
	dieOnGLError("glUniform3f on " + uniformName);
}

void setUniformVec4(GLuint program, const string& uniformName, const algebra3::vec4& v) {
	int loc = glGetUniformLocation(program, uniformName.c_str());
	dieOnInvalidIndex(loc, uniformName);
	glUniform4f(loc, v[0], v[1], v[2], v[3]);
	dieOnGLError("glUniform4f on " + uniformName);
}

void setUniformMat3(GLuint program, const string& uniformName, const algebra3::mat3& v) {
	int loc = glGetUniformLocation(program, uniformName.c_str());
	dieOnInvalidIndex(loc, uniformName);
	float glMat[9]; getGLMat3(v, glMat);
	glUniformMatrix3fv(loc, 1, GL_FALSE, glMat);
	dieOnGLError("glUniformMatrix3fv on " + uniformName);
}

void setUniformMat4(GLuint program, const string& uniformName, const algebra3::mat4& v) {
	int loc = glGetUniformLocation(program, uniformName.c_str());
	dieOnInvalidIndex(loc, uniformName);
	float glMat[16]; getGLMat4(v, glMat);
	glUniformMatrix4fv(loc, 1, GL_FALSE, glMat);
	dieOnGLError("glUniformMatrix4fv on " + uniformName);
}

string getCodeFromSource(const string& fName) {
	ifstream fin(fName);
	if (!fin.is_open()) {
		cout << "The shader source file could not be found" << endl;
		die();
	}
	string temp;
	string finalOut = "";
	while (true) {
		getline(fin, temp);
		finalOut.append(temp);
		if (fin.eof())
			break;
		else
			finalOut.append("\n");
	}
	fin.close();
	return finalOut;
}

GLuint createShaderProgram(const string& vShaderFilename, const string& fShaderFilename) {
	string vShaderSource = getCodeFromSource(vShaderFilename);
	string fShaderSource = getCodeFromSource(fShaderFilename);
	const char * str = NULL;

	GLuint vShader = glCreateShader(GL_VERTEX_SHADER);
	dieOnGLError("glCreateShader for GL_VERTEX_SHADER");
	str = vShaderSource.c_str();
	glShaderSource(vShader, 1, &str, NULL);
	glCompileShader(vShader);
	dieOnGLError("glCompileShader for vShader");

	GLuint fShader = glCreateShader(GL_FRAGMENT_SHADER);
	dieOnGLError("glCreateShader for GL_FRAGMENT_SHADER");
	str = fShaderSource.c_str();
	glShaderSource(fShader, 1, &str, NULL);
	glCompileShader(fShader);
	dieOnGLError("glCompileShader for fShader");

	GLuint program = glCreateProgram();
	dieOnGLError("glCreateProgram");
	
	glAttachShader(program, vShader);
	dieOnGLError("glAttachShader for vShader");
	glAttachShader(program, fShader);
	dieOnGLError("glAttachShader for fShader");
	
	glLinkProgram(program);
	dieOnGLError("glLinkProgram");
	GLint status = GL_FALSE;
	glGetProgramiv(program, GL_LINK_STATUS, &status);
	if (status == GL_FALSE) {
		GLchar buff[1024];
		int len;
		glGetProgramInfoLog(program, 1023, &len, buff);
		cout << "Program could not be linked. Error Log: " << endl << buff << endl;
		die();
	}

	glUseProgram(program);
	dieOnGLError("glUseProgram");
	glGetProgramiv(program, GL_VALIDATE_STATUS, &status);
	if (status == GL_FALSE) {
		GLchar buff[1024];
		int len;
		glGetProgramInfoLog(program, 1023, &len, buff);
		cout << "Program could not be validated. Error Log: " << endl << buff << endl;
		die();
	}
	return program;
}

bool ReadPLY(string filename, vector<float> &vertices, vector<float> &colors, vector<unsigned int> &faces)
{
	char line[1024];
	char dummy[100];
	int dummy2 = 0;
	int noVerts;
	int noTris;
	float vertList[3];
	unsigned int faceList[3];
	int colorList[3];
	ifstream fin;
	fin.open(filename);
	if (fin.is_open())
	{
		do
		{
			fin.getline(line, 1024);
		} while (strncmp(line, "element vertex", 14) != 0);
		sscanf(line, "%s %s %d", dummy, dummy, &noVerts);
		do
		{
			fin.getline(line, 1024);
		} while (strncmp(line, "element face", 12) != 0);
		sscanf(line, "%s %s %d", dummy, dummy, &noTris);
		do
		{
			fin.getline(line, 1024);
		} while (strncmp(line, "end_header", 10) != 0);
		vertices.reserve(3 * noVerts);
		colors.reserve(3 * noVerts);
		faces.reserve(3 * noTris);
		for (int i = 0; i < noVerts; i++)
		{
			fin.getline(line, 1024);
			sscanf(line, "%f %f %f %d %d %d %d", &vertList[0], &vertList[1], &vertList[2], &colorList[0], &colorList[1], &colorList[2], &dummy2);
			vertices.push_back(vertList[0] );
			vertices.push_back(vertList[1] );
			vertices.push_back(vertList[2] );
			colors.push_back(static_cast<float>(colorList[0]) / 255.0);
			colors.push_back(static_cast<float>(colorList[1]) / 255.0);
			colors.push_back(static_cast<float>(colorList[2]) / 255.0);
		}
		for (int i = 0; i < noTris; i++)
		{
			fin.getline(line, 1024);
			sscanf(line, "%d %d %d %d", &dummy2, &faceList[0], &faceList[1], &faceList[2]);
			faces.push_back(faceList[0]);
			faces.push_back(faceList[1]);
			faces.push_back(faceList[2]);
		}
		fin.close();
	}
	else
	{
		std::cout << "Cannot open mesh file" << std::endl;
	}
	std::cout <<"Loaded mesh with " <<noVerts << " Vertices and " <<noTris <<" Faces"<<std::endl;
	return true;
}

bool ReadPLY(string filename, vector<float> &vertices, vector<float> &colors, vector<float> &normals, ,vector<unsigned int> &faces)
{
	char line[1024];
	char dummy[100];
	int dummy2 = 0;
	int noVerts;
	int noTris;
	float vertList[3];
	unsigned int faceList[3];
	int colorList[3];
	ifstream fin;
	fin.open(filename);
	if (fin.is_open())
	{
		do
		{
			fin.getline(line, 1024);
		} while (strncmp(line, "element vertex", 14) != 0);
		sscanf(line, "%s %s %d", dummy, dummy, &noVerts);
		do
		{
			fin.getline(line, 1024);
		} while (strncmp(line, "element face", 12) != 0);
		sscanf(line, "%s %s %d", dummy, dummy, &noTris);
		do
		{
			fin.getline(line, 1024);
		} while (strncmp(line, "end_header", 10) != 0);
		vertices.reserve(3 * noVerts);
		colors.reserve(3 * noVerts);
		faces.reserve(3 * noTris);
		for (int i = 0; i < noVerts; i++)
		{
			fin.getline(line, 1024);
			sscanf(line, "%f %f %f %d %d %d %d", &vertList[0], &vertList[1], &vertList[2], &colorList[0], &colorList[1], &colorList[2], &dummy2);
			vertices.push_back(vertList[0]);
			vertices.push_back(vertList[1]);
			vertices.push_back(vertList[2]);
			colors.push_back(static_cast<float>(colorList[0]) / 255.0);
			colors.push_back(static_cast<float>(colorList[1]) / 255.0);
			colors.push_back(static_cast<float>(colorList[2]) / 255.0);
		}
		for (int i = 0; i < noTris; i++)
		{
			fin.getline(line, 1024);
			sscanf(line, "%d %d %d %d", &dummy2, &faceList[0], &faceList[1], &faceList[2]);
			faces.push_back(faceList[0]);
			faces.push_back(faceList[1]);
			faces.push_back(faceList[2]);
		}
		fin.close();
	}
	else
	{
		std::cout << "Cannot open mesh file" << std::endl;
	}
	std::cout << "Loaded mesh with " << noVerts << " Vertices and " << noTris << " Faces" << std::endl;
	return true;
}


RenderObject::RenderObject(vector<float> *vertices, vector<float> *colors, vector<unsigned int> *indices):m_vertices(vertices),m_colors(colors),m_indices(indices)
{
	if (vertices == NULL || colors == NULL || indices == NULL)
	{
		std::cout << "Couldn't Create Render Objects. Check if all inputs have been initialized" << std::endl;
	}
	else
	{
		glGenVertexArrays(1, &m_vao);
		glBindVertexArray(m_vao);
		glGenBuffers(1, &m_vbo);
		glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
		glBufferData(GL_ARRAY_BUFFER, vertices->size() * sizeof(GLfloat), vertices->data(), GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glGenBuffers(1, &m_cbo);
		glBindBuffer(GL_ARRAY_BUFFER, m_cbo);
		glBufferData(GL_ARRAY_BUFFER, colors->size() * sizeof(GLfloat), colors->data(), GL_STATIC_DRAW);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer((GLuint)1, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glGenBuffers(1, &m_elembuf);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_elembuf);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices->size() * sizeof(GLuint), indices->data(), GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glBindVertexArray(0);
		m_indCount = m_indices->size();
		m_vertCount = m_vertices->size() / 3;
		m_vertices = new vector<float>();
		m_colors = new vector<float>();
		m_indices = new vector<unsigned int>();
		*m_vertices = *vertices;
		*m_colors = *colors;
		*m_indices = *indices;

		m_hasVerts = true;
		m_hasColors = true;
		m_hasFaces = true;
	}
}

RenderObject::RenderObject(vector<float>* vertices, vector<float>* colors)
{
	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);
	glGenBuffers(1, &m_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glBufferData(GL_ARRAY_BUFFER, vertices->size() * sizeof(GLfloat), vertices->data(), GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glGenBuffers(1, &m_cbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_cbo);
	glBufferData(GL_ARRAY_BUFFER, colors->size() * sizeof(GLfloat), colors->data(), GL_STATIC_DRAW);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer((GLuint)1, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);
	glBindVertexArray(0);
	m_indCount = 0;
	m_vertCount = m_vertices->size()/3;
	m_hasVerts = true;
	m_hasColors = true;
	m_hasFaces = false;
}

bool RenderObject::AddTexCoords(vector<float>* texCoords)
{
	if (texCoords == NULL)
	{
		std::cout << "Couldn't add textures. Check if all inputs have been initialized" << std::endl;
		return false;
	}
	else if(!m_hasVerts || !m_hasFaces)
	{
		std::cout << "This render object is not a mesh" << std::endl;
		return false;
	}
	else if (!m_hasColors)
	{
		if (m_vertCount == texCoords->size() / 2)
		{
			glBindVertexArray(m_vao);
			glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
			glBufferData(GL_ARRAY_BUFFER, m_vertices->size() * sizeof(GLfloat), m_vertices->data(), GL_STATIC_DRAW);
			glEnableVertexAttribArray(0);
			glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);
			glGenBuffers(1, &m_tbo);
			glBindBuffer(GL_ARRAY_BUFFER, m_tbo);
			glBufferData(GL_ARRAY_BUFFER, texCoords->size() * sizeof(GLfloat), texCoords->data(), GL_STATIC_DRAW);
			glEnableVertexAttribArray(2);
			glVertexAttribPointer((GLuint)2, 2, GL_FLOAT, GL_FALSE, 0, 0);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_elembuf);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices->size() * sizeof(GLuint), m_indices->data(), GL_STATIC_DRAW);
			glEnableVertexAttribArray(0);
			glBindVertexArray(0);

			m_texCoords = new vector<float>;
			*m_texCoords = *texCoords;
			return true;
		}
		else
		{
			std::cout << "Size of texCoords buffer is not correct" << std::endl;
			return false;
		}
	}
	else
	{
		if (m_vertCount == texCoords->size() / 2)
		{
			glBindVertexArray(m_vao);
			glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
			glBufferData(GL_ARRAY_BUFFER, m_vertices->size() * sizeof(GLfloat), m_vertices->data(), GL_STATIC_DRAW);
			glEnableVertexAttribArray(0);
			glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);
			glBindBuffer(GL_ARRAY_BUFFER, m_cbo);
			glBufferData(GL_ARRAY_BUFFER, m_colors->size() * sizeof(GLfloat), m_colors->data(), GL_STATIC_DRAW);
			glEnableVertexAttribArray(1);
			glVertexAttribPointer((GLuint)1, 3, GL_FLOAT, GL_FALSE, 0, 0);
			glGenBuffers(1, &m_tbo);
			glBindBuffer(GL_ARRAY_BUFFER, m_tbo);
			glBufferData(GL_ARRAY_BUFFER, texCoords->size() * sizeof(GLfloat), texCoords->data(), GL_STATIC_DRAW);
			glEnableVertexAttribArray(2);
			glVertexAttribPointer((GLuint)2, 2, GL_FLOAT, GL_FALSE, 0, 0);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_elembuf);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices->size() * sizeof(GLuint), m_indices->data(), GL_STATIC_DRAW);
			glEnableVertexAttribArray(0);
			glBindVertexArray(0);

			m_texCoords = new vector<float>;
			*m_texCoords = *texCoords;
			return true;
		}
		else
		{
			std::cout << "Size of texCoords buffer is not correct" << std::endl;
			return false;
		}
	}
	return false;
}


RenderObject::RenderObject(): m_indCount(0), m_vertCount(0)
{
}

RenderObject::~RenderObject()
{
}

int RenderObject::GetIndCount()
{
	return m_indCount;
}
int RenderObject::GetVertCount()
{
	return m_vertCount;
}
vector<unsigned int>* RenderObject::GetIndices()
{
	return m_indices;
}
vector<float>* RenderObject::GetVertices()
{
	return m_vertices;
}
vector<float>* RenderObject::GetColors()
{
	return m_colors;
}

vector<float>* RenderObject::GetNormals()
{
	return m_normals;
}

GLuint  RenderObject::GetVAO()
{
	return m_vao;
}

GLuint  RenderObject::GetVBO()
{
	return m_vbo;
}

GLuint  RenderObject::GetCBO()
{
	return m_cbo;
}
GLuint RenderObject::GetElemBuff()
{
	return m_elembuf;
}

bool RenderObject::GetHasFaces()
{
	return m_hasFaces;
}


Texture::Texture()
{
}

Texture::~Texture()
{}

Texture::Texture(cv::Mat image, GLint minMagFiler, GLint wrapMode)
{
	if (image.rows > 0 && image.cols > 0)
	{
		glGenTextures(1, &m_tex);
		glBindTexture(GL_TEXTURE_2D, m_tex);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minMagFiler);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, minMagFiler);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapMode);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapMode);
		glTexImage2D(GL_TEXTURE_2D, 0,  GL_RGBA, image.cols, image.rows,0, GL_BGRA, GL_UNSIGNED_BYTE, (GLvoid*)image.data);
		glBindTexture(GL_TEXTURE_2D, 0);
	}
	else
	{
		std::cout << "Could not create texture" << std::endl;
	}
}

bool Texture::UpdateTexture(cv::Mat image)
{
	if (image.rows > 0 && image.cols > 0)
	{
		glBindTexture(GL_TEXTURE_2D, m_tex);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.cols, image.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, (GLvoid*)image.data);
		glBindTexture(GL_TEXTURE_2D, 0);
		return true;
	}
	else
	{
		std::cout << "Could not create texture" << std::endl;
		return false;
	}
}

GLuint Texture::GetTexture()
{
	return m_tex;
}
