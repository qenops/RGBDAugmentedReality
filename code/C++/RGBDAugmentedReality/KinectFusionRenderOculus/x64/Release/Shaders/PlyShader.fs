#version 330 core

// Interpolated values from the vertex shaders
in vec3 VColor;

// Ouput data
out vec4 color;

void main()
{
	color.rgb = VColor; // Vertex Color

	color.a = 1.0;
}