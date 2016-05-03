#version 330 core

// Interpolated values from the vertex shaders
in vec2 UV;

// Ouput data
out vec4 color;

// Values that stay constant for the whole mesh.
uniform sampler2D textureSampler;

void main()
{
	// Material properties
	vec3 MaterialDiffuseColor = texture2D( textureSampler, UV ).rgb;

	color.rgb = MaterialDiffuseColor; // Texture Color

	color.a = 1.0;
}