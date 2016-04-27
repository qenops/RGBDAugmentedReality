#version 330

uniform sampler2D texUnit;

in vec2 f_texcoord;
out vec3 finalColor;

void main() {
	finalColor = texture(texUnit, f_texcoord).xyz;
}
