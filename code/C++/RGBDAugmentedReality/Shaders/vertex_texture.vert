#version 330

uniform mat4 modelview;
uniform mat4 projection;
uniform mat4 camera;

in vec3 v_position;
in vec2 v_texcoord;

out vec2 f_texcoord;

void main() {
	gl_Position = projection * (camera * (modelview * vec4(v_position, 1.0)));
    f_texcoord = v_texcoord;
}
