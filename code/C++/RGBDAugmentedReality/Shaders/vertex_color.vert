#version 330

uniform mat4 projection;
uniform mat4 modelview;
uniform mat4 camera;

in vec3 vPosition;
in vec3 vColor;

out vec3 fColor;

void main() {
    vec4 wp = camera * modelview * vec4(vPosition, 1.0);
    gl_Position = projection * wp;
    fColor = vColor;
}