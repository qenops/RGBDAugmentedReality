#version 330

uniform mat4 projection;
uniform mat4 modelview;
uniform vec4 K; // [fx, fy, cx, cy]
uniform float depthScale;

in vec3 vPosition;
in vec3 vColor;

out vec4 fColor;

void main() {
	if (vPosition.z <= 0) {
		// Dont draw the point if the depth is invalid
		gl_Position = projection * vec4(0.0, 0.0, 0.0, 1.0);
		fColor = vec4(0.0, 0.0, 0.0, 0.0);
		return;
	}

	// Negative sign with the x-coordinate because the images are flipped along the vertical
	vec3 point3d = depthScale * vPosition.z * vec3(-(vPosition.x - K[2]) / K[0], (vPosition.y - K[3]) / K[1], 1.0);
	gl_Position = projection * modelview * vec4(point3d, 1.0);
	fColor = vec4(vColor/255.0, 1.0);
}