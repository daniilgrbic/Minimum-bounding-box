#version 330 core

layout(location = 0) in vec3 vertexPosition_modelspace;
layout(location = 1) in vec3 vertexNormal_modelspace;

out vec3 Position_worldspace;
out vec3 Normal_cameraspace;
out vec3 EyeDirection_cameraspace;
out vec3 LightDirection_cameraspace;

uniform mat4 u_MVP;
uniform mat4 u_V;
uniform mat4 u_M;
uniform vec3 u_LightPosition_world;

void main(){

	gl_Position =  u_MVP * vec4(vertexPosition_modelspace,1);

	Position_worldspace = (u_M * vec4(vertexPosition_modelspace,1)).xyz;

	vec3 vertexPosition_cameraspace = ( u_V * u_M * vec4(vertexPosition_modelspace,1)).xyz;
	EyeDirection_cameraspace = vec3(0,0,0) - vertexPosition_cameraspace;

	vec3 LightPosition_cameraspace = ( u_V * vec4(u_LightPosition_world,1)).xyz;
	LightDirection_cameraspace = LightPosition_cameraspace + EyeDirection_cameraspace;

	Normal_cameraspace = ( u_V * u_M * vec4(vertexNormal_modelspace,0)).xyz;
}
