#version 330 core

in vec3 Position_worldspace;
in vec3 Normal_cameraspace;
in vec3 EyeDirection_cameraspace;
in vec3 LightDirection_cameraspace;

out vec4 color;

uniform mat4 MV;
uniform vec3 u_LightPosition_world;
uniform float u_Transparency;
uniform vec3 u_LightColor;

void main(){
	float LightPower = 500.0f;

	vec3 MaterialDiffuseColor = vec3(0.5, 0.5, 0.5);
	vec3 MaterialAmbientColor = vec3(0.2,0.2,0.2) * MaterialDiffuseColor;
	vec3 MaterialSpecularColor = vec3(0.7,0.7,0.7);

	float distance = length( u_LightPosition_world - Position_worldspace );

	vec3 n = normalize( Normal_cameraspace );
	vec3 l = normalize( LightDirection_cameraspace );
	float cosTheta = clamp( dot( n,l ), 0,1 );

	vec3 eye = normalize(EyeDirection_cameraspace);
	vec3 refelct_direction = reflect(-l,n);
	float cosAlpha = clamp( dot( eye,refelct_direction ), 0,1 );
	
	color.xyz =
		MaterialAmbientColor +
		MaterialDiffuseColor * u_LightColor * LightPower * cosTheta / (distance*distance) +
		MaterialSpecularColor * u_LightColor * LightPower * pow(cosAlpha,5) / (distance*distance);
    color.a = u_Transparency;
}
