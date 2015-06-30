#version 330

// varying variables
in vec3 varColor;
in vec3 varTexCoord;
in vec3 varNormal;
in vec3 varPosition;

// output color
out vec4 outputF;

// uniform variables
uniform int mode;	// 1 -- color / 2 -- texture
uniform sampler2D tex0;

uniform int shadowState;	// 1 -- normal / 2 -- shadow
uniform mat4 light_mvpMatrix;
uniform vec3 lightDir;
uniform sampler2D shadowMap;

float shadowCoef(){
	vec4 shadow_coord2 = light_mvpMatrix * vec4(varPosition, 1.0);
	vec3 ProjCoords = shadow_coord2.xyz / shadow_coord2.w;
	vec2 UVCoords;
	UVCoords.x = 0.5 * ProjCoords.x + 0.5;
    UVCoords.y = 0.5 * ProjCoords.y + 0.5;
    float z = 0.5 * ProjCoords.z + 0.5;
	
	float visibility = 1.0f;
	if (texture2D(shadowMap, UVCoords).z  <  z) {
		visibility = 0;
	}
	return visibility;
}

void main()
{
	// for color mode
	outputF = vec4(varColor, 1.0);

	// shadow
	if (shadowState == 2) {
		return;
	}

	if (mode == 2) { // for texture mode
		outputF = texture(tex0, varTexCoord.rg);
	}

	// lighting
	vec4 ambient = vec4(0.2, 0.2, 0.2, 1.0);
	vec4 diffuse = vec4(0.8, 0.8, 0.8, 1.0) * max(0.0, dot(-lightDir, varNormal));

	outputF = (ambient + diffuse) * outputF;

	float shadow_coef = 0.95;
	shadow_coef= shadowCoef();
	outputF = (ambient + (shadow_coef + 0.05) * diffuse) * outputF;
}

