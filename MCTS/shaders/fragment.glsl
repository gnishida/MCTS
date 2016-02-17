#version 330

// varying variables
in vec4 fColor;
in vec3 fTexCoord;
in vec3 fNormal;
in vec3 fPosition;
noperspective in vec3 dist;

// output color
out vec4 outputF;

uniform int pass;	// 1 -- 1st pass / 2 -- 2nd pass

// uniform variables
uniform int textureEnabled;		// 1 -- texture / 0 -- color only
//uniform int wireframeEnalbed;	// 0 -- no wireframe / 1 -- add wireframe
uniform int depthComputation;  // 1 -- depth computation / 0 - otherwise

uniform int useShadow;		// 1 -- use shadow / 0 - no shadow 
uniform mat4 light_mvpMatrix;
uniform vec3 lightDir;
uniform sampler2D tex0;
uniform sampler2D shadowMap;
uniform sampler2D normalMap;
uniform sampler2D depthMap;
uniform int screenWidth;
uniform int screenHeight;
uniform int renderingMode;		// 1 -- regular / 2 -- wireframe / 3 -- line / 4 -- sketchy
//uniform int lineRendering;     // 1 -- line rendering / 0 - otherwise
uniform float depthSensitivity;
uniform float normalSensitivity;
uniform int useThreshold;	// 1 -- use threshold / 0 -- otherwise
uniform float threshold;
uniform int seed;

float shadowCoef(){
	vec4 shadow_coord2 = light_mvpMatrix * vec4(fPosition, 1.0);
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

void main() {
	// for color mode
	float opacity = fColor.w;
	outputF = vec4(fColor.xyz, 1);

	// depth computation
	if (depthComputation == 1) return;

	if (textureEnabled == 1) { // for texture mode
		outputF = outputF * texture(tex0, fTexCoord.rg);
	}

	// lighting
	vec4 ambient = vec4(0.2, 0.2, 0.2, 1.0);
	vec4 diffuse = vec4(0.8, 0.8, 0.8, 1.0) * max(0.0, dot(-lightDir, fNormal));

	float shadow_coef = 1.0;
	if (useShadow == 1) {
		shadow_coef= shadowCoef();
	}
	outputF = (ambient + (shadow_coef * 0.95 + 0.05) * diffuse) * outputF;

	outputF.w = opacity;
}

