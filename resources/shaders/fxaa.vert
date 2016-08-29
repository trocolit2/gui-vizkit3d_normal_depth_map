// based on https://github.com/mattdesl/glsl-fxaa/blob/master/texcoords.glsl
#version 130

attribute vec2 position;
uniform vec2 image_resolution;

out vec2 v_rgbNW;
out vec2 v_rgbNE;
out vec2 v_rgbSW;
out vec2 v_rgbSE;
out vec2 v_rgbM;
out vec2 vUv;

void texcoords(vec2 fragCoord, vec2 resolution,
			out vec2 v_rgbNW, out vec2 v_rgbNE,
			out vec2 v_rgbSW, out vec2 v_rgbSE,
			out vec2 v_rgbM) {
	vec2 inverseVP = 1.0 / resolution.xy;
	v_rgbNW = (fragCoord + vec2(-1.0, -1.0)) * inverseVP;
	v_rgbNE = (fragCoord + vec2(1.0, -1.0)) * inverseVP;
	v_rgbSW = (fragCoord + vec2(-1.0, 1.0)) * inverseVP;
	v_rgbSE = (fragCoord + vec2(1.0, 1.0)) * inverseVP;
	v_rgbM = vec2(fragCoord * inverseVP);
}


void main(){
    vec2 temp_pos = (gl_ModelViewMatrix * gl_Vertex).xy;
    gl_Position = vec4(temp_pos, 0.0, 1.0);

    vUv = (temp_pos + 1.0) * 0.5;
    vUv.y = 1.0 - vUv.y;
    vec2 fragCoord = vUv * image_resolution;

    texcoords( fragCoord, image_resolution, v_rgbNW,
               v_rgbNE, v_rgbSW, v_rgbSE, v_rgbM);
}
