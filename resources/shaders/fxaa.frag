#version 130

uniform sampler2D original_image;
uniform vec2 image_resolution;

//import our fxaa shader
//#pragma glslify: fxaa = require('../')

void main() {

  //vec2 iResolution = vec2(500,500);

  vec2 uv = vec2(gl_FragCoord.xy / image_resolution.xy);
  uv.y = 1.0 - uv.y;

  //can also use gl_FragCoord.xy
  vec2 fragCoord = uv * image_resolution;

  gl_FragColor = texture2D(original_image, uv);
}
