// Custom shader developed by Brandon Minor
varying float LightIntensity;
varying vec2  ModelPos;
varying vec3 ModelXYZ;
varying vec3 LightDir;
varying vec3 EyeDir;
varying vec3 normal;
uniform float time;

// Crystal!
// Modified from the OpenGL Programming Guide 8ed, ex 8.10

uniform float CrystalDensity;
uniform float CrystalSize;

void main() {
  vec4 surface_color = vec4(0.0, 0.5, 0.5, 1.0);
  vec2 center = CrystalDensity * ModelPos;
  vec2 perturb = fract(center) - vec2(0.5);
  // Reflect if the Light and normal are in a good range
  float dist = dot(LightDir, normal);
  float norm_factor = inversesqrt(dist + 1.0);
  if (dist <= CrystalSize) {
    perturb = vec2(0.0);
    norm_factor = 1.0;
    surface_color.g = 0.1;
  }
  vec3 norm_delta = vec3(perturb.x, perturb.y, 1.0) * norm_factor;
  vec3 color = surface_color.rgb * max(dot(norm_delta, LightDir), 0.1);
  vec3 reflect_dir = reflect(LightDir, norm_delta);
  float spec = max(dot(EyeDir, reflect_dir), 0.0);
  color = min(color, vec3(1.0));
  gl_FragColor = vec4(color, surface_color.a);
}
