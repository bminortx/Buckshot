#version 430 core

// Light elements
uniform vec3 light_position;
uniform vec4 light_ambient;
uniform vec4 light_diffuse;
uniform vec4 light_specular;

uniform sampler2D tex;
in vec4 FragColor;
in vec3 FragPosition;
in vec3 FragNormal;
in vec2 FragTexCoord;
in vec3 FragLight;
in vec3 FragView;

//  Fragment color
layout (location=0) out vec4 Frontcolor;

void main() {
  // Start with ambient light
  Frontcolor = light_ambient * FragColor;
  vec3 N = normalize(FragNormal);
  vec3 L = normalize(FragLight);
  float diffuse = dot(N, L);
  if (diffuse > 0.0) {
    Frontcolor += diffuse * light_diffuse;
    // Add Reflection
    vec3 R = reflect(-L, N);
    vec3 V = normalize(FragView);
    float specular = dot(V, R);
    if (specular > 0.0) {
      // SUCH SPECULAR
      Frontcolor += pow(specular, 128) * light_specular;
    }
  }
  Frontcolor = Frontcolor * texture2D(tex, FragTexCoord);
}
