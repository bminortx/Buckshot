#version 430 core

uniform mat4 ModelViewMatrix;
uniform mat4 ProjectionMatrix;
uniform mat4 NormalMatrix;
uniform vec3 light_position;
layout(location = 0) in vec4 Vertex;
layout(location = 1) in vec3 Normal;
layout(location = 2) in vec3 Color;
layout(location = 3) in vec2 TexCoord;
out vec4 FragColor;
out vec3 FragPosition;
out vec3 FragNormal;
out vec2 FragTexCoord;
out vec3 FragView;
out vec3 FragLight;

void main() {
  FragColor = vec4(Color, 1.0);
  FragNormal = vec3(ModelViewMatrix * vec4(Normal, 0));
  // Get position in eye coordinates
  vec4 view_pos = ModelViewMatrix * Vertex;
  FragPosition = vec3(view_pos / view_pos.w);
  FragTexCoord = TexCoord;
  FragView = -FragPosition;
  FragLight = vec3(light_position - FragPosition);
  gl_Position =  ProjectionMatrix * ModelViewMatrix * Vertex;
}
