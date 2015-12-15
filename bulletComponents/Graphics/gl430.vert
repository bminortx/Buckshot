// Vertex shader for brick and mandelbrot shaders
// Derived from Orange Book Chapter 6

//  Light intensity and model position required by fragment shader
varying float LightIntensity;
varying vec2 ModelPos;
varying vec3 LightDir;
varying vec3 EyeDir;

// Center and zoom (for Mandelbrot set)
uniform vec3 loc;
varying vec3 normal;

//  Phong lighting intensity only
float phong()
{
//  P is the vertex coordinate on body
vec3 P = vec3(gl_ModelViewMatrix * gl_Vertex);
//  N is the object normal at P
vec3 N = normalize(gl_NormalMatrix * gl_Normal);
normal = N;
//  Light Position for light 0
vec3 LightPos = vec3(gl_LightSource[0].position);
//  L is the light vector
vec3 L = normalize(LightPos - P);
LightDir = L;
//  R is the reflected light vector R = 2(L.N)N - L
vec3 R = reflect(-L, N);
//  V is the view vector (eye at the origin)
vec3 V = normalize(-P);
EyeDir = V;
//  Diffuse light intensity is cosine of light and normal vectors
float Id = max(dot(L,N) , 0.0);
//  Shininess intensity is cosine of light and reflection vectors to a power
float Is = (Id>0.0) ? pow(max(dot(R,V) , 0.0) , gl_FrontMaterial.shininess) : 0.0;

//  Vertex color (ignores emission and global ambient)
vec3 color = gl_FrontLightProduct[0].ambient.rgb
             + Id*gl_FrontLightProduct[0].diffuse.rgb
             + Is*gl_FrontLightProduct[0].specular.rgb;

//  Vertex intensity
return length(color);
}

void main()
{
//  Scalar light intensity (for fragment shader)
LightIntensity = phong();
//  Save model coordinates (for fragment shader)
ModelPos = loc.z*gl_Vertex.xy - loc.xy;
//  Return fixed transform coordinates for this vertex
gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
