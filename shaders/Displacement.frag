#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_3;
uniform vec2 u_texture_3_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_3, uv).r;
}

void main() {
  // YOUR CODE HERE
  mat3 TBN = mat3(vec3(v_tangent),cross(vec3(v_normal),vec3(v_tangent)),vec3(v_normal));

  float dU = (h(vec2((v_uv.x+1)/u_texture_3_size.x,v_uv.y))-h(v_uv)) * u_normal_scaling * u_height_scaling;
  float dV = (h(vec2(v_uv.x, v_uv.y+1/u_texture_3_size.y))-h(v_uv))* u_normal_scaling * u_height_scaling;
  vec3 no = vec3(-dU, -dV, 1);
  vec3 nd = TBN * no;

  float ka = 0.1;
  float kd=0.7;
  float ks=0.7;

  vec3 Ia = vec3(1,1,1);
  float p =30;

  vec3 r = u_light_pos - vec3(v_position);
  float diffuse = max(dot(normalize(r),nd), 0);
  out_color = vec4(kd*u_light_intensity * diffuse/dot(r, r), 1.0);
  vec3 h = normalize(u_light_pos + u_cam_pos - 2 * vec3(v_position));
  out_color += vec4(ks * (u_light_intensity / dot(r, r)) * pow(max(0, dot(nd, h) ),p),1);
  out_color += vec4(ka * Ia,1);
  // (Placeholder code. You will want to replace it.)
//  out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
//  out_color.a = 1;
}

