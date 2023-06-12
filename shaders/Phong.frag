#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
    float ka = 0.1;
    float kd=1;
    float ks=0.5;

    vec3 Ia = vec3(1,1,1);
    float p =20;

    vec3 r = u_light_pos - vec3(v_position);
    float diffuse = max(dot(normalize(r),vec3(v_normal)), 0);
    out_color = vec4(u_light_intensity * diffuse/dot(r, r), 1.0);
    vec3 h = normalize(u_light_pos + u_cam_pos - 2 * vec3(v_position));
    out_color += vec4(ks * (u_light_intensity / dot(r, r)) * pow(max(0, dot(vec3(v_normal), h) ),p),1);
    out_color += vec4(ka * Ia,1);

}

