#version 150

uniform mat4 u_modelviewMatrix;
uniform mat4 u_projMatrix;
uniform mat4 u_transformMatrix;

in vec3 v_position;
in vec3 v_color;
in vec3 v_normal;
in vec2 v_texcoord;

out vec4 f_color;
out vec4 f_normal;
out vec4 f_light1;
out vec4 f_light2;
out vec4 f_light3;
out vec2 f_texcoord;

void main()
{
    f_color = vec4(v_color, 1.0);
    f_normal = transpose(inverse(u_modelviewMatrix)) * vec4(v_normal, 0.0);
    f_light1 =normalize(u_modelviewMatrix * vec4(0.0, 0.707, -0.707, 0.0));
    f_light2 =normalize(u_modelviewMatrix * vec4(-0.707, -0.0, 0.707, 0.0));
    f_light3 =normalize(u_modelviewMatrix * vec4(0.707, -0.0, 0.707, 0.0));
	f_texcoord = v_texcoord;

    gl_Position = u_projMatrix * u_modelviewMatrix * u_transformMatrix * vec4(v_position, 1.0);
}
