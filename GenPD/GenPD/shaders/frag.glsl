#version 150

uniform int u_choose_tex;
uniform sampler2D u_sampler1;

in vec4 f_color;
in vec4 f_normal;
in vec4 f_light1;
in vec4 f_light2;
in vec4 f_light3;
in vec2 f_texcoord;

out vec4 out_Color;
void main()
{
    vec4 diffuseColor = f_color;//material color
    
    float diffuseTerm1 = clamp(dot(f_normal, f_light1), 0.0, 1.0);
    float diffuseTerm2 = clamp(dot(f_normal, f_light2), 0.0, 1.0);
    float diffuseTerm3 = clamp(dot(f_normal, f_light3), 0.0, 1.0);

	float totalTerm = (diffuseTerm1 + diffuseTerm2 + diffuseTerm3) / 2 * 0.7 + 0.3;

	if (u_choose_tex != 0)
		out_Color = texture( u_sampler1, vec2(f_texcoord)) * totalTerm;
	else
		out_Color = diffuseColor * totalTerm;
}
