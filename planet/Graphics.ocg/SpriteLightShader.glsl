uniform mat3x2 lightTransform;
#ifdef HAVE_NORMALMAP
uniform sampler2D normalTex;
#endif

slice(texture+4)
{
	// prepare texture coordinate for light lookup in LightShader.c
	vec2 lightCoord = lightTransform * vec3(gl_FragCoord.xy, 1.0);
}

slice(normal)
{
#ifdef HAVE_NORMALMAP
	vec4 normalPx = texture2D(normalTex, texcoord.xy);
	vec3 normalPxDir = 2.0 * (normalPx.xyz - vec3(0.5, 0.5, 0.5));
	vec3 normal = normalize(gl_NormalMatrix * normalPxDir);
#else
	vec3 normal = vec3(0.0, 0.0, 1.0);
#endif
}
