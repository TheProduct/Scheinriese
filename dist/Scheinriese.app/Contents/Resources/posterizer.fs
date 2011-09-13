uniform sampler2D tex0;
const int THRESHOLD_SIZE = 8;
uniform float thresholds[THRESHOLD_SIZE];

void main()
{
	vec4 mPosterizedColor;
	vec4 mColor =  texture2D(tex0, gl_TexCoord[0].st);
	float intensity = max(0.0, min(1.0, ( mColor.r + mColor.g + mColor.b ) / 3.0 ));
	
	mPosterizedColor = vec4(thresholds[THRESHOLD_SIZE - 1], thresholds[THRESHOLD_SIZE - 1], thresholds[THRESHOLD_SIZE - 1], 1.0);

	for (int i=0; i < THRESHOLD_SIZE - 1; i++) {
		if (intensity >= thresholds[i] && intensity <= thresholds[i + 1]) { 
			mPosterizedColor = vec4(thresholds[i], thresholds[i], thresholds[i], 1.0);
		}	
	}

	gl_FragColor = gl_Color * mPosterizedColor;
}
