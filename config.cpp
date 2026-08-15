#include "Config.h"

float gsampleRate;
int debounceSamples;

void configSetup(BelaContext *context)
{
	gsampleRate = context->audioSampleRate;
	debounceSamples = (int)(0.02 * context->audioSampleRate); // 20ms
}