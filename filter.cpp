#include "Filter.h"
#include <cmath>

const unsigned int filterFrequencyPin = 0;
const unsigned int filterQPin = 1;

float gA1 = 0, gA2 = 0;
float gB0 = 1, gB1 = 0, gB2 = 0;

float previousInput = 0, previousOutput = 0, previousInput2 = 0, previousOutput2 = 0;

void calculate_coefficients(float sampleRate, float frequency, float q)
{
	float k = tanf(M_PI * frequency / sampleRate);
	float norm = 1.0 / (1 + k / q + k * k);

	gB0 = k * k * norm;
	gB1 = 2.0 * gB0;
	gB2 = gB0;
	gA1 = 2 * (k * k - 1) * norm;
	gA2 = (1 - k / q + k * k) * norm;
}