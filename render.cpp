#include <Bela.h>
#include "Config.h"
#include "Filter.h"
#include "Recording.h"
#include "Cropping.h"
#include "Trillsensors.h"
#include "Distsensor.h"

// Volume pot
const unsigned int volumePin = 2;

// Speed pot (always positive for now: 0.25x to 3x)
const unsigned int speedPin = 4;
const float minSpeed = 0.25f;
const float maxSpeed = 3.0f;

// Scale pot for the 3rd string (analog pin, no pinMode needed)
const unsigned int tuningPin = 5;

int printCount = 0;

// Precise reading position in finalSample (float, keeps the fractions of the
// speed). finalReadPointer is only its integer part, used to index the array.
float readPosition = 0;

bool setup(BelaContext *context, void *userData)
{
	configSetup(context);
	recordingSetup(context);
	croppingSetup(context, dataFlowlenght);

	calculate_coefficients(context->audioSampleRate, 1000, 0.707);

	if (!trillSensorsSetup()) {
		return false;
	}

	if (!distanceSensorSetup(context)) {
		return false;
	}

	return true;
}

void render(BelaContext *context, void *userData)
{
	float filterFrequencyVal = 1000;
	float filterQVal = 0.707;
	float volumeVal = 1.0f;
	float speedVal = 1.0f;
	float tuningRaw = 0;

	// Scale currently applied to the 3rd string (-1 = not set yet)
	static int currentScale = -1;

	// Analog readings
	for (int i = 0; i < context->analogFrames; i++) {
		filterFrequencyVal = map(analogRead(context, i, filterFrequencyPin), 0, 1, 100, 1000);
		filterQVal = map(analogRead(context, i, filterQPin), 0, 1, 0.5, 10);
		volumeVal = analogRead(context, i, volumePin);
		speedVal = map(analogRead(context, i, speedPin), 0, 1, minSpeed, maxSpeed);
		tuningRaw = analogRead(context, i, tuningPin); // between 0 and 1

		distanceSensorReadVolume(context, i);

		calculate_coefficients(context->audioSampleRate, filterFrequencyVal, filterQVal);
	}

	// Scale choice: 6 positions (0 = continuous mapping, 1 to 5 = scales).
	// A small margin around each boundary avoids flickering between two scales
	// when the pot sits right between them.
	float pos = tuningRaw * 6.0f;
	if (currentScale < 0 || pos < currentScale - 0.1f || pos > currentScale + 1.1f) {
		currentScale = (int)pos;
		if (currentScale > 5) currentScale = 5;
		distanceSensorSetScale(currentScale);
		rt_printf("Gamme : %d\n", currentScale);
	}

	printCount++;
	if (printCount >= 4410) { // every 100ms at 44100 Hz
		rt_printf("Flex: touches=%d loc=%f | Ring: touches=%d loc=%f | Volume=%.2f | Vitesse=%.2f\n",
			gNumActiveTouchesFlex, gTouchLocationCycleFlex, gNumActiveTouches, gTouchLocationCycle, volumeVal, speedVal);
		printCount = 0;
	}

	// Bounds of the cropped sample preview (once per block)
	int previewStart = (int)map(provisionnalBeginCrop, 0, 1, 0, finalSample.size());
	int previewEnd = (int)map(provisionnalEndingCrop, 0, 1, 0, finalSample.size());
	if (previewStart >= (int)finalSample.size()) previewStart = finalSample.size() - 1;
	if (previewEnd >= (int)finalSample.size()) previewEnd = finalSample.size() - 1;
	if (previewStart < 0) previewStart = 0;

	for (int i = 0; i < context->audioFrames; i++) {

		// mic + record button
		recordingProcessSample(context, i);

		// crop button
		croppingProcessSample(context, i);

		// --- Reading: readPosition is the reference. Check it, then read, then advance ---
		float in;

		if (gNumActiveTouchesFlex > 0) {
			// Preview of the sample being cropped, bounded by the provisional crop
			if (readPosition < previewStart || readPosition >= previewEnd + 1) {
				readPosition = previewStart;
			}
			finalReadPointer = (int)readPosition;
			in = finalSample[finalReadPointer];
			readPosition += speedVal;
		} else {
			// Last validated crop, no input on the Flex
			if (readPosition < 0 || readPosition >= (float)finalSample.size()) {
				readPosition = 0;
			}
			finalReadPointer = (int)readPosition;
			in = finalSample[finalReadPointer];
			readPosition += speedVal;
		}

		float out = gB0 * in + gB1 * previousInput + gB2 * previousInput2 - gA1 * previousOutput - gA2 * previousOutput2;

		previousInput2 = previousInput;
		previousInput = in;
		previousOutput2 = previousOutput;
		previousOutput = out;

		// --- Ring: browsing and scratching. It writes readPosition directly,
		// so the speed continues from the position chosen with the finger ---
		if (gNumActiveTouches > 0) {
			if (gNumActiveTouchesFlex > 0) {
				// Scratch inside the preview zone
				readPosition = previewStart + map(gTouchLocationCycle, 0, 1, 0, previewEnd - previewStart + 1);
				if (readPosition >= previewEnd + 1) readPosition = previewEnd;
			} else {
				// Scratch in the validated sample
				readPosition = map(gTouchLocationCycle, 0, 1, 0, (float)finalSample.size());
				if (readPosition >= (float)finalSample.size()) readPosition = finalSample.size() - 1;
			}
		}

		float droneSample = distanceSensorProcessSample(context, i); // once per sample (not per channel)

		for (int c = 0; c < context->audioOutChannels; c++) audioWrite(context, i, c, out * volumeVal + droneSample);
	}
}

void cleanup(BelaContext *context, void *userData)
{
	distanceSensorCleanup();
}