#include <Bela.h>
#include "Config.h"
#include "Filter.h"
#include "Recording.h"
#include "Cropping.h"
#include "Trillsensors.h"

// Pin du potentiometre de volume (propre a render.cpp, pas de module dedie
// pour un seul potard)
const unsigned int volumePin = 2;

int printCount = 0;

bool setup(BelaContext *context, void *userData)
{
	configSetup(context);
	recordingSetup(context);
	croppingSetup(context, dataFlowlenght);

	calculate_coefficients(context->audioSampleRate, 1000, 0.707);

	if (!trillSensorsSetup()) {
		return false;
	}

	return true;
}

void render(BelaContext *context, void *userData)
{
	float filterFrequencyVal;
	float filterQVal;
	float volumeVal = 1.0f;

	for (int i = 0; i < context->analogFrames; i++) {
		filterFrequencyVal = map(analogRead(context, i, filterFrequencyPin), 0, 1, 100, 1000);
		filterQVal = map(analogRead(context, i, filterQPin), 0, 1, 0.5, 10);
		volumeVal = analogRead(context, i, volumePin); // deja entre 0 et 1

		calculate_coefficients(context->audioSampleRate, filterFrequencyVal, filterQVal);
	}

	printCount++;
	if (printCount >= 4410) { // environ toutes les 100ms a 44100 Hz
		rt_printf("Flex: touches=%d loc=%f | Ring: touches=%d loc=%f | Volume=%.2f\n",
			gNumActiveTouchesFlex, gTouchLocationCycleFlex, gNumActiveTouches, gTouchLocationCycle, volumeVal);
		printCount = 0;
	}

	// Bornes de previsualisation du crop, basees sur finalSample (une fois par bloc)
	int previewStart = (int)map(provisionnalBeginCrop, 0, 1, 0, finalSample.size());
	int previewEnd = (int)map(provisionnalEndingCrop, 0, 1, 0, finalSample.size());
	if (previewStart >= (int)finalSample.size()) previewStart = finalSample.size() - 1;
	if (previewEnd >= (int)finalSample.size()) previewEnd = finalSample.size() - 1;
	if (previewStart < 0) previewStart = 0;

	for (int i = 0; i < context->audioFrames; i++) {

		// Capture micro + bouton record (module recording)
		recordingProcessSample(context, i);

		// Bouton crop (module cropping)
		croppingProcessSample(context, i);

		// --- Lecture filtree, avec previsualisation dynamique du crop ---
		float in;

		if (gNumActiveTouchesFlex > 0) {
			// Mode previsualisation live : lecture dans finalSample, bornee par le crop provisoire
			if (finalReadPointer < previewStart || finalReadPointer > previewEnd) {
				finalReadPointer = previewStart;
			}
			in = finalSample[finalReadPointer];

			finalReadPointer++;
			if (finalReadPointer > previewEnd) finalReadPointer = previewStart;
		} else {
			// Mode normal : lecture du dernier crop valide
			in = finalSample[finalReadPointer];

			finalReadPointer++;
			if (finalReadPointer >= (int)finalSample.size()) finalReadPointer = 0;
		}

		float out = gB0 * in + gB1 * previousInput + gB2 * previousInput2 - gA1 * previousOutput - gA2 * previousOutput2;

		previousInput2 = previousInput;
		previousInput = in;
		previousOutput2 = previousOutput;
		previousOutput = out;

		// --- Scratch via le Ring, adapte au mode actif ---
		if (gNumActiveTouches > 0) {
			if (gNumActiveTouchesFlex > 0) {
				// Scratch a l'interieur de la zone de previsualisation
				finalReadPointer = previewStart + (int)map(gTouchLocationCycle, 0, 1, 0, previewEnd - previewStart + 1);
				if (finalReadPointer > previewEnd) finalReadPointer = previewEnd;
			} else {
				// Scratch dans le sample deja valide
				finalReadPointer = (int)map(gTouchLocationCycle, 0, 1, 0, finalSample.size());
				if (finalReadPointer >= (int)finalSample.size()) finalReadPointer = finalSample.size() - 1;
			}
		}

		for (int c = 0; c < context->audioOutChannels; c++) audioWrite(context, i, c, out * volumeVal);
	}
}

void cleanup(BelaContext *context, void *userData)
{
}