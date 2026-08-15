#include "Recording.h"
#include "Cropping.h"   // pour reinitialiser finalSample / finalReadPointer
#include "Config.h"
#include <libraries/AudioFile/AudioFile.h>
#include <cmath>

std::vector<float> DataFlow;
int dataFlowlenght;
int writingPointer = 0;

std::vector<float> orderedBuffer;

std::string gFilename = "rawSample.wav";
AuxiliaryTask writeFileTask;
const unsigned int recButtonPin = 0;

// Debounce du bouton record (independant des autres boutons)
static bool candidateState = true;
static int candidateCounter = 0;
static bool stableButtonState = true;
static bool previousStableState = true;

void recordingSetup(BelaContext *context)
{
	pinMode(context, 0, recButtonPin, INPUT);

	dataFlowlenght = context->audioSampleRate * 10; // 10 secondes
	DataFlow.resize(dataFlowlenght);
	orderedBuffer.resize(dataFlowlenght);

	writeFileTask = Bela_createAuxiliaryTask(&writeToFile, 50, "write-file-task");
}

void recordingProcessSample(BelaContext *context, int n)
{
	// Capture continue du micro dans le buffer circulaire
	DataFlow[writingPointer] = audioRead(context, n, 0);
	writingPointer++;
	if (writingPointer >= (int)DataFlow.size()) writingPointer = 0;

	// Debounce du bouton record
	bool rawState = digitalRead(context, n, recButtonPin);
	if (rawState == candidateState) {
		candidateCounter++;
	} else {
		candidateState = rawState;
		candidateCounter = 0;
	}
	if (candidateCounter >= debounceSamples && candidateState != stableButtonState) {
		previousStableState = stableButtonState;
		stableButtonState = candidateState;

		if (stableButtonState == false && previousStableState == true) {
			rt_printf("Bouton presse (valide)\n");
			Bela_scheduleAuxiliaryTask(writeFileTask);
		}
	}
}

void writeToFile(void*)
{
	int firstSegmentLength = DataFlow.size() - writingPointer;

	for (int f = 0; f < firstSegmentLength; f++) {
		orderedBuffer[f] = DataFlow[writingPointer + f];
	}
	for (int f = 0; f < writingPointer; f++) {
		orderedBuffer[firstSegmentLength + f] = DataFlow[f];
	}

	// Diagnostic : verifie que le buffer contient bien un signal exploitable
	float maxVal = 0;
	for (int f = 0; f < dataFlowlenght; f++) {
		if (fabsf(orderedBuffer[f]) > maxVal) {
			maxVal = fabsf(orderedBuffer[f]);
		}
	}
	rt_printf("Valeur max absolue dans le buffer : %f\n", maxVal);

	AudioFileUtilities::write(gFilename, orderedBuffer.data(), 1, dataFlowlenght, (unsigned int)gsampleRate);
	rt_printf("Fichier .wav ecrit avec succes\n");

	// Nouvel enregistrement = reset du crop, tout le sample redevient disponible
	finalSample = orderedBuffer;
	finalReadPointer = 0;
}