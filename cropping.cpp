#include "Cropping.h"
#include "Config.h"

std::vector<float> finalSample;
int finalReadPointer = 0;

float provisionnalBeginCrop = 0;
float provisionnalEndingCrop = 1;

const unsigned int cropButtonPin = 1;
AuxiliaryTask cropTask;

// Debounce du bouton crop (independant des autres boutons)
static bool cropCandidateState = true;
static int cropCandidateCounter = 0;
static bool cropStableState = true;
static bool previousCropStableState = true;

void croppingSetup(BelaContext *context, int dataFlowlenght)
{
	pinMode(context, 0, cropButtonPin, INPUT);
	finalSample.resize(dataFlowlenght); // depart = taille complete
	cropTask = Bela_createAuxiliaryTask(&cropBuffer, 50, "crop-task");
}

void croppingProcessSample(BelaContext *context, int n)
{
	bool cropRawState = digitalRead(context, n, cropButtonPin);

	if (cropRawState == cropCandidateState) {
		cropCandidateCounter++;
	} else {
		cropCandidateState = cropRawState;
		cropCandidateCounter = 0;
	}

	if (cropCandidateCounter >= debounceSamples && cropCandidateState != cropStableState) {
		previousCropStableState = cropStableState;
		cropStableState = cropCandidateState;

		if (cropStableState == false && previousCropStableState == true) {
			rt_printf("Crop demande\n");
			Bela_scheduleAuxiliaryTask(cropTask);
		}
	}
}

void cropBuffer(void*)
{
	// Le crop se fait sur finalSample (pas orderedBuffer) pour permettre un affinage successif
	int startIdx = (int)map(provisionnalBeginCrop, 0, 1, 0, finalSample.size());
	int endIdx = (int)map(provisionnalEndingCrop, 0, 1, 0, finalSample.size());

	if (startIdx >= (int)finalSample.size()) startIdx = finalSample.size() - 1;
	if (endIdx >= (int)finalSample.size()) endIdx = finalSample.size() - 1;
	if (startIdx < 0) startIdx = 0;

	int newLength = endIdx - startIdx + 1;
	if (newLength < 1) newLength = 1;

	std::vector<float> cropped(newLength);
	for (int f = 0; f < newLength; f++) {
		cropped[f] = finalSample[startIdx + f];
	}

	finalSample = cropped; // remplace le contenu ET la taille
	finalReadPointer = 0;

	rt_printf("finalSample croppe, nouvelle taille : %d\n", newLength);
}