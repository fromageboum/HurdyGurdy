#include <Bela.h>
#include "Config.h"
#include "Filter.h"
#include "Recording.h"
#include "Cropping.h"
#include "Trillsensors.h"
#include "Distsensor.h"


// Volume pot
const unsigned int volumePin = 2;

//Tuning pot
const unsigned int tuningPin = 5;
const unsigned int minScale= 1;
const unsigned int maxScale=5;

// Speed pot
const unsigned int speedPin = 4;
const int minSpeed = -2;
const unsigned int maxSpeed = 3;
int printCount = 0;
float readPosition;

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
	
	pinMode(context, 0, tuningPin, INPUT);

	return true;
	
}

void render(BelaContext *context, void *userData)
{
	float filterFrequencyVal;
	float filterQVal;
	float volumeVal = 1.0f;
	float speedVal ;
	int tuningValue;

	//Analog readings
	for (int i = 0; i < context->analogFrames; i++) {
		filterFrequencyVal = map(analogRead(context, i, filterFrequencyPin), 0, 1, 100, 1000);
		filterQVal = map(analogRead(context, i, filterQPin), 0, 1, 0.5, 10);
		volumeVal = analogRead(context, i, volumePin);
		speedVal = map(analogRead(context,i,speedPin),0,1,minSpeed,maxSpeed);
		tuningValue = map(analogRead(context,i,tuningPin),0,1,(int)minScale,(int)maxScale); 
		 
		

		distanceSensorReadVolume(context, i);

		calculate_coefficients(context->audioSampleRate, filterFrequencyVal, filterQVal);
	}

	printCount++;
	if (printCount >= 4410) { // every 100ms at 4410 hz
		rt_printf("Flex: touches=%d loc=%f | Ring: touches=%d loc=%f | Volume=%.2f\n",
			gNumActiveTouchesFlex, gTouchLocationCycleFlex, gNumActiveTouches, gTouchLocationCycle, volumeVal);
		printCount = 0;
	}

	// Defining values for the cropped sample
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

		//Pre-vizualisation of the to be cropped file
		float in;

		if (gNumActiveTouchesFlex > 0) {

			if (finalReadPointer < previewStart || finalReadPointer > previewEnd) {
				finalReadPointer = previewStart;
				
			}
			in = finalSample[finalReadPointer];

			readPosition+=speedVal;
			
			if (readPosition > previewEnd) {
				readPosition = previewStart;
			}
			finalReadPointer = (int)readPosition;

		} else {
			// Reading of the last sample with no input on flex Trill
			readPosition+=speedVal;
			finalReadPointer = (int)readPosition;
			if (readPosition >= (int)finalSample.size()) readPosition = 0;

			in = finalSample[finalReadPointer];
			
			if(readPosition < 0) readPosition =0;
		}

		float out = gB0 * in + gB1 * previousInput + gB2 * previousInput2 - gA1 * previousOutput - gA2 * previousOutput2;

		previousInput2 = previousInput;
		previousInput = in;
		previousOutput2 = previousOutput;
		previousOutput = out;

		//Using ring Trill for browsing and scratching through the sample
		if (gNumActiveTouches > 0) {
			if (gNumActiveTouchesFlex > 0) {
				//Scratch in preview
				finalReadPointer = previewStart + (int)map(gTouchLocationCycle, 0, 1, 0, previewEnd - previewStart + 1);
				if (finalReadPointer > previewEnd) finalReadPointer = previewEnd;
			} else {
				// Scratch in final sample
				finalReadPointer = (int)map(gTouchLocationCycle, 0, 1, 0, finalSample.size());
				if (finalReadPointer >= (int)finalSample.size()) finalReadPointer = finalSample.size() - 1;
			}
		}

		float droneSample = distanceSensorProcessSample(context, i); // une seule fois par echantillon (pas par canal)

		for (int c = 0; c < context->audioOutChannels; c++) audioWrite(context, i, c, out * volumeVal + droneSample);
	}
}

void cleanup(BelaContext *context, void *userData)
{
	distanceSensorCleanup();
}