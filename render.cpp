#include <Bela.h>
#include <libraries/AudioFile/AudioFile.h>
#include <libraries/Gui/Gui.h>
#include <cmath>
#include <libraries/Trill/Trill.h>


AudioFileWriter recorder;

// Buffer used to generate .wav file
std::vector<float> DataFlow;
int dataFlowlenght;

int printCount;

// Recorded buffer (reordered chronologically after button press)
std::vector<float> orderedBuffer;

// Cropped buffer, plays in a loop on the output
std::vector<float> finalSample;
int finalReadPointer = 0;

bool taskStatus = false;
bool previousTaskStatus = false;

// Writing/reading head for sound file
int writingPointer;
int readPointer;

// Constants to declare number pins for Hardware connexion
const unsigned int recButtonPin = 0;
const unsigned int cropButtonPin = 1;
const unsigned int filterFrequencyPin = 0;
const unsigned int filterQPin = 1;
const unsigned int volumePin = 2;

// Record button state
bool recButtonState = true;
bool previousButtonState;

// String to call and play sample file
std::string gFilename = "rawSample.wav";

// Declare auxiliary tasks
AuxiliaryTask writeFileTask;
AuxiliaryTask trillReadTask;
AuxiliaryTask cropTask;

// Debouncing record button
bool stableButtonState = true;
bool previousStableState = true;
bool candidateState = true;
int candidateCounter = 0;
int debounceSamples; // seuil en nombre d'echantillons (calcule dans setup())

// Debouncing crop button (independent from record button)
bool cropCandidateState = true;
int cropCandidateCounter = 0;
bool cropStableState = true;
bool previousCropStableState = true;

// Filter coefficients
float gA1 = 0, gA2 = 0;
float gB0 = 1, gB1 = 0, gB2 = 0;

// Filter memory
float previousInput = 0, previousOutput = 0, previousInput2 = 0, previousOutput2 = 0;

// Trill Ring sensor
Trill touchSensor;
// Trill Flex sensor
Trill FLEXTouchSensor;

// Location of touch on Trill Ring
float gTouchLocationCycle = 0;
float gTouchSizeCycle = 0;

#define NUM_TOUCH_RING 5
#define NUM_TOUCH_FLEX 4

float gTouchLocation[NUM_TOUCH_RING] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
float gTouchSize[NUM_TOUCH_RING] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
unsigned int gNumActiveTouches = 0;

// Trill Flex variables
float gTouchLocationCycleFlex = 0;
float gTouchSizeCycleFlex = 0;

float gTouchLocationFlex[NUM_TOUCH_FLEX] = {0.0, 0.0, 0.0, 0.0};
float gTouchSizeFlex[NUM_TOUCH_FLEX] = {0.0, 0.0, 0.0, 0.0};
int gNumActiveTouchesFlex = 0;

// Provisional crop bounds (0-1), updated live from Trill Flex
float provisionnalBeginCrop = 0;
float provisionnalEndingCrop = 1;

float gsampleRate;

void writeToFile(void*)
{
	int firstSegmentLength = DataFlow.size() - writingPointer;

	for (int f = 0; f < firstSegmentLength; f++) {
		orderedBuffer[f] = DataFlow[writingPointer + f];
	}
	for (int f = 0; f < writingPointer; f++) {
		orderedBuffer[firstSegmentLength + f] = DataFlow[f];
	}

	// TEST DE DIAGNOSTIC : trouver la valeur max absolue dans le buffer
	float maxVal = 0;
	for (int f = 0; f < dataFlowlenght; f++) {
		if (fabsf(orderedBuffer[f]) > maxVal) {
			maxVal = fabsf(orderedBuffer[f]);
		}
	}
	rt_printf("Valeur max absolue dans le buffer : %f\n", maxVal);

	AudioFileUtilities::write(gFilename, orderedBuffer.data(), 1, dataFlowlenght, (unsigned int)gsampleRate);
	rt_printf("Fichier .wav ecrit avec succes\n");

	taskStatus = true;

	// Nouvel enregistrement = reset du crop, tout le sample redevient disponible
	finalSample = orderedBuffer;
	finalReadPointer = 0;
}

void trillRead(void*)
{
	while (!Bela_stopRequested()) {

		touchSensor.readI2C();
		FLEXTouchSensor.readI2C();

		gNumActiveTouches = touchSensor.getNumTouches();
		gNumActiveTouchesFlex = FLEXTouchSensor.getNumTouches();

		for (unsigned int i = 0; i < gNumActiveTouches; i++) {
			gTouchLocation[i] = touchSensor.touchLocation(i);
			gTouchSize[i] = touchSensor.touchSize(i);
		}
		for (unsigned int i = gNumActiveTouches; i < NUM_TOUCH_RING; i++) {
			gTouchLocation[i] = 0.0;
			gTouchSize[i] = 0.0;
		}

		for (unsigned int i = 0; i < (unsigned int)gNumActiveTouchesFlex; i++) {
			gTouchLocationFlex[i] = FLEXTouchSensor.touchLocation(i);
			gTouchSizeFlex[i] = FLEXTouchSensor.touchSize(i);
		}
		for (unsigned int i = (unsigned int)gNumActiveTouchesFlex; i < NUM_TOUCH_FLEX; i++) {
			gTouchLocationFlex[i] = 0.0;
			gTouchSizeFlex[i] = 0.0;
		}

		// Assignation APRES mise a jour des tableaux (valeurs fraiches, pas du tour precedent)
		gTouchLocationCycle = gTouchLocation[0];
		gTouchLocationCycleFlex = gTouchLocationFlex[0];

		// Defining provisional crop bounds from Trill Flex touches
		if (gNumActiveTouchesFlex == 0) {
			provisionnalBeginCrop = 0;
			provisionnalEndingCrop = 1;
		}
		else if (gNumActiveTouchesFlex == 2) {
			if (FLEXTouchSensor.touchLocation(0) < FLEXTouchSensor.touchLocation(1)) {
				provisionnalBeginCrop = FLEXTouchSensor.touchLocation(0);
				provisionnalEndingCrop = FLEXTouchSensor.touchLocation(1);
			} else {
				provisionnalBeginCrop = FLEXTouchSensor.touchLocation(1);
				provisionnalEndingCrop = FLEXTouchSensor.touchLocation(0);
			}
		}
		else {
			provisionnalBeginCrop = FLEXTouchSensor.touchLocation(0);
			provisionnalEndingCrop = 1;
		}

		usleep(12000); // pause avant la prochaine lecture
	}
}

// Calculate the filter coefficients based on the given parameters
// Borrows code from the Bela Biquad library, itself based on code by
// Nigel Redmon
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

	finalSample = cropped;
	finalReadPointer = 0;

	rt_printf("finalSample croppe, nouvelle taille : %d\n", newLength);
}

bool setup(BelaContext *context, void *userData)
{
	// Configure buttons pins
	pinMode(context, 0, recButtonPin, INPUT);
	pinMode(context, 0, cropButtonPin, INPUT);

	// Max size of the buffer (10 secondes)
	dataFlowlenght = context->audioSampleRate * 10;

	// Set size of the buffers
	DataFlow.resize(dataFlowlenght);
	orderedBuffer.resize(dataFlowlenght);
	finalSample.resize(dataFlowlenght); // depart = taille complete

	// Initialize auxiliary tasks
	writeFileTask = Bela_createAuxiliaryTask(&writeToFile, 50, "write-file-task");
	cropTask = Bela_createAuxiliaryTask(&cropBuffer, 50, "crop-task");

	// Stock the audioSampleRate on a global level
	gsampleRate = context->audioSampleRate;

	debounceSamples = (int)(0.02 * context->audioSampleRate); // 20ms

	// Calculate initial filter coefficients
	calculate_coefficients(context->audioSampleRate, 1000, 0.707);

	// Initialize Trill sensors
	if (touchSensor.setup(1, Trill::RING) != 0) {
		fprintf(stderr, "Unable to initialise Trill Ring\n");
		return false;
	}

	if (FLEXTouchSensor.setup(1, Trill::FLEX) != 0) {
		fprintf(stderr, "Unable to initialise Trill Flex\n");
		return false;
	}

	trillReadTask = Bela_runAuxiliaryTask(&trillRead, 50);

	return true;
}

void render(BelaContext *context, void *userData)
{
	float filterFrequencyVal;
	float filterQVal;
	float volumeVal = 1.0f;

	for (int i = 0; i < context->analogFrames; i++) {
		// Reading values from potentiometers
		filterFrequencyVal = map(analogRead(context, i, filterFrequencyPin), 0, 1, 100, 1000);
		filterQVal = map(analogRead(context, i, filterQPin), 0, 1, 0.5, 10);
		volumeVal = analogRead(context, i, volumePin); // deja entre 0 et 1, ideal pour un facteur de volume

		calculate_coefficients(context->audioSampleRate, filterFrequencyVal, filterQVal);
	}

	printCount++;
	if (printCount >= 4410) { // environ toutes les 100ms a 44100 Hz
		rt_printf("Flex: touches=%d loc=%f | Ring: touches=%d loc=%f | Volume=%.2f\n",
			gNumActiveTouchesFlex, gTouchLocationCycleFlex, gNumActiveTouches, gTouchLocationCycle, volumeVal);
		printCount = 0;
	}

	// Calcul des bornes de previsualisation (une fois par bloc, pas par echantillon)
	// Basees sur finalSample pour permettre un crop successif
	int previewStart = (int)map(provisionnalBeginCrop, 0, 1, 0, finalSample.size());
	int previewEnd = (int)map(provisionnalEndingCrop, 0, 1, 0, finalSample.size());
	if (previewStart >= (int)finalSample.size()) previewStart = finalSample.size() - 1;
	if (previewEnd >= (int)finalSample.size()) previewEnd = finalSample.size() - 1;
	if (previewStart < 0) previewStart = 0;

	// Recording dynamically from mic and storing it in the buffer
	for (int i = 0; i < context->audioFrames; i++) {
		// Write every audio input in DataFlow
		DataFlow[writingPointer] = audioRead(context, i, 0);
		writingPointer++;
		if (writingPointer >= (int)DataFlow.size()) writingPointer = 0;

		// --- Debounce record button (independent) ---
		bool rawState = digitalRead(context, i, recButtonPin);
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

		// --- Debounce crop button (independent) ---
		bool cropRawState = digitalRead(context, i, cropButtonPin);
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

		// --- Filter processing, avec previsualisation dynamique du crop ---
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

		// --- Ring scratch control, adapte au mode actif (previsualisation ou normal) ---
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