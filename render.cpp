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

std::vector<float> orderedBuffer(dataFlowlenght);

bool taskStatus = false;
bool previousTaskStatus = false;

// Writing/reading head for sound file
int writingPointer;
int readPointer;

// Constants to declare number pins for Hardware connexion
const unsigned int recButtonPin = 0;
const unsigned int filterFrequencyPin = 0;
const unsigned int filterQPin = 1;
// Record button state
bool recButtonState = true;
bool previousButtonState;


// String to call and play sample file
std::string gFilename = "rawSample.wav";

// Declare an auxiliary task representing the writing task
AuxiliaryTask writeFileTask;

//Trill read auxialiary task
AuxiliaryTask trillReadTask; // variable globale, comme writeFileTask

// Debouncing button inputs
bool stableButtonState = true;   // l'état "validé" du bouton, après anti-rebond
bool previousStableState = true; // l'état stable précédent (pour détecter la transition)
bool candidateState = true;      // le dernier état brut lu, en cours de vérification
int candidateCounter = 0;        // depuis combien de samples cet état brut est stable
int debounceSamples;              // seuil en nombre d'échantillons (calculé dans setup())

// Additional variables for filter coeficients
float gA1 = 0, gA2 = 0;
float gB0 = 1, gB1 = 0, gB2 = 0;

// Variables to rememeber the previous states of the output
float previousInput = 0, previousOutput = 0, previousInput2 = 0, previousOutput2 = 0;

// Declaration for Ring Trill sensor 
Trill touchSensor; // Trill object declaration

// Location of touch on Trill Ring
float gTouchLocationCycle = 0;
// Size of touch on Trill Ring
float gTouchSizeCycle = 0;

// Location of touches on Trill Ring
#define NUM_TOUCH 5

float gTouchLocation[NUM_TOUCH] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
// Size of touches on Trill Ring
float gTouchSize[NUM_TOUCH] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
// Number of active touches
unsigned int gNumActiveTouches = 0;


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
}

void trillRead (void*)
{
	while (!Bela_stopRequested()) {
		
		touchSensor.readI2C();
		gTouchLocationCycle = gTouchLocation[0];
		gNumActiveTouches = touchSensor.getNumTouches();

		for (unsigned int i = 0; i < gNumActiveTouches; i++) {
			gTouchLocation[i] = touchSensor.touchLocation(i);
			gTouchSize[i] = touchSensor.touchSize(i);
		}
		for (unsigned int i = gNumActiveTouches; i < NUM_TOUCH; i++) {
			gTouchLocation[i] = 0.0;
			gTouchSize[i] = 0.0;
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

/*void applyFilters(sampledBuffer){


}*/

bool setup(BelaContext *context, void *userData)
{
// Configure record button pin
//
pinMode(context, 0, recButtonPin, INPUT);

// Max size of the buffer
dataFlowlenght = context->audioSampleRate*10;

// Set size of the buffer
DataFlow.resize(dataFlowlenght);
orderedBuffer.resize(dataFlowlenght);

// Initialze auxiliary task
writeFileTask = Bela_createAuxiliaryTask(&writeToFile, 50, "write-file-task");

// Stock the audioSampleRate on a global level
gsampleRate = context->audioSampleRate;

debounceSamples = (int)(0.02 * context->audioSampleRate); // 20ms

calculate_coefficients(context->audioSampleRate,1000, 0.707);

// Initialize Trill sensor
if(touchSensor.setup(1, Trill::RING) != 0) {
	fprintf(stderr, "Unable to initialise Trill Ring\n");
		return false;
}
	
trillReadTask = Bela_runAuxiliaryTask(&trillRead, 50);




	return true;

}

void render(BelaContext *context, void *userData)
{
	float filterFrequencyVal;
	float filterQVal;

for(int i = 0; i < context->analogFrames; i++){
		//Reading values from both potentiometers
	filterFrequencyVal = map(analogRead(context,i,filterFrequencyPin), 0,1, 100, 1000);
	filterQVal = map(analogRead(context, i,filterQPin),0,1,0.5,10);
	
	calculate_coefficients(context->audioSampleRate, filterFrequencyVal, filterQVal);
}

printCount++;
if (printCount >= 4410) { // environ toutes les 100ms à 44100 Hz
	rt_printf("TouchLocation = %f\n", gTouchLocationCycle);
	printCount = 0;
}
	
	// Recording dynamically from mic and storing it in the buffer
	// Looping on all audioframes
for (int i = 0; i < context->audioFrames; i++){
	// Write every audio input in DataFlow
	DataFlow[writingPointer] = audioRead(context, i, 0);
	writingPointer++;
	if (writingPointer >= DataFlow.size()) writingPointer = 0;
	
	// Debounce button input
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

	// Valid buton input, running auxiliary task !
		if (stableButtonState == false && previousStableState == true) {
			rt_printf("Bouton presse (valide)\n");
			Bela_scheduleAuxiliaryTask(writeFileTask);
		}
	}
        
        float in = orderedBuffer[readPointer];
        float out = gB0*in + gB1*previousInput + gB2*previousInput2 - gA1*previousOutput - gA2*previousOutput2;
        
        previousInput2 = previousInput;
        previousInput = in;
        
        previousOutput2 = previousOutput;
        previousOutput = out;
        

        readPointer++;
        if (readPointer >= orderedBuffer.size()) readPointer = 0;
        
	if (orderedBuffer.size() != 0 && gNumActiveTouches > 0) {
		readPointer = (int)map(gTouchLocationCycle, 0, 1, 0, orderedBuffer.size());
		if (readPointer >= orderedBuffer.size()) readPointer = orderedBuffer.size() - 1;
}


    	for(int c = 0; c < context-> audioOutChannels; c++) audioWrite(context, i, c, out);
}


}

void cleanup(BelaContext *context, void *userData)
{
	
}