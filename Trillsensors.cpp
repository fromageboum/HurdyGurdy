#include "Trillsensors.h"
#include "Cropping.h" // pour provisionnalBeginCrop / provisionnalEndingCrop

Trill touchSensor;      // Trill Ring
Trill FLEXTouchSensor;  // Trill Flex

float gTouchLocation[NUM_TOUCH_RING] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
float gTouchSize[NUM_TOUCH_RING] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
float gTouchLocationCycle = 0;
unsigned int gNumActiveTouches = 0;

float gTouchLocationFlex[NUM_TOUCH_FLEX] = {0.0, 0.0, 0.0, 0.0};
float gTouchSizeFlex[NUM_TOUCH_FLEX] = {0.0, 0.0, 0.0, 0.0};
float gTouchLocationCycleFlex = 0;
int gNumActiveTouchesFlex = 0;

AuxiliaryTask trillReadTask;

bool trillSensorsSetup()
{
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

		// Bornes provisoires de crop, a partir des touchers du Flex
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