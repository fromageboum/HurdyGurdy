#pragma once
#include <Bela.h>
#include <libraries/Trill/Trill.h>

#define NUM_TOUCH_RING 5
#define NUM_TOUCH_FLEX 4

// Trill Ring : position/nombre de touchers, pour le scratch
extern float gTouchLocation[NUM_TOUCH_RING];
extern float gTouchSize[NUM_TOUCH_RING];
extern float gTouchLocationCycle;
extern unsigned int gNumActiveTouches;

// Trill Flex : position/nombre de touchers, pour le crop
extern float gTouchLocationFlex[NUM_TOUCH_FLEX];
extern float gTouchSizeFlex[NUM_TOUCH_FLEX];
extern float gTouchLocationCycleFlex;
extern int gNumActiveTouchesFlex;

extern AuxiliaryTask trillReadTask;

// Initialise les deux capteurs Trill (bus I2C 1) et lance la tache de lecture continue
// Retourne false si un des deux capteurs ne repond pas
bool trillSensorsSetup();

// Tourne en boucle sur son propre thread : lit les deux capteurs et met a jour
// les bornes provisoires du crop (dans cropping.cpp) a partir du Trill Flex
void trillRead(void*);