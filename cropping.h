#pragma once
#include <Bela.h>
#include <vector>

// Buffer croppe, joue en boucle en sortie (rempli par recording.cpp au moment
// de l'enregistrement, puis reduit successivement par cropBuffer())
extern std::vector<float> finalSample;
extern int finalReadPointer;

// Bornes provisoires du crop (0-1), mises a jour en direct par trillSensors.cpp
// a partir du Trill Flex
extern float provisionnalBeginCrop;
extern float provisionnalEndingCrop;

// Bouton de validation du crop
extern const unsigned int cropButtonPin;
extern AuxiliaryTask cropTask;

// Prepare le module (pin, taille par defaut de finalSample)
void croppingSetup(BelaContext *context, int dataFlowlenght);

// A appeler une fois par echantillon audio, dans render() : lit le bouton crop,
// gere l'anti-rebond, et declenche cropTask() sur une pression valide
void croppingProcessSample(BelaContext *context, int n);

// Tache auxiliaire : reduit reellement finalSample selon les bornes provisoires
void cropBuffer(void*);