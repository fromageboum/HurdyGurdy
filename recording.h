#pragma once
#include <Bela.h>
#include <vector>
#include <string>

// Buffer circulaire de capture continue du micro (10 secondes)
extern std::vector<float> DataFlow;
extern int dataFlowlenght;
extern int writingPointer;

// Buffer reordonne chronologiquement, produit au moment de l'appui du bouton record
extern std::vector<float> orderedBuffer;

extern std::string gFilename;
extern AuxiliaryTask writeFileTask;
extern const unsigned int recButtonPin;

// Prepare le module (pin, tailles des buffers, calcul de dataFlowlenght)
void recordingSetup(BelaContext *context);

// A appeler une fois par echantillon audio, dans render() : capture le micro
// dans DataFlow, lit le bouton record, gere l'anti-rebond, et declenche
// writeFileTask sur une pression valide
void recordingProcessSample(BelaContext *context, int n);

// Tache auxiliaire : reordonne DataFlow en orderedBuffer, l'exporte en .wav,
// et reinitialise finalSample (dans cropping.cpp) a partir du nouvel enregistrement
void writeToFile(void*);