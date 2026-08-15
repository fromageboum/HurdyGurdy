#pragma once

// Constants pins des potentiometres du filtre
extern const unsigned int filterFrequencyPin;
extern const unsigned int filterQPin;

// Coefficients du filtre biquad
extern float gA1, gA2;
extern float gB0, gB1, gB2;

// Memoire du filtre (echantillons precedents)
extern float previousInput, previousOutput, previousInput2, previousOutput2;

// Calcule les coefficients a partir de la frequence de coupure et du facteur Q
// (formule du filtre passe-bas resonnant, Bela Biquad library / Nigel Redmon)
void calculate_coefficients(float sampleRate, float frequency, float q);