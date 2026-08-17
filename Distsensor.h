#pragma once
#include <Bela.h>

// Initialise le capteur VL53L1X (bus I2C 1, adresse 0x29), l'oscillateur
// associe, et le bouton d'activation/desactivation (pin digitale dediee).
// Retourne false si le capteur ne repond pas (ex: cable non branche
// via le Trill Hub).
bool distanceSensorSetup(BelaContext *context);

// A appeler une fois par echantillon audio, dans render() : lit le bouton
// toggle (avec anti-rebond) et renvoie l'echantillon courant de l'oscillateur
// (drone) si la fonctionnalite est activee, sinon 0.
float distanceSensorProcessSample(BelaContext *context, int n);

// Ferme proprement la connexion I2C au capteur (a appeler dans cleanup())
void distanceSensorCleanup();

// Derniere distance mesuree (mm), utile pour du diagnostic/print ailleurs.
// -1 tant qu'aucune mesure valide n'est arrivee.
extern volatile int gLatestDistanceMM;

// Etat actuel (active/desactive), utile pour du diagnostic/print ailleurs.
extern bool gDistanceSensorEnabled;