#pragma once
#include <Bela.h>

// Initialise le capteur VL53L1X (bus I2C 1, adresse 0x29), l'oscillateur
// associe, le bouton d'activation/desactivation, et les 4 boutons de
// selection de forme d'onde. Retourne false si le capteur ne repond pas
// (ex: cable non branche via le Trill Hub).
bool distanceSensorSetup(BelaContext *context);

// A appeler une fois par bloc, dans la boucle sur analogFrames de render()
// (comme pour les autres potentiometres) : lit le potard de volume dedie au
// drone et met a jour son amplitude en interne.
void distanceSensorReadVolume(BelaContext *context, int analogFrameIndex);

// A appeler une fois par echantillon audio, dans render() : lit tous les
// boutons du module (toggle + formes d'onde, avec anti-rebond), et renvoie
// l'echantillon courant de l'oscillateur (drone) si la fonctionnalite est
// activee, sinon 0.
float distanceSensorProcessSample(BelaContext *context, int n);

// Ferme proprement la connexion I2C au capteur (a appeler dans cleanup())
void distanceSensorCleanup();

// Derniere distance mesuree (mm), utile pour du diagnostic/print ailleurs.
// -1 tant qu'aucune mesure valide n'est arrivee.
extern volatile int gLatestDistanceMM;

// Etat actuel (active/desactive), utile pour du diagnostic/print ailleurs.
extern bool gDistanceSensorEnabled;