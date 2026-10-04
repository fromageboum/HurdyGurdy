#pragma once
#include <Bela.h>

// Initialises the VL53L1X (I2C bus 1, address 0x29), the oscillator, the
// toggle button and the 4 waveform buttons. Returns false if the sensor
// does not answer (e.g. cable not plugged through the Trill Hub).
bool distanceSensorSetup(BelaContext *context);

// Call once per analog frame, inside the analogFrames loop of render()
// (like the other potentiometers): reads the drone volume pot.
void distanceSensorReadVolume(BelaContext *context, int analogFrameIndex);

// Chooses the scale of the 3rd string.
// 0 = no scale (continuous distance -> frequency mapping)
// 1 = C major, 2 = five-EDO, 3 = eight-EDO, 4 = pelog, 5 = centaur
// Does nothing if the scale did not change, so it is safe to call every block.
void distanceSensorSetScale(int scale);

// Call once per audio sample inside render(): reads the buttons (with
// debounce) and returns the current oscillator sample (0 when disabled).
float distanceSensorProcessSample(BelaContext *context, int n);

// Closes the I2C connection (call in cleanup())
void distanceSensorCleanup();

// Last measured distance (mm), -1 until a valid measurement arrives
extern volatile int gLatestDistanceMM;

// Current state (enabled / disabled)
extern bool gDistanceSensorEnabled;