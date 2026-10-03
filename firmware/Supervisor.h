#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <Arduino.h>

// Variables del supervisor de vuelo
extern float baseThrottleDinamico;
extern int THROTTLE_HOVER;

// Prototipo de la función
void ejecutarSupervisorVuelo();

#endif