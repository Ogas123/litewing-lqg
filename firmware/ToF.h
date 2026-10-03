#ifndef TOF_H
#define TOF_H

// Variable global para que el PID de altura la pueda leer
extern float dist_tof_m; 

void initToF();
bool leerToF();

#endif