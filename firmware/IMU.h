#ifndef IMU_H
#define IMU_H

// Variables en crudo de la IMU
extern float RateRoll, RatePitch, RateYaw;
extern float AccX, AccY, AccZ;

// Ángulos brutos del acelerómetro (opcionales para graficar)
extern float AngleRoll_Acc;
extern float AnglePitch_Acc;

extern float Temp;

void initIMU();
void leerIMU();

#endif