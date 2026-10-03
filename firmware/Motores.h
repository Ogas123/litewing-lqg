#ifndef MOTORES_H
#define MOTORES_H

#include <Arduino.h>

// Funciones principales
void initMotores();
void actualizarMotores(bool armado, int throttleBase, float controlRoll, float controlPitch, float controlYaw);
void apagarMotores();

// Función para actualizar la compensación por batería
void actualizarBateria();

#endif