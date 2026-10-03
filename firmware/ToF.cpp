#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include "Config.h"
#include "ToF.h"

VL53L1X sensorToF;
int distanciaAlturaMM = 0;
float dist_tof_m = 0;

void initToF() {
  // 1. Iniciamos el SEGUNDO bus I2C (Wire1) asignando sus pines específicos
  //Wire1.begin(PIN_TOF_SDA, PIN_TOF_SCL);
  //Wire1.setClock(400000); // Reloj a 400kHz (Fast Mode)

  // 2. Le decimos a la librería de Pololu que NO use el "Wire" principal (el de la IMU), sino que se comunique exclusivamente por "Wire1".
  //sensorToF.setBus(&Wire1);

  sensorToF.setTimeout(500);
  if (!sensorToF.init()) {
    Serial.println("¡Error crítico! No se detecta el VL53L1X");
  } else {
    Serial.println("Sensor ToF VL53L1X inicializado");
  }

  // Configuramos el sensor para vuelo:
  // Modo "Short" llega hasta 1.3 metros (Ideal para interiores, más rápido)
  sensorToF.setDistanceMode(VL53L1X::Short); 
  
  // Timing Budget: 33ms (Equilibrio entre velocidad y precisión)
  sensorToF.setMeasurementTimingBudget(33000); 

  // Arranca a medir continuamente cada 33ms
  sensorToF.startContinuous(33); 
}

bool leerToF() {
  // Pregunta si hay un dato nuevo sin bloquear el procesador a 250 Hz
  if (sensorToF.dataReady()) {
    uint16_t dist = sensorToF.read(false);
    if (sensorToF.ranging_data.range_status == 0 && dist > 10 && dist <= 2000) {
      dist_tof_m = dist / 1000.0f;
      return true;
    }
  }
  return false;
}

