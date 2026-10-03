#ifndef COMUNICACIONES_H
#define COMUNICACIONES_H

enum EstadoDron { APAGADO, DESPEGANDO, VOLANDO, ATERRIZANDO };

extern EstadoDron estadoActual;

void initComunicaciones();
void enviarMensajeUDP(const char* mensaje);
void recibirComandosUDP();

// Nota: usamos uint8_t* para enviar bytes puros y size_t para el tamaño
void enviarTelemetriaBinaria(const uint8_t* datos, size_t longitud);
void tareaTelemetria(void *pvParameters);

// Le decimos al compilador que empaquete esto byte por byte sin espacios vacíos
struct __attribute__((packed)) TelemetriaDron {
  uint32_t timestamp; // Útil para saber si se perdieron paquetes (micros o millis)
  
  // Crudos (3 floats = 12 bytes)
  float accX; float accY; float accZ;
  
  // Roll (4 floats = 16 bytes)
  float rollAcc; float rollGyr; float rollKalman; float rollRateKalman;
  
  // Pitch (4 floats = 16 bytes)
  float pitchAcc; float pitchGyr; float pitchKalman; float pitchRateKalman;
  
  // Yaw (2 floats = 8 bytes)
  float yawRateGyr; float yawRateKalman;
  
  // Altura (3 floats = 12 bytes)
  float altToF; float altKalman; float vzKalman;

  // Batería (1 float = 4 bytes)
  float vBat; 
  //Temperatura
  float temp;
}; // TOTAL NUEVO: 76 bytes

#endif
