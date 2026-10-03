#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "Config.h"
#include "Telemetria.h"
#include "Motores.h"

WiFiUDP udp;

// Instanciamos la variable de estado inicializándola en APAGADO
EstadoDron estadoActual = APAGADO;

// ==========================================================
// VARIABLES GLOBALES
// ==========================================================
// IMU Cruda
extern float AccX, AccY, AccZ;
extern float AngleRoll_Acc, AnglePitch_Acc;
extern float RateRoll, RatePitch, RateYaw;

// ToF
extern float dist_tof_m; 

// Kalman
extern float x_hat_roll[2];
extern float x_hat_pitch[2];
extern float x_hat_yaw[1];
extern float x_hat_alt[2];

// Batería
extern float VoltajeBateriaReal;
// Temperatura
extern float Temp;


void initComunicaciones() {
  // 1. Configurar ESP32 como Access Point
  Serial.println("\nIniciando red Wi-Fi SoftAP...");
  WiFi.mode(WIFI_AP);

  // Obligamos al ESP32 a usar la IP, Gateway y Subnet que definimos en Config.h
  WiFi.softAPConfig(DRON_IP, DRON_GATEWAY, DRON_SUBNET);

  // Levantamos la red
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.println("Wi-Fi listo. IP del Dron: " + IP.toString());

  // 2. Iniciar el servidor UDP
  udp.begin(UDP_PORT);
  Serial.println("Escuchando por UDP en el puerto " + String(UDP_PORT));
}

void enviarTelemetriaBinaria(const uint8_t* datos, size_t longitud) {
  udp.beginPacket("255.255.255.255", UDP_PORT);
  udp.write(datos, longitud);
  udp.endPacket();
}

void recibirComandosUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[255];
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = '\0';
    }
    
    String comando = String(incomingPacket);
    comando.trim(); 
    comando.toUpperCase(); 

    Serial.printf("Comando UDP recibido: %s\n", comando.c_str());

    // ---------------------------------------------------------
    // LÓGICA DE TRANSICIONES POR WI-FI
    // ---------------------------------------------------------
    if (comando == "1" && estadoActual == APAGADO) {
      estadoActual = DESPEGANDO; // Dispara la rampa de despegue autónomo
      Serial.println("OK: Iniciando secuencia de DESPEGUE...");
    } 
    else if (comando == "2" && (estadoActual == VOLANDO || estadoActual == DESPEGANDO)) {
      estadoActual = ATERRIZANDO; // Dispara la rampa de descenso autónomo
      Serial.println("OK: Iniciando secuencia de ATERRIZAJE...");
    }
    else if (comando == "0") { 
      estadoActual = APAGADO; // BOTÓN DE PÁNICO VITAL
      actualizarMotores(false, 0, 0, 0, 0); // <-- CORTA EL HARDWARE AQUÍ MISMO
      Serial.println("EMERGENCIA: MOTORES CORTADOS INSTANTÁNEAMENTE");
    }
  }
}


// ==========================================================
// TAREA DE TELEMETRÍA (CORE 0) - ASÍNCRONA
// ==========================================================
void tareaTelemetria(void *pvParameters) {
  TelemetriaDron paquete; // Creamos el objeto del paquete

  for(;;) {
    // 1. Revisar si llegó un comando de armado/desarmado (UDP RX)
    recibirComandosUDP();

    // Leemos el ADC a 50Hz, completamente asilado del lazo de control
    actualizarBateria();

    // Aceleraciones crudas
    //Serial.print("AccX:"); Serial.print(AccX); Serial.print(",");
    //Serial.print("AccY:"); Serial.print(AccY); Serial.print(",");
    //Serial.print("AccZ:"); Serial.print(AccZ); Serial.print(",");

    // Actitud ROLL (Acelerómetro vs Giroscopio vs Estimación Óptima)
    //Serial.print("Roll_acc:"); Serial.print(AngleRoll_Acc); Serial.print(",");
    //Serial.print("Roll_gyr:"); Serial.print(RateRoll); Serial.print(",");
    //Serial.print("Roll_Kalman:"); Serial.print(x_hat_roll[0]); Serial.print(",");

    // Actitud PITCH
    //Serial.print("Pitch_acc:"); Serial.print(AnglePitch_Acc); Serial.print(",");
    //Serial.print("Pitch_gyr:"); Serial.print(RatePitch); Serial.print(",");
    //Serial.print("Pitch_Kalman:"); Serial.print(x_hat_pitch[0]); Serial.print(",");

    // Actitud YAW
    //Serial.print("Yaw_gyr:"); Serial.print(RateYaw); Serial.print(",");
    //Serial.print("Yaw_Kalman:"); Serial.print(x_hat_yaw[0]); Serial.print(",");

    // Altitud
    //Serial.print("Alt_ToF_Raw:"); Serial.print(dist_tof_m); Serial.print(",");
    //Serial.print("Alt_Kalman:"); Serial.print(x_hat_alt[0]);

    //Serial.println();

    // 1. Llenar la estructura con los datos actuales
    paquete.timestamp = micros();
    
    paquete.accX = AccX; 
    paquete.accY = AccY; 
    paquete.accZ = AccZ;

    paquete.rollAcc = AngleRoll_Acc; 
    paquete.rollGyr = RateRoll; 
    paquete.rollKalman = x_hat_roll[0]; 
    paquete.rollRateKalman = x_hat_roll[1];

    paquete.pitchAcc = AnglePitch_Acc; 
    paquete.pitchGyr = RatePitch; 
    paquete.pitchKalman = x_hat_pitch[0]; 
    paquete.pitchRateKalman = x_hat_pitch[1];

    paquete.yawRateGyr = RateYaw; 
    paquete.yawRateKalman = x_hat_yaw[0];

    paquete.altToF = dist_tof_m; 
    paquete.altKalman = x_hat_alt[0]; 
    paquete.vzKalman = x_hat_alt[1];

    paquete.vBat = VoltajeBateriaReal;
    paquete.temp = Temp;

    // 2. Enviar el paquete casteando el struct a puntero de bytes
    enviarTelemetriaBinaria((const uint8_t*)&paquete, sizeof(TelemetriaDron));
    
    // 3. Relajamos la tarea para no saturar el Wi-Fi ni el procesador.
    // 20 ms = 50 Hz de tasa de refresco.
    vTaskDelay(pdMS_TO_TICKS(20)); 
  }
}
