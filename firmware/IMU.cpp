#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "IMU.h"

// Variables de estado (Velocidad y Aceleración)
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;

float offsetRoll = 0, offsetPitch = 0, offsetYaw = 0;

// Variables para los ángulos brutos del acelerómetro
float AngleRoll_Acc, AnglePitch_Acc;

float Temp;

int16_t AccXLSB  = 0;
int16_t AccYLSB  = 0;
int16_t AccZLSB  = 0;
int16_t TempLSB  = 0;
int16_t GyroXLSB = 0;
int16_t GyroYLSB = 0;
int16_t GyroZLSB = 0;

void initIMU() {
  pinMode(PIN_LED_BLUE, OUTPUT);
  
  // 1. Iniciar bus I2C en los pines D4(SDA) y D5(SCL) del ESP32-S3
  Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL); 
  Wire.setClock(400000);  // Reloj I2C al máximo (Fast Mode)
  
  // Si el bus I2C se traba, se aborta rápido para no arruinar el Lazo de 4ms (250Hz)
  Wire.setTimeOut(2);  delay(250);   // Tiempo para que la IMU estabilice su energía

  // 2. Despertar a la MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Registro PWR_MGMT_1
  Wire.write(0x00); // 0x00 = Despierto
  Wire.endTransmission();

  // --------------------------------------------------------------
  // 3. Configurar el FILTRO PASA BAJOS (DLPF) - ANTIALIASING
  // --------------------------------------------------------------
  Wire.beginTransmission(0x68);  
  Wire.write(0x1A); // Registro CONFIG
  Wire.write(0x02); // Filtro a ~98 Hz (retraso de 2.8 ms)
  Wire.endTransmission();
  
  // 4. Configurar la escala del Giroscopio
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // Registro GYRO_CONFIG
  Wire.write(0x08); // Escala a ±500 °/s
  Wire.endTransmission();

  // 5. Configurar la escala del Acelerometro
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // Registro ACCEL_CONFIG
  Wire.write(0x10); // Escala a ±8g

  Wire.endTransmission();

  // ----------------------------------------------------
  // 6. CALIBRACION Offset del Giroscopio
  // ----------------------------------------------------
  // Tomo las primeras 2000 muestras y las promedio
  Serial.println("Calibrando giroscopio... NO MOVER");
  
  float sumRoll = 0, sumPitch = 0, sumYaw = 0;

  delay(100);
  for(int i = 0; i < 2000; i++){
    leerIMU();
    sumRoll += RateRoll;
    sumPitch += RatePitch;
    sumYaw += RateYaw;
    delay(1);
  }
  offsetRoll  = sumRoll  / 2000.0f;
  offsetPitch = sumPitch / 2000.0f;
  offsetYaw   = sumYaw   / 2000.0f;

  digitalWrite(PIN_LED_BLUE, HIGH); 
  Serial.println("Calibración completada!");
}

void leerIMU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Registro inicial ACCEL_XOUT_H
  Wire.endTransmission(false); 

  if (Wire.requestFrom(0x68, 14) == 14) { // Pide los 14 bytes de corrido
    AccXLSB  = Wire.read() << 8 | Wire.read();
    AccYLSB  = Wire.read() << 8 | Wire.read();
    AccZLSB  = Wire.read() << 8 | Wire.read();
    TempLSB  = Wire.read() << 8 | Wire.read();
    GyroXLSB = Wire.read() << 8 | Wire.read();
    GyroYLSB = Wire.read() << 8 | Wire.read();
    GyroZLSB = Wire.read() << 8 | Wire.read();
  }

  // CÁLCULO DE TEMPERATURA
  Temp = (TempLSB / 340.0f) + 36.53f;

  // ---------------------------------
  // --- GIROSCOPIO ---
  // ---------------------------------
  // Lecturas brutas en el marco del Sensor (S)
  float GyroX_s = (float)GyroXLSB / 65.5f;
  float GyroY_s = (float)GyroYLSB / 65.5f;
  float GyroZ_s = (float)GyroZLSB / 65.5f;

  // Transformación a Body Frame (NED) corregida según telemetría física:
  // - Roll: Ala derecha abajo  -> RateRoll POSITIVO (-GyroY_s)
  // - Pitch: Nariz arriba       -> RatePitch POSITIVO (-GyroX_s)
  // - Yaw: Giro horario (CW)    -> RateYaw POSITIVO (-GyroZ_s)
  RateRoll  = -GyroY_s - offsetRoll;
  RatePitch = -GyroX_s - offsetPitch;
  RateYaw   = -GyroZ_s - offsetYaw;

  // ---------------------------------
  // --- ACELERÓMETRO ---
  // ---------------------------------
  // 1. Lectura directa en m/s^2
  float AccX_crudo = ((float)AccXLSB / 4096.0f) * GRAVEDAD_F;
  float AccY_crudo = ((float)AccYLSB / 4096.0f) * GRAVEDAD_F;
  float AccZ_crudo = ((float)AccZLSB / 4096.0f) * GRAVEDAD_F;

  // 2. Restar el Offset (Vector 'b' en Marco Sensor)
  float a_x_1 = AccX_crudo - B_X;
  float a_y_1 = AccY_crudo - B_Y;
  float a_z_1 = AccZ_crudo - B_Z;

  // 3. Multiplicar por matriz de Escala (Matriz 'K')
  float a_x_2 = a_x_1 * S_X;
  float a_y_2 = a_y_1 * S_Y;
  float a_z_2 = a_z_1 * S_Z;

  // 4. Multiplicar por la matriz de Desalineación (Marco Sensor)
  float AccX_s = a_x_2;
  float AccY_s = (ALFA_YX * a_x_2) + a_y_2;
  float AccZ_s = (ALFA_ZX * a_x_2) + (ALFA_ZY * a_y_2) + a_z_2;

  // 5. Mapeo a Body Frame (NED) según orientación física real
  AccX = -AccY_s; // Eje X Body (Longitudinal: Nariz) -> Positivo al subir la nariz
  AccY =  AccX_s; // Eje Y Body (Transversal: Ala Derecha) -> Positivo al bajar ala derecha
  AccZ =  AccZ_s; // Eje Z Body (Vertical: Abajo)

  // 6. Cálculo de Ángulos de Euler (Estándar Aeronáutico NED)
  // Variantes 'f' de libm (atan2f/sqrtf) y constante RAD2DEG_F en precisión
  // simple: el LX7 resuelve todo esto en su FPU de 32 bits. Con atan2/sqrt/
  // RAD_TO_DEG la expresión se promueve a double y se emula por software.
  const float accX2 = AccX * AccX;
  const float accY2 = AccY * AccY;
  const float accZ2 = AccZ * AccZ;

  // Pitch (theta): Nariz arriba -> AnglePitch_Acc POSITIVO
  AnglePitch_Acc = atan2f(AccX, sqrtf(accY2 + accZ2)) * RAD2DEG_F;

  // Roll (phi): Ala derecha abajo -> AngleRoll_Acc POSITIVO
  AngleRoll_Acc  = atan2f(AccY, sqrtf(accX2 + accZ2)) * RAD2DEG_F;
}