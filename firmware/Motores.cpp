#include <Arduino.h>
#include "Config.h"
#include "Motores.h"

// ====================================================================
// CONFIGURACIÓN DE BATERÍA 
// ====================================================================
float FactorCompensacionBateria = 1.0f;
float VoltajeBateriaReal = 4.2f;

const float FACTOR_ADC_A_VOLTIOS = 0.001688f; 
const float VOLTAJE_NOMINAL_HOVER = 3.7f; 

void actualizarBateria() {
  float v_crudo = analogRead(PIN_BATERIA) * FACTOR_ADC_A_VOLTIOS;
  VoltajeBateriaReal = (0.95f * VoltajeBateriaReal) + (0.05f * v_crudo);
  float v_seguro = constrain(VoltajeBateriaReal, 3.0f, 4.3f);
  FactorCompensacionBateria = VOLTAJE_NOMINAL_HOVER / v_seguro;
}

// ====================================================================
// DISPOSICIÓN FÍSICA Y MEZCLADOR DE MOTORES
// ====================================================================
void initMotores() {
  ledcAttach(PIN_MOTOR_1, 1000, 12);
  ledcAttach(PIN_MOTOR_2, 1000, 12);
  ledcAttach(PIN_MOTOR_3, 1000, 12);
  ledcAttach(PIN_MOTOR_4, 1000, 12);
  
  apagarMotores(); 
  pinMode(PIN_BATERIA, INPUT);
  VoltajeBateriaReal = analogRead(PIN_BATERIA) * FACTOR_ADC_A_VOLTIOS; 
}

void actualizarMotores(bool armado, int throttleBase, float controlRoll, float controlPitch, float controlYaw) {
  
  if (!armado || throttleBase <= 0) {
    apagarMotores();
    return; 
  }

  // 1. EL MIXER BRUTO (Estándar Aeronáutico NED)
  float m1_raw = (float)throttleBase - controlRoll + controlPitch + controlYaw;
  float m2_raw = (float)throttleBase - controlRoll - controlPitch - controlYaw;
  float m3_raw = (float)throttleBase + controlRoll - controlPitch + controlYaw;
  float m4_raw = (float)throttleBase + controlRoll + controlPitch - controlYaw;

  // 2. COMPENSACIÓN POR BATERÍA
  m1_raw *= FactorCompensacionBateria;
  m2_raw *= FactorCompensacionBateria;
  m3_raw *= FactorCompensacionBateria;
  m4_raw *= FactorCompensacionBateria;

  // 3. DESATURACIÓN SUPERIOR PRIORITARIA
  float max_motor = max(max(m1_raw, m2_raw), max(m3_raw, m4_raw));
  if (max_motor > 4095.0f) {
    float exceso = max_motor - 4095.0f;
    m1_raw -= exceso;
    m2_raw -= exceso;
    m3_raw -= exceso;
    m4_raw -= exceso;
  }

  // Casteamos el resultado final a enteros
  int pwmMotor1 = (int)m1_raw;
  int pwmMotor2 = (int)m2_raw;
  int pwmMotor3 = (int)m3_raw;
  int pwmMotor4 = (int)m4_raw;

  // 4. SATURACIÓN INFERIOR (Corte seguro)
  // Como bajamos los 4 motores, alguno podría quedar por debajo de 0.
  if(pwmMotor1 < 0) pwmMotor1 = 0; else if(pwmMotor1 > 4095) pwmMotor1 = 4095;
  if(pwmMotor2 < 0) pwmMotor2 = 0; else if(pwmMotor2 > 4095) pwmMotor2 = 4095;
  if(pwmMotor3 < 0) pwmMotor3 = 0; else if(pwmMotor3 > 4095) pwmMotor3 = 4095;
  if(pwmMotor4 < 0) pwmMotor4 = 0; else if(pwmMotor4 > 4095) pwmMotor4 = 4095;

  // 5. ESCRITURA EN HARDWARE
  ledcWrite(PIN_MOTOR_1, pwmMotor1); 
  ledcWrite(PIN_MOTOR_2, pwmMotor2); 
  ledcWrite(PIN_MOTOR_3, pwmMotor3); 
  ledcWrite(PIN_MOTOR_4, pwmMotor4); 
}

void apagarMotores() {
  ledcWrite(PIN_MOTOR_1, 0);
  ledcWrite(PIN_MOTOR_2, 0);
  ledcWrite(PIN_MOTOR_3, 0);
  ledcWrite(PIN_MOTOR_4, 0);
}