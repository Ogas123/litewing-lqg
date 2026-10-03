#include <Arduino.h>
#include "Kalman.h"
#include "Config.h"

// Traemos las aceleraciones crudas en Body Frame (NED) desde IMU.cpp
extern float AccX, AccY, AccZ;

// ====================================================================
// VARIABLES DE ESTADO Y COVARIANZA
// ====================================================================
float x_hat_roll[2]  = {0.0f, 0.0f};
float P_roll[2][2]   = {{10.0f, 0.0f}, {0.0f, 10.0f}}; 

float x_hat_pitch[2] = {0.0f, 0.0f};
float P_pitch[2][2]  = {{10.0f, 0.0f}, {0.0f, 10.0f}};

float x_hat_alt[2]   = {0.0f, 0.0f};
float P_alt[2][2]    = {{10.0f, 0.0f}, {0.0f, 10.0f}};

float x_hat_yaw[1]   = {0.0f};
float P_yaw[1]       = {10.0f};

// ====================================================================
// FUNCIÓN RECURSIVA: FILTRO DE KALMAN 2x2 (C = Identidad)
// ====================================================================
void updateKalmanRecursive2x2(float x[2], float P[2][2], float u, const float y[2], 
                              const float Gamma[2], const float Q[2][2], const float R[2][2]) {
  
  float x_pred[2];
  float P_pred[2][2];
  float S[2][2];
  float K[2][2];

  // 1. PREDICCIÓN
  x_pred[0] = Phi_2x2[0][0]*x[0] + Phi_2x2[0][1]*x[1] + Gamma[0]*u;
  x_pred[1] = Phi_2x2[1][0]*x[0] + Phi_2x2[1][1]*x[1] + Gamma[1]*u;

  P_pred[0][0] = P[0][0] + h*P[1][0] + h*(P[0][1] + h*P[1][1]) + Q[0][0];
  P_pred[0][1] = P[0][1] + h*P[1][1] + Q[0][1];
  P_pred[1][0] = P[1][0] + h*P[1][1] + Q[1][0];
  P_pred[1][1] = P[1][1] + Q[1][1];

  // 2. GANANCIA DE KALMAN
  S[0][0] = P_pred[0][0] + R[0][0];
  S[0][1] = P_pred[0][1] + R[0][1];
  S[1][0] = P_pred[1][0] + R[1][0];
  S[1][1] = P_pred[1][1] + R[1][1];

  float detS = S[0][0]*S[1][1] - S[0][1]*S[1][0];
  float invS[2][2];
  invS[0][0] =  S[1][1] / detS;
  invS[0][1] = -S[0][1] / detS;
  invS[1][0] = -S[1][0] / detS;
  invS[1][1] =  S[0][0] / detS;

  K[0][0] = P_pred[0][0]*invS[0][0] + P_pred[0][1]*invS[1][0];
  K[0][1] = P_pred[0][0]*invS[0][1] + P_pred[0][1]*invS[1][1];
  K[1][0] = P_pred[1][0]*invS[0][0] + P_pred[1][1]*invS[1][0];
  K[1][1] = P_pred[1][0]*invS[0][1] + P_pred[1][1]*invS[1][1];

  // 3. CORRECCIÓN
  float err_0 = y[0] - x_pred[0];
  float err_1 = y[1] - x_pred[1];

  x[0] = x_pred[0] + K[0][0]*err_0 + K[0][1]*err_1;
  x[1] = x_pred[1] + K[1][0]*err_0 + K[1][1]*err_1;

  P[0][0] = P_pred[0][0] - (K[0][0]*P_pred[0][0] + K[0][1]*P_pred[1][0]);
  P[0][1] = P_pred[0][1] - (K[0][0]*P_pred[0][1] + K[0][1]*P_pred[1][1]);
  P[1][0] = P_pred[1][0] - (K[1][0]*P_pred[0][0] + K[1][1]*P_pred[1][0]);
  P[1][1] = P_pred[1][1] - (K[1][0]*P_pred[0][1] + K[1][1]*P_pred[1][1]);
}

// ====================================================================
// FILTRO DE KALMAN PARA ALTURA (C = [1, 0]) CON TILT COMPENSATION
// ====================================================================
void updateKalmanAltura(float x[2], float P[2][2], float acc_z_ms2, float dist_tof_m, float roll_est_deg, float pitch_est_deg) {
  
  // 1. Convertimos los ángulos estimados a radianes (constante float: sin promoción a double)
  float roll_rad  = roll_est_deg  * DEG2RAD_F;
  float pitch_rad = pitch_est_deg * DEG2RAD_F;

  // Evaluamos los cuatro senos/cosenos UNA sola vez con las variantes 'f' de
  // libm: sinf/cosf operan en la FPU de 32 bits del LX7, mientras que sin/cos
  // fuerzan la emulación de doble precisión por software. Reutilizarlos aquí
  // ahorra además las 4 evaluaciones trigonométricas duplicadas del paso 4.
  const float sin_roll  = sinf(roll_rad);
  const float cos_roll  = cosf(roll_rad);
  const float sin_pitch = sinf(pitch_rad);
  const float cos_pitch = cosf(pitch_rad);

  // 2. TILT COMPENSATION: Rotamos el vector de aceleración 3D al marco de la Tierra (Earth Frame Z-Down)
  // Matriz de Cosenos Directores (DCM) estándar NED:
  // a_z_earth = -AccX * sin(theta) + AccY * sin(phi) * cos(theta) + AccZ * cos(phi) * cos(theta)
  float acc_z_suelo = -AccX * sin_pitch + 
                      AccY * (sin_roll * cos_pitch) + 
                      acc_z_ms2 * (cos_roll * cos_pitch);

  // 3. Aceleración neta libre de gravedad
  float a_net = acc_z_suelo - GRAVEDAD_F;

  float x_pred[2];
  // Predicción cinemática usando a_net ya compensada
  x_pred[0] = x[0] + h * x[1] + 0.5f * h * h * a_net;
  x_pred[1] = x[1] + h * a_net;

  float P_pred[2][2];
  P_pred[0][0] = P[0][0] + h*P[1][0] + h*(P[0][1] + h*P[1][1]) + Q_alt[0][0];
  P_pred[0][1] = P[0][1] + h*P[1][1] + Q_alt[0][1];
  P_pred[1][0] = P[1][0] + h*P[1][1] + Q_alt[1][0];
  P_pred[1][1] = P[1][1] + Q_alt[1][1];

  // 4. Compensación de inclinación en la distancia del ToF (Proyección vertical real al suelo)
  float dist_tof_suelo = dist_tof_m * cos_roll * cos_pitch;

  // 5. CORRECCIÓN CONTINUA A 250 Hz (Evita acumulación de covarianza y saltos tipo serrucho)
  float S = P_pred[0][0] + R_alt_scalar; 
  
  float K[2];
  K[0] = P_pred[0][0] / S;
  K[1] = P_pred[1][0] / S;

  float err = dist_tof_suelo - x_pred[0];

  x[0] = x_pred[0] + K[0] * err;
  x[1] = x_pred[1] + K[1] * err;

  P[0][0] = P_pred[0][0] - K[0] * P_pred[0][0];
  P[0][1] = P_pred[0][1] - K[0] * P_pred[0][1];
  P[1][0] = P_pred[1][0] - K[1] * P_pred[0][0];
  P[1][1] = P_pred[1][1] - K[1] * P_pred[0][1];
}

// ====================================================================
// BUCLE PRINCIPAL DE ESTIMACIÓN (LQG)
// ====================================================================
void actualizarFiltrosLQG(float u_roll, float u_pitch, float u_yaw, float u_alt,
                          float y_roll[2], float y_pitch[2], float y_yaw, 
                          float dist_tof_m, float acc_z_ms2) { 
  
  // Buffers locales para que el control no lea variables a medias
  float temp_roll[2]  = {x_hat_roll[0], x_hat_roll[1]};
  float temp_pitch[2] = {x_hat_pitch[0], x_hat_pitch[1]};
  float temp_yaw[1]   = {x_hat_yaw[0]};
  float temp_alt[2]   = {x_hat_alt[0], x_hat_alt[1]};

  // 1. Filtros de Actitud
  updateKalmanRecursive2x2(temp_roll, P_roll, u_roll, y_roll, Gamma_roll_pitch, Q_roll_pitch, R_roll_pitch);
  updateKalmanRecursive2x2(temp_pitch, P_pitch, u_pitch, y_pitch, Gamma_roll_pitch, Q_roll_pitch, R_roll_pitch);
  
  // 2. Filtro de Yaw
  float x_pred_yaw = temp_yaw[0] + Gamma_yaw * u_yaw; 
  float P_pred_yaw = P_yaw[0] + Q_yaw;
  float K_yaw_gain = P_pred_yaw / (P_pred_yaw + R_yaw);
  
  temp_yaw[0] = x_pred_yaw + K_yaw_gain * (y_yaw - x_pred_yaw);
  P_yaw[0] = (1.0f - K_yaw_gain) * P_pred_yaw;

  // 3. Filtro de Altura (Le pasamos los ángulos de Roll y Pitch recién estimados)
  updateKalmanAltura(temp_alt, P_alt, acc_z_ms2, dist_tof_m, temp_roll[0], temp_pitch[0]);

  // Actualización global final
  x_hat_roll[0]  = temp_roll[0];
  x_hat_roll[1]  = temp_roll[1];
  x_hat_pitch[0] = temp_pitch[0];
  x_hat_pitch[1] = temp_pitch[1];
  x_hat_yaw[0]   = temp_yaw[0];
  x_hat_alt[0]   = temp_alt[0];
  x_hat_alt[1]   = temp_alt[1];
}