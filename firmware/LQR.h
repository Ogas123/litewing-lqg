#ifndef LQR_H
#define LQR_H

// ====================================================================
// SETPOINTS DEL SISTEMA (Referencias enviadas por el piloto/código)
// ====================================================================
extern float DesiredAngleRoll;   // [grados o radianes, según tu modelo]
extern float DesiredAnglePitch;  // [grados o radianes]
extern float DesiredRateYaw;     // [grados/s o rad/s]
extern float DesiredAltitude;    // [metros o mm, según tu modelo]

// ====================================================================
// SALIDAS DE CONTROL ÓPTIMO (u)
// Valores resultantes de la ley LQR: u = -L * \hat{x}
// Estas variables deben sumarse al PWM base en el Mixer de Motores.
// ====================================================================
extern float u_roll;
extern float u_pitch;
extern float u_yaw;
extern float u_alt;

// ====================================================================
// FUNCIONES PÚBLICAS DEL CONTROLADOR
// ====================================================================

/**
 * @brief Inicializa las salidas de control en 0.
 * Se debe llamar en el setup() del ESP32.
 */
void initControl();

/**
 * @brief Calcula la ley de control de estado estacionario LQR para los 4 canales.
 * Lee internamente los estados estimados (x_hat_*) del Filtro de Kalman.
 * Se debe llamar en el loop() inmediatamente después de actualizar el Kalman.
 */
void calcularControl();

#endif // LQR_H