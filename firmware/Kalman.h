#ifndef KALMAN_H
#define KALMAN_H

// Variables externas para que el main las vea
extern float x_hat_roll[2];
extern float x_hat_pitch[2];
extern float x_hat_yaw[1];
extern float x_hat_alt[2];

void updateKalmanRecursive2x2(float x[2], float P[2][2], float u, const float y[2], 
                              const float Gamma[2], const float Q[2][2], const float R[2][2]);

// Nueva función exclusiva para la Altura (C = [1, 0])
void updateKalmanAltura(float x[2], float P[2][2], float acc_z_g, float dist_tof_m, float roll_est_deg, float pitch_est_deg);

// Firma actualizada
void actualizarFiltrosLQG(float u_roll, float u_pitch, float u_yaw, float u_alt,
                          float y_roll[2], float y_pitch[2], float y_yaw, 
                          float dist_tof_m, float acc_z_g);

#endif