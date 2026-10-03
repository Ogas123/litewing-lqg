#include <Arduino.h>
#include "Config.h"
#include "IMU.h"
#include "ToF.h"
#include "Kalman.h"
#include "LQR.h"
#include "Motores.h"
#include "Telemetria.h"
#include "Supervisor.h"

// ==========================================================
// VARIABLES GLOBALES DE RTOS Y HARDWARE TIMER
// ==========================================================
// Puntero al temporizador de hardware del ESP32 que dictará el ritmo de 250 Hz
hw_timer_t * controlTimer = NULL;
// Semáforo binario para sincronizar la Interrupción (ISR) con el Loop principal
SemaphoreHandle_t timerSemaphore;

// Importamos sensores para el Lazo de Control (Core 1)
extern float AccX, AccY, AccZ;
extern float AngleRoll_Acc, AnglePitch_Acc;
extern float RateRoll, RatePitch, RateYaw;
extern float dist_tof_m; 

// ==========================================================
// RUTINA DE INTERRUPCIÓN (ISR) - HARDWARE TIMER (250 Hz)
// ==========================================================
// IRAM_ATTR asegura que esta función se cargue en la RAM interna del ESP32 
// y no en la memoria Flash, garantizando una ejecución ultrarrápida y evitando crashes.
void IRAM_ATTR onTimer() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
  // Liberamos (Give) el semáforo para avisarle al 'loop' que ya pasaron los 4ms
  xSemaphoreGiveFromISR(timerSemaphore, &xHigherPriorityTaskWoken);
  
  // Si liberar el semáforo despertó una tarea de mayor prioridad, forzamos un cambio de contexto
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR(); 
  }
}

// ==========================================================
// SETUP
// ==========================================================
void setup() {
  // Aumentamos los baudios a 500k para no saturar el buffer serial con la data a alta frecuencia
  Serial.begin(500000); 

  // 1. Inicialización de todos los periféricos y módulos de software
  initIMU();
  initToF();
  initControl();
  initMotores();
  initComunicaciones();

  // 2. Creación de la Tarea de Telemetría (Wi-Fi/UDP) en el Core 0
  // Al aislar la telemetría en el núcleo 0, evitamos que los retardos de red
  // interfieran con el lazo de control crítico que correrá en el núcleo 1.
  xTaskCreatePinnedToCore(
    tareaTelemetria,   // Puntero a la función que ejecuta la tarea
    "TaskTelemetria",  // Etiqueta de texto para debugging de FreeRTOS
    4096,              // Tamaño de memoria RAM asignada al Stack (en bytes)
    NULL,              // Parámetros a pasar a la función (ninguno en este caso)
    1,                 // Prioridad (1 es baja, el control manda)
    NULL,              // Handle (puntero) de la tarea, no lo necesitamos
    0                  // Asignación estricta al Núcleo 0 (Pro CPU)
  );

  // 3. Configuración del Sincronizador (Semáforo) y Timer de Control
  timerSemaphore = xSemaphoreCreateBinary();
  
  // timerBegin: Configuramos la frecuencia base del timer a 1 MHz
  // Esto significa que 1 "tick" del temporizador equivale exactamente a 1 microsegundo.
  controlTimer = timerBegin(1000000); 
  
  // Atamos nuestra función ISR (onTimer) al temporizador de hardware configurado
  timerAttachInterrupt(controlTimer, &onTimer);
  
  // timerAlarm: Disparamos la alarma cada 4000 ticks (4000 us = 4 ms = 250 Hz)
  // Parámetros: (timer, valor alarma, auto-reload true, contador de recargas)
  timerAlarm(controlTimer, 4000, true, 0); 
  
  Serial.println("SISTEMA ARMADO. LQG EN CORE 1 (250Hz) / TELEMETRÍA EN CORE 0.");
}

// ==========================================================
// LOOP PRINCIPAL (CORE 1) - CONTROL LQG ESTRICTO
// ==========================================================
void loop() {
  // El loop se "bloquea" aquí (sin consumir CPU) esperando a que la ISR libere el semáforo.
  // Esto garantiza que el lazo se ejecute EXACTAMENTE a 250 Hz sin el temblor de la función delay().
  if (xSemaphoreTake(timerSemaphore, portMAX_DELAY) == pdTRUE) {
    
    // 1. Leer Sensores
    leerIMU();
    leerToF();

    float y_roll[2]  = {AngleRoll_Acc, RateRoll};
    float y_pitch[2] = {AnglePitch_Acc, RatePitch};
    float y_yaw      = RateYaw;
    
    // 2. Filtro de Kalman Dinámico
    actualizarFiltrosLQG(u_roll, u_pitch, u_yaw, u_alt, 
                         y_roll, y_pitch, y_yaw, 
                         dist_tof_m, AccZ);

    // 3. Ejecutar Máquina de Estados (Supervisor)
    ejecutarSupervisorVuelo();
  }
}