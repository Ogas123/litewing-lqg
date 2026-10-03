#include "Supervisor.h"
#include "Config.h"
#include "LQR.h"
#include "Motores.h"
#include "Telemetria.h"

// ====================================================================
// CONFIGURACIÓN DE VUELO
// ====================================================================
int   THROTTLE_HOVER      = 1600;  // Empuje de sustentación [PWM]. Ajustar al peso.
float AlturaObjetivoFinal = 0.50f; // [m]
float baseThrottleDinamico = 0.0f;

// Adelanto máximo de la referencia sobre la altura estimada. Limita la
// velocidad de ascenso y descenso. Ajustados en vuelo.
constexpr float ADELANTO_MAX       = 0.17f; // [m] ascenso y descenso
constexpr float ADELANTO_MAX_FLARE = 0.10f; // [m] bajo ALTURA_FLARE

// --- Despegue ---
constexpr float RAMPA_DESPEGUE     = 15.0f; // [PWM/ciclo] rampa a lazo abierto
constexpr float ALTURA_DESPEGUE    = 0.06f; // [m] separación del piso (el ToF apoyado da ~0.037)
constexpr float VEL_ASCENSO        = 0.25f; // [m/s]
constexpr float TOLERANCIA_LLEGADA = 0.03f; // [m]
constexpr float VZ_LLEGADA         = 0.15f; // [m/s]

// --- Aterrizaje ---
constexpr float VEL_DESCENSO   = 0.25f;  // [m/s] descenso normal
constexpr float VEL_FLARE      = 0.10f;  // [m/s] bajo ALTURA_FLARE
constexpr float ALTURA_FLARE   = 0.15f;  // [m]
constexpr float ALTURA_REF_MIN = 0.025f; // [m] piso de la referencia

// Corte: cerca del piso se suelta el lazo de altura y la potencia sólo baja.
constexpr float ALTURA_CORTE     = 0.12f; // [m] o si el dron dejó de bajar
constexpr float VZ_SIN_BAJAR     = 0.05f; // [m/s]
constexpr int   CICLOS_SIN_BAJAR = 125;   // 0.5 s
constexpr float RAMPA_CORTE      = 3.0f;  // [PWM/ciclo] 750 PWM/s

// Apoyado: bajo y quieto. Ahí se cortan los motores de una, sin rampa: seguir
// bajando potencia con el dron ya en el piso es lo que lo hacía deslizarse.
constexpr float ALTURA_APOYADO = 0.055f; // [m]
constexpr float VZ_APOYADO     = 0.06f;  // [m/s] (en reposo Vz estimada ~ -0.04)
constexpr int   CICLOS_APOYADO = 10;     // 40 ms de confirmación

// Variables de otros módulos
extern float u_roll, u_pitch, u_yaw, u_alt;
extern float x_hat_alt[2];
extern float DesiredAltitude;
extern EstadoDron estadoActual;

// Estado interno de las secuencias
static bool  haDespegado    = false;
static bool  enCorte        = false;
static float throttleCorte  = 0.0f;
static int   ciclosSinBajar = 0;
static int   ciclosApoyado  = 0;

static void reiniciarSecuencias() {
  haDespegado    = false;
  enCorte        = false;
  throttleCorte  = 0.0f;
  ciclosSinBajar = 0;
  ciclosApoyado  = 0;
}

// Cuenta ciclos consecutivos en que se cumple una condición.
static bool sostenido(bool condicion, int &contador, int ciclos) {
  contador = condicion ? (contador + 1) : 0;
  return (contador >= ciclos);
}

void ejecutarSupervisorVuelo() {

  // Detección de entrada de estado. estadoActual lo escribe la telemetría en el
  // Core 0, así que la transición puede caer en cualquier punto del ciclo.
  static EstadoDron estadoPrevio = APAGADO;
  if (estadoActual != estadoPrevio) {
    if (estadoActual == DESPEGANDO || estadoActual == ATERRIZANDO) {
      reiniciarSecuencias();
      DesiredAltitude = x_hat_alt[0]; // arrancar en la altura real, sin escalón
    }
    estadoPrevio = estadoActual;
  }

  switch (estadoActual) {

    // ================================================================
    // DESPEGUE
    //   1) Rampa a lazo abierto hasta separarse del piso.
    //   2) Ascenso con la referencia limitada en adelanto.
    // ================================================================
    case DESPEGANDO:
      if (!haDespegado) {
        DesiredAltitude = x_hat_alt[0];
        calcularControl();
        u_alt = 0.0f; // apoyado no hay nada que regular en altura

        baseThrottleDinamico += RAMPA_DESPEGUE;
        if (baseThrottleDinamico < (0.70f * THROTTLE_HOVER)) u_yaw = 0.0f;

        // El lazo se cierra al SEPARARSE del piso, no al llegar a
        // THROTTLE_HOVER: esperar lo segundo lo dejaba acelerando a lazo
        // abierto con todo el empuje extra del efecto suelo.
        if (x_hat_alt[0] >= ALTURA_DESPEGUE ||
            baseThrottleDinamico >= (float)THROTTLE_HOVER) {
          haDespegado = true;
          baseThrottleDinamico = (float)THROTTLE_HOVER;
          DesiredAltitude = x_hat_alt[0];
          Serial.println("INFO: Despegue detectado. Lazo de altura cerrado.");
        }
        actualizarMotores(true, (int)baseThrottleDinamico, u_roll, u_pitch, u_yaw);

      } else {
        // Limitar el adelanto de la referencia limita la velocidad de ascenso.
        if ((DesiredAltitude - x_hat_alt[0]) < ADELANTO_MAX) {
          DesiredAltitude += VEL_ASCENSO * h;
        }
        if (DesiredAltitude > AlturaObjetivoFinal) DesiredAltitude = AlturaObjetivoFinal;

        calcularControl();

        // La transición mira la altura REAL, no la referencia.
        if (x_hat_alt[0] >= (AlturaObjetivoFinal - TOLERANCIA_LLEGADA) &&
            fabsf(x_hat_alt[1]) < VZ_LLEGADA) {
          estadoActual = VOLANDO;
          Serial.println("INFO: Altura objetivo alcanzada. Transición a VOLANDO.");
        }
        actualizarMotores(true, THROTTLE_HOVER + (int)u_alt, u_roll, u_pitch, u_yaw);
      }
      break;

    case VOLANDO:
      DesiredAltitude = AlturaObjetivoFinal;
      calcularControl();
      actualizarMotores(true, THROTTLE_HOVER + (int)u_alt, u_roll, u_pitch, u_yaw);
      break;

    // ================================================================
    // ATERRIZAJE
    //   1) Descenso a VEL_DESCENSO, y a VEL_FLARE bajo ALTURA_FLARE.
    //   2) Corte: se suelta el lazo de altura, la potencia sólo baja y los
    //      motores se apagan de una en cuanto el dron se apoya.
    //   La actitud se sigue corrigiendo en las dos fases.
    // ================================================================
    case ATERRIZANDO:
      if (!enCorte) {
        bool  flare       = (x_hat_alt[0] <= ALTURA_FLARE);
        float velDescenso = flare ? VEL_FLARE : VEL_DESCENSO;
        float adelanto    = flare ? ADELANTO_MAX_FLARE : ADELANTO_MAX;

        if ((x_hat_alt[0] - DesiredAltitude) < adelanto) {
          DesiredAltitude -= velDescenso * h;
        }
        if (DesiredAltitude < ALTURA_REF_MIN) DesiredAltitude = ALTURA_REF_MIN;

        calcularControl();

        // Se entra al corte por altura, o porque el dron dejó de bajar y ya no
        // va a bajar más por sí solo.
        bool dejoDeBajar = sostenido(fabsf(x_hat_alt[1]) < VZ_SIN_BAJAR,
                                     ciclosSinBajar, CICLOS_SIN_BAJAR);
        if (x_hat_alt[0] <= ALTURA_CORTE || dejoDeBajar) {
          enCorte = true;
          throttleCorte = (float)THROTTLE_HOVER + u_alt; // sin salto de empuje
          ciclosApoyado = 0;
          Serial.println("INFO: Cerca del suelo. Bajando potencia.");
        }
        actualizarMotores(true, THROTTLE_HOVER + (int)u_alt, u_roll, u_pitch, u_yaw);

      } else {
        DesiredAltitude = x_hat_alt[0];
        calcularControl();
        u_alt = 0.0f; // lazo de altura fuera de juego

        throttleCorte -= RAMPA_CORTE;

        bool apoyado = sostenido(x_hat_alt[0] <= ALTURA_APOYADO &&
                                 fabsf(x_hat_alt[1]) < VZ_APOYADO,
                                 ciclosApoyado, CICLOS_APOYADO);

        if (apoyado || throttleCorte <= 0.0f) {
          estadoActual = APAGADO;
          Serial.println("INFO: Aterrizaje completado. Motores apagados.");
          actualizarMotores(false, 0, 0, 0, 0);
        } else {
          actualizarMotores(true, (int)throttleCorte, u_roll, u_pitch, u_yaw);
        }
      }
      break;

    case APAGADO:
    default:
      u_roll = 0.0f; u_pitch = 0.0f; u_yaw = 0.0f; u_alt = 0.0f;
      baseThrottleDinamico = 0.0f;
      DesiredAltitude = 0.0f;
      reiniciarSecuencias();
      actualizarMotores(false, 0, 0, 0, 0);
      break;
  }
}
