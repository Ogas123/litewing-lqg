# Telemetría y mando (Anexo D)

`Telemetria.py` es la estación en tierra del dron: recibe la telemetría por WiFi, la grafica en vivo, la guarda en
CSV y envía los comandos de despegue, aterrizaje y emergencia.

## Requisitos

Python 3 con `numpy`, `pyqtgraph` y `PyQt6` (ver [`requirements.txt`](../requirements.txt)).

## Uso

1. Encender el dron. El ESP32-S3 levanta una red WiFi propia (*SoftAP*): **`LiteWing_Agus`**, clave `12345678`.
2. Conectar la computadora a esa red.
3. Ejecutar:
   ```bash
   python telemetria/Telemetria.py
   ```
   La ventana empieza a graficar en cuanto llega el primer paquete.

| Control | Tecla | Comando UDP | Efecto |
| :--- | :--- | :--- | :--- |
| DESPEGAR / ATERRIZAR | Espacio | `1` / `2` | Despegue automático hasta 0,50 m / descenso controlado y apagado al apoyarse |
| EMERGENCIA | Esc | `0` | Corte inmediato de los cuatro motores, desde cualquier estado |

La IP (`192.168.4.1`) y el puerto (`4210`) del dron se pueden cambiar en la barra superior de la ventana.

## Protocolo

El firmware envía a 50 Hz un paquete UDP binario de 76 bytes, *little-endian*: un `uint32` con la estampa de tiempo
en µs seguido de 18 `float` (formato `struct` de Python `'<I18f'`). El detalle byte por byte está en
[`firmware/README.md`](../firmware/README.md#-9-paquete-binario-de-telemetría-udp-76-bytes).

## Registros de vuelo (`vuelos/`)

Cada sesión se guarda en `vuelos/vuelo_AAAAMMDD_HHMMSS.csv`. El archivo se crea con el primer paquete recibido, así
que abrir la ventana sin el dron encendido no deja archivos vacíos.

| Columna | Unidad | Contenido |
| :--- | :--- | :--- |
| `timestamp_us` | µs | Tiempo del microcontrolador desde el arranque |
| `AccX`, `AccY`, `AccZ` | m/s² | Aceleraciones corregidas por la calibración |
| `Roll_Acc`, `Pitch_Acc` | ° | Ángulos calculados con el acelerómetro |
| `Roll_Gyr`, `Pitch_Gyr`, `YawRate_Gyr` | °/s | Velocidades angulares del giróscopo |
| `Roll_Kalman`, `Pitch_Kalman` | ° | Ángulos estimados |
| `RollRate_Kalman`, `PitchRate_Kalman`, `YawRate_Kalman` | °/s | Velocidades angulares estimadas |
| `Alt_ToF` | m | Distancia medida por el VL53L1X |
| `Alt_Kalman`, `Vz_Kalman` | m, m/s | Altura y velocidad vertical estimadas |
| `VBat` | V | Tensión de batería filtrada |
| `temp` | °C | Temperatura interna de la IMU |

Si la batería se agota en vuelo, el microcontrolador se reinicia y `timestamp_us` vuelve a cero dentro del mismo
archivo; el cuaderno de análisis descarta lo posterior a ese salto.

Los dos registros que usa el PIP son `vuelo_final.csv` y `vuelo_final_ajustado.csv` (ver
[`analisis/README.md`](../analisis/README.md)). El resto son las pruebas de vuelo del ajuste, de agosto y septiembre
de 2026.
