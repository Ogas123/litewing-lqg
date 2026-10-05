# litewing-lqg

Control LQG embebido para la estabilización de actitud y altura de un micro-cuadricóptero de 66 g.

Repositorio del Proyecto Integrador Profesional (PIP) de Ingeniería Electrónica:
*"Diseño e implementación de un sistema embebido para la estabilización de actitud y altitud de un micro-cuadricóptero"*,
Universidad Nacional del Comahue. Autor: **Agustín Schwerdt**.

El objetivo es mantener un vuelo estacionario estable a una altura fija (0,50 m) en interiores. El control corre por
completo a bordo de un ESP32-S3, a 250 Hz: un banco de filtros de Kalman estima los siete estados y cuatro reguladores
LQR desacoplados (alabeo, cabeceo, guiñada y altura) calculan el empuje de los motores. No hay control de posición
horizontal ni acción integral.

## Contenido

```
litewing-lqg/
├── firmware/      Firmware del ESP32-S3 (Arduino): Kalman, LQR, supervisor de vuelo, telemetría UDP
├── analisis/      Cuaderno con el diseño de los reguladores y el análisis de los vuelos (figuras del Cap. 3 y 4)
├── calibracion/   Cuaderno de calibración del acelerómetro por Levenberg-Marquardt
├── telemetria/    Aplicación de telemetría y mando en Python, y los registros de vuelo (vuelos/*.csv)
└── requirements.txt
```

Cada carpeta tiene su propio README con el detalle.

| Carpeta | Anexo del PIP | Qué produce |
| :--- | :--- | :--- |
| [`analisis/`](analisis/) | A | Modelo discreto, ganancias LQR y de Kalman, simulaciones y Figuras 3.5 y 4.2–4.7 |
| [`calibracion/`](calibracion/) | B | Los 9 parámetros de calibración del acelerómetro que usa `Config.h` |
| [`firmware/`](firmware/) | C | El programa que vuela el dron |
| [`telemetria/`](telemetria/) | D | Visualización en vivo, comandos de despegue/aterrizaje/emergencia y registro CSV |

## Hardware

La plataforma es la **LiteWing V2.5C**, un diseño abierto de J. Joseph (Semicon Media) publicado por Circuit Digest.
El proyecto original, con los archivos de diseño del hardware y su firmware de fábrica, está en
[Circuit-Digest/LiteWing](https://github.com/Circuit-Digest/LiteWing); la documentación, en
<https://circuitdigest.com/wiki/litewing/>. La placa se fabricó a partir de ese proyecto sin modificar el circuito,
por eso el hardware no forma parte de este repositorio. Componentes principales: ESP32-S3, IMU MPU6050, sensor de
distancia láser VL53L1X, cuatro motores sin núcleo 720 y una batería LiPo de una celda.

## Uso rápido

**Firmware.** Ver [`firmware/README.md`](firmware/README.md) para compilarlo y cargarlo.

**Python** (telemetría y cuadernos). Probado con Python 3.14:

```bash
python3 -m venv .venv
source .venv/bin/activate          # en Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

- Telemetría: `python telemetria/Telemetria.py`
- Cuadernos: `jupyter lab` y abrir `analisis/Resultados.ipynb` o `calibracion/Levenberg-Marquardt.ipynb`

## Reproducir los resultados del PIP

`analisis/Resultados.ipynb` regenera todas las figuras de datos del PIP a partir de dos registros:

- `telemetria/vuelos/vuelo_final.csv`: vuelo con las covarianzas de medición asumidas; con él se midió el ruido real
  de los sensores (§4.2).
- `telemetria/vuelos/vuelo_final_ajustado.csv`: vuelo con el firmware final, desde batería llena hasta agotarla (§4.3).

El resto de los archivos de `telemetria/vuelos/` son los registros de las pruebas de vuelo hechas durante el ajuste
(agosto y septiembre de 2026). Se conservan como datos crudos, pero ningún resultado del PIP depende de ellos.

## Versiones

El tag [`v1.0-PIP`](../../tree/v1.0-PIP) corresponde exactamente a la versión presentada en el PIP.
Los enlaces del PIP apuntan a ese tag, de modo que cambios posteriores en `main` no alteran lo que se cita.

## Licencia

El código y los cuadernos se distribuyen bajo la licencia [MIT](LICENSE). El diseño del hardware
[LiteWing](https://github.com/Circuit-Digest/LiteWing) pertenece a sus autores y no está incluido en esta licencia.
