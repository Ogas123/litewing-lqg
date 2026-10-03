# Análisis: diseño de los reguladores y resultados de vuelo (Anexo A)

`Resultados.ipynb` reúne todos los cálculos que respaldan los Capítulos 3 y 4 del PIP: desde el modelo discreto
hasta las figuras de los vuelos. Las constantes que imprime son las que se copian a `firmware/Config.h`.

## Secciones

| § | Contenido | En el PIP |
| :--- | :--- | :--- |
| 1 | Modelo discreto por retención de orden cero ($h$ = 4 ms) de los cuatro subsistemas | §3.3.2, §3.3.4 |
| 2 | Solución de la DARE y ganancias LQR, impresas como líneas de `Config.h` | §3.3 |
| 3 | Ganancias de Kalman de régimen con las covarianzas $R$ asumidas | Tabla 4.3 |
| 4 | Simulación del lazo cerrado desde una condición inicial perturbada | Figura 3.5 |
| 5 | Ruido medido de los sensores y ajuste de $R$ (`vuelo_final.csv`) | §4.2, Figura 4.2, Tabla 4.2 |
| 6 | Ganancias de Kalman con las $R$ ajustadas | Tabla 4.3 |
| 7 | Simulación del observador: $R$ asumidas frente a $R$ ajustadas | Figura 4.3 |
| 8 | Vuelo final con el diseño ajustado (`vuelo_final_ajustado.csv`) | §4.3, Figura 4.4 |
| 9 | Verificación del ruido en el vuelo final | Figura 4.5 |
| 10 | Estados medidos frente a estimados por Kalman | Figura 4.6 |
| 11 | Beneficio del filtro: ángulo frente a velocidad angular | §4.3 |
| 12 | Polos en el plano z: planta, lazo cerrado y observador | §4.4, Figura 4.7 |

## Cómo ejecutarlo

Desde la raíz del repositorio, con el entorno de [`requirements.txt`](../requirements.txt) instalado:

```bash
jupyter lab analisis/Resultados.ipynb
```

y ejecutar todas las celdas (*Run → Run All Cells*). Tarda menos de un minuto.

- **Entradas:** `../telemetria/vuelos/vuelo_final.csv` y `../telemetria/vuelos/vuelo_final_ajustado.csv`. La ruta se
  define una sola vez, en la variable `VUELOS` de la primera celda de código.
- **Salidas:** las figuras del PIP en `analisis/figuras/tesis_fig_*.png`, dibujadas a su tamaño final
  (16 cm de ancho, 300 dpi). La carpeta no se versiona porque se regenera en cada ejecución.

## Convenciones

Unidades de firmware en todo el cuaderno: ángulos en grados, velocidades angulares en °/s, altura en m y el esfuerzo de
control en cuentas PWM (0…4095). Ejes del cuerpo en convención NED. El detalle está en
[`firmware/README.md`](../firmware/README.md#-2-convenio-de-unidades).
