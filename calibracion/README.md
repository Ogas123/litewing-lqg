# Calibración del acelerómetro por Levenberg-Marquardt (Anexo B)

`Levenberg-Marquardt.ipynb` estima los nueve parámetros que corrigen las lecturas del acelerómetro del MPU6050:
tres de desalineación de ejes ($\alpha_{yx}$, $\alpha_{zx}$, $\alpha_{zy}$), tres factores de escala ($S_x$, $S_y$, $S_z$)
y tres sesgos ($B_x$, $B_y$, $B_z$). Sigue el método de Tedaldi, Pretto y Menegatti (ICRA 2014), que no necesita
mesa giratoria ni equipamiento externo.

## Método

1. **Adquisición.** La IMU se deja en reposo unos segundos y luego se apoya a mano en 36 a 50 posturas estáticas
   distintas, de 5 a 10 s cada una.
2. **Detector de reposo.** Una ventana móvil de 10 muestras descarta los tramos en movimiento: sólo se usan las muestras
   cuyo desvío en los tres ejes es menor que 0,05 m/s².
3. **Ajuste.** En reposo el módulo de la aceleración corregida debe valer $g$ = 9,80665 m/s². `scipy.optimize.least_squares`
   (Levenberg-Marquardt) minimiza la suma de $\lVert T K (a - b) \rVert^2 - g^2$ sobre todas las posturas.
4. **Exportación.** El cuaderno imprime las nueve constantes como líneas `constexpr float` listas para pegar en
   `firmware/Config.h`. `IMU.cpp` las aplica en cada ciclo de 4 ms.

## Cómo ejecutarlo

```bash
jupyter lab calibracion/Levenberg-Marquardt.ipynb
```

**Entrada:** `calibracion/datos_imu.csv`, con columnas `AccX`, `AccY` y `AccZ` en m/s² (lecturas **sin corregir**),
muestreadas a 50 Hz.

**Salida:** los nueve parámetros, el gráfico de la serie antes y después de la corrección y el bloque C++ para
`Config.h`.

## Resultado usado en el firmware

```cpp
constexpr float ALFA_YX = 0.000278f;   constexpr float S_X = 1.005936f;   constexpr float B_X = 0.313151f;
constexpr float ALFA_ZX = 0.001603f;   constexpr float S_Y = 0.997343f;   constexpr float B_Y = 0.016393f;
constexpr float ALFA_ZY = 0.000864f;   constexpr float S_Z = 0.991658f;   constexpr float B_Z = 0.223452f;
```
