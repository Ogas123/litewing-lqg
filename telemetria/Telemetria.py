import sys
import os
import socket
import struct
import datetime
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

UDP_PORT = 4210
MAX_POINTS = 600

# Carpeta de registro, anclada a la ubicacion de este archivo (no al directorio
# desde el que se lanza el script)
CARPETA_VUELOS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vuelos")

# struct TelemetriaDron de Telemetria.h: uint32_t timestamp + 18 floats = 76 bytes
FORMATO = '<I18f'
TAM = struct.calcsize(FORMATO)

# Orden exacto de los 18 floats del struct
CAMPOS = ['AccX', 'AccY', 'AccZ',
          'Roll_Acc', 'Roll_Gyr', 'Roll_Kalman', 'RollRate_Kalman',
          'Pitch_Acc', 'Pitch_Gyr', 'Pitch_Kalman', 'PitchRate_Kalman',
          'YawRate_Gyr', 'YawRate_Kalman',
          'Alt_ToF', 'Alt_Kalman', 'Vz_Kalman',
          'VBat', 'temp']
IDX = {c: i for i, c in enumerate(CAMPOS)}

pg.setConfigOption('background', '#121212')
pg.setConfigOption('foreground', '#E0E0E0')
pg.setConfigOption('antialias', False)


class Telemetria(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Telemetria Dron")
        self.resize(1350, 920)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", UDP_PORT))
        self.sock.setblocking(False)

        self.datos = np.zeros((len(CAMPOS), MAX_POINTS))
        self.tiempo = np.zeros(MAX_POINTS)
        self.ptr = 0
        self.n = 0
        self.muestra = 0
        self.total = 0
        self.cuenta = 0
        self.t_fps = QtCore.QTime.currentTime()

        # El CSV se crea con el primer paquete, para no dejar archivos vacios
        # cada vez que se abre la ventana
        self.csv = None
        self.filas = 0

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        self.estado = QtWidgets.QLabel(f"Escuchando UDP en el puerto {UDP_PORT}...")
        self.estado.setStyleSheet(
            "color:#00E5FF; font-size:13px; font-family:Menlo,monospace;"
            "background-color:#1E1E1E; padding:6px; border-radius:4px; border:1px solid #333;")
        layout.addWidget(self.estado)

        layout.addWidget(self._panel_control())

        self.win = pg.GraphicsLayoutWidget()
        layout.addWidget(self.win)
        self._crear_graficos()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.actualizar)
        self.timer.start(16)

    # ---------------- control del dron ----------------

    def _panel_control(self):
        panel = QtWidgets.QFrame()
        panel.setStyleSheet(
            "QFrame { background-color:#1E1E1E; border-radius:4px; border:1px solid #333; }"
            "QLabel { color:#E0E0E0; font-size:13px; font-weight:bold; font-family:Menlo,monospace; }"
            "QLineEdit { background-color:#121212; color:#00E5FF; border:1px solid #444;"
            "            border-radius:4px; padding:4px 8px; font-family:monospace; }")
        fila = QtWidgets.QHBoxLayout(panel)

        self.ip = QtWidgets.QLineEdit("192.168.4.1")
        self.ip.setFixedWidth(120)
        self.puerto = QtWidgets.QLineEdit("4210")
        self.puerto.setFixedWidth(65)

        self.btn_vuelo = QtWidgets.QPushButton("DESPEGAR (espacio)")
        self.btn_vuelo.clicked.connect(self.despegar_aterrizar)
        self.volando = False
        self._color_boton(True)

        btn_emerg = QtWidgets.QPushButton("EMERGENCIA (Esc)")
        btn_emerg.setStyleSheet(
            "QPushButton { background-color:#D50000; color:#FFF; font-weight:bold;"
            " font-size:13px; border-radius:4px; padding:6px 18px; border:none; }")
        btn_emerg.clicked.connect(self.emergencia)

        # Sin foco: si no, la barra espaciadora activaria el boton que quedo
        # seleccionado y el comando se enviaria dos veces
        self.btn_vuelo.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        btn_emerg.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)

        self.tx = QtWidgets.QLabel("TX: listo")
        self.tx.setStyleSheet("color:#AAA; font-size:12px; font-family:monospace;")

        fila.addWidget(QtWidgets.QLabel("IP Dron:"))
        fila.addWidget(self.ip)
        fila.addWidget(QtWidgets.QLabel("Puerto:"))
        fila.addWidget(self.puerto)
        fila.addWidget(self.btn_vuelo)
        fila.addWidget(btn_emerg)
        fila.addWidget(self.tx)
        fila.addStretch()
        return panel

    def _color_boton(self, verde):
        color = "#00E676" if verde else "#FF9100"
        self.btn_vuelo.setStyleSheet(
            f"QPushButton {{ background-color:{color}; color:#000; font-weight:bold;"
            f" font-size:13px; border-radius:4px; padding:6px 18px; border:none; }}")

    def enviar(self, comando):
        try:
            ip = self.ip.text().strip()
            puerto = int(self.puerto.text().strip())
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.sendto(comando.encode(), (ip, puerto))
            s.close()
            self.tx.setText(f"TX: '{comando}' -> {ip}:{puerto}")
        except Exception as e:
            self.tx.setText(f"TX ERROR: {e}")

    def despegar_aterrizar(self):
        if self.volando:
            self.enviar("2")
            self.volando = False
            self.btn_vuelo.setText("DESPEGAR (espacio)")
            self._color_boton(True)
        else:
            self.enviar("1")
            self.volando = True
            self.btn_vuelo.setText("ATERRIZAR (espacio)")
            self._color_boton(False)

    def emergencia(self):
        self.enviar("0")
        self.volando = False
        self.btn_vuelo.setText("DESPEGAR (espacio)")
        self._color_boton(True)

    def keyPressEvent(self, evento):
        # Espacio: despegar / aterrizar. isAutoRepeat evita que mantener la
        # tecla apretada mande una rafaga de comandos.
        if evento.key() == QtCore.Qt.Key.Key_Space and not evento.isAutoRepeat():
            self.despegar_aterrizar()
        elif evento.key() == QtCore.Qt.Key.Key_Escape:
            self.emergencia()
        else:
            super().keyPressEvent(evento)

    # ---------------- graficos ----------------

    def _crear_graficos(self):
        self.curvas = {}
        graficos = []

        def grafico(fila, col, titulo, series):
            p = self.win.addPlot(row=fila, col=col, title=titulo)
            p.showGrid(x=True, y=True, alpha=0.2)
            p.addLegend(offset=(10, 10))
            for campo, color, ancho, nombre in series:
                self.curvas[campo] = p.plot(pen=pg.mkPen(color, width=ancho), name=nombre)
            graficos.append(p)

        AZUL, VERDE, AMARILLO, ROJO = '#00E5FF', '#00E676', '#FFEA00', '#FF3D00'

        grafico(0, 0, "Pitch: angulo [deg]",
                [('Pitch_Acc', AZUL, 1.5, "Acelerometro"),
                 ('Pitch_Kalman', VERDE, 2.5, "Kalman")])
        grafico(0, 1, "Pitch: velocidad angular [deg/s]",
                [('Pitch_Gyr', AMARILLO, 1.5, "Giroscopo"),
                 ('PitchRate_Kalman', VERDE, 2.5, "Kalman")])
        grafico(1, 0, "Roll: angulo [deg]",
                [('Roll_Acc', AZUL, 1.5, "Acelerometro"),
                 ('Roll_Kalman', VERDE, 2.5, "Kalman")])
        grafico(1, 1, "Roll: velocidad angular [deg/s]",
                [('Roll_Gyr', AMARILLO, 1.5, "Giroscopo"),
                 ('RollRate_Kalman', VERDE, 2.5, "Kalman")])
        grafico(2, 0, "Altura [m]",
                [('Alt_ToF', ROJO, 1.5, "ToF"),
                 ('Alt_Kalman', VERDE, 2.5, "Kalman")])
        grafico(2, 1, "Velocidad vertical Vz [m/s]",
                [('Vz_Kalman', AZUL, 2.5, "Kalman")])
        grafico(3, 0, "Yaw: velocidad angular [deg/s]",
                [('YawRate_Gyr', AMARILLO, 1.5, "Giroscopo"),
                 ('YawRate_Kalman', VERDE, 2.5, "Kalman")])
        grafico(3, 1, "IMU: aceleracion lineal",
                [('AccX', '#FF1744', 1.8, "Acc X"),
                 ('AccY', '#76FF03', 1.8, "Acc Y"),
                 ('AccZ', '#2979FF', 1.8, "Acc Z")])

        for p in graficos[1:]:
            p.setXLink(graficos[0])

    # ---------------- recepcion ----------------

    def actualizar(self):
        nuevos = False
        while True:
            try:
                paquete, _ = self.sock.recvfrom(1024)
            except BlockingIOError:
                break
            if len(paquete) == TAM:
                self.guardar(paquete)
                self.total += 1
                self.cuenta += 1
                nuevos = True

        if nuevos:
            self.dibujar()

        ahora = QtCore.QTime.currentTime()
        ms = self.t_fps.msecsTo(ahora)
        if ms >= 1000:
            hz = self.cuenta * 1000.0 / ms
            self.estado.setText(
                f"Paquetes: {self.total} | Frecuencia: {hz:.1f} Hz | "
                f"Bateria: {self.datos[IDX['VBat'], self.ptr - 1]:.2f} V | "
                f"Temp: {self.datos[IDX['temp'], self.ptr - 1]:.1f} C | "
                f"CSV: {self.filas} filas")
            self.cuenta = 0
            self.t_fps = ahora

    def abrir_csv(self):
        os.makedirs(CARPETA_VUELOS, exist_ok=True)
        nombre = f"vuelo_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"
        self.ruta_csv = os.path.join(CARPETA_VUELOS, nombre)
        self.csv = open(self.ruta_csv, 'w', buffering=1)
        self.csv.write('timestamp_us,' + ','.join(CAMPOS) + '\n')
        print(f"Registrando vuelo en: {self.ruta_csv}")

    def guardar(self, paquete):
        valores = struct.unpack(FORMATO, paquete)

        if self.csv is None:
            self.abrir_csv()
        self.csv.write(','.join(f'{v:g}' for v in valores) + '\n')
        self.filas += 1

        self.muestra += 1
        self.tiempo[self.ptr] = self.muestra
        self.datos[:, self.ptr] = valores[1:]
        self.ptr = (self.ptr + 1) % MAX_POINTS
        if self.n < MAX_POINTS:
            self.n += 1

    def dibujar(self):
        if self.n < MAX_POINTS:
            t = self.tiempo[:self.n]
            datos = self.datos[:, :self.n]
        else:
            t = np.roll(self.tiempo, -self.ptr)
            datos = np.roll(self.datos, -self.ptr, axis=1)

        for campo, curva in self.curvas.items():
            curva.setData(t, datos[IDX[campo]])

    def closeEvent(self, evento):
        if self.csv is not None:
            self.csv.close()
            print(f"Vuelo guardado en: {self.ruta_csv} ({self.filas} filas)")
        self.sock.close()
        evento.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    ventana = Telemetria()
    ventana.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
