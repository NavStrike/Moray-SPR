"""
SPR Sensor Monitoring Application - Main Entry Point
====================================================

Autor: moyraTech - Proyecto ProInnova SensorSPR
File Name: main.py
"""

from pathlib import Path
import sys
import math
from PySide6.QtCore import QObject, Signal, Slot, QTimer
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine
from datetime import datetime
import csv

import os
os.environ.setdefault("QT_QPA_PLATFORM", "wayland")

import re
import time

# ===== Serial opcional (ESP32) =====
try:
    import serial
    from serial.tools import list_ports
    _serial_available = True
except Exception as _e:
    print(f"pyserial no disponible: {_e}")
    _serial_available = False

# ===== LED opcional con gpiozero =====
try:
    from gpiozero import LED
    _led_available = True
except ImportError:
    print("gpiozero no disponible, LED deshabilitado.")
    _led_available = False


class Backend(QObject):
    """
    Backend para comunicación entre Python y QML.
    """

    newLDRSample = Signal(float, float, float)
    newSample = Signal(float, float, float)
    activeChanged = Signal(bool)
    angleUpdate = Signal(float, float)
    newLDRSampleWithAngle = Signal(float, float, float, float, float)
    serialStatusChanged = Signal(str)
    csvSaved = Signal(str)
    csvError = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        # Estado interno
        self._active = False
        self._t = 0.0
        self._dt = 0.003              # 50 ms ≈ 20 Hz
        self._use_ads = False

        # Parámetros divisor
        self._vcc = 3.3
        self._r_fixed = 100_000.0
        self._ldr_to_vcc = False

        # LED (láser) opcional
        self.laser = None
        if _led_available:
            try:
                self.laser = LED(17)
                print("LED inicializado en GPIO17")
            except Exception as e:
                print(f"No se pudo inicializar LED: {e}")
                self.laser = None

        # Timer adquisición
        self.timer = QTimer(self)
        self.timer.setInterval(int(self._dt * 1000))
        self.timer.timeout.connect(self._on_timeout)

        # Serial ESP32
        self.ser = None
        self._serial_buf = b""
        self._last_abs = float("nan")
        self._last_rel = float("nan")
        self.serial_timer = QTimer(self)
        self.serial_timer.setInterval(10)
        she = self.serial_timer.timeout.connect(self._poll_serial)
        self._last_serial_rx_ms = 0

        if _serial_available:
            self._open_serial()
        else:
            print("Serial ESP32 deshabilitado (pyserial no disponible)")

        # ADS1115 (si disponible)
        try:
            import board
            from adafruit_ads1x15.ads1115 import ADS1115, P0, P1
            from adafruit_ads1x15.analog_in import AnalogIn

            print("Inicializando ADS1115...")
            i2c = board.I2C()
            self.ads = ADS1115(i2c, address=0x48)
            self.ads.gain = 1
            self.ads.data_rate = 860

            self.chan0 = AnalogIn(self.ads, P0)
            self.chan1 = AnalogIn(self.ads, P1)

            self._use_ads = True
            print("ADS1115 inicializado correctamente")
        except Exception as e:
            print(f"Hardware no disponible, usando simulación: {e}")
            self._use_ads = False
            self._sim_phase = 0.0
            self._sim_phase2 = math.pi

    # ===== Utilidades de exportación =====
    def _exports_dir(self) -> Path:
        out_dir = Path(__file__).parent / "exports"
        out_dir.mkdir(parents=True, exist_ok=True)
        return out_dir

    def _timestamp(self) -> str:
        return datetime.now().strftime("%Y%m%d_%H%M%S")

    @Slot(result=str)
    def getExportsDir(self) -> str:
        return str(self._exports_dir())

    # ===== CSV (compat) -> ahora usa sufijo _datageneral =====
    @Slot('QVariantList')
    def saveCsv(self, data_list):
        """
        Compatibilidad con tu QML actual.
        Guarda crudo como <fecha_hora>_datageneral.csv
        """
        try:
            out_dir = self._exports_dir()
            fname = f"{self._timestamp()}_datageneral.csv"
            fpath = out_dir / fname
            with open(fpath, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["angle_deg", "ch1_ohm", "ch2_ohm"])
                for row in data_list:
                    angle = float(row.get("angle", float("nan")))
                    ch1   = float(row.get("ch1",   float("nan")))
                    ch2   = float(row.get("ch2",   float("nan")))
                    w.writerow([angle, ch1, ch2])
            self.csvSaved.emit(str(fpath))
            print(f"CSV guardado: {fpath}")
        except Exception as e:
            msg = f"Error guardando CSV: {e}"
            self.csvError.emit(msg)
            print(msg)

    # ===== CSV crudo explícito (idéntico al anterior; útil para exportAll) =====
    @Slot('QVariantList')
    def saveRawDataCsv(self, data_list):
        try:
            out_dir = self._exports_dir()
            fname = f"{self._timestamp()}_datageneral.csv"
            fpath = out_dir / fname
            with open(fpath, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["angle_deg", "ch1_ohm", "ch2_ohm"])
                for row in data_list:
                    angle = float(row.get("angle", float("nan")))
                    ch1   = float(row.get("ch1",   float("nan")))
                    ch2   = float(row.get("ch2",   float("nan")))
                    w.writerow([angle, ch1, ch2])
            self.csvSaved.emit(str(fpath))
            print(f"[EXPORT] CSV crudo guardado: {fpath}")
        except Exception as e:
            msg = f"Error guardando CSV crudo: {e}"
            self.csvError.emit(msg)
            print(msg)

    # ===== CSV ángulo vs ciclo =====
    @Slot('QVariantList', 'QVariantList')
    def saveAngleVsCycleCsv(self, ch1_angles, ch2_angles):
        """
        Guarda serie ángulo vs ciclo:
        cycle_index, ch1_angle_deg, ch2_angle_deg
        Nombre: <fecha_hora>_angulovsciclo.csv
        """
        try:
            out_dir = self._exports_dir()
            fname = f"{self._timestamp()}_angulovsciclo.csv"
            fpath = out_dir / fname

            n1 = len(ch1_angles) if ch1_angles is not None else 0
            n2 = len(ch2_angles) if ch2_angles is not None else 0
            N  = max(n1, n2)

            with open(fpath, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["cycle_index", "ch1_angle_deg", "ch2_angle_deg"])
                for i in range(N):
                    a1 = float(ch1_angles[i]) if i < n1 else float("nan")
                    a2 = float(ch2_angles[i]) if i < n2 else float("nan")
                    w.writerow([i+1, a1, a2])

            self.csvSaved.emit(str(fpath))
            print(f"[EXPORT] CSV ángulo vs ciclo guardado: {fpath}")
        except Exception as e:
            msg = f"Error guardando CSV ángulo vs ciclo: {e}"
            self.csvError.emit(msg)
            print(msg)

    # ===== Conversión Vout -> R_LDR (Ω) =====
    def _vout_to_rldr(self, vout: float) -> float:
        eps = 1e-6
        v = max(min(vout, self._vcc - eps), eps)
        if self._ldr_to_vcc:
            # Según tu implementación actual:
            return v / self._r_fixed
        else:
            return self._r_fixed * (v / (self._vcc - v))

    # ===== Serial =====
    def _open_serial(self):
        port_env = os.environ.get("ESP32_PORT")
        baud = 115200

        candidates = []
        if port_env:
            candidates.append(port_env)
        candidates.extend(["/dev/ttyUSB0"])

        try:
            for p in list_ports.comports():
                if p.device not in candidates:
                    candidates.append(p.device)
        except Exception as e:
            print(f"No se pudo listar puertos: {e}")

        print(f"Puertos candidatos ESP32: {candidates}")

        for dev in candidates:
            try:
                self.ser = serial.Serial(dev, baudrate=baud, timeout=0)
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    try:
                        self.ser.setDTR(False)
                        self.ser.setRTS(False)
                    except Exception:
                        pass
                except Exception:
                    pass
                print(f"Conectado a ESP32 en {dev} @ {baud}")
                self.serialStatusChanged.emit(f"ESP32 conectado: {dev}")
                self.serial_timer.start()
                return
            except Exception:
                continue

        msg = "No se pudo abrir ningún puerto serial para ESP32. Defina ESP32_PORT."
        print(msg)
        self.serialStatusChanged.emit("ESP32 desconectado")

    def _send_serial(self, data: str):
        try:
            if self.ser and self.ser.writable():
                payload = data.encode("utf-8")
                print(f"SER TX -> {data.strip()}")
                self.ser.write(payload)
                try:
                    self.ser.flush()
                except Exception:
                    pass
        except Exception as e:
            print(f"Error enviando serial: {e}")

    def _poll_serial(self):
        if not self.ser:
            return
        try:
            chunk = self.ser.read(1024)
            if not chunk:
                now_ms = int(time.monotonic() * 1000)
                if self._last_serial_rx_ms == 0:
                    self._last_serial_rx_ms = now_ms
                elif now_ms - self._last_serial_rx_ms > 2000:
                    self.serialStatusChanged.emit("ESP32 sin datos (2s)")
                    self._last_serial_rx_ms = now_ms
                return
            self._serial_buf += chunk
            while b"\n" in self._serial_buf:
                line, self._serial_buf = self._serial_buf.split(b"\n", 1)
                try:
                    s = line.decode("utf-8", errors="ignore").strip()
                    if s:
                        print(f"SER RX <- {s}")
                except Exception:
                    continue
                m = re.search(r"abs=([-+]?\d+\.?\d*)\s+rel=([-+]?\d+\.?\d*)", s)
                if m:
                    try:
                        abs_deg = float(m.group(1))
                        rel_deg = float(m.group(2))
                        self._last_abs = abs_deg
                        self._last_rel = rel_deg
                        self._last_serial_rx_ms = int(time.monotonic() * 1000)
                        self.serialStatusChanged.emit("ESP32 recibiendo…")
                        self.angleUpdate.emit(abs_deg, rel_deg)
                    except Exception:
                        pass
        except Exception as e:
            print(f"Error leyendo serial: {e}")
            self.serialStatusChanged.emit("Error serial - reconectando…")
            try:
                if self.ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
            finally:
                self.ser = None
            try:
                self.serial_timer.stop()
            except Exception:
                pass
            QTimer.singleShot(1000, self._open_serial)

    # ===== Slots de control =====
    @Slot()
    def setRelativeZero(self):
        self._send_serial("ZC\n")

    @Slot(float)
    def setAbsoluteZero(self, abs_deg: float):
        try:
            self._send_serial(f"Z{float(abs_deg):.2f}\n")
        except Exception:
            pass

    @Slot(bool)
    def setActive(self, on: bool):
        if self._active == on:
            return
        self._active = on
        if on:
            print("Iniciando adquisición de datos...")
            self.timer.start()
            try:
                if self.laser is not None:
                    self.laser.on()
                    print("LED encendido")
            except Exception as e:
                print(f"Advertencia: no se pudo encender LED: {e}")
            self._send_serial("o\n")
        else:
            print("Deteniendo adquisición de datos...")
            self.timer.stop()
            try:
                if self.laser is not None:
                    self.laser.off()
                    print("LED apagado")
            except Exception as e:
                print(f"Advertencia: no se pudo apagar LED: {e}")
            self._send_serial("s\n")
        self.activeChanged.emit(self._active)

    def isActive(self) -> bool:
        return self._active

    # ===== Adquisición periódica =====
    def _on_timeout(self):
        self._t += self._dt

        if self._use_ads:
            try:
                ch1_voltage = float(self.chan0.voltage)
                ch2_voltage = float(self.chan1.voltage)
            except Exception as e:
                print(f"Error leyendo ADS1115: {e}")
                ch1_voltage = float("nan")
                ch2_voltage = float("nan")
        else:
            self._sim_phase += self._dt
            self._sim_phase2 += self._dt
            ch1_voltage = 1.65 + math.sin(self._sim_phase * 2 * math.pi / 2.0) * 1.0
            ch2_voltage = 1.65 + math.sin(self._sim_phase2 * 2 * math.pi / 2.5) * 0.8

        ch1_res = self._vout_to_rldr(ch1_voltage) if math.isfinite(ch1_voltage) else float("nan")
        ch2_res = self._vout_to_rldr(ch2_voltage) if math.isfinite(ch2_voltage) else float("nan")
        print(f"ch1_res_current: {ch1_res}, ch2_res_current: {ch2_res}")

        # emitir
        self.newLDRSample.emit(self._t, ch1_res, ch2_res)
        self.newSample.emit(self._t, ch1_res, ch2_res)
        self.newLDRSampleWithAngle.emit(self._t, ch1_res, ch2_res, self._last_abs, self._last_rel)


def main():
    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()

    backend = Backend()
    engine.rootContext().setContextProperty("backend", backend)

    def _cleanup():
        try:
            if backend.laser is not None:
                backend.laser.off()
                print("LED apagado (salida)")
        except Exception as e:
            print(f"Advertencia: no se pudo apagar LED al salir: {e}")
    app.aboutToQuit.connect(_cleanup)

    qml_path = Path(__file__).parent / "ui" / "LDRMonitor_maximo.qml"   # modificado a LDRMonitor.qml
    print(f"Cargando interfaz: {qml_path}")
    engine.load(str(qml_path))

    if not engine.rootObjects():
        print("Error: No se pudo cargar la interfaz QML")
        sys.exit(1)

    print("Aplicación iniciada. Use los controles en pantalla para activar/desactivar.")
    print("Presione Ctrl+C o use el botón 'Salir' para terminar.")

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
