"""
SPR Sensor Monitoring Application - Main Entry Point
====================================================
Autor: moyraTech - Proyecto ProInnova SensorSPR
File Name: main.py
"""

from PySide6.QtCore import QObject, Signal, Slot, QTimer, QThread
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine

from datetime import datetime
from pathlib import Path
import sys
import os
import math
import csv
import re
import time
import json

from functions.adminData import accessData
from functions.printInfo import print_info, print_warning, print_error, print_debug

# Modo de ejecución (debug o production)
try:
    data = accessData().data
    production_mode = data['production'] if data is not None else True
    print_debug(f"Modo de ejecución: {'PRODUCTION' if production_mode else 'DEVELOPMENT'}")
    print("************************************")
except:
    production_mode = True
    print_debug("No se pudo determinar el modo de ejecución, se asume PRODUCTION por defecto.")

print("************************************")

# En modo production, se fuerza el uso de Wayland para evitar problemas de rendimiento en Linux.
if production_mode:
    os.environ.setdefault("QT_QPA_PLATFORM", "wayland")

# ===== Serial opcional (ESP32) =====
try:
    import serial
    from serial.tools import list_ports
    _serial_available = True
except Exception as _e:
    print_warning(f"pyserial no disponible: {_e}")
    _serial_available = False

# ===== LED opcional (LASER) =====
try:
    from gpiozero import LED # type: ignore
    _led_available = True
except ImportError:
    print_warning("gpiozero no disponible, LED deshabilitado.")
    _led_available = False

# Interrupción para la lectura de datos del encoder y ADC
class SerialReaderThread(QThread):
    data_received = Signal(dict)    # Señal con los datos decodificados
    def __init__(self):
        pass

# ===== Backend para comunicación entre Python y QML ===== 
class Backend(QObject):
    # Señales para comunicación con QML

    # Transmicion de datos
    newLDRSample = Signal(float, float, float)
    newLDRSampleWithAngle = Signal(float, float, float, float, float)
    angleUpdate = Signal(float, float)
    timeUpdate = Signal(str)
    # Cambios de estado
    activeChanged = Signal(bool)
    angleMaxMin = Signal(float, float)
    speedMaxMin = Signal(float, float)
    substanceAct = Signal(list, list, str)
    serialStatusChanged = Signal(str)
    adqDeviceChanged = Signal(str, str)
    currentChanged = Signal(float)
    # Guardado de archivos
    csvSaved = Signal(str)
    csvError = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        # ---------------------------------------------
        # Designacion de variables y estados internos
        # ---------------------------------------------

        # Parametros de simulación (si no hay hardware)
        self._angRelaIn = 65.00
        self._angRelaTest = self._angRelaIn
        self._angRelaFin = 85.00
        self._advance = 0.1

        # Estado interno
        self._active = False
        self._use_ads = False

        # Variables de tiempo
        self._t = 0.0
        self._dt = 0.001              # 1 ms ≈ 1 kHz

        self._t_init = 0.0
        self._t_present = 0.0
        self._t_pause = 0.0
        self._t_pause_init = []
        self._t_pause_end = []

        # ESP32 Serial
        self._serial = False
        self.ser = None
        self._serial_buf = b""
        self._last_serial_rx_ms = 0     # Lectura serial del ESP32
        self._last_abs = float("nan")
        self._last_rel = float("nan")

        # Angulo cero relativo (ajustable)
        self.angzeroRel = 13.7607

        # Ángulos de barrido
        self.angMin = 70.0
        self.angMax = 80.0

        # Velocidades min y max
        self.velMin = 4
        self.velMax = 12

        # Valor de la corriente
        self.current = 550

        # Sustancia actual
        self.listSubstances = []
        self.anglesSubstances = []
        self.substance = ""

        # Parámetro de guardado en CSV
        self.saveDataName = "default"
        self.saveDataPath = "exports"

        # Parámetros divisor
        self._adq_device = "ldr" # "ldr" or "photodetector"
        self._unites_device = "resistance" # "resistance" or "current"
        self._vcc = 3.3
        self._r_fixed_ldr = 100_000.0 # Resistencia circuito divisor LDR (ohms)
        self._r_fixed_pho = 4_700.0 # Resistencia circuito divisor fotodiodo (ohms)

        # Variables acumulativas de datos
        self.dataAll = []   # [ciclo_1, ...] Acumulativo de todos los ciclos
        self.dataPeaks = []  # [(Angulo_rel, ch1_peak, ch2_peak), ...] Datos de picos por ciclo
        self.dataCycle = [] # [(Angulo_rel, ch1, ch2), ...]
        self.dataProcess = [] # Data procesada para visualización

        # ---------------------------------------------
        # Inicializacion de hardware
        # ---------------------------------------------

        # LED (láser) opcional
        self.laser = None
        if _led_available:
            try:
                self.laser = LED(17)
                print("LED inicializado en GPIO17")
            except Exception as e:
                print(f"No se pudo inicializar LED: {e}")
                self.laser = None

        if _serial_available and production_mode:
            self._open_serial()
        else:
            print_warning("Serial ESP32 deshabilitado (pyserial no disponible)")

        # ADS1115 (si disponible)
        try:
            import board # type: ignore
            from adafruit_ads1x15.ads1115 import ADS1115, P0, P1, P2    # type: ignore
            from adafruit_ads1x15.analog_in import AnalogIn             # type: ignore

            print_info("Inicializando ADS1115...")
            i2c = board.I2C()
            self.ads = ADS1115(i2c, address=0x48)
            self.ads.gain = 1
            self.ads.data_rate = 860

            self.chan0 = AnalogIn(self.ads, P0) # Canal de sensor 1
            self.chan1 = AnalogIn(self.ads, P1) # Canal de sensor 2
            self.chanRef0 = AnalogIn(self.ads, P2) # Canal de voltaje referencia (3V3/2)

            self._use_ads = True
            print_info("ADS1115 inicializado correctamente")
        except Exception as e:
            print_warning(f"Hardware no disponible, usando simulación: {e}")
            self._use_ads = False
            self._sim_phase = 0.0
            self._sim_phase2 = math.pi

        # Timer adquisición
        self.timer = QTimer(self)
        self.timer.setInterval(int(self._dt * 1000))
        self.timer.timeout.connect(self._on_worktime)

        # Timer visualización: angulo y tiempo (real y de funcionamiento)
        self.timer_window = QTimer(self)
        self.timer_window.setInterval(1)
        self.timer_window.timeout.connect(self._on_timeout)
        self.timer_window.start()

        # ---------------------------------------------
        # Lectura de datos guardados y configuración inicial
        # ---------------------------------------------

        # Actualizacion de los valores a los almacenados por el programa
        try:
            self._setValues()
            print_info("Se cargaron los datos correctamente")
        except Exception as e:
            print_error(f"Ha ocurrido un error al cargar los datos: {e}")

        # Establecimiento de los ángulos min y max de barrido
        try:
            self.setMaxMinAngles(self.angMin, self.angMax)
        except Exception as e:
            print_error(f"Error estableciendo los ángulos: {e}")

        # Establecimiento del ángulo zero relativo por defecto
        try:
            self.setAbsoluteZero(self.angzeroRel)
            print_info(f"Se ha establecido el cero en {self.angzeroRel}")
        except Exception as e:
            print_error(f"No se ha podido establecer el cero en {self.angzeroRel}: {e}")

        # Establecimiento de las velocidades min y max de barrido
        try:
            self.setMaxMinVel(self.velMin, self.velMax)
        except Exception as e:
            print_error(f"Error estableciendo las velocidades: {e}")

    # ===== Carga de variables para QML =====
    def chargeVariablesToQml(self):
        try:
            self.viewAngles()
            self.viewSpeeds()
            self.viewSubstance()
            self.viewDevice()
            self.viewCurrent()
            print_info("Variables cargadas correctamente para QML")
        except Exception as e:
            print_error(f"Error cargando variables para QML: {e}")

    # ===== Lectura de datos guardados =====
    def _setValues(self):
        p1 = accessData().data
        if p1 is None: raise ValueError("No se pudieron cargar los datos de configuración")

        self.angMin = p1["angles"]["angMin"]
        self.angMax = p1["angles"]["angMax"]
        self.angzeroRel = p1["angles"]["angZeroRel"]

        self.velMin = p1["speeds"]["velMin"]
        self.velMax = p1["speeds"]["velMax"]

        self._adq_device = p1["device"]["name"]
        self._unites_device = p1["device"]["unites"]

        self.saveDataName = p1["saveData"]["name"]
        self.saveDataPath = p1["saveData"]["path"]

        p2 = accessData("substances.json").data
        if p2 is None: raise ValueError("No se pudieron cargar los datos de sustancias")

        subs = p2["data"]
        self.listSubstances = [s["name"] for s in subs]; self.listSubstances.insert(0,"Otra sustancia")
        self.anglesSubstances = [s["spr_angles"] for s in subs]; self.anglesSubstances.insert(0,[0, 0])

    # ===== Utilidades de exportación =====
    def _exports_dir(self) -> Path:
        if self.saveDataName:
            out_dir = Path(__file__).parent / self.saveDataPath / self.saveDataName
        else:
            out_dir = Path(__file__).parent / self.saveDataPath

        out_dir.mkdir(parents=True, exist_ok=True)
        return out_dir

    def _timestamp(self, option = "all") -> str:
        if option == "date":
            return datetime.now().strftime("%Y/%m/%d")
        elif option == "time":
            return datetime.now().strftime("%H:%M:%S")
        else:
            return datetime.now().strftime("%Y%m%d_%H%M%S")

    @Slot(list, list, list, list)
    def saveJSON(self, data, dataProcessed, maxMinAngles, maxMinSpeeds):
        try:
            out_dir = self._exports_dir()
            fname = f"{self._timestamp()}_dataprocessed.json"
            fpath = out_dir / fname

            export_data = {
                "metadata": {
                    "date": self._timestamp("date"),
                    "time": self._timestamp("time"),
                    "source": "SPR Sensor Monitoring Application",
                    "version": "1.0",
                    "description": "Datos procesados de ángulo vs señal para análisis y visualización"
                },
                "data": data,
                "processed_data": dataProcessed,
                "max_min_angles": maxMinAngles,
                "max_min_speeds": maxMinSpeeds
            }

            with open(fpath, "w", encoding="utf-8") as f:
                json.dump(export_data, f, ensure_ascii=False, indent=4)

            self.csvSaved.emit(str(fpath))
            print_info(f"[EXPORT] JSON procesado guardado: {fpath}")
        except Exception as e:
            msg = f"Error guardando JSON procesado: {e}"
            self.csvError.emit(msg)
            print_error(msg)

    # ===== CSV crudo explícito (idéntico al anterior; útil para exportAll) =====
    @Slot(list)
    def saveRawDataCsv(self, data_list):
        # Guarda crudo como:
        # Name: <fecha_hora>_datageneral.csv
        try:
            out_dir = self._exports_dir()
            fname = f"{self._timestamp()}_datageneral.csv"
            fpath = out_dir / fname

            with open(fpath, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f, delimiter=";")
                w.writerow(["n_cycle", "time", "angle", "ch1", "ch2"])
                for row in data_list:
                    cycle = int(row.get("cycle", float("nan")))
                    time = float(row.get("time", float("nan")))
                    angle = float(row.get("angle", float("nan")))
                    ch1   = float(row.get("ch1",   float("nan")))
                    ch2   = float(row.get("ch2",   float("nan")))
                    w.writerow([cycle, time, angle, ch1, ch2])
            self.csvSaved.emit(str(fpath))
            print_info(f"[EXPORT] CSV crudo guardado: {fpath}")
        except Exception as e:
            msg = f"Error guardando CSV crudo: {e}"
            self.csvError.emit(msg)
            print_error(msg)

    # ===== CSV ángulo vs ciclo =====
    @Slot(list, list, list, list)
    def saveAngleVsTimeCsv(self, ch1_times, ch2_times, ch1_angles, ch2_angles):

        # Guarda serie ángulo vs ciclo como:
        # Nombre: <fecha_hora>_angulovsciclo.csv
        # cycle_index, ch1_angle_deg, ch2_angle_deg

        try:
            out_dir = self._exports_dir()
            fname = f"{self._timestamp()}_angulovsciclo.csv"
            fpath = out_dir / fname

            n1 = len(ch1_times) if ch1_angles is not None else 0
            n2 = len(ch2_times) if ch2_angles is not None else 0
            N  = max(n1, n2)

            with open(fpath, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f, delimiter=";")
                w.writerow(["cycle_index", "ch1_time", "ch1_angle_deg", "ch2_time", "ch2_angle_deg"])
                for i in range(N):
                    t1 = float(ch1_times[i]) if i < n1 else float("nan")
                    a1 = float(ch1_angles[i]) if i < n1 else float("nan")
                    t2 = float(ch2_times[i]) if i < n1 else float("nan")
                    a2 = float(ch2_angles[i]) if i < n2 else float("nan")
                    w.writerow([i+1, t1, a1, t2, a2])

            self.csvSaved.emit(str(fpath))
            print_info(f"[EXPORT] CSV ángulo vs tiempo guardado: {fpath}")
        except Exception as e:
            msg = f"Error guardando CSV ángulo vs tiempo: {e}"
            self.csvError.emit(msg)
            print_error(msg)

    def _saveProcessedDataCsv(self, data_list):
        pass

    # ===== Conversión Vout -> R_LDR (Ω) =====
    def _vout_to_signal(self, vout: float) -> float:

        eps = 1e-6
        vol = max(min(vout, self._vcc - eps), eps)

        if self._adq_device == "ldr":

            if self._unites_device == "current":
                return vol / self._r_fixed_ldr*(1000)  # ms
            else:
                return self._r_fixed_ldr * (vol / (self._vcc - vol))*(1/1000)    # kohm
            
        elif self._adq_device == "photodetector":

            if self._unites_device == "current":
                return vol/self._r_fixed_pho*(1000)    # ms
            else:
                return self._r_fixed_pho * vol / (self._vcc - vol)*(1/1000)  # kohm

        else:
            return vol

    # ===== Serial =====
    def _open_serial(self):
        port_env = os.environ.get("ESP32_PORT")
        baud = 921600

        candidates = []
        if port_env:
            candidates.append(port_env)
        candidates.extend(["/dev/ttyUSB0"])

        try:
            for p in list_ports.comports():
                if p.device not in candidates:
                    candidates.append(p.device)
        except Exception as e:
            print_error(f"No se pudo listar puertos: {e}")

        print_info(f"Puertos candidatos ESP32: {candidates}")

        for dev in candidates:
            try:
                self.ser = serial.Serial(dev, baudrate=baud, timeout=0)
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    try:
                        self.ser.setDTR(False) # type: ignore
                        self.ser.setRTS(False) # type: ignore
                    except Exception:
                        pass
                except Exception:
                    pass
                print_info(f"Conectado a ESP32 en {dev} @ {baud}")
                self.serialStatusChanged.emit(f"ESP32 conectado: {dev}")
                self._serial = True
                return
            except Exception:
                continue

        print_error("No se pudo abrir ningún puerto serial para ESP32. Defina ESP32_PORT.")
        self.serialStatusChanged.emit("ESP32 desconectado")

    def _send_serial(self, data: str):
        try:
            if self.ser and self.ser.writable():
                payload = data.encode("utf-8")
                print_info(f"SER TX -> {data.strip()}")
                self.ser.write(payload)
                try:
                    self.ser.flush()
                except Exception:
                    pass
        except Exception as e:
            print_error(f"Error enviando serial: {e}")

    # ===== Slots de control =====
    @Slot()
    def setRelativeZero(self):
        try:
            #self._send_serial("ZC\n")
            n_ang_zero = self.angzeroRel + self._last_rel - 360*math.floor((self.angzeroRel + self._last_rel)/360)
            self._send_serial(f"Z{n_ang_zero:.2f}\n")
            print(self._last_abs, self._last_rel, n_ang_zero)
            # Actualización del cero relativo para el cálculo interno
            accessData().changeZeroRel(n_ang_zero)
        except Exception as e:
            print_error(f"Error actualizando cero relativo: {e}")

    @Slot(float)
    def setAbsoluteZero(self, abs_deg: float):
        try:
            self._send_serial(f"Z{float(abs_deg):.2f}\n")
            # Actualización del cero relativo para el cálculo interno
            accessData().changeZeroRel(abs_deg)
        except Exception as e:
            print_error(f"Error actualizando cero absoluto: {e}")

    @Slot(str)
    def changeTimer(self, op: str):
        if op == "reset":
            self._t_init = 0
            self._t_pause_init = []
            self._t_pause_init = []
            self._t_pause = 0
        elif op == "pause":
            self._t_pause_init.append(time.perf_counter())
        elif op == "continue":
            if len(self._t_pause_init) - len(self._t_pause_end) == 1:
                self._t_pause_end.append(time.perf_counter())
                self._t_pause += self._t_pause_end[-1] - self._t_pause_init[-1]


    @Slot(bool)
    def setActive(self, on: bool):
        if self._active == on:
            return
        self._active = on
        if on:
            print_info("Iniciando adquisición de datos...")
            self. changeTimer("continue") # continuación el timer de adquisición
            self.timer.start()  # inicio del timer de adquisición

            try:
                if self.laser is not None:
                    self.laser.on()
                    print_info("LED encendido")
            except Exception as e:
                print_error(f"Advertencia: no se pudo encender LED: {e}")

            self._send_serial("o\n")
            
        else:
            print_info("Deteniendo adquisición de datos...")
            self. changeTimer("pause") # pausa el timer de adquisición
            self.timer.stop()   # pausa del timer de adquisición
            
            try:
                if self.laser is not None:
                    self.laser.off()
                    print("LED apagado")
            except Exception as e:
                print_warning(f"Advertencia: no se pudo apagar LED: {e}")
            self._send_serial("s\n")
        self.activeChanged.emit(self._active)
    
    @Slot()
    def viewAngles(self):
        self.angleMaxMin.emit(self.angMin, self.angMax)

    @Slot()
    def viewSpeeds(self):
        self.speedMaxMin.emit(self.velMin, self.velMax)

    @Slot()
    def viewDevice(self):
        self.adqDeviceChanged.emit(self._adq_device, self._unites_device)

    @Slot()
    def viewCurrent(self):
        self.currentChanged.emit(self.current)
    
    @Slot()
    def viewSubstance(self):
        self.substanceAct.emit(self.listSubstances, self.anglesSubstances, self.substance)

    @Slot(float, float)
    def setMaxMinAngles(self, aMin: float, aMax: float):
        self.angMin = aMin
        self.angMax = aMax

        time.sleep(0.05)
        self._send_serial(f"a{aMin}\n")
        time.sleep(0.05)
        self._send_serial(f"b{aMax}\n")
        time.sleep(0.05)

        # Actualizacion de los angulos guardados
        m = accessData()
        m.changeAngles([aMin, aMax])
        
        print_info(f"Ángulos actualizados: Min={self.angMin}, Max={self.angMax}")

    @Slot(float, float)
    def setMaxMinVel(self, vMin: float, vMax: float):
        self.velMin = vMin
        self.velMax = vMax

        time.sleep(0.05)
        self._send_serial(f"v1{self.velMin}\n")
        time.sleep(0.05)
        self._send_serial(f"v2{self.velMax}\n")
        time.sleep(0.05)

        # Actualizacion de los angulos guardados
        accessData().ChangeSpeeds([vMin, vMax])
        
        print_info(f"Velocidades actualizadas: Min={self.velMin}, Max={self.velMax}")

    def currentToReal(self, value):
        a=0.0008; b=-0.3039; c=73.123 # parabola -> f(x)=ax^2+bx+c
        h=189.9375; k=46.9369094 # vertice de parabola
        return ((value-k)/a)**0.5+h

    @Slot(int)
    def setCurrent(self, currentValue: int):
        self.current = currentValue

        time.sleep(0.05)
        # currentRealValue = self.currentToReal(self.current)
        self._send_serial(f"c{self.current}\n")
        time.sleep(0.05)

        # Actualización de la corriente motor guardada
        accessData().ChangeCurrent(self.current)
        
        print_info(f"Corriente actualizada: {self.current}")

    @Slot(str)
    def setNameFile(self, name: str):
        self.saveDataName = name
        print("El nombre del archivo ha sido asignado a:", name)

    @Slot(str)
    def setAdqDevice(self, device: str):
        if device not in ["ldr", "photodetector"]:
            print_warning(f"Dispositivo de adquisición desconocido: {device}")
            return
        self._adq_device = device
        unites = "resistance" if device == "ldr" else "current"
        self._unites_device = unites
        accessData().ChangeAdqDevice(device, unites)
        
        print_info(f"Dispositivo de adquisición establecido a: {device}, unidades: {unites}")
    
    def isActive(self) -> bool:
        return self._active
    
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
            
            # print("data inicio")
            while b"\n" in self._serial_buf:
                line, self._serial_buf = self._serial_buf.split(b"\n", 1)
                try:
                    s = line.decode("utf-8", errors="ignore").strip()
                    if s:
                        # print_info(f"SER RX <- {s}")
                        pass
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

                        if rel_deg > 90:
                            try:
                                self.setAbsoluteZero(self.angzeroRel)
                                print_info(f"Se ha establecido el cero en {self.angzeroRel}")
                            except Exception as e:
                                print_error(f"No se ha podido establecer el cero en {self.angzeroRel}: {e}")
                        
                        self.angleUpdate.emit(abs_deg, rel_deg)
                    except Exception:
                        pass
            # print("data fin")
        except Exception as e:
            print_error(f"Error leyendo serial: {e}")
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
                self._serial = False
            except Exception:
                pass

            QTimer.singleShot(1000, self._open_serial)
    
    # ===== Adquisición tiempo de trabajo =====
    def _on_worktime(self):
        self._t += self._dt

        if self._t_init == 0: self._t_init = time.perf_counter()
        else: self._t_present = time.perf_counter()
        
        diff_time = self._t_present - self._t_init - self._t_pause

        if self._serial:
            self._poll_serial()
        elif not production_mode:
            # Generacion de datos simulados para desarrollo sin hardware
            self._sim_phase += self._dt
            self._sim_phase2 += self._dt

            # Asignacion de valores de angulo relativo
            self._last_rel = self._angRelaTest

            # Codicional de angulo relativo
            if self._angRelaTest >= self._angRelaFin:
                self._angRelaTest = self._angRelaIn
            else:
                self._angRelaTest += self._advance

            # Asignacion de valores de angulo absoluto
            self._last_abs = self._last_rel

        if self._use_ads:
            try:
                noise_voltage = float(self.chanRef0.voltage) - 3.3/2.0
                ch1_voltage = float(self.chan0.voltage) - noise_voltage
                ch2_voltage = float(self.chan1.voltage) - noise_voltage

                if ch1_voltage > self._vcc:
                    print_warning("El voltaje del canal 1 es mayor a 3.3V")
                    ch1_voltage = float("nan")
            
                if ch2_voltage > self._vcc:
                    print_warning("El voltaje del canal 2 es mayor a 3.3V")
                    ch2_voltage = float("nan")

            except Exception as e:
                print_error(f"Error leyendo ADS1115: {e}")
                ch1_voltage = float("nan")
                ch2_voltage = float("nan")

        elif not production_mode: 
            # Generacion de datos simulados para desarrollo sin hardware
            # Asignación de lecturas simuladas con oscilaciones senoidales
            ch1_voltage = 0.05 + math.sin(self._sim_phase * 2 * math.pi / 2.0) * 0.5
            ch2_voltage = 0.05 + math.sin(self._sim_phase2 * 2 * math.pi / 2.5) * 0.5
        
        ch1_res = self._vout_to_signal(ch1_voltage) if math.isfinite(ch1_voltage) else float("nan")
        ch2_res = self._vout_to_signal(ch2_voltage) if math.isfinite(ch2_voltage) else float("nan")
            
        # emitir datos a QML
        self.newLDRSampleWithAngle.emit(diff_time, ch1_res, ch2_res, self._last_abs, self._last_rel)

    # ===== Adquisición tiempo fuera =====
    def _on_timeout(self):
        self._poll_serial()
        self.timeUpdate.emit(self._timestamp("time"))



def main():
    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()

    backend = Backend()
    engine.rootContext().setContextProperty("backend", backend)

    def _cleanup():
        try:
            backend.timer_window.stop()
            if backend.laser is not None:
                backend.laser.off()
                print_info("LED apagado (salida)")
        except Exception as e:
            print_error(f"Advertencia: no se pudo apagar LED al salir: {e}")

    app.aboutToQuit.connect(_cleanup)
    
    qml_path = Path(__file__).parent / "ui" / "app.qml"
    
    print_info(f"Cargando interfaz: {qml_path}")
    engine.load(str(qml_path))

    if not engine.rootObjects():
        print_error("Error: No se pudo cargar la interfaz QML")
        sys.exit(1)

    backend.chargeVariablesToQml()

    print_info("Aplicación iniciada. Use los controles en pantalla para activar/desactivar.")
    print_info("Presione Ctrl+C o use el botón 'Salir' para terminar.")

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
