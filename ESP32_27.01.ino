#include <Arduino.h>
#include <Wire.h>

// ==============================
//         PINES DRIVER
// ==============================
#define STEP_PIN 25
#define DIR_PIN  26
#define EN_PIN   27
#define MS1_PIN  14
#define MS2_PIN  12

// ==============================
//       PINES I2C / AS5600
// ==============================
#define SDA_PIN  21
#define SCL_PIN  22


#define AS5600_ADDR 0x36
#define REG_ANGLE   0x0E

// ==============================
//   PARÁMETROS DEL MOTOR
// ==============================
const int  FULLSTEPS_PER_REV = 200;
const int  MICROSTEPS        = 64;                                    // tu ajuste
const long STEPS_PER_REV     = (long)FULLSTEPS_PER_REV * MICROSTEPS;
const float DEG_PER_STEP     = 360.0f / (float)STEPS_PER_REV;

// ==============================
//  VELOCIDAD DE GIRO (FIJA)
// ==============================
const float TARGET_SPEED_DPS  = 1.0f;                                 // tu velocidad
const float STEP_RATE_SPS     = TARGET_SPEED_DPS / DEG_PER_STEP;
const uint32_t HALF_PERIOD_US = (uint32_t)(1e6f / (2.0f * STEP_RATE_SPS));

hw_timer_t* timer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool stepLevel = false;
volatile bool running   = false;

// ==============================
//   CONTROL POR ENCODER AS5600
// ==============================
// ANTES: const float ZERO_DEG = 342.8613f;
// AHORA: cero dinámico + comando para setearlo en caliente
const float DEFAULT_ZERO_DEG = 207.27f;
float zeroDeg = DEFAULT_ZERO_DEG;     // cero relativo DINÁMICO (0..360)
float targetDeg = 0.0f;               // objetivo relativo (-180..+180]
const float TOL_DEG = 0.2f;          // recomendado >= 0.1

// ---- Oscilación A <-> B ----
float angleA = 39.0f;                 // por defecto
float angleB = 50.0f;                 // por defecto
uint32_t dwellMs = 50;               // pausa por extremo (ms)

enum Mode { MODE_IDLE, MODE_GOTO, MODE_SWEEP };
Mode mode = MODE_IDLE;

enum SweepState { SWEEP_TO_A, SWEEP_DWELL_A, SWEEP_TO_B, SWEEP_DWELL_B };
SweepState sweepState = SWEEP_TO_A;
uint32_t dwellStartMs = 0;

// Secuencia automática de inicio: 0° -> +30°
enum AutoState { AUTO_OFF, AUTO_TO_ZERO, AUTO_TO_30, AUTO_DONE };
AutoState autoState = AUTO_OFF;

// ------------------------------
//  LECTURA I2C 12-bit del AS5600
// ------------------------------
uint16_t i2cRead12(uint8_t reg) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFFFF;
  if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return 0xFFFF;
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return (uint16_t)(((msb & 0x0F) << 8) | lsb);
}

// ------------------------------
//  ÁNGULO ABSOLUTO (0..360)
// ------------------------------
float angleDegreesAbs() {
  uint16_t raw = i2cRead12(REG_ANGLE);
  if (raw == 0xFFFF) return NAN;
  return (raw * 360.0f) / 4096.0f;
}

// ------------------------------
//  ENVOLVER A RANGO (-180,180]
// ------------------------------
static inline float wrap180(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

// ------------------------------
//  ÁNGULO RELATIVO (abs - zeroDeg)
// ------------------------------
float angleDegreesRel() {
  float a = angleDegreesAbs();
  if (isnan(a)) return NAN;
  return wrap180(a - zeroDeg);
}

// ==============================
//     ISR: TREN DE PULSOS STEP
// ==============================
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  stepLevel = !stepLevel;
  digitalWrite(STEP_PIN, stepLevel ? HIGH : LOW);
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ==============================
//   INICIO / PARO DEL GIRO
// ==============================
void startRun(bool dirCW) {
  portENTER_CRITICAL(&timerMux);

  // *** CAMBIO ***
  // habilitar driver SOLO cuando realmente vamos a mover
  digitalWrite(EN_PIN, LOW);   // LOW = habilitado en tu driver

  digitalWrite(DIR_PIN, dirCW ? HIGH : LOW);
  delayMicroseconds(5);
  stepLevel = false;
  digitalWrite(STEP_PIN, LOW);

  timerStop(timer);
  timerWrite(timer, 0);
  timerAlarm(timer, (uint64_t)HALF_PERIOD_US, true, 0);
  timerStart(timer);

  running = true;
  portEXIT_CRITICAL(&timerMux);
}

void stopRun() {
  portENTER_CRITICAL(&timerMux);
  running = false;
  stepLevel = false;
  digitalWrite(STEP_PIN, LOW);
  timerStop(timer);

  // *** CAMBIO ***
  // deshabilitar driver cuando paramos -> motor libre para moverlo a mano
  digitalWrite(EN_PIN, HIGH);  // HIGH = deshabilitado / sin torque

  portEXIT_CRITICAL(&timerMux);
}

// ==============================
//   SETUP (INICIALIZACIÓN)
// ==============================
void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  pinMode(EN_PIN,   OUTPUT);
  pinMode(MS1_PIN,  OUTPUT);
  pinMode(MS2_PIN,  OUTPUT);

  // Microstepping según tu driver
  digitalWrite(MS1_PIN, LOW);
  digitalWrite(MS2_PIN, HIGH);

  // *** CAMBIO IMPORTANTE ***
  // Al inicio: motor DESHABILITADO, libre
  digitalWrite(EN_PIN, HIGH);    // <- ahora arranca apagado, puedes girarlo con la mano

  Wire.begin(SDA_PIN, SCL_PIN);

  Wire.setClock(100000);

  timer = timerBegin(1000000);  // base 1 MHz
  timerAttachInterrupt(timer, &onTimer);

  delay(300);

  // *** CAMBIO ***
  // Antes arrancabas secuencia automática sola.
  // Eso hacía que el motor quiera moverse inmediatamente.
  // La apagamos. Comenzamos totalmente inactivo.
  //
  // targetDeg = 0.0f;
  // mode = MODE_GOTO;
  // autoState = AUTO_TO_ZERO;
  //
  // → En su lugar:
  mode = MODE_IDLE;
  autoState = AUTO_OFF;
  targetDeg = 0.0f;

  Serial.println("Comandos:");
  Serial.println("  z = secuencia auto (ir a 0°, luego +30°)  [NO SE EJECUTA HASTA QUE LA PIDAS]");
  Serial.println("  a<deg>, b<deg> = set extremos relativos, ej: a-20.5  b45");
  Serial.println("  w<ms> = dwell en ms por extremo, ej: w800");
  Serial.println("  o = iniciar oscilación A<->B (habilita motor)");
  Serial.println("  g<deg> = ir a ángulo relativo (habilita motor)");
  Serial.println("  s = stop (deshabilita motor)");
  Serial.println("  ZC = setear CERO al ángulo ABSOLUTO ACTUAL");
  Serial.println("  Z<deg> = setear CERO absoluto, ej: Z342.86");
}

// ==============================
//        LOOP PRINCIPAL
// ==============================
void loop() {
  // --- Comandos por Serial ---
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("z")) {
      // Ahora "z" activa la secuencia auto SOLO cuando tú quieras.
      targetDeg = 0.0f;
      mode = MODE_GOTO;
      autoState = AUTO_TO_ZERO;
      Serial.println("Secuencia auto: GOTO 0°, luego +30°");

    } else if (cmd.equalsIgnoreCase("ZC")) {
      float a = angleDegreesAbs();
      if (!isnan(a)) {
        while (a < 0.0f)   a += 360.0f;
        while (a >= 360.0f) a -= 360.0f;
        zeroDeg = a;
        Serial.printf("ZERO set (capture ABS): %.4f°\n", zeroDeg);
      } else {
        Serial.println("ERROR: no se pudo capturar ABS (encoder NaN).");
      }

    } else if (cmd.startsWith("Z")) {
      if (cmd.length() >= 2) {
        float v = cmd.substring(1).toFloat();
        while (v < 0.0f)   v += 360.0f;
        while (v >= 360.0f) v -= 360.0f;
        zeroDeg = v;
        Serial.printf("ZERO set (explicit): %.4f°\n", zeroDeg);
      } else {
        Serial.println("Uso: Z<grados> (ej: Z342.86) o ZC (capturar)");
      }

    } else if (cmd.startsWith("a") || cmd.startsWith("A")) {
      float v = cmd.substring(1).toFloat();
      angleA = wrap180(v);
      Serial.printf("Ángulo A = %.4f°\n", angleA);

    } else if (cmd.startsWith("b") || cmd.startsWith("B")) {
      float v = cmd.substring(1).toFloat();
      angleB = wrap180(v);
      Serial.printf("Ángulo B = %.4f°\n", angleB);

    } else if (cmd.startsWith("w") || cmd.startsWith("W")) {
      long v = cmd.substring(1).toInt();
      if (v < 0) v = 0;
      dwellMs = (uint32_t)v;
      Serial.printf("Dwell por extremo = %lums\n", (unsigned long)dwellMs);

    } else if (cmd.equalsIgnoreCase("o")) {
      // Este comando es el que tu UI manda en setActive(true)
      // => aquí sí vamos a permitir que mueva.
      float rel = angleDegreesRel();
      if (!isnan(rel)) {
        float dA = fabs(wrap180(angleA - rel));
        float dB = fabs(wrap180(angleB - rel));
        if (dA <= dB) {
          sweepState = SWEEP_TO_A;
          targetDeg  = angleA;
        } else {
          sweepState = SWEEP_TO_B;
          targetDeg  = angleB;
        }
        mode = MODE_SWEEP;
        Serial.printf("Oscilación ON: A=%.4f°, B=%.4f°, dwell=%lums\n",
                      angleA, angleB, (unsigned long)dwellMs);
      } else {
        Serial.println("Error: no se puede iniciar (encoder inválido).");
      }

    } else if (cmd.startsWith("g") || cmd.startsWith("G")) {
      float v = cmd.substring(1).toFloat();
      targetDeg = wrap180(v);
      mode = MODE_GOTO;
      autoState = AUTO_OFF;
      Serial.printf("Objetivo relativo (GOTO): %.4f°\n", targetDeg);

    } else if (cmd.equalsIgnoreCase("s")) {
      stopRun();
      mode = MODE_IDLE;
      autoState = AUTO_OFF;
      Serial.println("Stop (driver deshabilitado).");
    }
  }

  // --- Control de posición (bang-bang) ---
  float rel = angleDegreesRel();
  if (!isnan(rel)) {
    float delta = wrap180(targetDeg - rel);

    if (fabs(delta) <= TOL_DEG) {
      if (running) stopRun();

      // Secuencia automática: 0° -> +30°
      if (mode == MODE_GOTO) {
        if (autoState == AUTO_TO_ZERO) {
          targetDeg = 69.0f;
          autoState = AUTO_TO_30;
        } else if (autoState == AUTO_TO_30) {
          autoState = AUTO_DONE;
          mode = MODE_IDLE;
        }
      }
    } else {
      bool dirCW = (delta > 0);
      if (!running) {
        // *** CAMBIO ***
        // Al empezar a corregir posición recién energizamos
        startRun(dirCW);
      } else {
        bool dirActualCW = (digitalRead(DIR_PIN) == HIGH);
        if (dirActualCW != dirCW) {
          stopRun();
          startRun(dirCW);
        }
      }
    }

    // --- Máquina de estados de oscilación ---
    if (mode == MODE_SWEEP) {
      switch (sweepState) {
        case SWEEP_TO_A:
          if (fabs(wrap180(rel - angleA)) <= TOL_DEG) {
            stopRun();
            dwellStartMs = millis();
            sweepState = SWEEP_DWELL_A;
          }
          break;
        case SWEEP_DWELL_A:
          if (millis() - dwellStartMs >= dwellMs) {
            targetDeg  = angleB;
            sweepState = SWEEP_TO_B;
          }
          break;
        case SWEEP_TO_B:
          if (fabs(wrap180(rel - angleB)) <= TOL_DEG) {
            stopRun();
            dwellStartMs = millis();
            sweepState = SWEEP_DWELL_B;
          }
          break;
        case SWEEP_DWELL_B:
          if (millis() - dwellStartMs >= dwellMs) {
            targetDeg  = angleA;
            sweepState = SWEEP_TO_A;
          }
          break;
      }
    } else if (mode == MODE_IDLE) {
      if (running) stopRun(); // asegura que quede libre
    }
  } else {
    if (running) stopRun();   // encoder NaN -> seguridad
  }

  // --- Telemetría (cada ~10 ms) ---
  static uint32_t tLast = 0;
  uint32_t now = millis();
  if (now - tLast >= 10) {
    tLast = now;
    float aAbs = angleDegreesAbs();
    float aRel = angleDegreesRel();

    Serial.printf("abs=%.4f rel=%.4f tgt=%.2f run=%d mode=%d state=%d zero=%.2f\n",
                  aAbs, aRel, targetDeg, (int)running, (int)mode, (int)sweepState, zeroDeg);
  }
}
