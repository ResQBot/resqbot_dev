#include "TLE9879_Group.h"
#include <Arduino.h>
#include <SPI.h>

// ————————————————————————
// CONFIGURATION
// ————————————————————————
static const uint8_t MAX_MOTORS = 4;
static const uint8_t MOTOR_COUNT = 4;
static const uint8_t BOARD_ID[MAX_MOTORS] = { BOARD1, BOARD2, BOARD3, BOARD4 };

static const int16_t STOP_DELAY    = 5;    // ms per motor stop
static const int16_t REFRESH_RATE  = 20;    // Hz
static const int16_t DELAY_MS = (1000 / REFRESH_RATE) - (STOP_DELAY * MOTOR_COUNT);

static const int16_t DEFAULT_SPEED = 2900;  // motor‐speed scale

static const unsigned long PRINT_INTERVAL  = 50;  // ms
static const unsigned long POLL_INTERVAL   = 1;   // ms
static const unsigned long ERROR_THRESHOLD = 1000; // ms

// mechanical / Hall constants
static const int16_t FHBLDC_HALL_POLE_PAIRS = 2;
static const float GEAR_RATIO = 44.0f;
static const float WORM_GEAR_RATIO = 10.0f;
static const float TOTAL_GEAR_RATIO = 44.0f;
static const float CORRECTION_FACTOR = 1.2f;
static const float STEP_ELEC = 60.0f;
static const float STEP_CORR = STEP_ELEC * CORRECTION_FACTOR;
static const float TRANS_PER_OUT = (360.0f * FHBLDC_HALL_POLE_PAIRS * TOTAL_GEAR_RATIO) / STEP_CORR;

// ————————————————————————
// GLOBALS
// ————————————————————————
TLE9879_Group *shields;

// drive state
int16_t commanded[MAX_MOTORS];
int16_t modeState[MAX_MOTORS];

// estimator state
unsigned long lastPollTime[MAX_MOTORS];
unsigned long lastPrintTime[MAX_MOTORS];
unsigned long lastTransTime[MAX_MOTORS];
unsigned long transPeriod[MAX_MOTORS];
int32_t reportCount[MAX_MOTORS];
float lastDiscAngle[MAX_MOTORS];
int8_t lastIdx[MAX_MOTORS];      // zuletzt gesehenes Hall-Sektorfeld (0-5)

// Hall readings buffer
uint16_t hallBuf[MAX_MOTORS];

// ————————————————————————
// ANGLE HELPERS
// ————————————————————————
float normalizeAngle(float a) {
  float x = fmod(a, 360.0f);
  return (x < 0) ? x + 360.0f : x;
}

float getCounterClockwiseElectricalAngle(uint16_t hallPattern) {
  switch (hallPattern) {
    case 0b010: return 0.0f;
    case 0b011: return 60.0f;
    case 0b001: return 120.0f;
    case 0b101: return 180.0f;
    case 0b100: return 240.0f;
    case 0b110: return 300.0f;
    default:    return -1.0f;
  }
}

float getClockwiseElectricalAngle(uint16_t hallPattern) {
  switch (hallPattern) {
    case 0b001: return 0.0f;
    case 0b011: return 60.0f;
    case 0b010: return 120.0f;
    case 0b110: return 180.0f;
    case 0b100: return 240.0f;
    case 0b101: return 300.0f;
    default:    return -1.0f;
  }
}

// ————————————————————————
// TRANSITION HANDLING
// ————————————————————————
void handleTransition(int i, float currAng, unsigned long now){
    // 0°, 60°, … 300° → Index 0 … 5
    int8_t currIdx = (int)round(currAng / STEP_ELEC) % 6;

    if (lastIdx[i] < 0) {                  // erster gültiger Messwert
        lastIdx[i]       = currIdx;
        lastTransTime[i] = now;
        return;
    }

    /* --------  sichtbare Schrittweite 0 … ±5  -------- */
    int8_t diff;
    if (modeState[i] > 0)      diff = (currIdx - lastIdx[i] + 6) % 6;       // CCW
    else if (modeState[i] < 0) diff = -((lastIdx[i] - currIdx + 6) % 6);    // CW
    else                       diff = 0;

    /* --------  Fall 1: Index hat sich geändert  -------- */
    if (diff != 0) {
        unsigned long dt = now - lastTransTime[i];
        unsigned long stepTime = dt / abs(diff); // Zeit für **eine** Flanke

        reportCount[i] += diff;                  // ± sichtbare Schritte
        transPeriod[i]  = stepTime;              // neue Referenz­periode
        lastTransTime[i] = now;
        lastIdx[i]       = currIdx;
        return;
    }

    /* --------  Fall 2: Index gleich ⇒ evtl. ganze 6er-Pakete verpasst  -------- */
    if (transPeriod[i] == 0) return;             // noch keine Referenz

    unsigned long dt = now - lastTransTime[i];
    uint32_t missed  = dt / transPeriod[i];      // geschätzte Flanken

    if (missed == 0) return;                     // zu wenig Zeit vergangen

    int8_t sgn = (modeState[i] > 0) ? 1 : (modeState[i] < 0 ? -1 : 0);
    reportCount[i] += sgn * missed;              // alle fehlenden Flanken addieren
    lastTransTime[i] += missed * transPeriod[i]; // Zeitbasis nachführen
}




bool checkMotorError(int i, unsigned long now) {
  if (now - lastTransTime[i] > ERROR_THRESHOLD) {
    Serial.print("[ERROR] Motor ");
    Serial.print(i);
    Serial.println(" stuck or no hall!");
    return true;
  }
  return false;
}

float computeTotalElec(int i, unsigned long now) {
  float frac = 0;
  if (transPeriod[i] > 0) {
    frac = float(now - lastTransTime[i]) / transPeriod[i];
    if (frac > 1.0f) frac = 1.0f;
  }
  float tot = reportCount[i] + frac;
  float m   = fmod(tot, TRANS_PER_OUT);
  return m * STEP_CORR;
}

float electricalToMechanical(float e) {
  return e / FHBLDC_HALL_POLE_PAIRS;
}

float motorToOutputAngle(float mech) {
  return mech / TOTAL_GEAR_RATIO;
}

// ————————————————————————
// SETUP
// ————————————————————————
void setup() {
  shields = new TLE9879_Group(1);

  // initialize each enabled board
  for (int i = 0; i < MOTOR_COUNT; i++) {
    uint8_t b = BOARD_ID[i];
    shields->setMode(HALL, b);
    shields->setParameter(HALL_PWM_FREQ, 20000, b);
    shields->setParameter(HALL_POLE_PAIRS, 2, b);
    shields->setParameter(HALL_SPEED_IMIN, 0, b);
    shields->setParameter(HALL_SPEED_IMAX, 29, b);
    shields->setParameter(HALL_SPEED_PIMIN, 0, b);
    shields->setParameter(HALL_SPEED_PIMAX, 29, b);
    shields->setParameter(HALL_OFFSET_60DEGREE_EN, 1, b);
    shields->setMotorSpeed(0, b);
    shields->setMotorMode(START_MOTOR, b);
  }

  unsigned long t0 = millis();
  // clear all slots
  for (int i = 0; i < MAX_MOTORS; i++) {
    commanded[i]     = 0;
    modeState[i]     = 0;
    lastPollTime[i]  = t0;
    lastPrintTime[i] = t0;
    lastTransTime[i] = t0;
    transPeriod[i]   = 0;
    reportCount[i]   = 0;
    lastDiscAngle[i] = -1.0f;
    lastIdx[i]       = -1;   // <— NEU
  }
}

// ————————————————————————
// MAIN LOOP
// ————————————————————————
void loop() {
  // 1) Read serial for up to MOTOR_COUNT values
  if (Serial.available()) {
    String rx = Serial.readStringUntil('\n');
    int tmp[MAX_MOTORS] = {0,0,0,0};
    sscanf(rx.c_str(), "FL%iFR%iRL%iRR%i",
           &tmp[0], &tmp[1], &tmp[2], &tmp[3]);
    for (int i = 0; i < MOTOR_COUNT; i++) {
      commanded[i] = tmp[i];
    }
  }

  /*
  // 2) Flip rear motors (indices 2 and 3) if they exist
  for (int i = 2; i < MOTOR_COUNT; i++) {
    commanded[i] *= -1;
  }
  */

  unsigned long now = millis();

  // 3) Drive & mode transitions
  for (int i = 0; i < MOTOR_COUNT; i++) {
    uint8_t b = BOARD_ID[i];
    if (commanded[i] == modeState[i]) {
      delay(STOP_DELAY);
    } else {
      shields->setMotorMode(STOP_MOTOR, b);        // only when needed
      delay(STOP_DELAY);
      shields->setMotorMode(START_MOTOR, b);       // ENABLE the bridge first
      delay(1);                                    // HW needs ≥ 1 µs, be generous
      shields->setMotorSpeed(commanded[i] * DEFAULT_SPEED, b);
      modeState[i] = commanded[i];
    }
  }

  // 4) Read all enabled Hall patterns
  shields->readHallPatterns(hallBuf);

  // 5) Position estimation + print
  for (int i = 0; i < MOTOR_COUNT; i++) {
    // a) poll sensor
    if (now - lastPollTime[i] >= POLL_INTERVAL) {
      lastPollTime[i] = now;
      float ang = (modeState[i] > 0)
                ? getCounterClockwiseElectricalAngle(hallBuf[i])
                : (modeState[i] < 0)
                  ? getClockwiseElectricalAngle(hallBuf[i])
                  : -1.0f;
      if (ang >= 0) {
        handleTransition(i, ang, now);
        checkMotorError(i, now);
      }
    }
  }
  // b) print every PRINT_INTERVAL
  static unsigned long lastPrintTimeGlobal = 0;               // NEW

  if (now - lastPrintTimeGlobal >= PRINT_INTERVAL) {
    lastPrintTimeGlobal = now;

    String line = "";


    for (int i = 0; i < MOTOR_COUNT; ++i) {
      float elec = computeTotalElec(i, now);
      float mech = electricalToMechanical(elec);
      float out  = motorToOutputAngle(mech);
      const char* pfx = (i == 0 ? "FL"
                       : i == 1 ? "FR"
                       : i == 2 ? "RL"
                                : "RR");

        line += pfx;
        line += String((int)round(normalizeAngle(out)));    
    }

    Serial.println(line); // print the whole line once
  }

  // 6) enforce overall loop rate
  delay(DELAY_MS);
}
