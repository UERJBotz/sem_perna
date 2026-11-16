#include "robot.h"
#include "driver/gpio.h"
#include <Preferences.h>
#include <Wire.h>

// ============================================================================
// MOTORS
// ============================================================================
static void motorWrite(uint8_t in1, uint8_t in2, int speed) {
  speed = constrain(speed, -PWM_MAX, PWM_MAX);
  if (SLOW_DECAY) {
    if (speed >= 0) {
      ledcWrite(in1, PWM_MAX);
      ledcWrite(in2, PWM_MAX - speed);
    } else {
      ledcWrite(in1, PWM_MAX + speed);
      ledcWrite(in2, PWM_MAX);
    }
  } else {
    if (speed >= 0) {
      ledcWrite(in1, speed);
      ledcWrite(in2, 0);
    } else {
      ledcWrite(in1, 0);
      ledcWrite(in2, -speed);
    }
  }
}

static void writeAll(uint32_t duty) {
  ledcWrite(PIN_L_MOTOR_1, duty);
  ledcWrite(PIN_L_MOTOR_2, duty);
  ledcWrite(PIN_R_MOTOR_1, duty);
  ledcWrite(PIN_R_MOTOR_2, duty);
}

void motorsBegin() {
  ledcAttach(PIN_L_MOTOR_1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_L_MOTOR_2, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_R_MOTOR_1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_R_MOTOR_2, PWM_FREQ, PWM_RES);
  motorsCoast();
}

void setMotors(int left, int right) {
  if (L_MOTOR_INVERT) left = -left;
  if (R_MOTOR_INVERT) right = -right;
  motorWrite(PIN_L_MOTOR_1, PIN_L_MOTOR_2, left);
  motorWrite(PIN_R_MOTOR_1, PIN_R_MOTOR_2, right);
}

void motorsBrake() {
  writeAll(PWM_MAX);
}
void motorsCoast() {
  writeAll(0);
}

// ============================================================================
// ENCODERS
// ============================================================================
static volatile int32_t encL = 0, encR = 0;
static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR isrEncLeft() {
  bool a = gpio_get_level((gpio_num_t)PIN_L_ENC_A);
  bool b = gpio_get_level((gpio_num_t)PIN_L_ENC_B);
  portENTER_CRITICAL_ISR(&encMux);
  encL += (a == b) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encMux);
}

static void IRAM_ATTR isrEncRight() {
  bool a = gpio_get_level((gpio_num_t)PIN_R_ENC_A);
  bool b = gpio_get_level((gpio_num_t)PIN_R_ENC_B);
  portENTER_CRITICAL_ISR(&encMux);
  encR += (a == b) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encMux);
}

void encodersBegin() {
  pinMode(PIN_L_ENC_A, INPUT_PULLUP);
  pinMode(PIN_L_ENC_B, INPUT_PULLUP);
  pinMode(PIN_R_ENC_A, INPUT_PULLUP);
  pinMode(PIN_R_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_L_ENC_A), isrEncLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_R_ENC_A), isrEncRight, CHANGE);
}

void readEncoders(int32_t &left, int32_t &right) {
  portENTER_CRITICAL(&encMux);
  left = encL;
  right = encR;
  portEXIT_CRITICAL(&encMux);
  if (L_ENC_INVERT) left = -left;
  if (R_ENC_INVERT) right = -right;
}

void resetEncoders() {
  portENTER_CRITICAL(&encMux);
  encL = 0;
  encR = 0;
  portEXIT_CRITICAL(&encMux);
}

float distanceMm() {
  int32_t l, r;
  readEncoders(l, r);
  return (l + r) * 0.5f * MM_PER_TICK;
}

// ============================================================================
// SENSORS
// ============================================================================
CalData cal;
uint16_t raw[NUM_SENSORS];
uint16_t norm[NUM_SENSORS];
uint16_t sideNorm[2];

static int32_t lastPos = CENTER_POS;

static inline void selectMux(uint8_t ch) {
  digitalWrite(PIN_MUX_S0, ch & 0x01);
  digitalWrite(PIN_MUX_S1, (ch >> 1) & 0x01);
  digitalWrite(PIN_MUX_S2, (ch >> 2) & 0x01);
}

static uint16_t normalize(uint16_t v, uint16_t mn, uint16_t mx) {
  if (mx <= mn) return 0;
  int32_t n = ((int32_t)v - mn) * 1000 / (mx - mn);
  n = constrain(n, 0, 1000);
  return INVERT_SENSORS ? 1000 - n : n;
}

void sensorsBegin() {
  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  analogReadResolution(12);
}

void readRawSensors() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    selectMux(SENSOR_CHANNEL[i]);
    delayMicroseconds(MUX_SETTLE_US);
    raw[i] = analogRead(PIN_MUX_ADC);
  }
}

void readSideSensors() {
  sideNorm[0] = normalize(analogRead(PIN_SIDE_LEFT), cal.sideMin[0], cal.sideMax[0]);
  sideNorm[1] = normalize(analogRead(PIN_SIDE_RIGHT), cal.sideMin[1], cal.sideMax[1]);
}

int32_t readLinePosition(bool &onLine) {
  readRawSensors();
  uint32_t weighted = 0, sum = 0;
  onLine = false;

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    norm[i] = normalize(raw[i], cal.minV[i], cal.maxV[i]);
    if (norm[i] > LINE_ON_THRESHOLD) onLine = true;
    if (norm[i] > LINE_NOISE_FLOOR) {
      weighted += (uint32_t)norm[i] * i * 1000;
      sum += norm[i];
    }
  }

  // Line lost: return the extreme on the side where it was last seen
  if (!onLine) return (lastPos < CENTER_POS) ? 0 : (NUM_SENSORS - 1) * 1000;
  lastPos = weighted / sum;
  return lastPos;
}

int32_t lastLinePosition() {
  return lastPos;
}
void resetLinePosition() {
  lastPos = CENTER_POS;
}

uint8_t countActiveSensors() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
    if (norm[i] > 500) n++;
  return n;
}

// ============================================================================
// GYRO (MPU-6050, Z axis only)
// ============================================================================
static bool     gyroOk = false;
static bool     biasValid = false;
static float    gyroBias = 0;          // raw units
static float    gRate = 0;             // deg/s
static float    gHeading = 0;          // deg
static uint32_t gLastUs = 0;
static uint8_t  gErrors = 0;

// Offset tracking while the robot is idle and still
static int32_t  bSum = 0;
static int16_t  bMin = 32767, bMax = -32768;
static uint16_t bN = 0;
static uint32_t bStartMs = 0;

static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuReadGz(int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);                                  // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  gz = (int16_t)((hi << 8) | lo);
  return true;
}

void gyroBegin() {
  if (!GYRO_ENABLED) { Log.println("[GYRO] Disabled in config"); return; }
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  Wire.setTimeOut(3);                                // never block the loop for long

  uint8_t who = 0;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);                                  // WHO_AM_I
  bool found = Wire.endTransmission(false) == 0 &&
               Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1) == 1;
  if (found) who = Wire.read();
  if (!found) {
    Log.printf("[GYRO] MPU-6050 not found at 0x%02X, map uses wheel distances only\n", MPU_ADDR);
    return;
  }
  gyroOk = mpuWrite(0x6B, 0x01) &&                   // wake up, clock from gyro
           mpuWrite(0x1A, GYRO_DLPF) &&              // internal low-pass filter
           mpuWrite(0x1B, 0x18);                     // +/-2000 deg/s
  delay(50);
  gLastUs = micros();
  bStartMs = millis();
  Log.printf("[GYRO] %s (WHO_AM_I=0x%02X). Keep the robot still ~1 s to measure the offset.\n",
             gyroOk ? "MPU-6050 ready" : "setup failed", who);
}

static void biasTrack(int16_t raw) {
  bSum += raw;
  bMin = min(bMin, raw);
  bMax = max(bMax, raw);
  bN++;
  if (millis() - bStartMs < GYRO_BIAS_WINDOW_MS) return;
  if (bN > 100 && bMax - bMin <= GYRO_STILL_RANGE) {
    gyroBias = (float)bSum / bN;
    if (!biasValid) Log.printf("[GYRO] Offset measured: %.1f raw\n", gyroBias);
    biasValid = true;
  }
  bSum = 0; bN = 0; bMin = 32767; bMax = -32768;
  bStartMs = millis();
}

static void biasReset() {
  bSum = 0; bN = 0; bMin = 32767; bMax = -32768;
  bStartMs = millis();
}

void gyroService(bool idle) {
  if (!gyroOk) return;
  uint32_t now = micros();
  if (now - gLastUs < GYRO_PERIOD_US) return;
  float dt = (now - gLastUs) * 1e-6f;
  gLastUs = now;

  int16_t raw;
  if (!mpuReadGz(raw)) {
    if (++gErrors >= 20) {
      gyroOk = false;
      Log.println("[GYRO] Lost contact with the MPU-6050, gyro disabled until restart");
    }
    return;
  }
  gErrors = 0;

  if (idle) biasTrack(raw);
  else biasReset();

  float r = (raw - gyroBias) / GYRO_LSB_PER_DPS;
  if (GYRO_INVERT) r = -r;
  gRate = r;
  if (dt < 0.05f) gHeading += r * dt;                // skip big gaps (blocking calls)
}

bool  gyroReady()        { return gyroOk; }
bool  gyroOffsetValid()  { return biasValid; }
float gyroRate()         { return gRate; }
float gyroHeading()      { return gHeading; }
void  gyroResetHeading() { gHeading = 0; }

bool gyroCalibrate() {
  if (!gyroOk) { Log.println("[GYRO] Not available"); return false; }
  int32_t sum = 0; int16_t mn = 32767, mx = -32768; int n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 500) {
    int16_t raw;
    if (mpuReadGz(raw)) { sum += raw; mn = min(mn, raw); mx = max(mx, raw); n++; }
    delay(1);
  }
  if (n < 100 || mx - mn > GYRO_STILL_RANGE * 2) {
    Log.println("[GYRO] Robot moved during the measurement, try again with it still");
    return false;
  }
  gyroBias = (float)sum / n;
  biasValid = true;
  gLastUs = micros();
  Log.printf("[GYRO] Offset measured: %.1f raw\n", gyroBias);
  return true;
}

void printGyro() {
  if (!gyroOk) { Log.println("[GYRO] Not available"); return; }
  Log.printf("[GYRO] rate=%.1f deg/s  heading=%.1f deg  offset=%.1f raw (%s)\n",
             gRate, gHeading, gyroBias, biasValid ? "measured" : "NOT measured yet");
}

// ============================================================================
// CALIBRATION
// ============================================================================
static constexpr uint32_t CAL_MAGIC = 0x4C465231;  // "LFR1"
static Preferences prefs;

void setDefaultCalibration() {
  cal.magic = CAL_MAGIC;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    cal.minV[i] = 0;
    cal.maxV[i] = 4095;
  }
  for (uint8_t i = 0; i < 2; i++) {
    cal.sideMin[i] = 0;
    cal.sideMax[i] = 4095;
  }
}

void saveCalibration() {
  prefs.begin("lfr", false);
  prefs.putBytes("cal", &cal, sizeof(cal));
  prefs.end();
}

void loadCalibration() {
  prefs.begin("lfr", true);
  size_t n = prefs.getBytes("cal", &cal, sizeof(cal));
  prefs.end();
  if (n != sizeof(cal) || cal.magic != CAL_MAGIC) {
    Log.println("[CAL] No saved calibration, using defaults. Please calibrate.");
    setDefaultCalibration();
  } else {
    Log.println("[CAL] Calibration loaded from flash.");
  }
}

void printCalibration() {
  Log.println("[CAL] sensor: min / max");
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
    Log.printf("  S%u: %4u / %4u\n", i, cal.minV[i], cal.maxV[i]);
  Log.printf("  Side L: %4u / %4u\n", cal.sideMin[0], cal.sideMax[0]);
  Log.printf("  Side R: %4u / %4u\n", cal.sideMin[1], cal.sideMax[1]);
}

// Record min/max of every sensor (called continuously while calibrating)
static void calSample() {
  readRawSensors();
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    cal.minV[i] = min(cal.minV[i], raw[i]);
    cal.maxV[i] = max(cal.maxV[i], raw[i]);
  }
  uint16_t sl = analogRead(PIN_SIDE_LEFT), sr = analogRead(PIN_SIDE_RIGHT);
  cal.sideMin[0] = min(cal.sideMin[0], sl);
  cal.sideMax[0] = max(cal.sideMax[0], sl);
  cal.sideMin[1] = min(cal.sideMin[1], sr);
  cal.sideMax[1] = max(cal.sideMax[1], sr);
}

// Turn by angle using the gyro: +A, -A, +A, -A ... then back to 0.
// Returns false if it had to give up (timeout or turning the wrong way).
static bool calibrateByAngle() {
  // The robot is still right now: good moment to measure the gyro offset
  gyroCalibrate();
  gyroResetHeading();

  const int steps = CAL_SWEEPS * 2 + 1;              // last step returns to 0
  uint32_t t0 = millis();
  float    cmd = 0;                                  // current turn command, + = left
  uint32_t lastUs = micros();

  for (int step = 0; step < steps; step++) {
    float target = (step == steps - 1) ? 0.0f
                 : (step % 2 == 0 ? CAL_SWEEP_DEG : -CAL_SWEEP_DEG);
    Log.printf("[CAL] Turning to %+.0f deg\n", target);

    while (true) {
      gyroService(false);
      calSample();
      float h   = gyroHeading();
      float err = target - h;
      if (fabsf(err) <= CAL_ANGLE_TOL) {
        // Target reached: stop and wait a moment before turning back
        motorsBrake();
        cmd = 0;
        uint32_t p0 = millis();
        while (millis() - p0 < CAL_PAUSE_MS) { gyroService(false); calSample(); delay(1); }
        lastUs = micros();
        break;
      }

      if (millis() - t0 > CAL_GYRO_TIMEOUT_MS) {
        Log.println("[CAL] Gyro calibration timed out");
        return false;
      }
      if (fabsf(h) > CAL_SWEEP_DEG + 90.0f) {         // spinning away: gyro sign is wrong
        Log.println("[CAL] Robot turned too far: check GYRO_INVERT in config.h");
        return false;
      }

      // PD on the angle. Sign: + = turn left (heading goes up).
      //   P: proportional to the angle still to turn
      //   D: opposes the current turning speed, so the robot starts braking
      //      before it reaches the target instead of arriving fast
      float rate = gyroRate();
      float want = CAL_ANGLE_KP * err - CAL_ANGLE_KD * rate;
      want = constrain(want, (float)-CAL_SPEED, (float)CAL_SPEED);

      // Not moving yet and the command is too weak to start the motors:
      // give it the minimum speed toward the target
      if (fabsf(rate) < CAL_STALL_DPS && want * err > 0 && fabsf(want) < CAL_MIN_SPEED)
        want = (err > 0) ? CAL_MIN_SPEED : -CAL_MIN_SPEED;

      // Change the command gradually: speeding up is limited by CAL_ACCEL,
      // slowing down by CAL_DECEL (faster, so it can still stop on target)
      uint32_t now = micros();
      float dtMs = (now - lastUs) / 1000.0f;
      lastUs = now;
      bool slowingDown = fabsf(want) < fabsf(cmd) || (want * cmd < 0);
      float maxStep = (slowingDown ? CAL_DECEL : CAL_ACCEL) * dtMs;
      cmd += constrain(want - cmd, -maxStep, maxStep);

      setMotors(-(int)cmd, (int)cmd);
      delay(1);
    }
  }
  Log.printf("[CAL] Back at %.1f deg\n", gyroHeading());
  return true;
}

// Original method: turn for a fixed time (used without a gyro)
static void calibrateByTime() {
  uint32_t t0 = millis();
  while (millis() - t0 < CAL_TIME_MS) {
    if (CAL_AUTO_SPIN) {
      // Half swing right, then full swings L, R, L..., ending with half a
      // swing right -> ends where it started if CAL_TIME_MS is a multiple
      // of 2 x CAL_SWING_MS. Speed ramps down before every reversal.
      uint32_t t = millis() - t0;
      uint32_t s = (t + CAL_SWING_MS / 2) % CAL_SWING_MS;  // time into segment
      int dir = (((t + CAL_SWING_MS / 2) / CAL_SWING_MS) % 2 == 0) ? 1 : -1;
      float k = 1.0f;
      k = min(k, (float)s / CAL_RAMP_MS);                   // ramp up after reversal
      k = min(k, (float)(CAL_SWING_MS - s) / CAL_RAMP_MS);  // ramp down before reversal
      k = min(k, (float)t / CAL_RAMP_MS);                   // gentle start
      k = min(k, (float)(CAL_TIME_MS - t) / CAL_RAMP_MS);   // gentle stop
      int sp = (int)(CAL_SPEED * max(k, 0.0f));
      setMotors(dir * sp, -dir * sp);
    }
    calSample();
    delay(2);
  }
}

void runCalibration() {
  Log.println("[CAL] Calibrating...");

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    cal.minV[i] = 4095;
    cal.maxV[i] = 0;
  }
  for (uint8_t i = 0; i < 2; i++) {
    cal.sideMin[i] = 4095;
    cal.sideMax[i] = 0;
  }

  if (CAL_AUTO_SPIN && CAL_USE_GYRO && gyroReady()) {
    if (!calibrateByAngle()) {
      motorsBrake();
      delay(100);
      motorsCoast();
      Log.println("[CAL] Falling back to the time-based spin");
      calibrateByTime();
    }
  } else {
    calibrateByTime();
  }

  motorsBrake();
  delay(100);
  motorsCoast();

  bool ok = true;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (cal.maxV[i] - cal.minV[i] < CAL_MIN_RANGE) {
      Log.printf("[CAL] WARNING: sensor %u range too small\n", i);
      ok = false;
    }
  }
  for (uint8_t i = 0; i < 2; i++) {
    if (cal.sideMax[i] - cal.sideMin[i] < CAL_MIN_RANGE) {
      Log.printf("[CAL] WARNING: side sensor %s didn't see the line, "
                 "using full-scale range\n",
                 i ? "R" : "L");
      cal.sideMin[i] = 0;
      cal.sideMax[i] = 4095;
    }
  }

  cal.magic = CAL_MAGIC;
  saveCalibration();
  printCalibration();
  Log.println(ok ? "[CAL] Done, saved." : "[CAL] Done with warnings, saved.");
}

// ============================================================================
// TRACK MAP
// The track is split into sections at every mark (left or right). For each
// section the mapping run stores its length and how much the two wheels
// differed: on a straight both wheels travel the same distance.
// ============================================================================
struct Segment {
  float lenMm;
  float diffMm;       // left - right wheel distance (+ = left went further)
  float angleDeg;     // turned during the section (gyro), + = left
  bool  hasAngle;     // angle was measured (gyro working)
  uint16_t avgSpeed;  // mm/s
  bool straight;
};
struct TrackMap {
  uint32_t magic;
  uint16_t count;
  Segment seg[MAP_MAX_SEGMENTS];
};
static constexpr uint32_t MAP_MAGIC = 0x4C464D33;  // "LFM3" (old maps are discarded)

static TrackMap trackMap;  // map in use (from flash or the last mapping run)
static TrackMap rec;       // being recorded during a mapping run

int straightBoost = DEFAULT_STRAIGHT_BOOST;

int straightSpeed() {
  return min(baseSpeed + straightBoost, PWM_MAX);
}
bool fastEnabled = true;
static int finishMarks = FINISH_MARKS;

bool mapReady() {
  return trackMap.magic == MAP_MAGIC && trackMap.count > 0;
}

static void saveMap() {
  prefs.begin("lfr", false);
  prefs.putBytes("map", &trackMap, sizeof(trackMap));
  prefs.end();
}

static void loadSettings() {
  prefs.begin("lfr", true);
  size_t n = prefs.getBytes("map", &trackMap, sizeof(trackMap));
  finishMarks = prefs.getInt("fm", FINISH_MARKS);
  straightBoost = prefs.getInt("vb", DEFAULT_STRAIGHT_BOOST);
  prefs.end();
  if (n != sizeof(trackMap) || trackMap.magic != MAP_MAGIC) trackMap.count = 0;
  Log.printf("[MAP] %s. Finish marks: %d, straights: base + %d\n",
             mapReady() ? "Map loaded" : "No map, next run will map the track",
             finishMarks, straightBoost);
}

int finishMarkCount() {
  return finishMarks;
}

void setFinishMarks(int n) {
  finishMarks = constrain(n, 1, 20);
  prefs.begin("lfr", false);
  prefs.putInt("fm", finishMarks);
  prefs.end();
  Log.printf("[MAP] Finish marks = %d (saved)\n", finishMarks);
}

void setStraightBoost(int v) {
  straightBoost = constrain(v, 0, PWM_MAX);
  prefs.begin("lfr", false);
  prefs.putInt("vb", straightBoost);
  prefs.end();
  Log.printf("[MAP] Straights = base + %d = %d (saved)\n", straightBoost, straightSpeed());
}

// Straight if both wheels travelled the same distance within the tolerance
static float straightToleranceMm(float len) {
  return STRAIGHT_TOL_MM + len * STRAIGHT_TOL_PCT / 100.0f;
}

void clearMap() {
  trackMap.magic = 0;
  trackMap.count = 0;
  saveMap();
  Log.println("[MAP] Map cleared, next run will map the track");
}

void printMap() {
  if (!mapReady()) {
    Log.println("[MAP] No map yet");
    return;
  }
  Log.printf("[MAP] %u sections:\n", trackMap.count);
  for (uint16_t i = 0; i < trackMap.count; i++) {
    const Segment &g = trackMap.seg[i];
    if (g.hasAngle) {
      float rad = fabsf(g.angleDeg) * PI / 180.0f;
      if (rad > 0.02f)
        Log.printf("  #%u %5.0f mm  %+6.1f deg  R=%5.0f mm  %4u mm/s  %s\n", i, g.lenMm,
                   g.angleDeg, g.lenMm / rad, g.avgSpeed, g.straight ? "STRAIGHT" : "curve");
      else
        Log.printf("  #%u %5.0f mm  %+6.1f deg  R=   --     %4u mm/s  %s\n", i, g.lenMm,
                   g.angleDeg, g.avgSpeed, g.straight ? "STRAIGHT" : "curve");
    } else {
      Log.printf("  #%u %5.0f mm  L-R %+5.0f mm (tol +/-%.0f)  %4u mm/s  %s\n", i, g.lenMm,
                 g.diffMm, straightToleranceMm(g.lenMm), g.avgSpeed,
                 g.straight ? "STRAIGHT" : "curve");
    }
  }
}

// ============================================================================
// SIDE MARKERS
// ============================================================================
// Right marks = start/finish, left marks = curves.
// Crossings (both sides or most of the array at once) are ignored.
enum class MarkEvent { None,
                       Left,
                       Right };

static bool leftOn = false, rightOn = false;
static bool winOpen = false, winLeft = false, winRight = false, winCross = false;

static void markersReset() {
  leftOn = rightOn = false;
  winOpen = winLeft = winRight = winCross = false;
}

static MarkEvent markersUpdate() {
  readSideSensors();

  // Hysteresis
  if (!leftOn && sideNorm[0] > SIDE_ON) leftOn = true;
  if (leftOn && sideNorm[0] < SIDE_OFF) leftOn = false;
  if (!rightOn && sideNorm[1] > SIDE_ON) rightOn = true;
  if (rightOn && sideNorm[1] < SIDE_OFF) rightOn = false;

  // A window opens when any side sees a mark and closes when both are clear
  if (leftOn || rightOn) {
    if (!winOpen) {
      winOpen = true;
      winLeft = winRight = winCross = false;
    }
    winLeft |= leftOn;
    winRight |= rightOn;
    if ((leftOn && rightOn) || countActiveSensors() >= CROSS_SENSOR_COUNT)
      winCross = true;
  } else if (winOpen) {
    winOpen = false;
    if (!winCross) {
      if (winRight && !winLeft) return MarkEvent::Right;
      if (winLeft && !winRight) return MarkEvent::Left;
    }
  }
  return MarkEvent::None;
}

// ============================================================================
// CONTROLLER
// ============================================================================
float Kp = DEFAULT_KP;
float Ki = DEFAULT_KI;
float Kd = DEFAULT_KD;
int baseSpeed = DEFAULT_BASE_SPEED;

struct section shortcut = {
  SHORTCUT1_START, SHORTCUT1_END
};
float scKp = SHORTCUT_KP, scKi = SHORTCUT_KI, scKd = SHORTCUT_KD;

// Normal gains are saved when the shortcut starts and ALWAYS restored when it
// ends, when the run stops, or when a new run starts (so a run that stops in
// the middle of the shortcut can't leave the shortcut gains active).
static bool  shortcutActive = false;
static float savedKp, savedKi, savedKd;

static void shortcutEnter() {
  if (shortcutActive) return;
  savedKp = Kp; savedKi = Ki; savedKd = Kd;
  Kp = scKp; Ki = scKi; Kd = scKd;
  shortcutActive = true;
  Log.printf("[SHORTCUT] start: Kp=%.4f Ki=%.5f Kd=%.4f\n", Kp, Ki, Kd);
}

static void shortcutExit() {
  if (!shortcutActive) return;
  Kp = savedKp; Ki = savedKi; Kd = savedKd;
  shortcutActive = false;
  Log.printf("[SHORTCUT] end: gains restored (Kp=%.4f Ki=%.5f Kd=%.4f)\n", Kp, Ki, Kd);
}

// Drive with a steering correction. If the faster wheel would go past full
// PWM, both wheels are shifted down so the difference between them is kept.
static void driveWithCorrection(int base, int corr) {
  int l = base + corr;
  int r = base - corr;
  if (PRESERVE_STEERING) {
    int over = max(l, r) - PWM_MAX;
    if (over > 0) { l -= over; r -= over; }
    int under = -PWM_MAX - min(l, r);
    if (under > 0) { l += under; r += under; }
  }
  setMotors(l, r);
}

static RobotState state = RobotState::Idle;
static uint32_t countdownStart = 0;
static uint32_t countdownMs = START_DELAY_MS;
static uint32_t lastLoopUs = 0;
static float lastErr = 0, integral = 0;
static bool lineLost = false;
static uint32_t lostSince = 0;
static int rightMarks = 0, leftMarks = 0;
static float finishStartMm = 0;
static float lastRightMarkMm = -1e9f, lastLeftMarkMm = -1e9f;
static float holdUntilMm = -1e9f;  // drive straight until here (crossroads)
static uint32_t holdStartMs = 0;

// Track map, per run
static bool mappingRun = false;  // recording the map this run
static bool fastRun = false;     // using the map this run
static int segIdx = 0;           // section the robot is in
static int32_t segStartL = 0, segStartR = 0;
static float segStartMm = 0;
static uint32_t segStartMs = 0;
static float segStartDeg = 0;
static int targetSpeed = DEFAULT_BASE_SPEED;


// ---------------------------------------------------------------------------
static void startRun() {
  shortcutExit();
  resetEncoders();
  markersReset();
  resetLinePosition();
  rightMarks = leftMarks = 0;
  lastRightMarkMm = lastLeftMarkMm = -1e9f;
  integral = 0;
  lastErr = 0;
  lineLost = false;
  holdUntilMm = -1e9f;

  mappingRun = !mapReady();
  fastRun = !mappingRun && fastEnabled;
  rec.magic = MAP_MAGIC;
  rec.count = 0;
  segIdx = 0;
  segStartL = segStartR = 0;
  segStartMm = 0;
  segStartMs = millis();
  gyroResetHeading();
  segStartDeg = 0;
  targetSpeed = baseSpeed;
  if (gyroReady() && !gyroOffsetValid())
    Log.println("[GYRO] WARNING: offset not measured yet, angles may drift (keep robot still before starting)");

  lastLoopUs = micros();
  state = RobotState::Running;
  if (mappingRun) Log.println("[RUN] GO! (mapping run)");
  else if (fastRun) Log.printf("[RUN] GO! (fast run, %u sections mapped)\n", trackMap.count);
  else Log.println("[RUN] GO! (map not used)");
}

// Called at every accepted mark: ends the current section and starts the next
static void closeSegment() {
  int32_t l, r;
  readEncoders(l, r);
  float dL = (l - segStartL) * MM_PER_TICK;
  float dR = (r - segStartR) * MM_PER_TICK;
  float len = (dL + dR) * 0.5f;
  uint32_t ms = millis() - segStartMs;

  Segment g;
  g.lenMm = len;
  g.diffMm = dL - dR;
  g.hasAngle = gyroReady();
  g.angleDeg = g.hasAngle ? gyroHeading() - segStartDeg : 0;
  g.avgSpeed = ms ? (uint16_t)(len * 1000.0f / ms) : 0;
  // With the gyro: straight if it turned less than STRAIGHT_MAX_DEG.
  // Without it: straight if both wheels travelled the same distance (+/- tol).
  bool flat = g.hasAngle ? fabsf(g.angleDeg) <= STRAIGHT_MAX_DEG
                         : fabsf(g.diffMm) <= straightToleranceMm(len);
  g.straight = len >= STRAIGHT_MIN_MM && flat;

  if (mappingRun && rec.count < MAP_MAX_SEGMENTS) rec.seg[rec.count++] = g;

  if (g.hasAngle)
    Log.printf("[SEG] #%d %.0f mm %+.1f deg %u mm/s %s\n", segIdx, len,
               g.angleDeg, g.avgSpeed, g.straight ? "STRAIGHT" : "curve");
  else
    Log.printf("[SEG] #%d %.0f mm L-R %+.0f mm (tol +/-%.0f) %u mm/s %s\n", segIdx, len,
               g.diffMm, straightToleranceMm(len), g.avgSpeed,
               g.straight ? "STRAIGHT" : "curve");

  // In a fast run, check the track still matches the map
  if (fastRun) {
    bool bad = segIdx >= trackMap.count;
    if (!bad) {
      const Segment &m = trackMap.seg[segIdx];
      bad = fabsf(len - m.lenMm) > MAP_TOLERANCE * m.lenMm + 50.0f;
      if (!bad && g.hasAngle && m.hasAngle) {
        float tol = max(MAP_ANGLE_TOL_DEG, 0.25f * fabsf(m.angleDeg));
        bad = fabsf(g.angleDeg - m.angleDeg) > tol;
      }
    }
    if (bad) {
      fastRun = false;
      Log.printf("[MAP] Section #%d doesn't match the map, base speed for the rest of the run\n", segIdx);
    }
  }

  segIdx++;
  segStartL = l;
  segStartR = r;
  segStartMm = (l + r) * 0.5f * MM_PER_TICK;
  segStartMs = millis();
  segStartDeg = gyroHeading();
}

// Speed for this moment: faster on mapped straights, with a ramp up after
// entering and back down to base speed before the section ends.
static int computeTargetSpeed(float dist) {
  if (!fastRun || state != RobotState::Running || segIdx >= trackMap.count)
    return baseSpeed;
  const Segment &g = trackMap.seg[segIdx];
  int fast = straightSpeed();
  if (!g.straight || fast <= baseSpeed) return baseSpeed;

  float in = dist - segStartMm;
  float remaining = g.lenMm - in;
  float up = in / STRAIGHT_RAMP_MM;
  float down = (remaining - STRAIGHT_BRAKE_MARGIN_MM) / STRAIGHT_BRAKE_MM;
  float f = constrain(min(up, down), 0.0f, 1.0f);
  return baseSpeed + (int)((fast - baseSpeed) * f);
}

static void onRightMark() {
  float d = distanceMm();
  if (d - lastRightMarkMm < MARK_MIN_SPACING_MM) return;  // same mark / noise
  lastRightMarkMm = d;
  rightMarks++;
  closeSegment();
  Log.printf("[MARK] Right #%d at %.0f mm\n", rightMarks, d);
  if (rightMarks >= finishMarks && state == RobotState::Running) {
    state = RobotState::Finishing;
    finishStartMm = d;
    Log.println("[RUN] Finish line detected");
  }
}

static void onLeftMark() {
  float d = distanceMm();
  if (d - lastLeftMarkMm < MARK_MIN_SPACING_MM) return;  // same mark / noise
  lastLeftMarkMm = d;
  leftMarks++;
  closeSegment();
  Log.printf("[MARK] Left #%d at %.0f mm\n", leftMarks, d);

  if (leftMarks == shortcut.start) shortcutEnter();
  else if (leftMarks == shortcut.end) shortcutExit();
}

// A crossroads shows the line under BOTH ends of the array (or very many
// sensors at once). A 90-degree corner only fills one side, so it isn't confused.
static bool isCrossing() {
  bool leftEnd = norm[0] > 500;
  bool rightEnd = norm[NUM_SENSORS - 1] > 500;
  return (leftEnd && rightEnd) || countActiveSensors() >= CROSS_SENSOR_COUNT;
}

static void controlStep() {
  bool onLine;
  int32_t pos = readLinePosition(onLine);
  float dist = distanceMm();
  targetSpeed = computeTargetSpeed(dist);

  bool holding = CROSS_ENABLED && dist < holdUntilMm && millis() - holdStartMs < CROSS_HOLD_MAX_MS;
  if (CROSS_ENABLED && onLine && isCrossing()) {
    if (!holding) {
      holdStartMs = millis();
      Log.printf("[CROSS] at %.0f mm\n", dist);
    }
    holdUntilMm = dist + CROSS_HOLD_MM;
    holding = true;
  }

  if (holding) {
    lastPos = CENTER_POS;  // don't let the crossing bias the next reading
    lineLost = false;
    setMotors(targetSpeed, targetSpeed);
  } else {
    // Line-lost safety
    if (!onLine) {
      if (!lineLost) {
        lineLost = true;
        lostSince = millis();
      } else if (millis() - lostSince > LOST_TIMEOUT_MS) {
        requestStop("line lost");
        return;
      }
    } else {
      lineLost = false;
    }

    // PID
    float err = (float)(pos - CENTER_POS);
    integral = constrain(integral + err, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float deriv = err - lastErr;
    lastErr = err;
    float corr = Kp * err + Ki * integral + Kd * deriv;

    // Line to the right (pos > center) -> left wheel faster
    driveWithCorrection(targetSpeed, (int)corr);
  }

  switch (markersUpdate()) {
    case MarkEvent::Right: onRightMark(); break;
    case MarkEvent::Left: onLeftMark(); break;
    default: break;
  }

  if (state == RobotState::Finishing && distanceMm() - finishStartMm >= STOP_DISTANCE_MM) {
    if (mappingRun && rec.count > 0) {
      trackMap = rec;
      saveMap();
      mappingRun = false;
      Log.printf("[MAP] Lap complete, %u sections saved. Next runs will use it.\n", trackMap.count);
    }
    requestStop("finish");
  }
}

// ---------------------------------------------------------------------------
void controllerUpdate() {
  gyroService(state == RobotState::Idle);
  switch (state) {
    case RobotState::Countdown:
      if (millis() - countdownStart >= countdownMs) startRun();
      break;

    case RobotState::Running:
    case RobotState::Finishing:
      {
        uint32_t now = micros();
        if (now - lastLoopUs >= LOOP_PERIOD_US) {
          lastLoopUs += LOOP_PERIOD_US;
          if (now - lastLoopUs > 10 * LOOP_PERIOD_US) lastLoopUs = now;  // resync
          controlStep();
        }
        break;
      }

    default:
      break;
  }
}

void requestStart(const char *source, uint32_t delayMs) {
  if (state != RobotState::Idle) return;
  state = RobotState::Countdown;
  countdownStart = millis();
  countdownMs = delayMs;
  Log.printf("[RUN] Start requested by %s (in %lu ms)\n", source, (unsigned long)delayMs);
}

void requestStop(const char *reason) {
  motorsBrake();
  delay(150);
  motorsCoast();
  shortcutExit();
  if (state == RobotState::Running || state == RobotState::Finishing)
    Log.printf("[RUN] Stopped (%s). Distance: %.0f mm, marks R=%d L=%d\n",
               reason, distanceMm(), rightMarks, leftMarks);
  else if (state == RobotState::Countdown)
    Log.println("[RUN] Start cancelled");
  state = RobotState::Idle;
}

void requestCalibration(const char *source) {
  if (state != RobotState::Idle) return;
  Log.printf("[CAL] Requested by %s\n", source);
  state = RobotState::Calibrating;
  runCalibration();
  state = RobotState::Idle;
}

RobotState robotState() {
  return state;
}
bool isRunning() {
  return state == RobotState::Running || state == RobotState::Finishing;
}
float lastError() {
  return lastErr;
}
int rightMarkCount() {
  return rightMarks;
}
int leftMarkCount() {
  return leftMarks;
}
int currentSegment() {
  return segIdx;
}
int currentTargetSpeed() {
  return targetSpeed;
}

void robotBegin() {
  sensorsBegin();
  motorsBegin();
  encodersBegin();
  gyroBegin();
  loadCalibration();
  loadSettings();
}
