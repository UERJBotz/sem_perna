#pragma once
/*
  robot.h - Everything the robot does: motors, encoders, sensors,
  calibration, marker detection and the PID state machine.
*/
#include <Arduino.h>
#include "config.h"

// Console output: goes to USB Serial and, when a phone is connected, to BLE.
// Defined in line_follower.ino.
extern Print &Log;

// ============================================================================
// TYPES
// ============================================================================
struct section {
  int start, end;
};

// ============================================================================
// MOTORS
// ============================================================================
void motorsBegin();
void setMotors(int left, int right);   // -PWM_MAX .. +PWM_MAX
void motorsBrake();                    // short brake
void motorsCoast();                    // free wheel

// ============================================================================
// ENCODERS
// ============================================================================
void  encodersBegin();
void  readEncoders(int32_t &left, int32_t &right);  // ticks, sign-corrected
void  resetEncoders();
float distanceMm();                                 // average of both wheels

// ============================================================================
// SENSORS
// ============================================================================
struct CalData {
  uint32_t magic;
  uint16_t minV[NUM_SENSORS], maxV[NUM_SENSORS];
  uint16_t sideMin[2], sideMax[2];
};

extern CalData  cal;                    // active calibration
extern uint16_t raw[NUM_SENSORS];       // last raw ADC readings
extern uint16_t norm[NUM_SENSORS];      // last normalized readings (0..1000)
extern uint16_t sideNorm[2];            // [0] = left, [1] = right (0..1000)

void    sensorsBegin();
void    readRawSensors();
void    readSideSensors();
int32_t readLinePosition(bool &onLine); // 0 .. 7000, CENTER_POS = centered
int32_t lastLinePosition();
void    resetLinePosition();
uint8_t countActiveSensors();

// ============================================================================
// GYRO (MPU-6050)
// ============================================================================
void  gyroBegin();
void  gyroService(bool idle);            // reads the gyro (called from controllerUpdate)
bool  gyroReady();                       // found and working
bool  gyroOffsetValid();                 // offset measured since power-on
float gyroRate();                        // deg/s, + = turning left (counter-clockwise)
float gyroHeading();                     // deg turned since the last reset
void  gyroResetHeading();
bool  gyroCalibrate();                   // blocking ~0.5 s, robot must be still
void  printGyro();

// ============================================================================
// CALIBRATION (stored in flash)
// ============================================================================
void loadCalibration();
void saveCalibration();
void setDefaultCalibration();
void printCalibration();
void runCalibration();                  // blocking, moves the robot if CAL_AUTO_SPIN

// ============================================================================
// CONTROLLER (state machine + PID)
// ============================================================================
enum class RobotState { Idle, Countdown, Running, Finishing, Calibrating };

extern struct section shortcut;        // left-mark numbers where the shortcut gains start/end
extern float scKp, scKi, scKd;          // gains used inside the shortcut
extern float Kp, Ki, Kd;                // live-tunable
extern int   baseSpeed;

void       robotBegin();                // initializes all hardware above
void       controllerUpdate();          // call every loop()
void       requestStart(const char *source, uint32_t delayMs = START_DELAY_MS);
void       requestStop(const char *reason);
void       requestCalibration(const char *source);
RobotState robotState();
bool       isRunning();                 // Running or Finishing
float      lastError();
int        rightMarkCount();
int        leftMarkCount();

// ============================================================================
// TRACK MAP (first run records sections between marks, later runs use it)
// ============================================================================
extern int  straightBoost;              // straights run at baseSpeed + this
extern bool fastEnabled;                // false = never use the map

int  finishMarkCount();
void setFinishMarks(int n);             // saved in flash
void setStraightBoost(int v);           // saved in flash
int  straightSpeed();                   // baseSpeed + straightBoost, limited to PWM_MAX
bool mapReady();
void clearMap();                        // next run will map again
void printMap();
int  currentSegment();
int  currentTargetSpeed();
