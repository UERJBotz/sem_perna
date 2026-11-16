#pragma once
/*
  config.h - Pins and all tunable settings in one place.
*/
#include <Arduino.h>

// ============================================================================
// PINS (from the schematic)
// ============================================================================
constexpr uint8_t PIN_MUX_ADC    = 36;  // SENSOR_VP  <- 74HC4051 common output
constexpr uint8_t PIN_MUX_S0     = 17;
constexpr uint8_t PIN_MUX_S1     = 16;
constexpr uint8_t PIN_MUX_S2     = 4;
constexpr uint8_t PIN_SIDE_LEFT  = 34;  // Left_Sensor  (input only, analog)
constexpr uint8_t PIN_SIDE_RIGHT = 35;  // Right_Sensor (input only, analog)

constexpr uint8_t PIN_R_MOTOR_1  = 33;
constexpr uint8_t PIN_R_MOTOR_2  = 25;
constexpr uint8_t PIN_L_MOTOR_1  = 26;
constexpr uint8_t PIN_L_MOTOR_2  = 27;

constexpr uint8_t PIN_L_ENC_A    = 21;
constexpr uint8_t PIN_L_ENC_B    = 22;
constexpr uint8_t PIN_R_ENC_A    = 18;
constexpr uint8_t PIN_R_ENC_B    = 19;

constexpr uint8_t PIN_BTN_START  = 15;  // ALT_START     (strapping pin)
constexpr uint8_t PIN_BTN_CALIB  = 5;   // ALT_CALIBRATE (strapping pin)
constexpr uint8_t PIN_IR         = 13;
constexpr uint8_t PIN_SDA        = 23;  // SDA_MPU
constexpr uint8_t PIN_SCL        = 32;  // SCL_MPU

// Buttons connect the pin to 3V3 when pressed -> internal pull-down, active HIGH.
// Set false if a button connects to GND instead (internal pull-up, active LOW).
constexpr bool    BUTTONS_ACTIVE_HIGH = true;

// ============================================================================
// SENSORS
// ============================================================================
constexpr uint8_t  NUM_SENSORS    = 8;
constexpr int32_t  CENTER_POS     = (NUM_SENSORS - 1) * 1000 / 2;  // 3500

// true if the LINE gives LOWER raw readings than the background
constexpr bool     INVERT_SENSORS = true;

// Mux channel of each physical sensor, ordered LEFT -> RIGHT
constexpr uint8_t  SENSOR_CHANNEL[NUM_SENSORS] = {0, 1, 2, 3, 4, 5, 6, 7};

constexpr uint32_t MUX_SETTLE_US      = 8;
constexpr uint16_t LINE_ON_THRESHOLD  = 200;  // normalized 0..1000
constexpr uint16_t LINE_NOISE_FLOOR   = 50;

// Side markers
constexpr uint16_t SIDE_ON            = 750;  // hysteresis, normalized
constexpr uint16_t SIDE_OFF           = 500;
constexpr uint8_t  CROSS_SENSOR_COUNT = 6;    // this many array sensors on = crossing

// ============================================================================
// CALIBRATION
// ============================================================================
constexpr bool     CAL_AUTO_SPIN  = true;   // false = slide the robot by hand
constexpr uint32_t CAL_TIME_MS    = 6000;   // multiple of 2 x CAL_SWING_MS -> ends where it started
constexpr int      CAL_SPEED      = 600;    // lower = less heat (calibration doesn't need speed)
constexpr uint32_t CAL_RAMP_MS    = 100;    // ramp time before/after each reversal (< CAL_SWING_MS / 2)
constexpr uint16_t CAL_MIN_RANGE  = 300;    // warn if a sensor's range is smaller
constexpr uint32_t CAL_SWING_MS   = 1000;   // time turning to each side before reversing
constexpr uint32_t CAL_BUTTON_DELAY_MS = 1000; // wait after pressing CALIBRATE (time to remove your hand)
// Gyro calibration: instead of turning for a fixed time, the robot turns to
// +CAL_SWEEP_DEG, then -CAL_SWEEP_DEG, CAL_SWEEPS times, and returns to 0 deg,
// so every sensor passes over the line the same way. Without a working gyro
// the time-based spin above is used.
constexpr bool     CAL_USE_GYRO   = true;
constexpr float    CAL_SWEEP_DEG  = 90.0f;  // angle to each side of the starting direction
constexpr int      CAL_SWEEPS     = 5;      // left-right cycles
constexpr float    CAL_ANGLE_KP   = 13.0f;  // PWM per degree still to turn
constexpr float    CAL_ANGLE_KD   = 0.5f;   // PWM per deg/s of turning speed (brakes on approach)
constexpr float    CAL_STALL_DPS  = 20.0f;  // below this turning speed the robot counts as stopped
constexpr int      CAL_MIN_SPEED  = 300;    // slowest turning PWM that still moves the robot
constexpr float    CAL_ANGLE_TOL  = 3.0f;   // target reached when within this many degrees
constexpr uint32_t CAL_GYRO_TIMEOUT_MS = 12000; // give up if it takes longer than this
// Smooth direction changes (avoid wheel slip): stop briefly at each target and
// limit how fast the turning speed can change.
constexpr uint32_t CAL_PAUSE_MS   = 120;    // stopped at each target angle
constexpr float    CAL_ACCEL      = 2.0f;   // max speed increase, PWM per ms (0 -> 500 in 250 ms)
constexpr float    CAL_DECEL      = 5.0f;   // max speed decrease, PWM per ms

// ============================================================================
// MOTORS (RZ7889: 2 inputs per motor)
// ============================================================================
constexpr bool     L_MOTOR_INVERT = true;
constexpr bool     R_MOTOR_INVERT = true;
constexpr bool     SLOW_DECAY     = true;   // brake between PWM pulses (more linear)
// When base speed + correction goes above full PWM, shift BOTH wheels down so
// the speed difference (the steering) is kept. Without this, at high base
// speed the fast wheel is clipped and the robot steers weaker than the PID asked.
constexpr bool     PRESERVE_STEERING = true;

constexpr uint32_t PWM_FREQ       = 2000;  // Hz
constexpr uint8_t  PWM_RES        = 10;     // bits
constexpr int      PWM_MAX        = (1 << PWM_RES) - 1;

// ============================================================================
// ENCODERS
// ============================================================================
constexpr bool  L_ENC_INVERT      = true;
constexpr bool  R_ENC_INVERT      = false;

constexpr float ENC_PPR           = 7.0f;   // pulses per motor-shaft rev (one channel)
constexpr float GEAR_RATIO        = 51.5f;
constexpr float WHEEL_DIAMETER_MM = 32.0f;
// Every CHANGE of channel A is counted -> 2 ticks per pulse
constexpr float TICKS_PER_REV     = ENC_PPR * 2.0f * GEAR_RATIO;
constexpr float MM_PER_TICK       = (PI * WHEEL_DIAMETER_MM) / TICKS_PER_REV;

// ============================================================================
// CONTROL
// ============================================================================
constexpr float    DEFAULT_KP         = 0.34f;
constexpr float    DEFAULT_KI         = 0.0001f;
constexpr float    DEFAULT_KD         = 0.007f;
constexpr int      DEFAULT_BASE_SPEED = 950;     // 0..PWM_MAX

constexpr uint32_t LOOP_PERIOD_US     = 1000;    // 1 kHz control loop
constexpr float    INTEGRAL_LIMIT     = 50000.0f;
constexpr uint32_t START_DELAY_MS     = 5000;    // button / console: time to remove your hand
constexpr uint32_t IR_START_DELAY_MS  = 0;       // remote: start immediately
constexpr uint32_t LOST_TIMEOUT_MS    = 400;

constexpr int      FINISH_MARKS       = 2;       // default; change live with "fm <n>" (saved)
constexpr float    STOP_DISTANCE_MM   = 150.0f;  // keep driving after finish mark
// Crossroads: the robot drives straight while crossing and for this far after it
constexpr bool     CROSS_ENABLED      = true;    // false = plain PID, no crossroads handling
constexpr float    CROSS_HOLD_MM      = 25.0f;
constexpr uint32_t CROSS_HOLD_MAX_MS  = 150;     // hold never lasts longer than this
constexpr float    MARK_MIN_SPACING_MM = 30.0f;  // marks on the same side closer than this
                                                 // are treated as one (rejects sensor noise)

// ============================================================================
// TRACK MAP - the first run records the track between marks; later runs
// speed up on the sections that were straight.
// ============================================================================
constexpr int      DEFAULT_STRAIGHT_BOOST   = 200;    // straights run at baseSpeed + this; "vs <v>" (saved)
// A section is straight if the two wheels travelled the same distance, within
// +/- (STRAIGHT_TOL_MM + STRAIGHT_TOL_PCT % of the section length).
// Example: 800 mm section -> 20 + 5% of 800 = +/- 60 mm allowed.
constexpr float    STRAIGHT_TOL_MM          = 20.0f;
constexpr float    STRAIGHT_TOL_PCT         = 5.0f;
constexpr float    STRAIGHT_MIN_MM          = 200.0f; // shorter sections are never sped up
constexpr float    STRAIGHT_RAMP_MM         = 80.0f;  // accelerate over this distance after entering
constexpr float    STRAIGHT_BRAKE_MM        = 120.0f; // slow down over this distance...
constexpr float    STRAIGHT_BRAKE_MARGIN_MM = 60.0f;  // ...to be back at base speed this far before the end
constexpr float    MAP_TOLERANCE            = 0.25f;  // section length off by more than this -> stop using the map
// With the gyro working, straights are decided by the angle turned instead:
constexpr float    STRAIGHT_MAX_DEG         = 8.0f;   // section turned less than this (+/-) = straight
constexpr float    MAP_ANGLE_TOL_DEG        = 30.0f;  // angle off by more than this (or 25%) -> stop using the map
constexpr int      MAP_MAX_SEGMENTS         = 64;
// Shortcut: between these two LEFT mark numbers the robot uses the gains below
// (99 = never). Change live with shcut1 / shcut2 / skp / ski / skd.
constexpr int      SHORTCUT1_START          = 99;
constexpr int      SHORTCUT1_END            = 99;
constexpr float    SHORTCUT_KP              = 0.27f;
constexpr float    SHORTCUT_KI              = 0.0001f;
constexpr float    SHORTCUT_KD              = 0.008f;

// ============================================================================
// GYRO (MPU-6050 on I2C). Only the Z axis (turning) is used.
// ============================================================================
constexpr bool     GYRO_ENABLED      = true;    // false = ignore the MPU, map uses wheels only
constexpr uint8_t  MPU_ADDR          = 0x68;    // 0x69 if AD0 is tied to 3V3
constexpr bool     GYRO_INVERT       = false;   // set true if turning LEFT makes the heading go DOWN
constexpr uint32_t GYRO_PERIOD_US    = 1000;    // read every 1 ms
constexpr uint8_t  GYRO_DLPF         = 0x02;    // MPU internal filter, ~94 Hz
constexpr float    GYRO_LSB_PER_DPS  = 16.4f;   // +/-2000 deg/s range
// The gyro has a small offset that is measured automatically whenever the
// robot is idle and standing still (raw readings within this spread for 0.5 s)
constexpr int16_t  GYRO_STILL_RANGE  = 40;
constexpr uint32_t GYRO_BIAS_WINDOW_MS = 500;

// ============================================================================
// IR REMOTE (NEC command codes; unknown codes are printed on Serial)
// ============================================================================
constexpr uint16_t IR_CMD_START      = 0x40;
constexpr uint16_t IR_CMD_STOP       = 0xF1;
constexpr uint16_t IR_CMD_CALIBRATE  = 0xF2;
constexpr uint16_t IR_CMD_SPEED_UP   = 0x16;
constexpr uint16_t IR_CMD_SPEED_DOWN = 0x1A;
constexpr int      IR_SPEED_STEP     = 25;
// Many remotes send each press 2-3 times. The same command received again
// within this time is ignored (also absorbs frames queued during calibration).
constexpr uint32_t IR_DEDUP_MS       = 300;
// Report IR signals that couldn't be decoded (max once per second).
// Useful to tell "receiver sees nothing" from "signal corrupted by noise".
constexpr bool     IR_LOG_UNREADABLE = true;
// Core that samples the IR receiver. The control loop runs on core 1 and its
// analog reads briefly block interrupts there, which corrupts IR timing, so the
// receiver runs on core 0 (together with Bluetooth).
constexpr int      IR_TASK_CORE      = 0;

// ============================================================================
// DEBUG
// ============================================================================
constexpr bool     TELEMETRY           = true;
constexpr uint32_t TELEMETRY_PERIOD_MS = 200;
constexpr uint32_t SERIAL_BAUD         = 115200;

// ============================================================================
// BLUETOOTH LOW ENERGY (console over BLE, "Nordic UART" service)
// Library: NimBLE-Arduino by h2zero, v2.x
// ============================================================================
constexpr bool        BLE_ENABLED = true;
constexpr const char *BLE_NAME    = "LineFollower";   // name shown on the phone
constexpr uint32_t    BLE_MIN_GAP_MS     = 4;    // minimum time between BLE packets
constexpr int         BLE_QUEUE_LINES    = 80;   // lines waiting to be sent (never blocks the robot)
