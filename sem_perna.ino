/*
  ============================================================================
  Line Follower Robot - ESP32-WROOM-32D (ESP32-DevKitC-32D)
  ============================================================================
  Target   : Arduino-ESP32 core 3.x
  Libraries: IRremote by Armin Joachimsmeyer, v4.x
             NimBLE-Arduino by h2zero, v2.x

  Files
    config.h           pins and all settings  <- edit this one
    robot.h / .cpp     motors, encoders, sensors, calibration, PID
    line_follower.ino  user interface: buttons, IR remote, console (USB + BLE)
  ============================================================================
*/
#include <IRremote.hpp>      // must be included in only one file
#include <NimBLEDevice.h>
#include "config.h"
#include "robot.h"
#include <esp_system.h>

// ============================================================================
// BLE UART (Nordic UART Service - supported by "Serial Bluetooth Terminal")
// ============================================================================
static const char *NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *NUS_RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // phone -> robot
static const char *NUS_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // robot -> phone

static NimBLEServer         *bleServer = nullptr;
static NimBLECharacteristic *bleTx     = nullptr;
static volatile bool         bleConnected  = false;
static volatile uint16_t     bleConnHandle = 0;

// Bytes received from the phone. Written by the BLE task (core 0),
// read by loop() (core 1), so it's a small lock-protected ring buffer.
static char                bleRxBuf[256];
static volatile uint16_t   bleRxHead = 0, bleRxTail = 0;
static portMUX_TYPE        bleRxMux = portMUX_INITIALIZER_UNLOCKED;

static void bleRxPush(const uint8_t *data, size_t len) {
  portENTER_CRITICAL(&bleRxMux);
  for (size_t i = 0; i < len; i++) {
    uint16_t next = (bleRxHead + 1) % sizeof(bleRxBuf);
    if (next == bleRxTail) break;                  // buffer full: drop the rest
    bleRxBuf[bleRxHead] = (char)data[i];
    bleRxHead = next;
  }
  portEXIT_CRITICAL(&bleRxMux);
}

static int bleRxPop() {
  int c = -1;
  portENTER_CRITICAL(&bleRxMux);
  if (bleRxTail != bleRxHead) {
    c = (uint8_t)bleRxBuf[bleRxTail];
    bleRxTail = (bleRxTail + 1) % sizeof(bleRxBuf);
  }
  portEXIT_CRITICAL(&bleRxMux);
  return c;
}

class BleServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s, NimBLEConnInfo &info) override {
    bleConnHandle = info.getConnHandle();
    bleConnected = true;
  }
  void onDisconnect(NimBLEServer *s, NimBLEConnInfo &info, int reason) override {
    bleConnected = false;                          // advertising restarts automatically
  }
};

class BleRxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override {
    NimBLEAttValue v = c->getValue();
    bleRxPush(v.data(), v.length());
  }
};

void bleBegin() {
  if (!BLE_ENABLED) return;
  NimBLEDevice::init(BLE_NAME);

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new BleServerCallbacks());

  NimBLEService *svc = bleServer->createService(NUS_SERVICE_UUID);
  bleTx = svc->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic *rx = svc->createCharacteristic(
      NUS_RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(new BleRxCallbacks());
  svc->start();

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->setName(BLE_NAME);
  adv->start();
}

// Max bytes per BLE packet (negotiated with the phone, usually ~244 on Android)
static size_t blePayloadSize() {
  uint16_t mtu = bleServer ? bleServer->getPeerMTU(bleConnHandle) : 0;
  if (mtu < 23) mtu = 23;
  return mtu - 3;
}

// ============================================================================
// CONSOLE OUTPUT: everything printed goes to USB Serial right away and, when a
// phone is connected, into a queue that is sent over BLE a little at a time
// from loop(). Sending never waits, so printing during a run can't stall the
// 1 kHz control loop, and lines aren't dropped when many are printed at once.
// ============================================================================
class DualPrint : public Print {
public:
  size_t write(uint8_t c) override { return write(&c, 1); }

  size_t write(const uint8_t *buf, size_t n) override {
    Serial.write(buf, n);
    if (bleConnected) {
      for (size_t i = 0; i < n; i++) {
        line_[len_++] = buf[i];
        if (buf[i] == '\n' || len_ >= OUT_LINE_LEN) enqueue();
      }
    }
    return n;
  }

  // Call often (every loop): queues any partial line and sends one packet
  void service() {
    if (!bleConnected || !bleTx) { head_ = tail_ = 0; len_ = 0; off_ = 0; return; }
    if (len_ > 0) enqueue();
    if (head_ == tail_) return;                          // nothing to send
    if (millis() - lastMs_ < BLE_MIN_GAP_MS) return;     // pacing

    Slot &q = queue_[tail_];
    size_t chunk = std::min(blePayloadSize(), (size_t)(q.len - off_));
    bleTx->setValue(q.data + off_, chunk);
    if (!bleTx->notify()) return;                        // stack busy: retry next loop
    lastMs_ = millis();
    off_ += chunk;
    if (off_ >= q.len) {                                 // whole line sent
      off_ = 0;
      tail_ = (tail_ + 1) % BLE_QUEUE_LINES;
    }
  }

private:
  static constexpr size_t OUT_LINE_LEN = 128;
  struct Slot { uint8_t data[OUT_LINE_LEN]; uint8_t len; };

  void enqueue() {
    uint16_t next = (head_ + 1) % BLE_QUEUE_LINES;
    if (next != tail_) {                                 // full queue: drop the line
      memcpy(queue_[head_].data, line_, len_);
      queue_[head_].len = len_;
      head_ = next;
    }
    len_ = 0;
  }

  uint8_t  line_[OUT_LINE_LEN];
  size_t   len_ = 0;
  Slot     queue_[BLE_QUEUE_LINES];
  uint16_t head_ = 0, tail_ = 0;
  size_t   off_ = 0;                                     // bytes of the current line already sent
  uint32_t lastMs_ = 0;
};

DualPrint dualOut;
Print &Log = dualOut;          // used by robot.cpp and this file

// ============================================================================
// BUTTONS (debounced; polarity set by BUTTONS_ACTIVE_HIGH in config.h)
// ============================================================================
class Button {
public:
  explicit Button(uint8_t pin, uint32_t debounceMs = 30)
    : pin_(pin), debounceMs_(debounceMs) {}

  void begin() {
    // Active HIGH: button to 3V3, internal pull-down keeps the pin LOW when released
    pinMode(pin_, BUTTONS_ACTIVE_HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);
    stable_ = lastRead_ = digitalRead(pin_);   // no false press at startup
  }

  // true once per press (edge towards the active level, after debounce)
  bool pressed() {
    bool r = digitalRead(pin_);
    if (r != lastRead_) { lastRead_ = r; changedAt_ = millis(); }
    if (millis() - changedAt_ > debounceMs_ && r != stable_) {
      stable_ = r;
      return stable_ == ACTIVE;
    }
    return false;
  }

private:
  static constexpr bool ACTIVE = BUTTONS_ACTIVE_HIGH ? HIGH : LOW;
  uint8_t  pin_;
  uint32_t debounceMs_;
  bool     stable_ = !ACTIVE, lastRead_ = !ACTIVE;
  uint32_t changedAt_ = 0;
};

Button btnStart(PIN_BTN_START);
Button btnCalib(PIN_BTN_CALIB);

void handleButtons() {
  if (btnStart.pressed()) {
    if (robotState() == RobotState::Idle) requestStart("button");
    else requestStop("button");            // pressing START again stops
  }
  if (btnCalib.pressed() && robotState() == RobotState::Idle) {
    Log.printf("[CAL] Starting in %lu ms\n", (unsigned long)CAL_BUTTON_DELAY_MS);
    delay(CAL_BUTTON_DELAY_MS);
    requestCalibration("button");
  }
}

// ============================================================================
// IR REMOTE
// The receiver is sampled by a timer interrupt every 50 us. If that interrupt
// runs on the same core as the control loop, the analog reads (8+2 per ms)
// delay it and the remote's pulses get measured wrong. So all IRremote work
// happens in a small task on IR_TASK_CORE, which passes decoded commands to
// loop() through a queue.
// ============================================================================
struct IrEvent {
  decode_type_t protocol;
  uint16_t      command;
  uint8_t       flags;
};
static QueueHandle_t irQueue = nullptr;

static void irTask(void *) {
  IrReceiver.begin(PIN_IR, DISABLE_LED_FEEDBACK);   // timer interrupt lands on this core
  for (;;) {
    if (IrReceiver.decode()) {
      IrEvent e;
      e.protocol = IrReceiver.decodedIRData.protocol;
      e.command  = IrReceiver.decodedIRData.command;
      e.flags    = IrReceiver.decodedIRData.flags;
      IrReceiver.resume();
      xQueueSend(irQueue, &e, 0);                    // drop if loop() is busy and queue full
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void irBegin() {
  irQueue = xQueueCreate(8, sizeof(IrEvent));
  xTaskCreatePinnedToCore(irTask, "ir", 4096, nullptr, 2, nullptr, IR_TASK_CORE);
}

void handleIR() {
  IrEvent d;
  if (xQueueReceive(irQueue, &d, 0) != pdTRUE) return;

  if (d.protocol == UNKNOWN) {
    static uint32_t lastReport = 0;
    if (IR_LOG_UNREADABLE && millis() - lastReport > 1000) {
      Log.printf("[IR] unreadable signal (%s)\n", isRunning() ? "running" : "idle");
      lastReport = millis();
    }
    return;
  }
  if (d.flags & (IRDATA_FLAGS_IS_REPEAT | IRDATA_FLAGS_IS_AUTO_REPEAT)) return;

  // Ignore the same command arriving again shortly after (duplicate frames)
  static uint16_t lastCmd = 0xFFFF;
  static uint32_t lastTime = 0;
  bool duplicate = (d.command == lastCmd) && (millis() - lastTime < IR_DEDUP_MS);
  lastCmd = d.command;
  lastTime = millis();
  if (duplicate) return;

  Log.printf("[IR] command 0x%02X\n", d.command);

  switch (d.command) {
    case IR_CMD_START:     requestStart("IR", IR_START_DELAY_MS); break;
    case IR_CMD_STOP:      requestStop("IR"); break;
    case IR_CMD_CALIBRATE: requestCalibration("IR"); break;
    case IR_CMD_SPEED_UP:
      baseSpeed = min(baseSpeed + IR_SPEED_STEP, PWM_MAX);
      Log.printf("[IR] baseSpeed = %d\n", baseSpeed);
      break;
    case IR_CMD_SPEED_DOWN:
      baseSpeed = max(baseSpeed - IR_SPEED_STEP, 0);
      Log.printf("[IR] baseSpeed = %d\n", baseSpeed);
      break;
    default:
      Log.printf("[IR] Unmapped: protocol=%s command=0x%02X\n",
                 getProtocolString(d.protocol), d.command);
  }
  lastTime = millis();   // measure from the END of the action (calibration takes 3 s)
}

// ============================================================================
// CONSOLE COMMANDS (same commands over USB Serial and BLE)
// ============================================================================
void printHelp() {
  Log.println(
    "Commands:\n"
    "  kp <v>  ki <v>  kd <v>  v <speed>   set PID gains / base speed\n"
    "  vs <v>                              straights run at base + v (saved)\n"
    "  fm <n>                              number of finish marks (saved)\n"
    "  map | remap | fast <0/1>            show map / map again / use map\n"
    "  shcut1 <n>  shcut2 <n>              shortcut between left marks n1..n2 (99 = off)\n"
    "  skp <v>  ski <v>  skd <v>           gains used inside the shortcut\n"
    "  go | stop | cal                     run control\n"
    "  s                                   dump sensor readings\n"
    "  c                                   show calibration\n"
    "  e                                   show encoders\n"
    "  g | gz | gcal                       gyro reading / zero heading / measure offset\n"
    "  r                                   why the ESP32 last restarted\n"
    "  p                                   show parameters");
}

void printResetReason() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  Log.println("Reset reason: power on"); break;
    case ESP_RST_BROWNOUT: Log.println("Reset reason: BROWNOUT (supply voltage dipped)"); break;
    case ESP_RST_PANIC:    Log.println("Reset reason: crash (panic)"); break;
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      Log.println("Reset reason: watchdog"); break;
    case ESP_RST_EXT:      Log.println("Reset reason: EN/reset button"); break;
    default:               Log.printf("Reset reason: %d\n", (int)esp_reset_reason()); break;
  }
}

void printParams() {
  Log.printf("Kp=%.4f Ki=%.5f Kd=%.4f baseSpeed=%d\n", Kp, Ki, Kd, baseSpeed);
  Log.printf("straights=base+%d (%d) finishMarks=%d map=%s fast=%s\n", straightBoost,
             straightSpeed(), finishMarkCount(), mapReady() ? "ready" : "none",
             fastEnabled ? "on" : "off");
  Log.printf("shortcut: left marks %d..%d  Kp=%.4f Ki=%.5f Kd=%.4f\n",
             shortcut.start, shortcut.end, scKp, scKi, scKd);
}

void dumpSensors() {
  bool on;
  int32_t pos = readLinePosition(on);
  readSideSensors();
  Log.print("raw:  ");   for (auto v : raw)  Log.printf("%4u ", v);
  Log.print("\nnorm: "); for (auto v : norm) Log.printf("%4u ", v);
  Log.printf("\npos=%ld onLine=%d  sideL=%u sideR=%u\n",
             (long)pos, on, sideNorm[0], sideNorm[1]);
}

void processCommand(String line) {
  line.trim();
  if (line.isEmpty()) return;

  int sp = line.indexOf(' ');
  String cmd = (sp < 0) ? line : line.substring(0, sp);
  float  val = (sp < 0) ? 0 : line.substring(sp + 1).toFloat();
  cmd.toLowerCase();

  if      (cmd == "kp")     { Kp = val; printParams(); }
  else if (cmd == "ki")     { Ki = val; printParams(); }
  else if (cmd == "kd")     { Kd = val; printParams(); }
  else if (cmd == "shcut1") { shortcut.start = (int)val; printParams(); }
  else if (cmd == "shcut2") { shortcut.end   = (int)val; printParams(); }
  else if (cmd == "skp")    { scKp = val; printParams(); }
  else if (cmd == "ski")    { scKi = val; printParams(); }
  else if (cmd == "skd")    { scKd = val; printParams(); }
  else if (cmd == "v")      { baseSpeed = constrain((int)val, 0, PWM_MAX); printParams(); }
  else if (cmd == "vs")     setStraightBoost((int)val);
  else if (cmd == "fm")     setFinishMarks((int)val);
  else if (cmd == "map")    printMap();
  else if (cmd == "remap")  clearMap();
  else if (cmd == "fast")   { fastEnabled = val != 0; printParams(); }
  else if (cmd == "p")      printParams();
  else if (cmd == "go")     requestStart("console");
  else if (cmd == "stop")   requestStop("console");
  else if (cmd == "cal")    requestCalibration("console");
  else if (cmd == "c")      printCalibration();
  else if (cmd == "s")      dumpSensors();
  else if (cmd == "e") {
    int32_t l, r; readEncoders(l, r);
    Log.printf("encL=%ld encR=%ld dist=%.1f mm\n", (long)l, (long)r, distanceMm());
  }
  else if (cmd == "g")    printGyro();
  else if (cmd == "gz")   { gyroResetHeading(); printGyro(); }
  else if (cmd == "gcal") { if (robotState() == RobotState::Idle) gyroCalibrate(); }
  else if (cmd == "r")    printResetReason();
  else if (cmd == "help") printHelp();
  else Log.println("Unknown command, type 'help'");
}

// Collects characters into lines without blocking the control loop.
class LineReader {
public:
  // Returns true when a full line is ready in `out`
  bool feed(int c, String &out) {
    if (c == '\n' || c == '\r') {
      if (len_ == 0) return false;
      buf_[len_] = '\0';
      out = buf_;
      len_ = 0;
      return true;
    }
    if (len_ < sizeof(buf_) - 1) buf_[len_++] = (char)c;
    return false;
  }
private:
  char    buf_[64];
  uint8_t len_ = 0;
};

LineReader usbLine, bleLine;

void handleConsole() {
  String line;
  while (Serial.available())
    if (usbLine.feed(Serial.read(), line)) processCommand(line);

  int c;
  while ((c = bleRxPop()) >= 0)
    if (bleLine.feed(c, line)) processCommand(line);
}

// ============================================================================
// TELEMETRY
// ============================================================================
void telemetry() {
  static uint32_t last = 0;
  static int32_t lastL = 0, lastR = 0;
  static bool wasRunning = false;
  if (!TELEMETRY) return;

  if (!isRunning()) { wasRunning = false; return; }
  if (!wasRunning) {                       // run just started: encoders were reset
    readEncoders(lastL, lastR);
    last = millis();
    wasRunning = true;
    return;
  }
  if (millis() - last < TELEMETRY_PERIOD_MS) return;
  float dt = (millis() - last) / 1000.0f;
  last = millis();

  int32_t l, r;
  readEncoders(l, r);
  float vL = (l - lastL) * MM_PER_TICK / dt;
  float vR = (r - lastR) * MM_PER_TICK / dt;
  lastL = l; lastR = r;

  Log.printf("seg=%d spd=%d pos=%ld err=%.0f vL=%.0f vR=%.0f mm/s dist=%.0f hdg=%.0f\n",
               currentSegment(), currentTargetSpeed(),
               (long)lastLinePosition(), lastError(), vL, vR, distanceMm(), gyroHeading());
}

// ============================================================================
// SETUP / LOOP
// ============================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Log.println("\n=== Line Follower ESP32 ===");
  printResetReason();

  bleBegin();
  robotBegin();
  btnStart.begin();
  btnCalib.begin();
  irBegin();

  printHelp();
  if (BLE_ENABLED) Log.printf("BLE console: connect to \"%s\"\n", BLE_NAME);
  Log.println("Ready. START = run, CALIBRATE = calibrate.");
}

void loop() {
  handleIR();
  handleConsole();
  handleButtons();
  controllerUpdate();
  telemetry();
  dualOut.service();          // send queued output over BLE (never blocks)
}
