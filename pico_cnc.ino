/* ================================================================
   Pico CNC — G-code Controller (Arduino / RP2040)
   ================================================================ */

#include <Arduino.h>
#include <math.h>
// Dùng hardware PWM của RP2040 trực tiếp — không dùng Servo.h
// Servo.h trên RP2040 dùng software PWM → jitter khi core bận
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/time.h"

/* ================================================================
   CẤU HÌNH CHÂN
   ================================================================ */
#define X_STEP_PIN   2
#define X_DIR_PIN    3
#define Y_STEP_PIN   4
#define Y_DIR_PIN    5
#define EN_PIN       6
#define SPINDLE_PIN  7
#define Z_SERVO_PIN  8

#define ESP_SERIAL   Serial1
#define ESP_TX_PIN   0
#define ESP_RX_PIN   1
#define ESP_BAUD     115200

/* ================================================================
   THÔNG SỐ MÁY
   ================================================================ */
#define STEPS_PER_MM_X     80.0f
#define STEPS_PER_MM_Y     79.0f
#define MAX_SPEED_MM_S     150.0f
#define ACCEL_MM_S2        400.0f
#define INVERT_X           true
#define INVERT_Y           true
#define ARC_TOLERANCE      0.05f
#define DEFAULT_FEED_MM_S  150.0f

/* ================================================================
   THÔNG SỐ SERVO Z
   ================================================================ */
#define Z_ANGLE_MIN      50
#define Z_ANGLE_MAX      105
#define Z_ANGLE_UP       50
#define Z_ANGLE_DOWN     105
#define Z_SLEW_DELAY_MS  8      // ms mỗi độ — điều chỉnh tốc độ servo
#define Z_SETTLE_MS      120    // chờ sau khi servo đến đích (ms)

/* Map Z G-code → góc servo:
   0 = ngưỡng  : Z>0 → UP, Z≤0 → DOWN
   1 = tuyến tính trong [Z_LINEAR_MIN .. Z_LINEAR_MAX]            */
#define Z_MAP_MODE       0
#define Z_LINEAR_MIN     0.0f
#define Z_LINEAR_MAX     1.0f

/* ================================================================ */
#define X_DIR_LEVEL(dx)  ((INVERT_X ? (dx) <= 0 : (dx) >= 0) ? HIGH : LOW)
#define Y_DIR_LEVEL(dy)  ((INVERT_Y ? (dy) <= 0 : (dy) >= 0) ? HIGH : LOW)

/* ================================================================
   COMMAND BUFFER (tăng lên 64 để giảm drop khi servo settle)
   ================================================================ */
#define CMD_BUFFER_SIZE  64
#define MAX_LINE_LEN     96
String  cmdBuffer[CMD_BUFFER_SIZE];
volatile int bufHead = 0, bufTail = 0;
String  usbLine = "", espLine = "";

/* ================================================================
   TRẠNG THÁI MÁY
   ================================================================ */
long    posX      = 0, posY = 0;
float   feedRate  = DEFAULT_FEED_MM_S;
bool    absMode   = true;
bool    mmMode    = true;
bool    spindleOn = false;
volatile bool stopFlag = false;

/* ================================================================
   SERVO — Hardware PWM trực tiếp (không dùng Servo.h)
   RP2040 PWM: period 20ms, pulse 500–2500µs → 0°–180°
   Dùng volatile để core0 ↔ core1 đồng bộ.
   ================================================================ */
volatile int  zCurrentAngle  = Z_ANGLE_UP;
volatile int  zTargetAngle   = Z_ANGLE_UP;
volatile bool zMoving        = false;
volatile bool zSettling      = false;

static uint  _servoSlice = 0;
static uint  _servoChan  = 0;
/* Cache wrap+scale sau khi init để servoWriteUs() không recalculate */
static uint32_t _pwmWrap  = 0;
static uint64_t _pwmScale = 0;  // = sysClk / divider / 1e6 (nhân trước để tránh float)

/* Chuyển góc (độ) → pulse width (µs): 500µs=0°, 2500µs=180° */
static inline uint32_t angleToPulseUs(int angle) {
  return 500UL + (uint32_t)angle * 2000UL / 180UL;
}

/* Ghi pulse width lên hardware PWM — gọi được từ bất kỳ core nào */
void servoWriteUs(uint32_t pulseUs) {
  uint32_t level = (uint32_t)(pulseUs * _pwmScale / 1000000UL);
  pwm_set_chan_level(_servoSlice, _servoChan, level);
}

void servoWriteAngle(int angle) {
  servoWriteUs(angleToPulseUs(angle));
}

/* Khởi tạo hardware PWM 50Hz cho servo */
void servoHWInit(uint pin) {
  gpio_set_function(pin, GPIO_FUNC_PWM);
  _servoSlice = pwm_gpio_to_slice_num(pin);
  _servoChan  = pwm_gpio_to_channel(pin);

  pwm_config cfg = pwm_get_default_config();
  uint32_t sysClk = clock_get_hz(clk_sys);
  // divider=64 → clock vào PWM = 125MHz/64 ≈ 1.953MHz
  // wrap cho 50Hz: 1953125/50 - 1 = 39061
  pwm_config_set_clkdiv_int(&cfg, 64);
  uint32_t wrap = sysClk / 64 / 50 - 1;
  pwm_config_set_wrap(&cfg, wrap);
  pwm_init(_servoSlice, &cfg, true);

  _pwmWrap  = wrap;
  _pwmScale = (uint64_t)sysClk / 64ULL;

  servoWriteAngle(Z_ANGLE_UP);  // đặt vị trí ban đầu
}

/* ================================================================
   CORE 1 — Servo slew loop
   Chạy hoàn toàn độc lập, không ảnh hưởng core0 chút nào.
   - Mỗi Z_SLEW_DELAY_MS ms: tăng/giảm góc 1 độ tiến đến target
   - Khi đến đích: chuyển sang settling, chờ Z_SETTLE_MS ms
   - Khi idle: tight_loop_contents() để không ăn 100% CPU
   ================================================================ */
void setup1() { /* Hardware PWM đã init ở setup() (core0) */ }

void loop1() {
  static absolute_time_t nextStep;
  static absolute_time_t settleEnd;
  static bool timerInit = false;

  if (!timerInit) { nextStep = get_absolute_time(); timerInit = true; }

  if (zMoving) {
    int cur = zCurrentAngle, target = zTargetAngle;
    if (cur != target) {
      sleep_until(nextStep);                        // chờ đúng chu kỳ (µs accuracy)
      cur += (target > cur) ? 1 : -1;
      servoWriteAngle(cur);
      zCurrentAngle = cur;
      nextStep = delayed_by_ms(nextStep, Z_SLEW_DELAY_MS);
    } else {
      zMoving   = false;
      zSettling = true;
      settleEnd = delayed_by_ms(get_absolute_time(), Z_SETTLE_MS);
    }
  } else if (zSettling) {
    if (absolute_time_diff_us(get_absolute_time(), settleEnd) <= 0) {
      zSettling = false;
      nextStep  = get_absolute_time();              // reset để lần slew tiếp bắt đầu đúng
    }
  } else {
    tight_loop_contents();                          // idle: nhường CPU
  }
}

/* ================================================================
   FORWARD DECLARATION — phải đứng trước zServoWaitDone()
   ================================================================ */
void readSerialNonBlocking();

/* ================================================================
   API SERVO — gọi từ core0
   ================================================================ */

/* Đặt target và return ngay — không block core0 */
void zServoMoveTo(int targetAngle) {
  if (targetAngle < Z_ANGLE_MIN) targetAngle = Z_ANGLE_MIN;
  if (targetAngle > Z_ANGLE_MAX) targetAngle = Z_ANGLE_MAX;
  if (targetAngle == (int)zCurrentAngle && !zMoving && !zSettling) return;
  zTargetAngle = targetAngle;
  zMoving      = true;
}

/* Chờ servo hoàn toàn xong (slew + settle).
   Trong lúc chờ vẫn đọc serial để buffer không tràn. */
void zServoWaitDone() {
  while (zMoving || zSettling) {
    readSerialNonBlocking();
    tight_loop_contents();
  }
}

/* Map giá trị Z trong G-code → góc servo */
int zValueToAngle(float zVal) {
#if Z_MAP_MODE == 0
  return (zVal > 0.0f) ? Z_ANGLE_UP : Z_ANGLE_DOWN;
#else
  float t = (zVal - Z_LINEAR_MIN) / (Z_LINEAR_MAX - Z_LINEAR_MIN);
  t = constrain(t, 0.0f, 1.0f);
  return (int)(Z_ANGLE_MIN + t * (Z_ANGLE_MAX - Z_ANGLE_MIN) + 0.5f);
#endif
}

/* ================================================================
   DUAL PRINT & REPLY
   ================================================================ */
void dualPrintln(const String& s) {
  Serial.println(s);
  ESP_SERIAL.println(s);
}

void replyOk() {
  dualPrintln("ok X:" + String((float)posX / STEPS_PER_MM_X, 3) +
              " Y:"   + String((float)posY / STEPS_PER_MM_Y, 3) +
              " Z:"   + String((int)zCurrentAngle));
}

inline void enableMotors()  { digitalWrite(EN_PIN, LOW);  }
inline void disableMotors() { digitalWrite(EN_PIN, HIGH); }

/* ================================================================
   PARSE — sửa lỗi bỏ sót key ở đầu chuỗi
   Chấp nhận key ở vị trí đầu (i==0) hoặc sau space/tab
   ================================================================ */
float parseParm(const String& line, char key) {
  int len = line.length();
  for (int i = 0; i < len; i++) {
    if (line.charAt(i) == key) {
      bool atStart  = (i == 0);
      bool afterSep = (i > 0 && (line.charAt(i-1) == ' ' || line.charAt(i-1) == '\t'));
      if (atStart || afterSep) return line.substring(i + 1).toFloat();
    }
  }
  return NAN;
}

bool hasParm(const String& line, char key) {
  int len = line.length();
  for (int i = 0; i < len; i++) {
    if (line.charAt(i) == key) {
      bool atStart  = (i == 0);
      bool afterSep = (i > 0 && (line.charAt(i-1) == ' ' || line.charAt(i-1) == '\t'));
      if (atStart || afterSep) return true;
    }
  }
  return false;
}

/* ================================================================
   moveToRaw — Bresenham không accel (dùng cho arc segment)
   Kiểm tra STOP mỗi 256 step thay vì yield() mỗi step
   ================================================================ */
void moveToRaw(float tx_mm, float ty_mm, unsigned long delayUs) {
  if (stopFlag) return;
  long tx = (long)(tx_mm * STEPS_PER_MM_X), ty = (long)(ty_mm * STEPS_PER_MM_Y);
  long dx = tx - posX, dy = ty - posY;
  if (dx == 0 && dy == 0) return;

  digitalWrite(X_DIR_PIN, X_DIR_LEVEL(dx));
  digitalWrite(Y_DIR_PIN, Y_DIR_LEVEL(dy));
  delayMicroseconds(2);

  long adx = abs(dx), ady = abs(dy), total = max(adx, ady), err = adx - ady;

  for (long i = 0; i < total; i++) {
    if (stopFlag) return;
    long e2 = 2L * err;
    bool sX = (e2 > -ady), sY = (e2 < adx);
    if (e2 > -ady) err -= ady;
    if (e2 <  adx) err += adx;
    if (sX) { digitalWrite(X_STEP_PIN, HIGH); }
    if (sY) { digitalWrite(Y_STEP_PIN, HIGH); }
    delayMicroseconds(5);
    if (sX) { digitalWrite(X_STEP_PIN, LOW); posX += (dx > 0 ? 1 : -1); }
    if (sY) { digitalWrite(Y_STEP_PIN, LOW); posY += (dy > 0 ? 1 : -1); }
    delayMicroseconds(delayUs);

    // Kiểm tra STOP mỗi 256 step — nhẹ hơn yield()
    if ((i & 0xFF) == 0 && ESP_SERIAL.available()) {
      char c = ESP_SERIAL.peek();
      if (c == 'S' || c == 's') {
        String s = ESP_SERIAL.readStringUntil('\n'); s.trim();
        if (s.equalsIgnoreCase("STOP")) { stopFlag = true; return; }
      }
    }
  }
}

/* ================================================================
   moveTo — Bresenham + hình thang gia tốc
   Tính accelSteps = (v² / 2a) * stepsPerMM
   Ratio: [0..accelSteps] accel → cruise → [total-accelSteps..total] decel
   ================================================================ */
void moveTo(float tx_mm, float ty_mm, float spd_mm_s) {
  if (stopFlag) return;
  long tx = (long)(tx_mm * STEPS_PER_MM_X), ty = (long)(ty_mm * STEPS_PER_MM_Y);
  long dx = tx - posX, dy = ty - posY;
  if (dx == 0 && dy == 0) return;

  digitalWrite(X_DIR_PIN, X_DIR_LEVEL(dx));
  digitalWrite(Y_DIR_PIN, Y_DIR_LEVEL(dy));
  delayMicroseconds(2);

  long adx = abs(dx), ady = abs(dy), total = max(adx, ady);
  float spd    = constrain(spd_mm_s, 1.0f, MAX_SPEED_MM_S);
  float maxSPMM = max(STEPS_PER_MM_X, STEPS_PER_MM_Y);
  float dMin   = 1000000.0f / (spd * maxSPMM);   // us/step tốc độ cao nhất
  float dMax   = dMin * 4.0f;                      // us/step tốc độ thấp nhất

  long accelSteps = (long)((spd * spd) / (2.0f * ACCEL_MM_S2) * maxSPMM);
  accelSteps = constrain(accelSteps, 1L, total / 2);

  long err = adx - ady;

  for (long i = 0; i < total; i++) {
    if (stopFlag) return;

    // Hình thang: accel → cruise → decel
    float ratio;
    if      (i < accelSteps)             ratio = (float)(i + 1) / accelSteps;
    else if (i > total - accelSteps - 1) ratio = (float)(total - i) / accelSteps;
    else                                 ratio = 1.0f;
    ratio = constrain(ratio, 0.15f, 1.0f);

    unsigned long d = (unsigned long)(dMax - (dMax - dMin) * ratio);

    long e2 = 2L * err;
    bool sX = (e2 > -ady), sY = (e2 < adx);
    if (e2 > -ady) err -= ady;
    if (e2 <  adx) err += adx;
    if (sX) { digitalWrite(X_STEP_PIN, HIGH); }
    if (sY) { digitalWrite(Y_STEP_PIN, HIGH); }
    delayMicroseconds(5);
    if (sX) { digitalWrite(X_STEP_PIN, LOW); posX += (dx > 0 ? 1 : -1); }
    if (sY) { digitalWrite(Y_STEP_PIN, LOW); posY += (dy > 0 ? 1 : -1); }
    delayMicroseconds(d);

    // Kiểm tra STOP + đọc command mới mỗi 256 step
    if ((i & 0xFF) == 0 && ESP_SERIAL.available()) {
      char c = ESP_SERIAL.peek();
      if (c == 'S' || c == 's') {
        String s = ESP_SERIAL.readStringUntil('\n'); s.trim();
        if (s.equalsIgnoreCase("STOP")) { stopFlag = true; return; }
        int nh = (bufHead + 1) % CMD_BUFFER_SIZE;
        if (nh != bufTail) { cmdBuffer[bufHead] = s; bufHead = nh; }
      }
    }
  }
}

/* ================================================================
   arcInterpolate — CONTINUOUS STEP LOOP
   ─────────────────────────────────────────────────────────────────
   Một vòng Bresenham liên tục xuyên suốt toàn arc thay vì chia
   thành nhiều đoạn rồi gọi moveToRaw() nhiều lần (cách cũ gây gằn
   vì mỗi lần gọi có DIR setup + delay ngắt quãng).

   Gia tốc hình thang tính trên tổng step toàn arc → mượt đều.
   DIR pin chỉ set lại khi thực sự đổi chiều → không delay thừa.
   v_arc giới hạn thêm bởi sqrt(a*r)*0.75 để không trượt vòng cung.
   ================================================================ */
void arcInterpolate(float cx, float cy, float r,
                    float a_start, float a_end,
                    bool cw, float reqSpd) {
  if (r <= 0 || stopFlag) return;

  // Giới hạn tốc độ theo bán kính để không trượt
  float v_arc   = min(reqSpd, sqrtf(ACCEL_MM_S2 * r) * 0.75f);
  v_arc         = constrain(v_arc, 1.0f, MAX_SPEED_MM_S);
  float maxSPMM = max(STEPS_PER_MM_X, STEPS_PER_MM_Y);
  float dMin = 1000000.0f / (v_arc * maxSPMM);
  float dMax = dMin * 4.0f;

  // Tính delta góc, đảm bảo đúng chiều CW/CCW
  float delta = a_end - a_start;
  if (fabsf(delta) < 0.0001f) {
    delta = cw ? -2.0f * PI : 2.0f * PI;
  } else {
    if ( cw && delta > 0) delta -= 2.0f * PI;
    if (!cw && delta < 0) delta += 2.0f * PI;
  }

  // Số segment dựa trên chord tolerance ARC_TOLERANCE
  float tol  = min(ARC_TOLERANCE, r * 0.5f);
  float dAng = 2.0f * acosf(1.0f - tol / r);
  int   nSeg = constrain((int)ceilf(fabsf(delta) / dAng), 4, 512);

  // Ước lượng tổng step cho trapezoid velocity
  long totalEst   = max((long)(r * fabsf(delta) * maxSPMM), 1L);
  long accelSteps = (long)((v_arc * v_arc) / (2.0f * ACCEL_MM_S2) * maxSPMM);
  accelSteps      = constrain(accelSteps, 1L, totalEst / 2);

  long globalStep = 0;
  int  lastDirX   = -1, lastDirY = -1;

  for (int seg = 1; seg <= nSeg && !stopFlag; seg++) {
    float a  = a_start + delta * ((float)seg / nSeg);
    long  tx = (long)((cx + r * cosf(a)) * STEPS_PER_MM_X);
    long  ty = (long)((cy + r * sinf(a)) * STEPS_PER_MM_Y);
    long dx = tx - posX, dy = ty - posY;
    if (dx == 0 && dy == 0) continue;

    // Set DIR chỉ khi đổi chiều — tránh delay 2µs mỗi segment
    int dirX = X_DIR_LEVEL(dx), dirY = Y_DIR_LEVEL(dy);
    bool dirChanged = false;
    if (dirX != lastDirX) { digitalWrite(X_DIR_PIN, dirX); lastDirX = dirX; dirChanged = true; }
    if (dirY != lastDirY) { digitalWrite(Y_DIR_PIN, dirY); lastDirY = dirY; dirChanged = true; }
    if (dirChanged) delayMicroseconds(2);

    long adx = abs(dx), ady = abs(dy), total = max(adx, ady), err = adx - ady;

    for (long i = 0; i < total && !stopFlag; i++) {
      // Trapezoid dựa trên vị trí tuyệt đối trong toàn arc
      float ratio;
      if      (globalStep < accelSteps)            ratio = (float)(globalStep + 1) / accelSteps;
      else if (globalStep > totalEst - accelSteps) ratio = (float)(totalEst - globalStep) / accelSteps;
      else                                         ratio = 1.0f;
      ratio = constrain(ratio, 0.15f, 1.0f);

      unsigned long d = (unsigned long)(dMax - (dMax - dMin) * ratio);

      long e2 = 2L * err;
      bool sX = (e2 > -ady), sY = (e2 < adx);
      if (e2 > -ady) err -= ady;
      if (e2 <  adx) err += adx;
      if (sX) digitalWrite(X_STEP_PIN, HIGH);
      if (sY) digitalWrite(Y_STEP_PIN, HIGH);
      delayMicroseconds(5);
      if (sX) { digitalWrite(X_STEP_PIN, LOW); posX += (dx > 0 ? 1 : -1); }
      if (sY) { digitalWrite(Y_STEP_PIN, LOW); posY += (dy > 0 ? 1 : -1); }
      delayMicroseconds(d);

      globalStep++;

      if ((globalStep & 0xFF) == 0 && ESP_SERIAL.available()) {
        char c = ESP_SERIAL.peek();
        if (c == 'S' || c == 's') {
          String s = ESP_SERIAL.readStringUntil('\n'); s.trim();
          if (s.equalsIgnoreCase("STOP")) { stopFlag = true; return; }
        }
      }
    }
  }
}

/* ================================================================
   processGCode — phân tích và thực thi từng lệnh G-code
   Thứ tự xử lý:
   1. Strip comment (;) và block comment (...)
   2. Parse các tham số X Y Z I J F S
   3. Xử lý Z servo trước (non-blocking), chờ settle trước khi XY
   4. Dispatch lệnh G/M
   ================================================================ */
void processGCode(String line) {
  line.trim(); line.toUpperCase();
  int sc = line.indexOf(';'); if (sc >= 0) line = line.substring(0, sc);
  int p1 = line.indexOf('('), p2 = line.indexOf(')');
  if (p1 >= 0 && p2 > p1) line.remove(p1, p2 - p1 + 1);
  line.trim();
  if (line.length() == 0) { replyOk(); return; }

  float px = hasParm(line,'X') ? parseParm(line,'X') : NAN;
  float py = hasParm(line,'Y') ? parseParm(line,'Y') : NAN;
  float pz = hasParm(line,'Z') ? parseParm(line,'Z') : NAN;
  float pi = hasParm(line,'I') ? parseParm(line,'I') : 0.0f;
  float pj = hasParm(line,'J') ? parseParm(line,'J') : 0.0f;
  float pf = hasParm(line,'F') ? parseParm(line,'F') : NAN;
  float ps = hasParm(line,'S') ? parseParm(line,'S') : NAN;

  if (!isnan(pf)) {
    feedRate = constrain(pf / 60.0f, 0.1f, MAX_SPEED_MM_S);
    if (!mmMode) feedRate *= 25.4f;
  }

  float curX = (float)posX / STEPS_PER_MM_X;
  float curY = (float)posY / STEPS_PER_MM_Y;
  float uf   = mmMode ? 1.0f : 25.4f;
  float tx = isnan(px) ? curX : (absMode ? px * uf : curX + px * uf);
  float ty = isnan(py) ? curY : (absMode ? py * uf : curY + py * uf);

  /* Z: phát lệnh servo NGAY (non-blocking), chờ settle trước khi XY */
  if (!isnan(pz)) { zServoMoveTo(zValueToAngle(pz)); zServoWaitDone(); }

  if      (line.startsWith("G02") || line.startsWith("G2 ") || line.startsWith("G2\t")) {
    float cx = curX + pi, cy = curY + pj, r = sqrtf(pi*pi + pj*pj);
    arcInterpolate(cx, cy, r, atan2f(curY-cy, curX-cx), atan2f(ty-cy, tx-cx), true,  feedRate);
  }
  else if (line.startsWith("G03") || line.startsWith("G3 ") || line.startsWith("G3\t")) {
    float cx = curX + pi, cy = curY + pj, r = sqrtf(pi*pi + pj*pj);
    arcInterpolate(cx, cy, r, atan2f(curY-cy, curX-cx), atan2f(ty-cy, tx-cx), false, feedRate);
  }
  else if (line.startsWith("G01") || line.startsWith("G1 ") || line.startsWith("G1\t") || line.equals("G1"))
    moveTo(tx, ty, feedRate);
  else if (line.startsWith("G00") || line.startsWith("G0 ") || line.startsWith("G0\t") || line.equals("G0"))
    moveTo(tx, ty, MAX_SPEED_MM_S);
  else if (line.startsWith("G90")) absMode = true;
  else if (line.startsWith("G91")) absMode = false;
  else if (line.startsWith("G21")) mmMode  = true;
  else if (line.startsWith("G20")) mmMode  = false;
  else if (line.startsWith("G28")) {
    zServoMoveTo(Z_ANGLE_UP); zServoWaitDone();
    moveTo(0, 0, MAX_SPEED_MM_S);
  }
  else if (line.startsWith("G92")) {
    if (!isnan(px)) posX = (long)(px * uf * STEPS_PER_MM_X);
    if (!isnan(py)) posY = (long)(py * uf * STEPS_PER_MM_Y);
  }
  else if (line.startsWith("G04") || line.startsWith("G4 ")) {
    // Dwell: chờ P ms, vẫn đọc serial trong lúc chờ
    float dms = hasParm(line,'P') ? parseParm(line,'P') : 0.0f;
    unsigned long t0 = millis();
    while (millis() - t0 < (unsigned long)dms) {
      if (stopFlag) break;
      readSerialNonBlocking();
    }
  }
  else if (line.startsWith("M03") || line.startsWith("M3 ") || line.equals("M3")) {
    spindleOn = true;
    analogWrite(SPINDLE_PIN, isnan(ps) ? 128 : (int)constrain(ps / 1000.0f * 255, 0, 255));
    zServoMoveTo(Z_ANGLE_DOWN); zServoWaitDone();
  }
  else if (line.startsWith("M05") || line.equals("M5")) {
    spindleOn = false; analogWrite(SPINDLE_PIN, 0);
    zServoMoveTo(Z_ANGLE_UP); zServoWaitDone();
  }
  else if (line.startsWith("M17")) enableMotors();
  else if (line.startsWith("M18") || line.startsWith("M84")) disableMotors();
  else if (line.startsWith("M30") || line.startsWith("M2 ") || line.equals("M2")) {
    analogWrite(SPINDLE_PIN, 0); spindleOn = false;
    zServoMoveTo(Z_ANGLE_UP); zServoWaitDone();
  }
  /* Lệnh trực tiếp: "Z<angle>", ví dụ "Z30" → servo về 30° */
  else if (line.startsWith("Z") && line.length() > 1 &&
           (isDigit(line.charAt(1)) || line.charAt(1) == '-' || line.charAt(1) == '.')) {
    int a = constrain((int)line.substring(1).toFloat(), Z_ANGLE_MIN, Z_ANGLE_MAX);
    zServoMoveTo(a); zServoWaitDone();
  }
  else { Serial.print(">> skip: "); Serial.println(line); }

  replyOk();
}

/* ================================================================
   readSerialNonBlocking — đọc USB + ESP UART vào buffer
   - USB  : tích lũy ký tự vào usbLine, push khi gặp '\n'
   - ESP  : tương tự; nếu gặp "STOP" → set stopFlag + xóa buffer
   ================================================================ */
void readSerialNonBlocking() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbLine.length()) { usbLine.trim(); pushToBuffer(usbLine); usbLine = ""; }
    } else if (usbLine.length() < MAX_LINE_LEN) usbLine += c;
  }
  while (ESP_SERIAL.available()) {
    char c = ESP_SERIAL.read();
    if (c == '\n' || c == '\r') {
      if (espLine.length()) {
        espLine.trim();
        if (espLine.equalsIgnoreCase("STOP")) {
          stopFlag = true; bufHead = bufTail = 0;
          dualPrintln("ok X:- Y:- ; STOPPED");
        } else pushToBuffer(espLine);
        espLine = "";
      }
    } else if (espLine.length() < MAX_LINE_LEN) espLine += c;
  }
}

void pushToBuffer(const String& line) {
  if (!line.length()) return;
  int nh = (bufHead + 1) % CMD_BUFFER_SIZE;
  if (nh != bufTail) { cmdBuffer[bufHead] = line; bufHead = nh; }
  else dualPrintln("error: buffer full");
}

/* ================================================================
   SETUP — core0
   ================================================================ */
void setup() {
  Serial.begin(115200);
  ESP_SERIAL.setTX(ESP_TX_PIN); ESP_SERIAL.setRX(ESP_RX_PIN);
  ESP_SERIAL.begin(ESP_BAUD);

  pinMode(X_STEP_PIN,  OUTPUT); pinMode(X_DIR_PIN,   OUTPUT);
  pinMode(Y_STEP_PIN,  OUTPUT); pinMode(Y_DIR_PIN,   OUTPUT);
  pinMode(EN_PIN,      OUTPUT); pinMode(SPINDLE_PIN, OUTPUT);

  // Khởi động hardware PWM 50Hz cho servo
  zCurrentAngle = Z_ANGLE_UP; zTargetAngle = Z_ANGLE_UP;
  servoHWInit(Z_SERVO_PIN);
  delay(350);   // cho servo về vị trí UP ổn định

  enableMotors(); delay(100);

  dualPrintln("Pico CNC Online — HW PWM Servo + Optimized v4");
  dualPrintln("Z servo: GP" + String(Z_SERVO_PIN) +
              " UP=" + String(Z_ANGLE_UP) + "deg" +
              " DOWN=" + String(Z_ANGLE_DOWN) + "deg" +
              " slew=" + String(Z_SLEW_DELAY_MS) + "ms/deg [HW PWM]");
  replyOk();
}

/* ================================================================
   LOOP — core0
   Thứ tự: đọc serial → lấy lệnh từ buffer → thực thi
   - STOP  : set flag, xóa buffer
   - STATUS: báo cáo trạng thái đầy đủ (không gọi processGCode)
   - Các lệnh G/M: chuyển sang processGCode()
   ================================================================ */
void loop() {
  readSerialNonBlocking();
  if (bufHead == bufTail) return;

  String cmd = cmdBuffer[bufTail];
  bufTail = (bufTail + 1) % CMD_BUFFER_SIZE;

  if (stopFlag && bufHead == bufTail) stopFlag = false;
  if (stopFlag) return;

  if      (cmd.equalsIgnoreCase("PENUP"))   { zServoMoveTo(Z_ANGLE_UP);   zServoWaitDone(); replyOk(); }
  else if (cmd.equalsIgnoreCase("PENDOWN")) { zServoMoveTo(Z_ANGLE_DOWN); zServoWaitDone(); replyOk(); }
  else if (cmd.equalsIgnoreCase("STATUS")) {
    dualPrintln("ok X:" + String((float)posX / STEPS_PER_MM_X, 3) +
                " Y:"   + String((float)posY / STEPS_PER_MM_Y, 3) +
                " Z:"   + String((int)zCurrentAngle) +
                " F:"   + String(feedRate * 60, 0) +
                " "     + String(absMode ? "ABS" : "REL") +
                " SP:"  + String(spindleOn ? "ON" : "OFF") +
                " SERVO:" + String(zMoving ? "MOVING" : zSettling ? "SETTLE" : "IDLE"));
  }
  else if (cmd.equalsIgnoreCase("HOLD"))    { enableMotors();  dualPrintln("ok X:- Y:-"); }
  else if (cmd.equalsIgnoreCase("RELEASE")) { disableMotors(); dualPrintln("ok X:- Y:-"); }
  else if (cmd.equalsIgnoreCase("STOP"))    {
    stopFlag = true; bufHead = bufTail = 0;
    dualPrintln("ok X:- Y:- ; STOPPED");
  }
  else processGCode(cmd);
}
