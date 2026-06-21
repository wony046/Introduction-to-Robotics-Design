#include <EEPROM.h>

// ── EEPROM ──────────────────────────────────────────────────────────────────
#define EEPROM_ADDR  0
#define CAL_MAGIC    0xC0FFEE01
#define CAL_VERSION  3
#define CAL_STEPS    18

struct CalibData {
  uint32_t magic;
  uint16_t version;
  float    cal_speed_L[CAL_STEPS];
  float    cal_speed_R[CAL_STEPS];
  float    max_avg_speed;
};

// ── 핀 ──────────────────────────────────────────────────────────────────────
#define PWM_L  6
#define DIR1_L 7
#define DIR2_L 8
#define PWM_R  9
#define DIR1_R 10
#define DIR2_R 11
#define ENC_L_A 3
#define ENC_L_B 5
#define ENC_R_A 2
#define ENC_R_B 4

//____
#define LINE_DRIVE_V 0.12f   // 직진 속도 [m/s] (느리게 = 슬립↓)
#define LINE_SETTLE  40      // 정지 후 안정화 틱 (×5ms)

// ── 로봇 파라미터 ────────────────────────────────────────────────────────────
const float WHEEL_BASE = 0.1796f;
const float M_TO_PULSE = 4795.0f;

// ── 오도메트리 ──────────────────────────────────────────────────────────────
volatile long encLeft  = 0;
volatile long encRight = 0;
long  prevL = 0, prevR = 0;
float heading = 0.0f;
float odom_x  = 0.0f;   // m 단위 (전방 +Y, 좌측 +X)
float odom_y  = 0.0f;

const float WHEEL_RATIO = 0.00568f;
const float MPP_L = (1.0f / 4795.0f) * (1.0f - WHEEL_RATIO / 2.0f);
const float MPP_R = (1.0f / 4795.0f) * (1.0f + WHEEL_RATIO / 2.0f);

bool ffEnabled = true;

// ── 타임아웃 ─────────────────────────────────────────────────────────────────
const unsigned long CMD_TIMEOUT_MS = 500;
unsigned long lastCmdTime = 0;
bool timedOut = false;

// ── 타이밍 ───────────────────────────────────────────────────────────────────
unsigned long lastOdoTime     = 0;
unsigned long lastHeadingSend = 0;
unsigned long lastPrintTime   = 0;
const unsigned long ODO_INTERVAL_MS = 10;
const unsigned long HEADING_SEND_MS = 50;  // 20Hz 전송

// ── FF 캘리브레이션 ──────────────────────────────────────────────────────────
float cal_pwm[CAL_STEPS] = {
  50, 60, 70, 80, 90, 100,
  110, 120, 130, 140, 150, 160, 170, 180,
  200, 220, 240, 255
};
float cal_speed_L[CAL_STEPS];
float cal_speed_R[CAL_STEPS];
float max_avg_speed = 0.0f;


// ─────────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(PWM_L,  OUTPUT); pinMode(DIR1_L, OUTPUT); pinMode(DIR2_L, OUTPUT);
  pinMode(PWM_R,  OUTPUT); pinMode(DIR1_R, OUTPUT); pinMode(DIR2_R, OUTPUT);
  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeftA,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRightA, CHANGE);
  delay(500);

  Serial.println(F("3초 안에 'c' → 재캘리브레이션"));
  bool forceCalib = false;
  unsigned long t0 = millis();
  while (millis() - t0 < 3000) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'c' || c == 'C') { forceCalib = true; break; }
    }
  }
  bool loaded = false;
  if (!forceCalib) {
    loaded = loadCalibFromEEPROM();
    Serial.println(loaded ? F("[EEPROM] 로드 성공") : F("[EEPROM] 없음 → 캘리 진행"));
  }
  if (forceCalib || !loaded) {
    autoCalibrateMotors();
    saveCalibToEEPROM();
  }

  stopMotors();

  noInterrupts();
  prevL = encLeft;
  prevR = encRight;
  interrupts();
  heading = 0.0f;

  lastCmdTime = millis();

  Serial.println(F("[READY] 라즈베리파이 명령 대기"));
  Serial.println(F("  \"v w\\n\" : 속도 명령"));
  Serial.println(F("  PC: CAL / CHK / R / H"));
}

void runLine(float dist) {
  stopMotors(); delay(600);

  long startL, startR;
  noInterrupts(); startL = encLeft; startR = encRight; interrupts();

  // odom 거리 기준 dist 만큼 직진 (헤딩 피드백 없음 = 순수 구름거리 측정)
  float trav = 0.0f;
  while (trav < dist) {
    long cL, cR;
    noInterrupts(); cL = encLeft; cR = encRight; interrupts();
    trav = ((cL - startL) * MPP_L + (cR - startR) * MPP_R) * 0.5f;
    setVelocity(LINE_DRIVE_V, 0.0f);
    updateOdometry();
    delay(5);
  }
  stopMotors();
  for (int i = 0; i < LINE_SETTLE; i++) { updateOdometry(); delay(5); }

  long eL, eR;
  noInterrupts(); eL = encLeft; eR = encRight; interrupts();
  long dL  = eL - startL;
  long dR  = eR - startR;
  long avg = (dL + dR) / 2;

  float imbal = (avg != 0) ? (float)(dR - dL) / avg * 100.0f : 0.0f;

  Serial.print(F("[LINE] FF=")); Serial.print(ffEnabled ? F("ON") : F("OFF"));
  Serial.print(F(" odom="));     Serial.print(trav, 3);
  Serial.print(F("m  L="));      Serial.print(dL);
  Serial.print(F("  R="));       Serial.print(dR);
  Serial.print(F("  avg="));     Serial.print(avg);
  Serial.print(F("  불균형="));   Serial.print(imbal, 1); Serial.println(F("%"));

  Serial1.print(F("LINE_RESULT:"));
  Serial1.print(trav, 4); Serial1.print(',');
  Serial1.print(dL);      Serial1.print(',');
  Serial1.print(dR);      Serial1.print(',');
  Serial1.println(avg);
}


// ─────────────────────────────────────────────────────────────────────────────
// 오도메트리
// ─────────────────────────────────────────────────────────────────────────────
void updateOdometry() {
  long safeL, safeR;
  noInterrupts(); safeL = encLeft; safeR = encRight; interrupts();
  float dsL = (safeL - prevL) * MPP_L;
  float dsR = (safeR - prevR) * MPP_R;
  prevL = safeL; prevR = safeR;
  float ds  = (dsL + dsR) * 0.5f;
  heading  += (dsR - dsL) / WHEEL_BASE;
  odom_x   += ds * sinf(heading); # 오도메트리 x 위치 (mm, 좌측 +)
  odom_y   += ds * cosf(heading);
}


// ─────────────────────────────────────────────────────────────────────────────
// setVelocity
// ─────────────────────────────────────────────────────────────────────────────
void setVelocity(float v, float w) {
  if (abs(v) < 0.001f && abs(w) < 0.001f) { stopMotors(); return; }

  float v_left  = v - (WHEEL_BASE * w / 2.0f);
  float v_right = v + (WHEEL_BASE * w / 2.0f);

  float pwm_L, pwm_R;
  if (ffEnabled) {
    pwm_L = getFeedforwardPWM_L(abs(v_left)  * M_TO_PULSE);
    pwm_R = getFeedforwardPWM_R(abs(v_right) * M_TO_PULSE);
  } else {
    pwm_L = getRawPWM(abs(v_left)  * M_TO_PULSE);
    pwm_R = getRawPWM(abs(v_right) * M_TO_PULSE);
  }

  if (v_left  < 0) pwm_L = -pwm_L;
  if (v_right < 0) pwm_R = -pwm_R;

  driveMotorLeft(pwm_L);
  driveMotorRight(pwm_R);

  if (millis() - lastPrintTime >= 100) {
    Serial.print(F("vL="));  Serial.print(v_left, 3);
    Serial.print(F(" vR=")); Serial.print(v_right, 3);
    Serial.print(F(" hdg=")); Serial.print(heading * 57.3f, 1);
    Serial.print(F(" FF="));  Serial.print(ffEnabled ? F("ON") : F("OFF"));
    Serial.println();
    lastPrintTime = millis();
  }
}


// ─────────────────────────────────────────────────────────────────────────────
// Main Loop
// ─────────────────────────────────────────────────────────────────────────────
void loop() {

  // ── 오도메트리 (10ms) ────────────────────────────────────────────────────
  if (millis() - lastOdoTime >= ODO_INTERVAL_MS) {
    updateOdometry();
    lastOdoTime = millis();
  }

  // ── 헤딩 전송 (50ms) ─────────────────────────────────────────────────────
  if (millis() - lastHeadingSend >= HEADING_SEND_MS) {
    Serial1.print(F("O:"));
    Serial1.print(odom_x * 1000.0f, 0);   // m → mm
    Serial1.print(F(","));
    Serial1.print(odom_y * 1000.0f, 0);
    Serial1.print(F(","));
    Serial1.println(heading * 57.2958f, 1);
    lastHeadingSend = millis();
  }

  // ── 타임아웃 ─────────────────────────────────────────────────────────────
  if (millis() - lastCmdTime > CMD_TIMEOUT_MS) {
    if (!timedOut) {
      stopMotors();
      timedOut = true;
      Serial.println(F("[TIMEOUT] 속도 정지"));
    }
  }

  // ── PC 시리얼 (디버깅용) ──────────────────────────────────────────────────
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if      (cmd == "CAL" || cmd == "cal") { stopMotors(); autoCalibrateMotors(); saveCalibToEEPROM(); checkCalQuality(); checkFFHoldout(); }
    else if (cmd == "chk" || cmd == "CHK") { checkCalQuality(); checkFFHoldout(); }
    else if (cmd == "R"   || cmd == "r")   { heading = 0.0f; odom_x = 0.0f; odom_y = 0.0f; Serial.println(F("[R] 헤딩+위치 리셋")); }
    else if (cmd == "H"   || cmd == "h")   { Serial.print(F("[H] ")); Serial.print(heading * 57.3f, 1); Serial.println(F("°")); }
    else if (cmd == "FF"  || cmd == "ff")  { ffEnabled = !ffEnabled; Serial.print(F("[FF] ")); Serial.println(ffEnabled ? F("ON") : F("OFF")); }
  }

  // ── 라즈베리파이 UART 수신 ───────────────────────────────────────────────
  if (Serial1.available()) {
    String input = Serial1.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    lastCmdTime = millis();
    timedOut    = false;

    // ── 헤딩 리셋 명령 (라즈베리파이 시작 시 자동 전송) ─────────────────
    if (input == "R" || input == "r") {
      heading = 0.0f; odom_x = 0.0f; odom_y = 0.0f;
      Serial.println(F("[R] 헤딩+위치 리셋 (from RPi)"));
      return;
    }

    // ── FF 토글 ─────────────────────────────────────────────────────────────
    if (input == "FF" || input == "ff") {
      ffEnabled = !ffEnabled;
      Serial.print(F("[FF] ")); Serial.println(ffEnabled ? F("ON") : F("OFF"));
      return;
    }

    // ── FF 캘리브레이션 (원격 트리거) ─────────────────────────────────────────
    if (input == "CAL" || input == "cal") {
      stopMotors();
      autoCalibrateMotors();
      saveCalibToEEPROM();
      checkCalQuality();
      checkFFHoldout();
      lastCmdTime = millis();
      return;
    }

    // ── 거리 캘리브레이션 트리거 ───────────────────────────
    if (input.startsWith("LINE")) {
      float d = input.substring(4).toFloat();
      if (d <= 0) d = 3.0f;
      runLine(d);
      lastCmdTime = millis();   // 직후 타임아웃 메시지 방지
      return;
    }

    // ── v w 명령 파싱 ────────────────────────────────────────────────────
    int spaceIdx = input.indexOf(' ');
    if (spaceIdx > 0) {
      float v = input.substring(0, spaceIdx).toFloat();
      float w = input.substring(spaceIdx + 1).toFloat();
      setVelocity(v, w);
    } else {
      Serial.print(F("[WARN] 파싱실패: ")); Serial.println(input);
    }
  }
}


// ─────────────────────────────────────────────────────────────────────────────
// 모터 / ISR / FF / EEPROM / 캘리브레이션
// ─────────────────────────────────────────────────────────────────────────────
void driveMotorLeft(float output) {
  int pwm = constrain((int)abs(output), 0, 255);
  if      (output > 0) { digitalWrite(DIR1_L, HIGH); digitalWrite(DIR2_L, LOW); }
  else if (output < 0) { digitalWrite(DIR1_L, LOW);  digitalWrite(DIR2_L, HIGH); }
  else                 { digitalWrite(DIR1_L, LOW);  digitalWrite(DIR2_L, LOW);  }
  analogWrite(PWM_L, pwm);
}

void driveMotorRight(float output) {
  int pwm = constrain((int)abs(output), 0, 255);
  if      (output > 0) { digitalWrite(DIR1_R, HIGH); digitalWrite(DIR2_R, LOW); }
  else if (output < 0) { digitalWrite(DIR1_R, LOW);  digitalWrite(DIR2_R, HIGH); }
  else                 { digitalWrite(DIR1_R, LOW);  digitalWrite(DIR2_R, LOW);  }
  analogWrite(PWM_R, pwm);
}

void stopMotors() {
  digitalWrite(DIR1_L, LOW); digitalWrite(DIR2_L, LOW);
  digitalWrite(DIR1_R, LOW); digitalWrite(DIR2_R, LOW);
  analogWrite(PWM_L, 0); analogWrite(PWM_R, 0);
}

void isrLeftA()  { bool a=digitalRead(ENC_L_A),b=digitalRead(ENC_L_B); if(a==b) encLeft++;  else encLeft--;  }
void isrRightA() { bool a=digitalRead(ENC_R_A),b=digitalRead(ENC_R_B); if(a!=b) encRight++; else encRight--; }

float getFeedforwardPWM_L(float t) {
  if (t<=0) return 0;
  if (t<cal_speed_L[0]) return cal_pwm[0]*t/cal_speed_L[0];
  if (t>=cal_speed_L[CAL_STEPS-1]) return cal_pwm[CAL_STEPS-1];
  for (int i=0;i<CAL_STEPS-1;i++)
    if (t>=cal_speed_L[i]&&t<=cal_speed_L[i+1])
      return cal_pwm[i]+(t-cal_speed_L[i])*(cal_pwm[i+1]-cal_pwm[i])/(cal_speed_L[i+1]-cal_speed_L[i]);
  return 0;
}

float getFeedforwardPWM_R(float t) {
  if (t<=0) return 0;
  if (t<cal_speed_R[0]) return cal_pwm[0]*t/cal_speed_R[0];
  if (t>=cal_speed_R[CAL_STEPS-1]) return cal_pwm[CAL_STEPS-1];
  for (int i=0;i<CAL_STEPS-1;i++)
    if (t>=cal_speed_R[i]&&t<=cal_speed_R[i+1])
      return cal_pwm[i]+(t-cal_speed_R[i])*(cal_pwm[i+1]-cal_pwm[i])/(cal_speed_R[i+1]-cal_speed_R[i]);
  return 0;
}

// 좌우 동일 매핑 (평균 테이블 보간) — ffEnabled=false 시 사용
float getRawPWM(float t) {
  if (t <= 0) return 0;
  float s0 = (cal_speed_L[0]           + cal_speed_R[0])           * 0.5f;
  float sN = (cal_speed_L[CAL_STEPS-1] + cal_speed_R[CAL_STEPS-1]) * 0.5f;
  if (t <  s0) return cal_pwm[0] * t / s0;
  if (t >= sN) return cal_pwm[CAL_STEPS-1];
  for (int i = 0; i < CAL_STEPS-1; i++) {
    float si  = (cal_speed_L[i]   + cal_speed_R[i])   * 0.5f;
    float si1 = (cal_speed_L[i+1] + cal_speed_R[i+1]) * 0.5f;
    if (t >= si && t <= si1)
      return cal_pwm[i] + (t - si) * (cal_pwm[i+1] - cal_pwm[i]) / (si1 - si);
  }
  return 0;
}

void saveCalibToEEPROM() {
  CalibData d; d.magic=CAL_MAGIC; d.version=CAL_VERSION;
  for (int i=0;i<CAL_STEPS;i++){d.cal_speed_L[i]=cal_speed_L[i];d.cal_speed_R[i]=cal_speed_R[i];}
  d.max_avg_speed=max_avg_speed; EEPROM.put(EEPROM_ADDR,d);
  Serial.println(F("[EEPROM] 저장"));
}

bool loadCalibFromEEPROM() {
  CalibData d; EEPROM.get(EEPROM_ADDR,d);
  if (d.magic!=CAL_MAGIC||d.version!=CAL_VERSION) return false;
  for (int i=0;i<CAL_STEPS;i++){cal_speed_L[i]=d.cal_speed_L[i];cal_speed_R[i]=d.cal_speed_R[i];}
  max_avg_speed=d.max_avg_speed; return true;
}

// ── 캘리브레이션 양방향 헬퍼 (Serial + Serial1 동시 출력) ───────────────────
static void cPrint(const __FlashStringHelper* s)   { Serial.print(s);      Serial1.print(s); }
static void cPrint(float v, int d = 0)             { Serial.print(v, d);   Serial1.print(v, d); }
static void cPrint(int v)                          { Serial.print(v);      Serial1.print(v); }
static void cPrintln(const __FlashStringHelper* s) { Serial.println(s);    Serial1.println(s); }
static void cPrintln(float v, int d = 1)           { Serial.println(v, d); Serial1.println(v, d); }
static void cPrintln()                             { Serial.println();     Serial1.println(); }

// Serial 또는 Serial1 어느 쪽에서든 n/q 입력 대기
static char calReadChar() {
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'n' || c == 'N' || c == 'q' || c == 'Q') return c;
    }
    if (Serial1.available()) {
      char c = Serial1.read();
      if (c == 'n' || c == 'N' || c == 'q' || c == 'Q') return c;
    }
  }
}

void autoCalibrateMotors() {
  cPrintln(F("\n[CAL] 시작 (n=측정, q=중단) — PWM당 3회 중앙값"));
  delay(2000);
  const float CAL_WINDOW_S = 1.5f;
  for (int i = 0; i < CAL_STEPS; i++) {
    float sampL[3], sampR[3];
    for (int r = 0; r < 3; r++) {
      cPrint(F("\n[준비] PWM ")); cPrint(cal_pwm[i]);
      cPrint(F(" (")); cPrint(r + 1); cPrintln(F("/3) → 'n'"));
      char c = calReadChar();
      if (c == 'q' || c == 'Q') { stopMotors(); return; }
      digitalWrite(DIR1_L, HIGH); digitalWrite(DIR2_L, LOW);
      digitalWrite(DIR1_R, HIGH); digitalWrite(DIR2_R, LOW);
      analogWrite(PWM_L, (int)cal_pwm[i]); analogWrite(PWM_R, (int)cal_pwm[i]);
      delay(1000);
      long sL, sR; noInterrupts(); sL = encLeft; sR = encRight; interrupts();
      delay((unsigned long)(CAL_WINDOW_S * 1000));
      long eL, eR; noInterrupts(); eL = encLeft; eR = encRight; interrupts();
      sampL[r] = abs(eL - sL) / CAL_WINDOW_S;
      sampR[r] = abs(eR - sR) / CAL_WINDOW_S;
      stopMotors();
      cPrint(F("[측정")); cPrint(r + 1); cPrint(F("] spdL="));
      cPrint(sampL[r], 1); cPrint(F(" spdR=")); cPrintln(sampR[r], 1);
      if (r < 2) cPrintln(F("[원위치로 이동 후 'n']"));
    }
    // 3개 버블 정렬 → 중앙값(index 1) 채택
    if (sampL[0] > sampL[1]) { float t = sampL[0]; sampL[0] = sampL[1]; sampL[1] = t; }
    if (sampL[1] > sampL[2]) { float t = sampL[1]; sampL[1] = sampL[2]; sampL[2] = t; }
    if (sampL[0] > sampL[1]) { float t = sampL[0]; sampL[0] = sampL[1]; sampL[1] = t; }
    if (sampR[0] > sampR[1]) { float t = sampR[0]; sampR[0] = sampR[1]; sampR[1] = t; }
    if (sampR[1] > sampR[2]) { float t = sampR[1]; sampR[1] = sampR[2]; sampR[2] = t; }
    if (sampR[0] > sampR[1]) { float t = sampR[0]; sampR[0] = sampR[1]; sampR[1] = t; }
    cal_speed_L[i] = sampL[1];
    cal_speed_R[i] = sampR[1];
    cPrint(F("[중앙값] spdL=")); cPrint(cal_speed_L[i], 1);
    cPrint(F(" spdR=")); cPrintln(cal_speed_R[i], 1);
  }
  max_avg_speed = (cal_speed_L[CAL_STEPS-1] + cal_speed_R[CAL_STEPS-1]) / 2.0f;
  noInterrupts(); encLeft = 0; encRight = 0; interrupts();
  cPrintln(F("[CAL] 완료"));
}

void checkFFHoldout() {
  cPrintln(F("\n[FF CHECK 1] Hold-out"));
  for (int i=1;i<CAL_STEPS-1;i++) {
    float dL=cal_speed_L[i+1]-cal_speed_L[i-1],dR=cal_speed_R[i+1]-cal_speed_R[i-1];
    if(dL<0.5f||dR<0.5f) continue;
    float pL=cal_pwm[i-1]+((cal_speed_L[i]-cal_speed_L[i-1])/dL)*(cal_pwm[i+1]-cal_pwm[i-1]);
    float pR=cal_pwm[i-1]+((cal_speed_R[i]-cal_speed_R[i-1])/dR)*(cal_pwm[i+1]-cal_pwm[i-1]);
    cPrint(i); cPrint(F("\t")); cPrint(cal_pwm[i]); cPrint(F("\t"));
    cPrint(pL,1); cPrint(F("\t")); cPrint(pL-cal_pwm[i],1); cPrint(F("\t"));
    cPrint(pR,1); cPrint(F("\t")); cPrintln(pR-cal_pwm[i],1);
  }
}

void checkCalQuality() {
  cPrintln(F("\n[FF CHECK 2] 품질"));
  for (int i=0;i<CAL_STEPS;i++){
    cPrint(i); cPrint(F("\t")); cPrint(cal_pwm[i]);
    cPrint(F("\t")); cPrint(cal_speed_L[i]); cPrint(F("\t")); cPrintln(cal_speed_R[i], 0);
  }
}
