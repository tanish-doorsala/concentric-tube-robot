// Concentric Push-Pull Tube Robot — Closed Loop Control
// Serial: 9600 baud, newline terminated
//
// Commands:
//   R:<deg>   — rotation absolute
//   L:<mm>    — linear absolute (now supports negative and positive travel)
//   LR:<mm>   — linear RELATIVE (+ extends, - retracts)
//   S:<deg>   — stepper twist absolute
//   RKP/RKI/RKD/LKP/LKI/LKD:<val>  — live PID tuning
//   HOME      — zero all encoders, return to 0,0,0
//
// Reports every 100ms:
//   ENC:rot,lin,step|SP:rot,lin,step|OUT:rot,lin

#include <Encoder.h>
#include <PID_v1.h>

// ── Calibration ──────────────────────────────────────────────
const float ENC_COUNTS_PER_DEG_ROT = 7.96;     // was 8.30
const float ENC_COUNTS_PER_MM_LIN  = 79.26;    // ~2014 counts per inch
const float MAX_EXTENSION_MM       = 76.2;     // 3 inches (keep)
const float STEPPER_GEAR_RATIO     = 16.0;     // keep

// Derived
const float STEPS_PER_DEG = (200.0 * STEPPER_GEAR_RATIO) / 360.0;

// ── Pins ─────────────────────────────────────────────────────
#define DIR_PIN   10
#define STEP_PIN  11
#define MOT_A1    12    // Linear motor
#define MOT_A2    13
#define MOT_B1     9    // Rotational motor
#define MOT_B2     8
#define ENA        5    // Linear PWM enable
#define ENB        6    // Rotational PWM enable

// ── Encoders ─────────────────────────────────────────────────
Encoder encLin(2, 7);
Encoder encRot(3, 4);

// ── PID ──────────────────────────────────────────────────────
double encRot_val = 0, outRot = 0, spRot = 0;
double encLin_val = 0, outLin = 0, spLin = 0;

double Kp_R = 0.10, Ki_R = 0.00, Kd_R = 0.00;
double Kp_L = 0.30, Ki_L = 0.00, Kd_L = 0.00;

PID pidRot(&encRot_val, &outRot, &spRot, Kp_R, Ki_R, Kd_R, DIRECT);
PID pidLin(&encLin_val, &outLin, &spLin, Kp_L, Ki_L, Kd_L, DIRECT);

// ── Stepper ──────────────────────────────────────────────────
volatile bool     stepState     = false;
volatile long     stepPos       = 0;
long              stepTarget    = 0;
bool              stepRunning   = false;

// ── Deadzones ────────────────────────────────────────────────
const int DZ_PWM = 10;   // min PWM to avoid stall hum
const int DZ_ENC = 10;   // encoder window where motor stops

// ── Slew rate (PWM ramp) ─────────────────────────────────────
const int SLEW_MAX = 8;  // max PWM change per loop (~8 per ms = smooth ramp)
int lastPwmRot = 0;
int lastPwmLin = 0;

// ── Encoder spike filter ─────────────────────────────────────
const long ENC_MAX_JUMP = 150;  // max plausible counts between reads
double prevEncRot = 0;
double prevEncLin = 0;

// ── Serial ───────────────────────────────────────────────────
String serialBuf = "";
unsigned long lastReport = 0;



// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  Serial.println("CTR ready. Commands: R/L/LR/S:<val> | RKP..LKD:<val> | HOME");
 

  pinMode(STEP_PIN, OUTPUT); pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA, OUTPUT);      pinMode(ENB, OUTPUT);
  pinMode(MOT_A1, OUTPUT);   pinMode(MOT_A2, OUTPUT);
  pinMode(MOT_B1, OUTPUT);   pinMode(MOT_B2, OUTPUT);

  pidRot.SetOutputLimits(-180, 180); pidRot.SetMode(AUTOMATIC);
  pidLin.SetOutputLimits(-180, 180); pidLin.SetMode(AUTOMATIC);

  // Timer1 for stepper (CTC, prescaler 8 → 0.5µs/tick)
  noInterrupts();
  TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
  OCR1A = 400;
  TCCR1B |= (1 << WGM12) | (1 << CS11);
  interrupts();
}

void loop() {
  readSerial();

  // Read encoders with spike rejection
  double rawRot = (double)encRot.read();
  double rawLin = (double)encLin.read();

  if (abs(rawRot - prevEncRot) > ENC_MAX_JUMP) {
    encRot.write((long)prevEncRot);  // rewrite last-known-good value
    rawRot = prevEncRot;
  }
  if (abs(rawLin - prevEncLin) > ENC_MAX_JUMP) {
    encLin.write((long)prevEncLin);
    rawLin = prevEncLin;
  }
  encRot_val = rawRot;  prevEncRot = rawRot;
  encLin_val = rawLin;  prevEncLin = rawLin;

  pidRot.SetTunings(Kp_R, Ki_R, Kd_R);
  pidLin.SetTunings(Kp_L, Ki_L, Kd_L);
  pidRot.Compute();
  pidLin.Compute();

  lastPwmRot = driveMotorRamped(MOT_B1, MOT_B2, ENB, outRot, encRot_val, spRot, lastPwmRot);
  lastPwmLin = driveMotorRamped(MOT_A1, MOT_A2, ENA, outLin, encLin_val, spLin, lastPwmLin);

  if (stepRunning && abs(stepPos - stepTarget) < 2) {
    setStepperSpeed(0);
    stepRunning = false;
  }

  if (millis() - lastReport >= 100) {
    lastReport = millis();
    Serial.print("ENC:"); Serial.print(encRot_val); Serial.print(",");
    Serial.print(encLin_val); Serial.print(","); Serial.print(stepPos);
    Serial.print("|SP:"); Serial.print(spRot); Serial.print(",");
    Serial.print(spLin); Serial.print(","); Serial.print(stepTarget);
    Serial.print("|OUT:"); Serial.print(outRot); Serial.print(",");
    Serial.println(outLin); 
  
  }
}

// ── Drive a DC motor with slew-rate limiting ─────────────────
int driveMotorRamped(int fwd, int rev, int pwmPin, double output, double enc, double sp, int prevPwm) {
  int targetPwm;
  if (enc > sp - DZ_ENC && enc < sp + DZ_ENC) {
    targetPwm = 0;
  } else {
    targetPwm = constrain((int)abs(output), DZ_PWM, 255);
    if (output < 0) targetPwm = -targetPwm;  // encode direction in sign
  }

  // Slew-rate limit: ramp toward target
  int diff = targetPwm - prevPwm;
  if (diff > SLEW_MAX)       prevPwm += SLEW_MAX;
  else if (diff < -SLEW_MAX) prevPwm -= SLEW_MAX;
  else                        prevPwm = targetPwm;

  int pwmOut = abs(prevPwm);
  if (pwmOut < DZ_PWM) pwmOut = 0;  // below threshold = off

  digitalWrite(rev, prevPwm < 0 ? HIGH : LOW);
  digitalWrite(fwd, prevPwm > 0 ? HIGH : LOW);
  analogWrite(pwmPin, pwmOut);
  return prevPwm;
}

// ── Stepper speed: -255 to 255, 0 = stop ─────────────────────
void setStepperSpeed(int speed) {
  if (speed == 0) { TIMSK1 &= ~(1 << OCIE1A); return; }
  OCR1A = map(constrain(abs(speed), 1, 255), 1, 255, 1428, 400);
  digitalWrite(DIR_PIN, speed > 0 ? HIGH : LOW);
  TIMSK1 |= (1 << OCIE1A);
}

// ── Serial parser ─────────────────────────────────────────────
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuf.length() > 0) { parseCommand(serialBuf); serialBuf = ""; }
    } else {
      serialBuf += c;
    }
  }
}

void parseCommand(String cmd) {
  cmd.trim();

  if (cmd == "HOME") {
    encRot.write(0); encLin.write(0);
    stepPos = 0; stepTarget = 0;
    spRot = 0; spLin = 0;
    setStepperSpeed(0);
    Serial.println("HOME: zeroed.");
    return;
  }

  int start = 0;
  while (start < (int)cmd.length()) {
    int comma = cmd.indexOf(',', start);
    String token = (comma == -1) ? cmd.substring(start) : cmd.substring(start, comma);
    start = (comma == -1) ? cmd.length() : comma + 1;
    token.trim();

    int colon = token.indexOf(':');
    if (colon == -1) continue;
    String key = token.substring(0, colon);
    float  val = token.substring(colon + 1).toFloat();

    if      (key == "R")   spRot = val * ENC_COUNTS_PER_DEG_ROT;
    else if (key == "L") {
      // absolute linear target in mm, allow both directions
      float mm = constrain(val, -MAX_EXTENSION_MM, MAX_EXTENSION_MM);
      spLin = mm * ENC_COUNTS_PER_MM_LIN;
    }
    else if (key == "LR") {
      // relative linear move in mm, allow both directions
      float newMM = (spLin / ENC_COUNTS_PER_MM_LIN) + val;
      newMM = constrain(newMM, -MAX_EXTENSION_MM, MAX_EXTENSION_MM);
      spLin = newMM * ENC_COUNTS_PER_MM_LIN;
    }
    else if (key == "S") {
      stepTarget = (long)(val * STEPS_PER_DEG);
      long err = stepTarget - stepPos;
      if (abs(err) > 2) { stepRunning = true; setStepperSpeed(err > 0 ? 150 : -150); }
    }
    else if (key == "RKP") Kp_R = val;
    else if (key == "RKI") Ki_R = val;
    else if (key == "RKD") Kd_R = val;
    else if (key == "LKP") Kp_L = val;
    else if (key == "LKI") Ki_L = val;
    else if (key == "LKD") Kd_L = val;
    else { Serial.print("Unknown key: "); Serial.println(key); }
  }
}

// ── Stepper ISR ───────────────────────────────────────────────
ISR(TIMER1_COMPA_vect) {
  stepState = !stepState;
  digitalWrite(STEP_PIN, stepState);
  if (stepState) {
    if (digitalRead(DIR_PIN) == HIGH) stepPos++;
    else                              stepPos--;
  }
}