// ─────────────────────────────────────────────────────────────
// Linear (mm) + Rotary (deg) + Stepper (deg)
//
// Serial commands:
//   L:<mm>   Linear position   e.g. L:25.4 ~ 1 inch
//   R:<deg>  Rotary position   e.g. R:360
//   S:<deg>  Stepper angle     e.g. S:360
//   H        Home all axes
// ─────────────────────────────────────────────────────────────

#include <Encoder.h>
#include <PID_v1.h>
#include <AccelStepper.h>

// ── Pins ──────────────────────────────────────────────────────
#define DIR_PIN   10
#define STEP_PIN  11
#define MOT_A1    12
#define MOT_A2    13
#define MOT_B1     9
#define MOT_B2     8
#define ENA        5
#define ENB        6

// ── Encoders ──────────────────────────────────────────────────
Encoder encLin(3, 4);
Encoder encRot(2, 7);

// ── Stepper ───────────────────────────────────────────────────
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

const float STEPPER_GEAR_RATIO = 16.0;
const float STEPS_PER_DEG      = (200.0 * STEPPER_GEAR_RATIO) / 360.0;
const float MAX_STEPPER_DEG    = 360.0f;
const float STEPPER_MAX_SPEED  = 800.0;
const float STEPPER_ACCEL      = 400.0;

// ── Calibration ───────────────────────────────────────────────
const float ENC_COUNTS_PER_MM  = 1970.0 / 25.4f;
const float ENC_COUNTS_PER_DEG = 7.9;
const float MAX_EXTENSION_MM   = 76.2f;
const float MAX_ROTATION_DEG   = 360.0f;

// ── Linear PID ────────────────────────────────────────────────
double linPos = 0, linTarget = 0, linOutput = 0;
double linKp = 4.0, linKi = 0.5, linKd = 0.3;
const double LIN_DEADBAND = 0.3;
const int    LIN_PWM_MIN  = 45;
const int    LIN_PWM_MAX  = 120;
PID linPID(&linPos, &linOutput, &linTarget, linKp, linKi, linKd, DIRECT);

// ── Rotary PID ────────────────────────────────────────────────
double rotPos = 0, rotTarget = 0, rotOutput = 0;
double rotKp = 6.0;
double rotKi = 0.05;
double rotKd = 0.4;
const double ROT_DEADBAND = 1.5;
const int ROT_PWM_MIN = 40; 
const int    ROT_PWM_MAX  = 120;
PID rotPID(&rotPos, &rotOutput, &rotTarget, rotKp, rotKi, rotKd, DIRECT);

// ── State ─────────────────────────────────────────────────────
bool linMoving  = false;
bool rotMoving  = false;
bool stepMoving = false;

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(MOT_A1, OUTPUT); pinMode(MOT_A2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(MOT_B1, OUTPUT); pinMode(MOT_B2, OUTPUT); pinMode(ENB, OUTPUT);

  stopLinear(); stopRotary();
  encLin.write(0); encRot.write(0);

  linPID.SetOutputLimits(-LIN_PWM_MAX, LIN_PWM_MAX);
  linPID.SetSampleTime(20);
  linPID.SetMode(AUTOMATIC);

  rotPID.SetOutputLimits(-ROT_PWM_MAX, ROT_PWM_MAX);
  rotPID.SetSampleTime(20);
  rotPID.SetMode(AUTOMATIC);


  stepper.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper.setAcceleration(STEPPER_ACCEL);
  stepper.setCurrentPosition(0);

  Serial.println("Triple-axis ready.");
  Serial.println("L:<mm>  R:<deg>  S:<deg>  H=home all");
}

// ─────────────────────────────────────────────────────────────
void loop() {
  handleSerial();
  runLinear();
  runRotary();
  runStepper();
}

// ── Linear PID loop ───────────────────────────────────────────
void runLinear() {
  if (!linMoving) return;

  linPos = (encLin.read() * -1.0f) / ENC_COUNTS_PER_MM;
  double err = linTarget - linPos;

  if (abs(err) <= LIN_DEADBAND) {
    stopLinear();
    linMoving = false;
    Serial.print(">> LIN REACHED: ");
    Serial.print(linPos, 2);
    Serial.println(" mm");
    return;
  }

  linPID.Compute();
  int pwm = constrain((int)abs(linOutput), LIN_PWM_MIN, LIN_PWM_MAX);
  if (linOutput > 0) linExtend(pwm);
  else               linRetract(pwm);

  Serial.print("L: ");
  Serial.print(linPos, 2);
  Serial.print("mm  Err:");
  Serial.print(err, 2);
  Serial.print("  PWM:");
  Serial.println(pwm);
}

// ── Rotary PID loop ───────────────────────────────────────────
void runRotary() {
  if (!rotMoving) return;

  rotPos = encRot.read() / ENC_COUNTS_PER_DEG;
  double err = rotTarget - rotPos;

  if (abs(err) <= ROT_DEADBAND) {
    stopRotary();
    rotMoving = false;
    Serial.print(">> ROT REACHED: ");
    Serial.print(rotPos, 2);
    Serial.println(" deg");
    return;
  }

  rotPID.Compute();
  int pwm = constrain((int)abs(rotOutput), ROT_PWM_MIN, ROT_PWM_MAX);
  if (rotOutput > 0) rotForward(pwm);
  else               rotBackward(pwm);

  Serial.print("R: ");
  Serial.print(rotPos, 2);
  Serial.print("deg  Err:");
  Serial.print(err, 2);
  Serial.print("  PWM:");
  Serial.println(pwm);
}

// ── Stepper loop ──────────────────────────────────────────────
void runStepper() {
  if (!stepMoving) return;

  stepper.run();

  if (stepper.distanceToGo() == 0) {
    stepMoving = false;
    float arrivedDeg = stepper.currentPosition() / STEPS_PER_DEG;
    Serial.print(">> STEP REACHED: ");
    Serial.print(arrivedDeg, 2);
    Serial.println(" deg");
  }
}

// ── Serial parser ─────────────────────────────────────────────
void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.equalsIgnoreCase("H")) {
    linTarget = 0; linMoving = true;
    rotTarget = 0; rotMoving = true;
    stepper.moveTo(0); stepMoving = true;
    Serial.println(">> Homing all axes...");
  }
  else if (cmd.startsWith("L:") || cmd.startsWith("l:")) {
    float mm = constrain(cmd.substring(2).toFloat(), 0, MAX_EXTENSION_MM);
    linTarget = mm; linMoving = true;
    Serial.print(">> Linear -> "); Serial.print(mm, 1); Serial.println(" mm");
  }
  else if (cmd.startsWith("R:") || cmd.startsWith("r:")) {
    float deg = constrain(cmd.substring(2).toFloat(), -MAX_ROTATION_DEG, MAX_ROTATION_DEG);
    rotTarget = deg; rotMoving = true;
    Serial.print(">> Rotary -> "); Serial.print(deg, 1); Serial.println(" deg");
  }
  else if (cmd.startsWith("S:") || cmd.startsWith("s:")) {
    float deg = constrain(cmd.substring(2).toFloat(), -MAX_STEPPER_DEG, MAX_STEPPER_DEG);
    long targetSteps = (long)(deg * STEPS_PER_DEG);
    stepper.moveTo(targetSteps); stepMoving = true;
    Serial.print(">> Stepper -> "); Serial.print(deg, 1); Serial.println(" deg");
  }
}

// ── Motor helpers ─────────────────────────────────────────────
void linExtend(int pwm)   { digitalWrite(MOT_A1,HIGH); digitalWrite(MOT_A2,LOW);  analogWrite(ENA, pwm); }
void linRetract(int pwm)  { digitalWrite(MOT_A1,LOW);  digitalWrite(MOT_A2,HIGH); analogWrite(ENA, pwm); }
void stopLinear()         { digitalWrite(MOT_A1,LOW);  digitalWrite(MOT_A2,LOW);  analogWrite(ENA, 0);   }

void rotForward(int pwm)  { digitalWrite(MOT_B1,HIGH); digitalWrite(MOT_B2,LOW);  analogWrite(ENB, pwm); }
void rotBackward(int pwm) { digitalWrite(MOT_B1,LOW);  digitalWrite(MOT_B2,HIGH); analogWrite(ENB, pwm); }
void stopRotary()         { digitalWrite(MOT_B1,LOW);  digitalWrite(MOT_B2,LOW);  analogWrite(ENB, 0);   }
