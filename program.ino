// =============================
// ROBOT DIFFERENTIAL DRIVE - DUAL STRATEGY
// Mode A: Gerak Kurva (langsung ke target)
// Mode B: Rotasi Dulu, baru Maju Lurus
// Pilih mode via Keypad: tekan '1' atau '2'
// =============================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <ESP32Servo.h>
#include <math.h>

// =============================
// LCD & KEYPAD
// =============================
LiquidCrystal_I2C lcd(0x27, 16, 2);
const byte JUMLAH_BARIS = 4;
const byte JUMLAH_KOLOM = 3;
char tombol[JUMLAH_BARIS][JUMLAH_KOLOM] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte pinBaris[JUMLAH_BARIS] = {8, 18, 17, 16};
byte pinKolom[JUMLAH_KOLOM] = {15, 7, 6};
Keypad keypad = Keypad(makeKeymap(tombol), pinBaris, pinKolom, JUMLAH_BARIS, JUMLAH_KOLOM);

// =============================
// MOTOR & ENCODER PINS
// =============================
#define KIRI_MAJU   10
#define KIRI_MUNDUR 11
#define KIRI_PWM     9
#define KANAN_MAJU  13
#define KANAN_MUNDUR 14
#define KANAN_PWM   12
#define ENCODER_KIRI 35
#define ENCODER_KANAN 36

// =============================
// SERVO & LIFT
// =============================
#define SERVO_JEPIT 4
#define LIFT_NAIK   38
#define LIFT_TURUN  39
#define LIFT_PWM    37
Servo servoJepit;

// =============================
// PARAMETER FISIK ROBOT
// =============================
const float DIAMETER_RODA = 0.065;    // meter
const float WHEEL_RADIUS = DIAMETER_RODA / 2.0;
const float JARAK_RODA = 0.142;       // meter (wheelbase L)
const int ENCODER_PER_MOTOR_REV = 25;         
const float TICKS_PER_WHEEL_REV = ENCODER_PER_MOTOR_REV;
const float KELILING_RODA = 3.14159265358979323846 * DIAMETER_RODA;
const float METER_PER_TICK = KELILING_RODA / TICKS_PER_WHEEL_REV;

// =============================
// CONTROL GAINS & LIMITS
// =============================
const float Kp_linear = 3.5;    
const float Kp_angular = 3.9;   
const float Kp_wheel = 1.6;     

const float V_MAX = 0.25;       // m/s
const float OMEGA_MAX = 3.5;    // rad/s
const int PWM_MAX = 255;

// =============================
// STATE VARIABLES
// =============================
volatile long hitungKiri = 0, hitungKanan = 0;
float posisiX = 0.0, posisiY = 0.0, arahRobot = 0.0;
long lastHitungKiri = 0, lastHitungKanan = 0;
unsigned long lastLoopMillis = 0;
unsigned long tPrev_runPF = 0;

float ffGainL = 0.88; // coba awal 1.12 – 1.25
float ffGainR = 1.00;


// =============================
// MODE SELECTION
// =============================
enum ControlMode {
  MODE_KURVA = 1,        // Gerak langsung (kurva)
  MODE_ROTATE_MOVE = 2   // Rotasi dulu, baru maju
};

ControlMode selectedMode = MODE_KURVA;

// =============================
// STATE MACHINE UNTUK MODE 2
// =============================
enum MovePhase {
  PHASE_ROTATE,  
  PHASE_MOVE,    
  PHASE_DONE     
};

MovePhase currentPhase = PHASE_ROTATE;

// =============================
// INTERRUPTS
// =============================
void IRAM_ATTR bacaEncoderKiri()  { hitungKiri++; }
void IRAM_ATTR bacaEncoderKanan() { hitungKanan++; }

// =============================
// MOTOR CONTROL
// =============================
void setMotorPinsInit() {
  pinMode(KIRI_MAJU, OUTPUT);
  pinMode(KIRI_MUNDUR, OUTPUT);
  pinMode(KIRI_PWM, OUTPUT);
  pinMode(KANAN_MAJU, OUTPUT);
  pinMode(KANAN_MUNDUR, OUTPUT);
  pinMode(KANAN_PWM, OUTPUT);
}

void driveWheelLeft(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm > 0) {
    digitalWrite(KIRI_MAJU, HIGH);
    digitalWrite(KIRI_MUNDUR, LOW);
    analogWrite(KIRI_PWM, pwm);
  } else if (pwm < 0) {
    digitalWrite(KIRI_MAJU, LOW);
    digitalWrite(KIRI_MUNDUR, HIGH);
    analogWrite(KIRI_PWM, -pwm);
  } else {
    digitalWrite(KIRI_MAJU, LOW);
    digitalWrite(KIRI_MUNDUR, LOW);
    analogWrite(KIRI_PWM, 0);
  }
}

void driveWheelRight(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm > 0) {
    digitalWrite(KANAN_MAJU, HIGH);
    digitalWrite(KANAN_MUNDUR, LOW);
    analogWrite(KANAN_PWM, pwm);
  } else if (pwm < 0) {
    digitalWrite(KANAN_MAJU, LOW);
    digitalWrite(KANAN_MUNDUR, HIGH);
    analogWrite(KANAN_PWM, -pwm);
  } else {
    digitalWrite(KANAN_MAJU, LOW);
    digitalWrite(KANAN_MUNDUR, LOW);
    analogWrite(KANAN_PWM, 0);
  }
}

void berhenti() {
  driveWheelLeft(0);
  driveWheelRight(0);
}

// =============================
// CONVERSION FUNCTIONS
// =============================
float ticksPerSec_to_mps(float ticks_per_sec) {
  return ticks_per_sec * METER_PER_TICK;
}

float mps_to_ticksPerSec(float v_mps) {
  return v_mps / METER_PER_TICK;
}

int wheelControllerP(float targetTicksPerSec, float measuredTicksPerSec) {
  float error = targetTicksPerSec - measuredTicksPerSec;
  float pwmDelta = Kp_wheel * error;
  return (int)round(pwmDelta);
}

// =============================
// INPUT FUNCTIONS
// =============================
float inputKoordinat(const char* label) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Masukkan ");
  lcd.print(label);
  lcd.setCursor(0,1);
  String input = "";
  char t;
  while (true) {
    t = keypad.getKey();
    if (t) {
      if (t >= '0' && t <= '9') {
        input += t;
        lcd.print(t);
      } else if (t == '*') {
        input = "";
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Masukkan ");
        lcd.print(label);
        lcd.setCursor(0,1);
      } else if (t == '#') {
        break;
      }
    }
    delay(10);
  }
  return input.toFloat() / 100.0; // cm -> meter
}

ControlMode pilihMode() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Pilih Mode:");
  lcd.setCursor(0,1);
  lcd.print("1:Kurva 2:Rotasi");
  
  char t;
  while (true) {
    t = keypad.getKey();
    if (t == '1') {
      lcd.clear();
      lcd.print("Mode: KURVA");
      delay(1000);
      return MODE_KURVA;
    } else if (t == '2') {
      lcd.clear();
      lcd.print("Mode: ROTASI+MAJU");
      delay(1000);
      return MODE_ROTATE_MOVE;
    }
    delay(10);
  }
}

// =============================
// GRIPPER & LIFT
// =============================
void jepitBarang() { 
  servoJepit.attach(SERVO_JEPIT); 
  servoJepit.write(15); 
  delay(700); 
}

void lepasBarang() { 
  servoJepit.attach(SERVO_JEPIT); 
  servoJepit.write(120); 
  delay(700); 
}

void jepitBarangAWAL() { 
  servoJepit.attach(SERVO_JEPIT); 
  servoJepit.write(15); 
  delay(500); 
}

void lepasBarangAWAL() { 
  servoJepit.attach(SERVO_JEPIT); 
  servoJepit.write(120); 
  delay(500); 
}

void liftNaik() {
  digitalWrite(LIFT_NAIK, LOW);
  digitalWrite(LIFT_TURUN, HIGH);
  analogWrite(LIFT_PWM, 200);
  delay(1200);
  analogWrite(LIFT_PWM, 0);
}

void liftTurun() {
  digitalWrite(LIFT_NAIK, HIGH);
  digitalWrite(LIFT_TURUN, LOW);
  analogWrite(LIFT_PWM, 200);
  delay(1400);
  analogWrite(LIFT_PWM, 0);
}

// =============================
// RESET ODOMETRY
// =============================
void resetOdometry() {
  noInterrupts();
  hitungKiri  = 0;
  hitungKanan = 0;
  lastHitungKiri  = 0;
  lastHitungKanan = 0;
  interrupts();

  posisiX = 0.0;
  posisiY = 0.0;
  arahRobot = 0.0;

  lastLoopMillis = millis();
  tPrev_runPF = 0;
}

void resetStateMachine() {
  currentPhase = PHASE_ROTATE;
  tPrev_runPF = 0;
}

// =============================
// MODE 1: GERAK KURVA (LANGSUNG)
// =============================
bool runPathFollowing_Kurva(float targetX, float targetY) {
  const float DIST_TOL = 0.06;   
  const float ANGLE_TOL = 0.45;  

  unsigned long tNow = millis();
  if (tPrev_runPF == 0) tPrev_runPF = tNow;
  float dt = (tNow - tPrev_runPF) / 1000.0;
  if (dt <= 0.0) return false;
  tPrev_runPF = tNow;

  // BACA ENCODER
  noInterrupts();
  long curKL = hitungKiri;
  long curKR = hitungKanan;
  interrupts();

  long dTicksL = curKL - lastHitungKiri;
  long dTicksR = curKR - lastHitungKanan;
  lastHitungKiri = curKL;
  lastHitungKanan = curKR;

  float measuredTicksPerSecL = dTicksL / dt;
  float measuredTicksPerSecR = dTicksR / dt;

  float vL_meas = ticksPerSec_to_mps(measuredTicksPerSecL);
  float vR_meas = ticksPerSec_to_mps(measuredTicksPerSecR);

  // FORWARD KINEMATICS
  float v_robot = (vR_meas + vL_meas) / 2.0;
  float omega_robot = (vR_meas - vL_meas) / JARAK_RODA;

  // UPDATE ODOMETRY
  posisiX += v_robot * cos(arahRobot) * dt;
  posisiY += v_robot * sin(arahRobot) * dt;
  arahRobot += omega_robot * dt;
  while (arahRobot > M_PI) arahRobot -= 2.0*M_PI;
  while (arahRobot < -M_PI) arahRobot += 2.0*M_PI;

  // ERROR TO TARGET
  float dx = targetX - posisiX;
  float dy = targetY - posisiY;
  float distErr = sqrt(dx*dx + dy*dy);
  float angleToTarget = atan2(dy, dx);
  float angleErr = angleToTarget - arahRobot;
  while (angleErr > M_PI) angleErr -= 2.0*M_PI;
  while (angleErr < -M_PI) angleErr += 2.0*M_PI;

  // CEK SAMPAI
  if (distErr <= DIST_TOL) {
    berhenti();
    return true;
  }

  // KONTROL P
  float v_target = Kp_linear * distErr;
  if (v_target > V_MAX) v_target = V_MAX;

  float omega_target = Kp_angular * angleErr;
  if (omega_target > OMEGA_MAX) omega_target = OMEGA_MAX;
  if (omega_target < -OMEGA_MAX) omega_target = -OMEGA_MAX;

  // Jika error sudut besar, stop linear
  if (fabs(angleErr) > 0  .2) v_target = 0.0;

  // INVERSE KINEMATICS
  float vR_target = v_target + (omega_target * JARAK_RODA / 2.0);
  float vL_target = v_target - (omega_target * JARAK_RODA / 2.0);

  float ticksPerSec_R_target = mps_to_ticksPerSec(vR_target);
  float ticksPerSec_L_target = mps_to_ticksPerSec(vL_target);

  int pwmDeltaR = wheelControllerP(ticksPerSec_R_target, measuredTicksPerSecR);
  int pwmDeltaL = wheelControllerP(ticksPerSec_L_target, measuredTicksPerSecL);

  int pwmBaseR = (int)((fabs(vR_target)/V_MAX) * (PWM_MAX ));
  int pwmBaseL = (int)((fabs(vL_target)/V_MAX) * (PWM_MAX ));

  int pwmR = pwmBaseR + pwmDeltaR;
  int pwmL = pwmBaseL + pwmDeltaL;

  if (vR_target < 0) pwmR = -pwmR;
  if (vL_target < 0) pwmL = -pwmL;

  // APPLY MOTOR COMPENSATION
  pwmL = pwmL * ffGainL;
  pwmR = pwmR * ffGainR;

  // BATASI
  pwmR = constrain(pwmR, -255, 255);
  pwmL = constrain(pwmL, -255, 255);

  driveWheelLeft(pwmL);
  driveWheelRight(pwmR);


  return false;
}

// =============================
// MODE 2: ROTASI DULU, BARU MAJU
// =============================
bool runPathFollowing_RotateMove(float targetX, float targetY) {
  const float DIST_TOL = 0.08;         
  const float ANGLE_TOL = 0.025;        // toleransi rotasi (8.6 deg)
  const float ANGLE_MOVE_TOL = 0.45;   // toleransi saat gerak

  unsigned long tNow = millis();
  if (tPrev_runPF == 0) tPrev_runPF = tNow;
  float dt = (tNow - tPrev_runPF) / 1000.0;
  if (dt <= 0.0) return false;
  tPrev_runPF = tNow;

  // BACA ENCODER
  noInterrupts();
  long curKL = hitungKiri;
  long curKR = hitungKanan;
  interrupts();

  long dTicksL = curKL - lastHitungKiri;
  long dTicksR = curKR - lastHitungKanan;
  lastHitungKiri = curKL;
  lastHitungKanan = curKR;

  float measuredTicksPerSecL = dTicksL / dt;
  float measuredTicksPerSecR = dTicksR / dt;

  float vL_meas = ticksPerSec_to_mps(measuredTicksPerSecL);
  float vR_meas = ticksPerSec_to_mps(measuredTicksPerSecR);

  // FORWARD KINEMATICS
  float v_robot = (vR_meas + vL_meas) / 2.0;
  float omega_robot = (vR_meas - vL_meas) / JARAK_RODA ;

  // UPDATE ODOMETRY
  posisiX += v_robot * cos(arahRobot) * dt;
  posisiY += v_robot * sin(arahRobot) * dt;
  arahRobot += omega_robot * dt;
  while (arahRobot > M_PI) arahRobot -= 2.0*M_PI;
  while (arahRobot < -M_PI) arahRobot += 2.0*M_PI;

  // HITUNG ERROR
  float dx = targetX - posisiX;
  float dy = targetY - posisiY;
  float distErr = sqrt(dx*dx + dy*dy);
  float angleToTarget = atan2(dy, dx);
  float angleErr = angleToTarget - arahRobot;
  while (angleErr > M_PI) angleErr -= 2.0*M_PI;
  while (angleErr < -M_PI) angleErr += 2.0*M_PI;

  // CEK SAMPAI
  if (distErr <= DIST_TOL) {
    berhenti();
    currentPhase = PHASE_DONE;
    return true;
  }

  // =============================
  // STATE MACHINE 2 FASE
  // =============================
  
  if (currentPhase == PHASE_ROTATE) {
    // FASE 1: ROTASI DI TEMPAT
    lcd.setCursor(0, 1);
    lcd.print("Fase: ROTASI   ");
    
    if (fabs(angleErr) <= ANGLE_TOL) {
      berhenti();
      delay(100);
      currentPhase = PHASE_MOVE;
      lcd.setCursor(0, 1);
      lcd.print("Fase: MAJU     ");
      return false;
    }
    
    // Kontrol rotasi (v=0, omega≠0)
    float omega_target = Kp_angular * angleErr;
    omega_target = constrain(omega_target, -OMEGA_MAX , OMEGA_MAX);
    
    // IK untuk rotasi di tempat
    float vR_target = (omega_target * JARAK_RODA / 2.0);
    float vL_target = -(omega_target * JARAK_RODA / 2.0);
    
    float ticksPerSec_R_target = mps_to_ticksPerSec(vR_target);
    float ticksPerSec_L_target = mps_to_ticksPerSec(vL_target);
    
    int pwmDeltaR = wheelControllerP(ticksPerSec_R_target, measuredTicksPerSecR);
    int pwmDeltaL = wheelControllerP(ticksPerSec_L_target, measuredTicksPerSecL);
    
    int pwmBaseR = (int)((fabs(omega_target)/OMEGA_MAX) * (PWM_MAX * 0.7));
    int pwmBaseL = (int)((fabs(omega_target)/OMEGA_MAX) * (PWM_MAX * 0.7));
    
    int pwmR = pwmBaseR + pwmDeltaR;
    int pwmL = pwmBaseL + pwmDeltaL;

    if (pwmR < 80){
      pwmR = 100;
    };
    if (pwmL < 80){
      pwmL = 100;
    };
    
    
    if (vR_target < 0) pwmR = -pwmR;
    if (vL_target < 0) pwmL = -pwmL;
    
    pwmR = constrain(pwmR, -255, 255);
    pwmL = constrain(pwmL, -255, 255);
    
    driveWheelLeft(pwmL);
    driveWheelRight(pwmR);
    
    return false;
    
  } else if (currentPhase == PHASE_MOVE) {
    // FASE 2: MAJU LURUS
    
    // Jika menyimpang terlalu jauh, kembali rotasi
    // if (fabs(angleErr) > ANGLE_MOVE_TOL) {
    //   berhenti();
    //   delay(100);
    //   currentPhase = PHASE_ROTATE;
    //   return false;
    // }
    
    // Kontrol linear dengan koreksi sudut minimal
    float v_target = Kp_linear * distErr;
    v_target = constrain(v_target, 0, V_MAX);
    
    float omega_target = Kp_angular * angleErr * 0.5; // gain kecil
    omega_target = constrain(omega_target, -OMEGA_MAX*1.5, OMEGA_MAX*1.5);
    
    // IK
    float vR_target = v_target + (omega_target * JARAK_RODA / 2.0);
    float vL_target = v_target - (omega_target * JARAK_RODA / 2.0);
    
    float ticksPerSec_R_target = mps_to_ticksPerSec(vR_target);
    float ticksPerSec_L_target = mps_to_ticksPerSec(vL_target);
    
    int pwmDeltaR = wheelControllerP(ticksPerSec_R_target, measuredTicksPerSecR);
    int pwmDeltaL = wheelControllerP(ticksPerSec_L_target, measuredTicksPerSecL);
    
    int pwmBaseR = (int)((fabs(vR_target)/V_MAX) * (PWM_MAX * 1.5));
    int pwmBaseL = (int)((fabs(vL_target)/V_MAX) * (PWM_MAX * 1.5));
    
    int pwmR = pwmBaseR + pwmDeltaR;
    int pwmL = pwmBaseL + pwmDeltaL;
    if (pwmR < 80){
      pwmR = 120;
    };
    if (pwmL < 80){
      pwmL = 120;
    };
    
    if (vR_target < 0) pwmR = -pwmR;
    if (vL_target < 0) pwmL = -pwmL;
    
    pwmR = constrain(pwmR, -255, 255);
    pwmL = constrain(pwmL, -255, 255);
    
    driveWheelLeft(pwmL);
    driveWheelRight(pwmR);
    
    return false;
  }
  
  return false;
}

void balikHome_Sederhana(float jarakMeter) {
  lcd.clear();
  lcd.print("Balik Home Simple");
  delay(300);

  // Step 1: Rotasi 180 derajat
  lcd.clear();
  lcd.print("Rotasi 180");
  delay(200);

  int pwmRot = 140;
  int timeRotate = 900; // tuning berdasarkan kecepatan motor

  driveWheelLeft(-pwmRot);
  driveWheelRight(pwmRot);
  delay(timeRotate);
  berhenti();
  delay(300);

  // Step 2: Maju lurus sejauh jarakMeter
  lcd.clear();
  lcd.print("Maju Pulang");
  delay(200);

  noInterrupts();
  hitungKiri = 0;
  hitungKanan = 0;
  interrupts();

  float currentDistance = 0;

  while (currentDistance < jarakMeter) {
    driveWheelLeft(140);
    driveWheelRight(120);

    noInterrupts();
    long L = hitungKiri;
    long R = hitungKanan;
    interrupts();

    currentDistance = ((L + R) / 2.0) * METER_PER_TICK;
    delay(10);
  }

  berhenti();
  // delay(200);

  // // Step 3 (opsional): kembali hadap depan
  // lcd.clear();
  // lcd.print("Arah Depan");
  // delay(200);

  // driveWheelLeft(-pwmRot);
  // driveWheelRight(pwmRot);
  // delay(timeRotate);
  berhenti();
}



// =============================
// SETUP
// =============================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 47);
  lcd.init(); 
  lcd.backlight();
  lcd.print("Robot Dual Mode");
  lcd.setCursor(0,1);
  lcd.print("FK/IK + P-Ctrl");
  delay(1000);

  setMotorPinsInit();

  pinMode(ENCODER_KIRI, INPUT_PULLUP);
  pinMode(ENCODER_KANAN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_KIRI), bacaEncoderKiri, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_KANAN), bacaEncoderKanan, RISING);

  pinMode(LIFT_NAIK, OUTPUT);
  pinMode(LIFT_TURUN, OUTPUT);
  pinMode(LIFT_PWM, OUTPUT);

  servoJepit.attach(SERVO_JEPIT);
  servoJepit.write(120);

  noInterrupts();
  hitungKiri = 0; 
  hitungKanan = 0;
  interrupts();

  posisiX = 0.0; 
  posisiY = 0.0; 
  arahRobot = 0.0;
  lastHitungKiri = 0; 
  lastHitungKanan = 0;
  lastLoopMillis = millis();

  Serial.println("Setup selesai. Pilih mode di keypad.");
}

// =============================
// LOOP UTAMA
// =============================
void loop() {
  // PILIH MODE
  selectedMode = pilihMode();

  // INPUT TARGET
  float targetX = inputKoordinat("X (cm)");
  delay(200);
  float targetY = inputKoordinat("Y (cm)");
  delay(200);

  // RESET ODOMETRY & STATE MACHINE
  lcd.clear();
  lcd.print("Reset Odometry...");
  resetOdometry();
  if (selectedMode == MODE_ROTATE_MOVE) {
    resetStateMachine();
  }
  delay(250);

  // INISIALISASI GRIPPER
  jepitBarangAWAL(); 
  lepasBarangAWAL();
  
  // TAMPILKAN MODE
  lcd.clear(); 
  if (selectedMode == MODE_KURVA) {
    lcd.print("Mode: KURVA");
  } else {
    lcd.print("Mode: ROT+MAJU");
  }
  delay(500);

  // JALANKAN SESUAI MODE
  lcd.setCursor(0,0);
  lcd.print("Menuju target...");
  
  if (selectedMode == MODE_KURVA) {
    // MODE 1: GERAK KURVA
    while (!runPathFollowing_Kurva(targetX, targetY)) {
      delay(10);
    }
  } else {
    // MODE 2: ROTASI DULU, MAJU
    while (!runPathFollowing_RotateMove(targetX, targetY)) {
      delay(10);
    }
  }

  // PICKUP SEQUENCE
  lcd.clear(); 
  lcd.print("Jepit & Angkat");
  jepitBarang(); 
  delay(250);
  liftNaik(); 
  delay(250);

  // BALIK KE HOME
  float jarakBerangkat = sqrt(targetX*targetX + targetY*targetY);

  if (selectedMode == MODE_KURVA) {
    while (!runPathFollowing_Kurva(0.0, 0.0)) delay(10);
  } else {
    balikHome_Sederhana(jarakBerangkat);
  }




  // TURUN & LEPAS
  lcd.clear(); 
  lcd.print("Turun & Lepas");
  liftTurun(); 
  delay(200);
  lepasBarang();
  
  lcd.clear(); 
  lcd.print("Selesai!");
  lcd.setCursor(0,1);
  lcd.print("Tekan 1 atau 2");
  delay(2000);
}