#include <Arduino.h>
#include <math.h>

/* ===================== CONFIG ===================== */
const int voltagePin      = A0;
const int current30APin   = A1;
const int current20APins[] = {A2, A3, A4, A5, A6, A7};
const int numCurrent20A    = sizeof(current20APins) / sizeof(current20APins[0]);

// IO
const int systemEnablePin = 43;  // LOW = เปิดระบบ, HIGH = ปิดระบบ
const int relayRed    = 42;      // Active-LOW
const int relayYellow = 41;      // Active-LOW
const int relayGreen  = 40;      // Active-LOW
const int onoffPin    = 52;      // สวิตช์ NO -> กด = LOW
const int emergencyPin= 53;      // สวิตช์ NO -> กด = LOW

// ADC / Voltage divider
const float VREF_mV = 5000.0;  // AVcc ~5V
const float ADC_MAX = 1023.0;
const float R1 = 30000.0;      // divider: Vin -- R1 --+-- R2 -- GND, A0 at node(+)
const float R2 =  7500.0;      // 30k : 7.5k => scale ~5x

// ACS712 sensitivity (mV/A)
const float sensitivity30A = 66.0;    // ACS712-30A
// per-channel sensitivity (ตั้งให้ตรงรุ่นแต่ละตัว)
float sens20A[6] = {100, 100, 100, 100, 100, 100};  // ACS712-20A = 100 mV/A
// ถ้า A7 เป็นรุ่น 5A ให้แก้ sens20A[5] = 185;

// ทิศทางกระแส (+1 / -1) ต่อช่อง
int8_t sign30A = +1;
// ถ้าช่องไหนกลับขั้ว ให้สลับเป็น -1 ได้ทันที (เริ่มตั้ง A7 = -1 ตามอาการเดิม)
int8_t sign20A[6] = {+1, +1, +1, +1, +1, +1};

// Deadband (ตัด noise ใกล้ 0 A)
const float deadband30A = 0.05f;    // A
float dead20A[6] = {0.08, 0.08, 0.08, 0.08, 0.08, 0.10}; // A7 เคร่งกว่าเล็กน้อย

// Manual trims in Amps (ค่าที่จะ "ลบออก" จากที่คำนวณได้) — ตั้ง A7 ชดเชยไว้ให้
float trimAmp30A = 0.0f;
float trimAmp20A[6] = {0, 0, 0, 0, 0, 2.00f}; // A7 = index 5 (ปรับเลขนี้ได้หน้างาน)

// Sampling / Print rate
const int CURRENT_SAMPLES = 32;         // เร็วขึ้น, พอสำหรับลด noise
const unsigned long PRINT_MS = 200;     // แสดงผลทุก 0.2 วินาที

/* ===================== STATE ===================== */
String systemState = "IDLE";
bool emergencyLatched = false;
String lastCommandFromController = "IDLE";

uint16_t offsetCount30A = 512;      // จะคาลิเบรตตอนบูต
uint16_t offsetCount20A[6] = {512,512,512,512,512,512};

unsigned long lastPrint = 0;

/* ===================== PROTOTYPES ===================== */
void updatePilotLamp(const String &state);
float readVoltage_V(int pin);
uint16_t averageADC(int pin, int samples);
float countsToCurrentA(int avgCount, int offsetCount, float mVperA);
float applyDeadband(float val, float band);
void calibrateOffsets();

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);

  pinMode(relayRed, OUTPUT);
  pinMode(relayYellow, OUTPUT);
  pinMode(relayGreen, OUTPUT);

  pinMode(emergencyPin, INPUT_PULLUP);
  pinMode(onoffPin, INPUT_PULLUP);

  pinMode(systemEnablePin, OUTPUT);
  digitalWrite(systemEnablePin, HIGH);   // เริ่มต้นปิดระบบ
  digitalWrite(relayRed, HIGH);          // Active-LOW -> HIGH = ดับ
  digitalWrite(relayYellow, HIGH);
  digitalWrite(relayGreen, HIGH);

  analogReference(DEFAULT);              // ใช้ AVcc เป็น ref
  updatePilotLamp(systemState);          // แสดงไฟตาม state เริ่มต้น
  delay(100);                            // รอไฟเลี้ยง/ADC นิ่ง

  Serial.println("System Ready: Voltage + Current Monitor + Emergency");
  Serial.println("Calibrating zero-current offsets... (NO LOAD please)");
  calibrateOffsets();

  Serial.print("Offset30A(counts): "); Serial.println(offsetCount30A);
  for (int i=0;i<numCurrent20A;i++){
    Serial.print("Offset20A["); Serial.print(i); Serial.print("](counts): ");
    Serial.println(offsetCount20A[i]);
  }
  Serial.println("Calibration done.\n");
}

/* ===================== LOOP ===================== */
void loop() {
  // -------- Hotkeys over USB Serial --------
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'z' || c == 'Z') {
      Serial.println(">>> Re-calibrating offsets (NO LOAD)...");
      calibrateOffsets();
      Serial.print("Offset30A="); Serial.println(offsetCount30A);
      for (int i=0;i<numCurrent20A;i++){
        Serial.print("Offset20A["); Serial.print(i); Serial.print("]=");
        Serial.println(offsetCount20A[i]);
      }
    }
    if (c == 'd' || c == 'D') {
      Serial.println(">>> RAW ADC (counts):");
      Serial.print("A1="); Serial.println(averageADC(current30APin, CURRENT_SAMPLES));
      for (int i=0;i<numCurrent20A;i++){
        Serial.print("A"); Serial.print(i+2); Serial.print('=');
        Serial.println(averageADC(current20APins[i], CURRENT_SAMPLES));
      }
      Serial.println("-----------------------------");
    }
  }

  // -------- Power ON/OFF logic --------
  bool isSystemOn = (digitalRead(onoffPin) == LOW);  // NO: กด = LOW
  if (!isSystemOn) {
    if (systemState != "OFF") {
      systemState = "OFF";
      updatePilotLamp(systemState);
      Serial.println(">>> System is OFF (via onoff switch)");
      Serial2.println("OFF");
      digitalWrite(systemEnablePin, HIGH);  // ปิดระบบ
    }
    delay(100);
    return;
  } else if (systemState == "OFF") {
    systemState = "IDLE";
    updatePilotLamp(systemState);
    Serial.println(">>> System turned ON");
    Serial2.println("IDLE");
    digitalWrite(systemEnablePin, LOW);     // เปิดระบบ
  }

  // -------- Emergency logic --------
  bool emergencyPressed = (digitalRead(emergencyPin) == LOW);
  if (emergencyPressed && !emergencyLatched) {
    emergencyLatched = true;
    systemState = "EMERGENCY";
    updatePilotLamp(systemState);
    Serial.println(">>> EMERGENCY triggered by switch");
    Serial2.println("EMERGENCY");
  }

  if (!emergencyPressed && emergencyLatched && lastCommandFromController == "IDLE") {
    emergencyLatched = false;
    systemState = "IDLE";
    updatePilotLamp(systemState);
    Serial.println(">>> Emergency cleared by switch (motor idle)");
  }

  // -------- Receive controller state over Serial2 --------
  while (Serial2.available()) {
    String input = Serial2.readStringUntil('\n');
    input.trim();

    if (input == "EMERGENCY") {
      emergencyLatched = true;
      systemState = "EMERGENCY";
      updatePilotLamp(systemState);
      Serial.println(">>> EMERGENCY triggered by Serial2");
    }

    if (input == "RUN" || input == "IDLE") {
      lastCommandFromController = input;
      if (!emergencyLatched) {
        systemState = input;
        updatePilotLamp(systemState);
        Serial.print(">>> State set by Serial2: ");
        Serial.println(systemState);
      }
    }
  }

  // -------- Non-blocking print every PRINT_MS --------
  unsigned long now = millis();
  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;

    float voltage = readVoltage_V(voltagePin);

    // 30A channel
    uint16_t avg30 = averageADC(current30APin, CURRENT_SAMPLES);
    float i30 = sign30A * countsToCurrentA(avg30, offsetCount30A, sensitivity30A);
    i30 -= trimAmp30A;                         // ★ ชดเชยเป็นแอมป์
    i30 = applyDeadband(i30, deadband30A);

    Serial.print("Voltage: "); Serial.print(voltage, 2); Serial.println(" V");
    Serial.print("Current 30A (A1): "); Serial.print(i30, 2); Serial.println(" A");

    // Auto-trim offset เมื่อเข้าใกล้ 0A (ลด drift หลังสลับ power)
    if (fabs(i30) < 0.15f) {
      offsetCount30A = (uint16_t)((offsetCount30A * 99u + avg30) / 100u);
    }

    // 20A channels
    for (int i = 0; i < numCurrent20A; i++) {
      uint16_t avg20 = averageADC(current20APins[i], CURRENT_SAMPLES);
      float i20 = sign20A[i] * countsToCurrentA(avg20, offsetCount20A[i], sens20A[i]);
      i20 -= trimAmp20A[i];                   // ★ ชดเชยเป็นแอมป์ต่อช่อง (A7 ใช้ index 5)
      i20 = applyDeadband(i20, dead20A[i]);

      Serial.print("Current 20A (A"); Serial.print(i + 2); Serial.print("): ");
      Serial.print(i20, 2); Serial.println(" A");

      if (fabs(i20) < 0.10f) {
        offsetCount20A[i] = (uint16_t)((offsetCount20A[i] * 99u + avg20) / 100u);
      }
    }
    Serial.println("-----------------------------");
  }
}

/* ===================== HELPERS ===================== */
void updatePilotLamp(const String &state) {
  // ดับทั้งหมดก่อน (Active-LOW)
  digitalWrite(relayRed,    HIGH);
  digitalWrite(relayYellow, HIGH);
  digitalWrite(relayGreen,  HIGH);

  if (state == "EMERGENCY") {
    digitalWrite(relayRed, LOW);
  } else if (state == "RUN") {
    digitalWrite(relayYellow, LOW);
  } else if (state == "IDLE") {
    digitalWrite(relayGreen, LOW);
  } else if (state == "OFF") {
    // ดับทั้งหมด
  }
}

uint16_t averageADC(int pin, int samples) {
  analogRead(pin); // dummy read กันมัลติเพล็กซ์ยังไม่นิ่ง
  uint32_t sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  return (uint16_t)(sum / (uint32_t)samples);
}

float countsToCurrentA(int avgCount, int offsetCount, float mVperA) {
  float mV_per_count = VREF_mV / ADC_MAX;           // ~4.887 mV/count
  float delta_mV = (avgCount - offsetCount) * mV_per_count;
  return delta_mV / mVperA;                         // A
}

float readVoltage_V(int pin) {
  int raw = analogRead(pin);
  float vout = (raw / ADC_MAX) * (VREF_mV / 1000.0f); // V
  float vin  = vout / (R2 / (R1 + R2));               // แปลงกลับตาม divider
  return vin;
}

float applyDeadband(float val, float band) {
  if (fabs(val) < band) return 0.0f;
  return val;
}

void calibrateOffsets() {
  // อ่านค่าเฉลี่ยหลาย ๆ ครั้งเพื่อได้ศูนย์ที่นิ่ง (ควร NO LOAD)
  const int repeats = 25; // 25 * 32 = 800 reads/ช่อง

  // 30A
  uint32_t sum30 = 0;
  for (int r=0; r<repeats; r++) sum30 += averageADC(current30APin, CURRENT_SAMPLES);
  offsetCount30A = (uint16_t)(sum30 / repeats);

  // 20A x6
  for (int i=0;i<numCurrent20A;i++){
    uint32_t sum = 0;
    for (int r=0; r<repeats; r++) sum += averageADC(current20APins[i], CURRENT_SAMPLES);
    offsetCount20A[i] = (uint16_t)(sum / repeats);
  }
}
