/*
 * Arduino Program - LEFT HAND (5 Finger Control)
 * Port: COM3 (sesuaikan dengan koneksi Arduino kiri Anda)
 * 
 * Menerima data format: L,<Thumb>,<Index>,<Middle>,<Ring>,<Pinky>\n
 * Contoh: L,90,45,120,60,30\n
 * 
 * Pin Setup:
 * - Pin 3: Servo Thumb (Jempol)
 * - Pin 5: Servo Index (Telunjuk)
 * - Pin 6: Servo Middle (Tengah)
 * - Pin 9: Servo Ring (Manis)
 * - Pin 10: Servo Pinky (Kelingking)
 */

#include <Servo.h>

#define BAUDRATE 115200

// Pin untuk 5 servo (Thumb, Index, Middle, Ring, Pinky)
const uint8_t SERVO_PINS[5] = {3, 5, 6, 9, 10};

// Konfigurasi servo
const uint8_t MIN_LIMIT[5] = {0, 0, 0, 0, 0};
const uint8_t MAX_LIMIT[5] = {180, 180, 180, 180, 180};
const bool INVERT[5] = {false, false, false, false, false};  // Set true jika ingin membalik arah servo tertentu
const uint8_t SMOOTH_STEP[5] = {3, 3, 3, 3, 3};              // Smoothing step per servo

Servo servos[5];
String rxLine;
int currentDeg[5] = {90, 90, 90, 90, 90};  // Posisi awal servo (tengah)
int targetDeg[5] = {90, 90, 90, 90, 90};

void setup() {
  Serial.begin(BAUDRATE);
  while (!Serial) { ; }
  
  // Attach semua servo dan set posisi awal
  for (int i = 0; i < 5; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(currentDeg[i]);
  }
  
  Serial.println(F("READY - LEFT HAND 5 Fingers"));
  Serial.println(F("Pins: T=3, I=5, M=6, R=9, P=10"));
}

void loop() {
  // Baca data serial
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { 
      handleLine(rxLine); 
      rxLine = ""; 
    }
    else { 
      rxLine += c; 
      if (rxLine.length() > 64) rxLine = "";  // Prevent buffer overflow
    }
  }

  // Smooth servo movement untuk semua servo
  for (int i = 0; i < 5; i++) {
    int step = SMOOTH_STEP[i];
    if (step > 0) {
      if (currentDeg[i] < targetDeg[i]) {
        currentDeg[i] = min(targetDeg[i], currentDeg[i] + step);
      }
      else if (currentDeg[i] > targetDeg[i]) {
        currentDeg[i] = max(targetDeg[i], currentDeg[i] - step);
      }
      servos[i].write(currentDeg[i]);
    }
    else {
      // No smoothing
      servos[i].write(targetDeg[i]);
      currentDeg[i] = targetDeg[i];
    }
  }
  
  delay(5);
}

void handleLine(const String& line) {
  // Expected format: L,<Thumb>,<Index>,<Middle>,<Ring>,<Pinky>
  // Example: L,90,45,120,60,30
  
  if (!line.startsWith("L,")) return;
  
  int vals[5] = {0, 0, 0, 0, 0};
  int idx = 0;
  int start = 2;
  
  // Parse 5 nilai dari string
  while (idx < 5) {
    int comma = line.indexOf(',', start);
    String tok = (comma == -1) ? line.substring(start) : line.substring(start, comma);
    tok.trim();
    
    if (tok.length() == 0) return;
    
    vals[idx] = tok.toInt();
    idx++;
    
    if (comma == -1) break;
    start = comma + 1;
  }
  
  if (idx != 5) return;  // Harus dapat 5 nilai
  
  // Apply ke semua servo
  for (int i = 0; i < 5; i++) {
    int v = vals[i];
    
    // Apply constraints
    v = constrain(v, 0, 180);
    
    // Invert if needed
    if (INVERT[i]) {
      v = 180 - v;
    }
    
    // Apply min/max limits
    v = constrain(v, MIN_LIMIT[i], MAX_LIMIT[i]);
    
    targetDeg[i] = v;
  }
  
  // Debug output (uncomment jika ingin melihat nilai yang diterima)
  // Serial.print(F("LEFT: "));
  // for (int i = 0; i < 5; i++) {
  //   Serial.print(targetDeg[i]);
  //   Serial.print(F(" "));
  // }
  // Serial.println();
}
