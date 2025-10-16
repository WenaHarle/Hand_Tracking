#include <Servo.h>

#define BAUDRATE 115200

const uint8_t SERVO_PINS[5] = {3, 5, 6, 9, 10}; // Thumb, Index, Middle, Ring, Pinky
const uint8_t MIN_LIMIT[5]  = {0, 0, 0, 0, 0};
const uint8_t MAX_LIMIT[5]  = {180, 180, 180, 180, 180};
const bool    INVERT[5]     = {false, false, false, false, false};
const uint8_t SMOOTH_STEP[5]= {3, 3, 3, 3, 3};

Servo servos[5];
String rxLine;
int currentDeg[5] = {0, 0, 0, 0, 0};
int targetDeg[5]  = {0, 0, 0, 0, 0};

void setup() {
  Serial.begin(BAUDRATE);
  while (!Serial) { ; }
  for (int i=0;i<5;++i){ servos[i].attach(SERVO_PINS[i]); servos[i].write(currentDeg[i]); }
  Serial.println(F("READY 5-Servo"));
}

void loop() {
  while (Serial.available()) {
    char c=Serial.read();
    if (c=='\r') continue;
    if (c=='\n') { handleLine(rxLine); rxLine=""; }
    else { rxLine+=c; if (rxLine.length()>64) rxLine=""; }
  }

  for (int i=0;i<5;++i){
    int step=SMOOTH_STEP[i];
    if (step>0){
      if (currentDeg[i]<targetDeg[i]) currentDeg[i]=min(targetDeg[i], currentDeg[i]+step);
      else if (currentDeg[i]>targetDeg[i]) currentDeg[i]=max(targetDeg[i], currentDeg[i]-step);
      servos[i].write(currentDeg[i]);
    }else{
      servos[i].write(targetDeg[i]); currentDeg[i]=targetDeg[i];
    }
  }
  delay(5);
}

void handleLine(const String& line){
  if (!line.startsWith("F,")) return;
  int vals[5]={0,0,0,0,0}, idx=0;
  int start=2;
  while (idx<5){
    int comma=line.indexOf(',', start);
    String tok = (comma==-1)? line.substring(start) : line.substring(start, comma);
    tok.trim(); if (tok.length()==0) return;
    vals[idx]=tok.toInt(); idx++;
    if (comma==-1) break; start=comma+1;
  }
  if (idx!=5) return;

  for (int i=0;i<5;++i){
    int v=vals[i];
    v=constrain(v,0,180);
    if (INVERT[i]) v=180-v;
    v=constrain(v, MIN_LIMIT[i], MAX_LIMIT[i]);
    targetDeg[i]=v;
  }
}
