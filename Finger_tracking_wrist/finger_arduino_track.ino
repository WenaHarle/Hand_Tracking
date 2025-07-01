#include <Servo.h>
Servo servo1, servo2, servo3, servo4, servo5, servo6;
int angles[6] = {90, 90, 90, 90, 90, 90};
String inputString = "";
boolean stringComplete = false;

int servoOpen[6]  = {180, 180, 180, 180, 180, 180};
int servoClose[6] = {0, 0, 0, 0, 0, 0};
int currentPos[6] = {90, 90, 90, 90, 90, 90};
int servoSpeed = 4; // ms per step

unsigned long lastMoveTime[6] = {0, 0, 0, 0, 0, 0};
int moveStep[6] = {0, 0, 0, 0, 0, 0};

void setup() {
  Serial.begin(9600);
  servo1.attach(3);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(9);
  servo5.attach(10);
  servo6.attach(11);
  inputString.reserve(40);
}

void loop() {
  // Baca serial di loop utama
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
      break;
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    int commaCount = 0;
    for (unsigned int i = 0; i < inputString.length(); i++) {
      if (inputString[i] == ',') commaCount++;
    }
    if (commaCount == 5 && inputString.indexOf('<') != -1 && inputString.indexOf('>') != -1) {
      parseAngles(inputString);
      for (int i = 0; i < 6; i++) {
        moveStep[i] = (currentPos[i] < angles[i]) ? 1 : (currentPos[i] > angles[i]) ? -1 : 0;
      }
    }
    inputString = "";
    stringComplete = false;
  }

  // Gerakkan semua servo secara paralel (non-blocking)
  unsigned long now = millis();
  for (int i = 0; i < 6; i++) {
    if (moveStep[i] != 0 && now - lastMoveTime[i] >= servoSpeed) {
      currentPos[i] += moveStep[i];
      if ((moveStep[i] > 0 && currentPos[i] >= angles[i]) || (moveStep[i] < 0 && currentPos[i] <= angles[i])) {
        currentPos[i] = angles[i];
        moveStep[i] = 0;
      }
      switch (i) {
        case 0: servo1.write(currentPos[i]); break;
        case 1: servo2.write(currentPos[i]); break;
        case 2: servo3.write(currentPos[i]); break;
        case 3: servo4.write(currentPos[i]); break;
        case 4: servo5.write(currentPos[i]); break;
        case 5: servo6.write(currentPos[i]); break;
      }
      lastMoveTime[i] = now;
    }
  }
}

void parseAngles(String data) {
  int startIdx = data.indexOf('<');
  int endIdx = data.indexOf('>');
  if (startIdx != -1 && endIdx != -1 && endIdx > startIdx) {
    String values = data.substring(startIdx + 1, endIdx);
    int lastIdx = 0;
    for (int i = 0; i < 6; i++) {
      int commaIdx = values.indexOf(',', lastIdx);
      String valStr;
      if (i < 5) {
        valStr = values.substring(lastIdx, commaIdx);
        lastIdx = commaIdx + 1;
      } else {
        valStr = values.substring(lastIdx);
      }
      int val = valStr.toInt();
      // Untuk servo 1-5: 1=open, 0=close. Untuk servo 6: langsung sudut (0/180)
      if (i < 5) {
        int target = (val == 1) ? servoOpen[i] : servoClose[i];
        if (target < 0) target = 0;
        if (target > 180) target = 180;
        angles[i] = target;
      } else {
        // Servo 6 (wrist): langsung gunakan nilai dari Python (0 atau 180)
        if (val < 0) val = 0;
        if (val > 180) val = 180;
        angles[i] = val;
      }
    }
  }
}