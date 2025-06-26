#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

int angles[5] = {90, 90, 90, 90, 90};
String inputString = "";
boolean stringComplete = false;

void setup() {
  Serial.begin(9600);
  servo1.attach(3);  // Ganti pin sesuai kebutuhan
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(9);
  servo5.attach(10);
  inputString.reserve(30);
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
    parseBinary(inputString);
    servo1.write(angles[0]);
    servo2.write(angles[1]);
    servo3.write(angles[2]);
    servo4.write(angles[3]);
    servo5.write(angles[4]);
    inputString = "";
    stringComplete = false;
  }
}

void parseBinary(String data) {
  // Format: <b1,b2,b3,b4,b5>
  int startIdx = data.indexOf('<');
  int endIdx = data.indexOf('>');
  if (startIdx != -1 && endIdx != -1 && endIdx > startIdx) {
    String values = data.substring(startIdx + 1, endIdx);
    int lastIdx = 0;
    for (int i = 0; i < 5; i++) {
      int commaIdx = values.indexOf(',', lastIdx);
      String valStr;
      if (i < 4) {
        valStr = values.substring(lastIdx, commaIdx);
        lastIdx = commaIdx + 1;
      } else {
        valStr = values.substring(lastIdx);
      }
      int binVal = valStr.toInt();
      angles[i] = (binVal == 1) ? 180 : 0;
    }
  }
}
