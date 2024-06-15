#include <Servo.h>

#define WAIST_PIN 11
#define SHOULDER_PIN 9
#define ELBOW_PIN 10
#define WRIST_PIN 6
#define WRIST_TWIST_PIN 3
#define TOOL_PIN 5

#define WAIST_HOME 161
#define SHOULDER_HOME 147
#define ELBOW_HOME 120
#define WRIST_HOME 0
#define WRIST_TWIST_HOME 0
#define TOOL_TIP_HOME 35

Servo waist;
Servo shoulder;
Servo elbow;
Servo wrist;
Servo wrist_twist;
Servo tool_tip;

char received_chars[32];  // Buffer for the received serial data
int char_pos = 0;  // Position in the buffer

void setup() {
  waist.attach(WAIST_PIN);
  shoulder.attach(SHOULDER_PIN);
  elbow.attach(ELBOW_PIN);
  wrist.attach(WRIST_PIN);
  wrist_twist.attach(WRIST_TWIST_PIN);
  tool_tip.attach(TOOL_PIN);

  waist.write(WAIST_HOME);
  shoulder.write(SHOULDER_HOME);
  elbow.write(ELBOW_HOME);
  wrist.write(WRIST_HOME);
  wrist_twist.write(WRIST_TWIST_HOME);
  tool_tip.write(TOOL_TIP_HOME);

  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  Serial.print(waist.read());
  Serial.print(" ");
  Serial.print(shoulder.read());
  Serial.print(" ");
  Serial.print(elbow.read());
  Serial.print(" ");
  Serial.print(wrist.read());
  Serial.print(" ");
  Serial.println(tool_tip.read());
  delay(1000);

}
