#include <Servo.h>

#define WAIST_PIN 11
#define SHOULDER_PIN 9
#define ELBOW_PIN 10
#define WRIST_PIN 6
#define TOOL_PIN 5

#define WAIST_HOME 0
#define SHOULDER_HOME 0
#define ELBOW_HOME 0
#define WRIST_HOME 0
#define TOOL_TIP_HOME 35

Servo waist;
Servo shoulder;
Servo elbow;
Servo wrist;
Servo tool_tip;

char received_chars[32];  // Buffer for the received serial data
int char_pos = 0;  // Position in the buffer

void setup() {
  waist.attach(WAIST_PIN);
  shoulder.attach(SHOULDER_PIN);
  elbow.attach(ELBOW_PIN);
  wrist.attach(WRIST_PIN);
  tool_tip.attach(TOOL_PIN);

  waist.write(WAIST_HOME);
  shoulder.write(SHOULDER_HOME);
  elbow.write(ELBOW_HOME);
  wrist.write(WRIST_HOME);
  tool_tip.write(TOOL_TIP_HOME);

  Serial.begin(115200);
}

void loop() {
  while (Serial.available()) {
    char chr = Serial.read();
    if (chr == '?') {
      send_current_positions();
    } else if (chr == ',') {
      received_chars[char_pos] = '\0';  // Null-terminate the string

      // Process the command
      process_command(received_chars);

      // Reset the buffer for the next command
      char_pos = 0;
    } else {
      received_chars[char_pos] = chr;
      char_pos++;
      if (char_pos >= sizeof(received_chars) - 1) {
        char_pos = 0;  // Reset if overflow
      }
    }
  }
}

void process_command(const char *cmd) {
  // Parse the command
  char command_type;
  int angle;

  char *token = strtok(cmd, ",");
  while (token != NULL) {
    sscanf(token, "%c%d", &command_type, &angle);

    if (angle >= 0 && angle <= 180) {  // Check if angle is within valid range
      switch (command_type) {
        case 'b': move_servo(waist, angle); break;
        case 's': move_servo(shoulder, angle); break;
        case 'e': move_servo(elbow, angle); break;
        case 'w': move_servo(wrist, angle); break;
        case 'g': move_servo(tool_tip, angle); break;
        default: break;  // Invalid command type, do nothing
      }
    }
    token = strtok(NULL, ",");  // Get the next token
  }
}

void move_servo(Servo &motor, int end_angle) {
  int start_angle = motor.read();
  if (start_angle < end_angle) {
    for (int pos = start_angle; pos <= end_angle; pos++) {
      motor.write(pos);
      delay(1);  
    }
  } else {
    for (int pos = start_angle; pos >= end_angle; pos--) {
      motor.write(pos);
      delay(1);  
    }
  }
}

void send_current_positions() {
  Serial.print(waist.read());
  Serial.print(",");
  Serial.print(shoulder.read());
  Serial.print(",");
  Serial.print(elbow.read());
  Serial.print(",");
  Serial.print(wrist.read());
  Serial.print(",");
  Serial.print(tool_tip.read());
  Serial.println();
}
