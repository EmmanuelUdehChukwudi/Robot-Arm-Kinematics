#include <Servo.h>

#define WAIST_PIN 11
#define SHOULDER_PIN 9
#define ELBOW_PIN 10
#define WRIST_PIN 6
#define WRIST_TWIST_PIN 3
#define TOOL_PIN 5

Servo waist;
Servo shoulder;
Servo elbow;
Servo wrist;
Servo wrist_twist;
Servo tool_tip;

// Current positions of the servos
int current_waist = 0;
int current_shoulder = 0;
int current_elbow = 0;
int current_wrist = 0;
int current_wrist_twist = 0;
int current_tool_tip = 35;

char received_chars[32];  // Buffer for the received serial data
int char_pos = 0;  // Position in the buffer

void setup() {
  waist.attach(WAIST_PIN);
  shoulder.attach(SHOULDER_PIN);
  elbow.attach(ELBOW_PIN);
  wrist.attach(WRIST_PIN);
  wrist_twist.attach(WRIST_TWIST_PIN);
  tool_tip.attach(TOOL_PIN);

  waist.write(current_waist);
  shoulder.write(current_shoulder);
  elbow.write(current_elbow);
  wrist.write(current_wrist);
  wrist_twist.write(current_wrist_twist);
  tool_tip.write(current_tool_tip);

  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  while (Serial.available()) {
    char chr = Serial.read();
    if (chr == ',') {
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

  // Send the current positions back to the Python script
  send_current_positions();
}

void process_command(const char *cmd) {
  char command_type = cmd[0];
  int angle = atoi(&cmd[1]);

  if (angle >= 0 && angle <= 180) {  // Check if angle is within valid range
    switch (command_type) {
      case 'b': reach_goal(waist, current_waist, angle, current_waist); break;
      case 's': reach_goal(shoulder, current_shoulder, angle, current_shoulder); break;
      case 'e': reach_goal(elbow, current_elbow, angle, current_elbow); break;
      case 'w': reach_goal(wrist, current_wrist, angle, current_wrist); break;
      case 't': reach_goal(wrist_twist, current_wrist_twist, angle, current_wrist_twist); break;
      case 'g': reach_goal(tool_tip, current_tool_tip, angle, current_tool_tip); break;
      default: break;  // Invalid command type, do nothing
    }
  }
}

void reach_goal(Servo& motor, int start_pos, int goal, int &current_pos) {
  int step = (goal > start_pos) ? 1 : -1;

  for (int pos = start_pos; pos != goal; pos += step) {
    motor.write(pos);
    current_pos = pos;
    delay(10);  // Adjust the delay to control the speed of the motion
  }

  motor.write(goal);  // Ensure the motor reaches the final goal
  current_pos = goal;
}

void send_current_positions() {
  String positions = String(current_waist) + "," + String(current_shoulder) + "," + 
                     String(current_elbow) + "," + String(current_wrist) + "," + 
                     String(current_wrist_twist) + "," + String(current_tool_tip) + "\n";
  Serial.print(positions);
}
