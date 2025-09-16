#include <Servo.h>

Servo servo[6];  // base, shoulder, elbow, gripper, wrist_spin (left-right), wrist_tilt (up-down)
int default_angle[6] = {90, 90, 90, 90, 90, 90}; // Initial positions

void setup() {
    Serial.begin(115200);
    servo[0].attach(7);  // Base
    servo[1].attach(8);  // Shoulder
    servo[2].attach(9);  // Elbow
    servo[3].attach(10); // Gripper
    servo[4].attach(11); // Wrist Left-Right (Spinning)
    servo[5].attach(12); // Wrist Up-Down (Tilting)

    for (int i = 0; i < 6; i++) {
        servo[i].write(default_angle[i]);
    }

    Serial.println("Arduino Ready");
}

byte angle[6];
byte pre_angle[6];

void loop() {
    if (Serial.available() >= 6) {
        Serial.readBytes(angle, 6);

        // Base, Shoulder, Elbow, Gripper, Wrist Tilt (i = 0 to 2, 3, 5)
        for (int i = 0; i < 6; i++) {
            if (i == 4) continue; // skip wrist spin in this loop
            if (angle[i] != pre_angle[i]) {
                servo[i].write(angle[i]);
                pre_angle[i] = angle[i];
            }
        }

        // Wrist Left-Right (Continuous rotation servo)
        if (angle[4] < 90) {
            servo[4].write(180); // Clockwise
        } else if (angle[4] > 90) {
            servo[4].write(0);   // Counter-clockwise
        } else {
            servo[4].write(90);  // Stop
        }
    }
}





