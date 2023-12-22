#include <Servo.h>
#include <math.h>

Servo servo1;  // create servo object to control a servo
Servo servo2;  // create servo object to control a servo
// Servo servoBase; // create servo object to control the base
Servo servoGrip1; // create servo for left hand
Servo servoGrip2; // create servo for right hand

// Function to calculate theta1 and theta2
void calculate_theta(float x, float y, float l1, float l2, float &theta1, float &theta2) {
  // Check if the point is within the robot's workspace
  float d = sqrt(pow(x, 2) + pow(y, 2));
  if (d > l1 + l2 || d < abs(l1 - l2)) {
    Serial.println("The point is outside the robot's workspace.");
    return;
  }

  // Calculate theta2
  float numerator = pow(x, 2) + pow(y, 2) - pow(l1, 2) - pow(l2, 2);
  float denominator = 2 * l1 * l2;
  float cos_theta2 = numerator / denominator;

  // Check if cos_theta2 is within the valid range
  if (cos_theta2 < -1 || cos_theta2 > 1) {
    Serial.println("Invalid value for cos(theta2).");
    return;
  }

  theta2 = acos(cos_theta2);

  // Calculate theta1
  theta1 = atan2(y, x) - atan2((l2 * sin(theta2)), (l1 + l2 * cos(theta2)));
}

// Function to gradually move a servo from current position to target position
void moveServoGradually(Servo &servo, int targetPos) {
  int currentPos = servo.read(); // Get the current position of the servo
  while (currentPos != targetPos) {
    // Move in steps of 1 degree
    if (currentPos < targetPos) currentPos++;
    else currentPos--;

    servo.write(currentPos); // Move the servo to the new position
    delay(15); // Short delay to slow down the movement
  }
}

// void rotateBase(int fromAngle, int toAngle) {
//   if (fromAngle < toAngle) { // For rotation from a lower angle to a higher angle
//     for (int angle = fromAngle; angle <= toAngle; angle++) {
//       servoBase.write(angle);
//       delay(15); // Adjust for a smoother rotation
//     }
//   } else { // For rotation from a higher angle to a lower angle
//     for (int angle = fromAngle; angle >= toAngle; angle--) {
//       servoBase.write(angle);
//       delay(15); // Adjust for a smoother rotation
//     }
//   }
// }


void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  
  // Attach the servos to the pins
  servo1.attach(12);  // attaches the servo on pin 9
  servo2.attach(11);  // attaches the servo on pin 10
  // servoBase.attach(6);
  servoGrip1.attach(9);
  servoGrip2.attach(10);

  // Variables for the lengths of the links
  float l1 = 18.30, l2 = 13.80;

  // Variables for the angles
  float theta1, theta2;

  // Set the initial servo positions to 90 degrees
  moveServoGradually(servo1, 90);
  moveServoGradually(servo2, 90);
  delay(2000);  // Wait for 2 seconds

  // Move the arm to pick up the object at (20, 20)
  calculate_theta(28.0, 10.0, l1, l2, theta1, theta2);
  moveServoGradually(servo1, (int) (theta1 * 180.0 / PI));
  moveServoGradually(servo2, (int) (theta2 * 180.0 / PI));

  delay(2000);  // Wait for 2 seconds

  //Create the end effector fun
   moveServoGradually(servoGrip1, 100);
  moveServoGradually(servoGrip2, 80);
  delay(2000);
  moveServoGradually(servoGrip1, 60);
  moveServoGradually(servoGrip2, 120);
  delay(2000);

  moveServoGradually(servoGrip1, 90);
  moveServoGradually(servoGrip2, 90);
  delay(2000);

  // Move the arm to drop the object at (14, 28)
  calculate_theta(14.0, 28.0, l1, l2, theta1, theta2);
  moveServoGradually(servo1, (int) (theta1 * 180.0 / PI));
  moveServoGradually(servo2, (int) (theta2 * 180.0 / PI));

  delay(2000);  // Wait for 2 seconds

    // Attach your servo to the appropriate pin
  // moveServoGradually(servoBase, 90);
  // delay(3000);
  // rotateBase(90, 180);
  // delay(3000);

// Move the arm to pick up the object at (20, 20)
  calculate_theta(-10.0, 0.0, l1, l2, theta1, theta2);
  moveServoGradually(servo1, (int) (theta1 * 180.0 / PI));
  moveServoGradually(servo2, (int) (theta2 * 180.0 / PI));

  delay(2000);  // Wait for 2 seconds

  moveServoGradually(servoGrip1, 60);
  moveServoGradually(servoGrip2, 120);
  delay(2000);

  moveServoGradually(servoGrip1, 100);
  moveServoGradually(servoGrip2, 80);
  delay(2000);

  // Move the arm to drop the object at (14, 28)
  calculate_theta(14.0, 28.0, l1, l2, theta1, theta2);
  moveServoGradually(servo1, (int) (theta1 * 180.0 / PI));
  moveServoGradually(servo2, (int) (theta2 * 180.0 / PI));

  delay(2000);  // Wait for 2 seconds

  // moveServoGradually(servoBase, 90);
  // delay(3000); 

  // Return the arm to the initial position
  moveServoGradually(servo1, 90);
  moveServoGradually(servo2, 90);
}

void loop() {
  // The loop function is intentionally left empty
}
