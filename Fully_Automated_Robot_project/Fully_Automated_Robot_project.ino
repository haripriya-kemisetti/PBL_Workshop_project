#include <Pixy2.h>
#include <Pixy2CCC.h>
#include <Servo.h>  // Commenting out servo library as per request

// Pixy camera instance
Pixy2 pixy;

// Ultrasonic sensor pins
#define TRIGGER_PIN_1  19 // Left sensor trigger
#define ECHO_PIN_1     18 // Left sensor echo
#define TRIGGER_PIN_2  3  // Center sensor trigger
#define ECHO_PIN_2     2  // Center sensor echo
#define TRIGGER_PIN_3  5  // Right sensor trigger
#define ECHO_PIN_3     4  // Right sensor echo
#define MAX_DISTANCE   200

// Motor control pins
#define MOTOR_IN1      22
#define MOTOR_IN2      23
#define MOTOR_IN3      24
#define MOTOR_IN4      25
#define MOTOR_ENA      8
#define MOTOR_ENB      9

// Motor speed
const int spa = 100; // Left
const int spb = 75; // Right

// Ultrasonic sensor variables
double distance_L, distance_M, distance_R;
double tmp_L, tmp_M, tmp_R;
const int dangerThresh = 15;

// Commenting out servo initialization
Servo armServo;
Servo clawServo;

// Global variable 'ok' to control the chase_collect_cube function
int ok = 0;

void setup() {
  pinMode(TRIGGER_PIN_1, OUTPUT);  // Left sensor trigger
  pinMode(ECHO_PIN_1, INPUT);      // Left sensor echo
  pinMode(TRIGGER_PIN_2, OUTPUT);  // Center sensor trigger
  pinMode(ECHO_PIN_2, INPUT);      // Center sensor echo
  pinMode(TRIGGER_PIN_3, OUTPUT);  // Right sensor trigger
  pinMode(ECHO_PIN_3, INPUT);      // Right sensor echo

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_ENB, OUTPUT);

  pixy.init();
  Serial.begin(115200);

  // Commenting out servo attachment
  armServo.attach(11); // arm component
  armServo.write(165);
  clawServo.attach(17); // catch component
  clawServo.write(0);
}

void loop() {
  int blocks = pixy.ccc.getBlocks();
  Serial.print("blocks=");
  Serial.println(blocks);

  if (pixy.ccc.numBlocks) {
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      Serial.print("  block ");
      Serial.print(i);
      Serial.print(": ");
      pixy.ccc.blocks[i].print();
    }
  }

  measureDistance();

  // Use the minimum distance for general navigation
  double distance = min(distance_L, min(distance_M, distance_R));

  if (pixy.ccc.numBlocks) {
    Serial.println("aaa");
    s();
    delay(10);
    // Commenting out the ball-chasing function
    chase_collect_cube();
  } else {
    if (distance > dangerThresh) {
      // If no obstacle in front, move forward
      s();
      delay(10);
      forward_normal();
      delay(100);
    } else {
      // Determine the direction to turn based on sensor readings
      if (distance_R > distance_L) {
        // Prefer turning right if right is clearer
        s();
        delay(50);
        backward_normal();
        delay(300);
        s();
        delay(50);
        turnRight_normal();
        delay(500);
      } else {
        // Prefer turning left if left is clearer
        s();
        delay(50);
        backward_normal();
        delay(300);
        s();
        delay(50);
        turnRight_normal();
        delay(500);
      }
    }
  }
}

// Ultrasonic sensor functions
void measureDistance() {
  p_L();
  p_M();
  p_R();
  distance_L = 0.01723 * tmp_L;
  distance_M = 0.01723 * tmp_M;
  distance_R = 0.01723 * tmp_R;
  Serial.print("Left cm = ");
  Serial.println(distance_L);
  Serial.print("Middle cm = ");
  Serial.println(distance_M);
  Serial.print("Right cm = ");
  Serial.println(distance_R);
}

void p_L() {
  digitalWrite(TRIGGER_PIN_1, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIGGER_PIN_1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_1, LOW);
  tmp_L = pulseIn(ECHO_PIN_1, HIGH);
}

void p_M() {
  digitalWrite(TRIGGER_PIN_2, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIGGER_PIN_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_2, LOW);
  tmp_M = pulseIn(ECHO_PIN_2, HIGH);
}

void p_R() {
  digitalWrite(TRIGGER_PIN_3, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIGGER_PIN_3, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_3, LOW);
  tmp_R = pulseIn(ECHO_PIN_3, HIGH);
}

// Motor control functions
void s() {
  analogWrite(MOTOR_ENA, 0);
  analogWrite(MOTOR_ENB, 0);
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, HIGH);
}

void forward_normal() {
  Serial.println("Forward");
  analogWrite(MOTOR_ENA, spa * 1.3);
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENB, spb * 1.3);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
}

void backward_normal() {
  Serial.println("back");
  analogWrite(MOTOR_ENA, spa);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  analogWrite(MOTOR_ENB, spb);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);
}

void turnLeft_normal() {
  Serial.println("Left");
  analogWrite(MOTOR_ENA, spa * 1.5);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  analogWrite(MOTOR_ENB, spb * 1.5);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
}

void turnRight_normal() {
  Serial.println("RIGHT");
  analogWrite(MOTOR_ENA, spa * 1.5);
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENB, spb * 1.5);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);
}

void forward_CH() {
  Serial.println("Forward_CH");
  analogWrite(MOTOR_ENA, spa * 1.3);
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENB, spb * 1.3);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
}

void backward_CH() {
  Serial.println("back_CH");
  analogWrite(MOTOR_ENA, spa * 1.7);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  analogWrite(MOTOR_ENB, spb * 1.7);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);
}

void turnLeft_CH() {
  Serial.println("Left_CH");
  analogWrite(MOTOR_ENA, spa * 1.7);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  analogWrite(MOTOR_ENB, spb * 1.7);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
}

void turnRight_CH() {
  Serial.println("RIGHT_CH");
  analogWrite(MOTOR_ENA, spa * 1.7);
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENB, spb * 1.7);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);
}

// Commenting out ball capture and chase functions
void capture() {
  s();        
  // Move arm down
  armServo.write(0);  // Adjust this value to the correct down position
  delay(1500);
  
  forward_CH();
  delay(450);        // Wait for the arm to move

  s();
  // Close claw to grab the object
  clawServo.write(180); // Adjust this value to close the claw
  delay(1500);          // Wait for the claw to move

  backward_CH();
  delay(500);

  s();  

  // Move arm back up
  armServo.write(165);  // Arm up position
  delay(1500);          // Wait for the arm to move

  // Open claw
  clawServo.write(0);   // Open the claw
  delay(1500);
}

void chase_collect_cube(){
  s();
  ok = 0;
  while (ok == 0) {
    int blocks = pixy.ccc.getBlocks();

    // Ensure there are detected blocks before accessing the array
    if (pixy.ccc.numBlocks == 0) break;

    int x = pixy.ccc.blocks[0].m_x;
    int y = pixy.ccc.blocks[0].m_y;

    // Adjust direction based on the block's x-coordinate
    if (x < 145){
      turnLeft_CH();
      delay(50);
      s();  // Stop briefly after turning
      delay(50);
    }
    else if(x > 160){
      turnRight_CH();
      delay(50);
      s();  // Stop briefly after turning
      delay(50);
    }
    else{
      // If within the x-range, check the y-coordinate
      if (y >= 155){
        s();  // Stop the robot before capturing
        delay(70);
        capture();  // Capture the object
        ok = 1;  // End the loop after capturing
        break;
      }
      else{
        forward_CH();  // Move forward towards the object
        delay(100);
      }
    }
  }
}
