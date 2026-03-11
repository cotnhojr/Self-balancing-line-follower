#include <MPU6050.h>
#include <Wire.h>
#include <Kalman.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Định nghĩa chân cảm biến IR
const int IR1 = 6;
const int IR2 = 7;
const int IR3 = 8;
const int IR4 = 9;
const int IR5 = 10;

// Định nghĩa tham số PID
#define IRTrack_Trun_KP 50.0
#define IRTrack_Trun_KI 0.015
#define IRTrack_Trun_KD 0.25
#define IRTrack_Middle 0

// Biến PID
int8_t error = 0;
float Move_X = 0;  // Tốc độ tiến
float Move_Z = 0;  // Góc quay


// Motor control Pins
int leftpwm = 46;
int leftmotor1 = 50;
int leftmotor2 = 48;
int righpwm = 44;
int righmotor1 = 40;
int righmotor2 = 42;


// Encoder variables
volatile long leftencoder;
volatile long righencoder;
volatile long leftencoder_1;
volatile long righencoder_1;
volatile long leftencoder_2;
volatile long righencoder_2;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 100;

int leftencoder_a = 3;
int leftencoder_b = 5;
int righencoder_a = 2;
int righencoder_b = 4;

bool readyToMove = false;
unsigned long stableTime = 0;

// IMU Data
float theta, psi, phi, theta_1, phi_1, en;
float thetadot, psidot, phidot;
float thetaold, phiold;
float AcX, AcY, AcZ;
float Gxro;
float Gxrate;
float pitch, offset, GHPWM;
uint32_t timer;
uint8_t i2cData[14];
unsigned long last_time;

float K[6] = { 30.5, 1.35, 0.2, 0.22, 1.5, 0.5 };  

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  offset = 1.0;
  GHPWM = 250;

  // Khởi tạo chân cảm biến IR
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);

  pinMode(leftpwm, OUTPUT);
  pinMode(righpwm, OUTPUT);
  pinMode(leftmotor1, OUTPUT);
  pinMode(leftmotor2, OUTPUT);
  pinMode(righmotor1, OUTPUT);
  pinMode(righmotor2, OUTPUT);
  pinMode(leftencoder_a, INPUT_PULLUP);
  pinMode(leftencoder_b, INPUT_PULLUP);
  pinMode(righencoder_a, INPUT_PULLUP);
  pinMode(righencoder_b, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftencoder_a), left_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(righencoder_a), righ_isr, RISING);

  Wire.begin();
  while (i2cWrite(0x6B, 0x01, true))
    ;
  while (i2cRead(0x75, i2cData, 1))
    ;
  if (i2cData[0] != 0x68) {
    Serial.print(F("Error reading sensor"));
    while (1)
      ;
  }
  delay(100);
  while (i2cRead(0x3B, i2cData, 6))
    ;
  AcX = ((i2cData[0] << 8) | i2cData[1]);
  AcZ = ((i2cData[4] << 8) | i2cData[5]);
  pitch = (atan2(AcX, AcZ)) * RAD_TO_DEG;
  kalmanX.setAngle(pitch);
  timer = micros();

  Serial.println("pitch,output1,output2,phi_desired,IR1,IR2,IR3,IR4,IR5");
}

void PID_track_get() {
  int ir1 = digitalRead(IR1);
  int ir2 = digitalRead(IR2);
  int ir3 = digitalRead(IR3);
  int ir4 = digitalRead(IR4);
  int ir5 = digitalRead(IR5);

  if (ir1 == 1 && ir2 == 1 && ir3 == 0 && ir4 == 1 && ir5 == 1) {
    error = 0;  
  } else if (ir1 == 1 && ir2 == 0 && ir3 == 0 && ir4 == 1 && ir5 == 1) {
    error = 1;  
  } else if (ir1 == 1 && ir2 == 0 && ir3 == 1 && ir4 == 1 && ir5 == 1) {
    error = 2;  
  } else if (ir1 == 0 && ir2 == 0 && ir3 == 1 && ir4 == 1 && ir5 == 1) {
    error = 3;  
  } else if (ir1 == 0 && ir2 == 1 && ir3 == 1 && ir4 == 1 && ir5 == 1) {
    error = 5;  
  } else if (ir1 == 1 && ir2 == 1 && ir3 == 0 && ir4 == 0 && ir5 == 1) {
    error = -1;  
  } else if (ir1 == 1 && ir2 == 1 && ir3 == 1 && ir4 == 0 && ir5 == 1) {
    error = -2; 
  } else if (ir1 == 1 && ir2 == 1 && ir3 == 1 && ir4 == 0 && ir5 == 0) {
    error = -3;  
  } else if (ir1 == 1 && ir2 == 1 && ir3 == 1 && ir4 == 1 && ir5 == 0) {
    error = -5;  
  } else {
    error = 0; 
  }
}


int Turn_IRTrack_PD(float gyro) {
  static float prev_output = 0;
  static float IRTrack_Integral = 0;

  PID_track_get();  
  float err = error - IRTrack_Middle;
  IRTrack_Integral += err;

  // Tính PID thô
  float raw_output = IRTrack_Trun_KP * err
                     + IRTrack_Trun_KI * IRTrack_Integral
                     + IRTrack_Trun_KD * gyro;

  float filtered_output = 0.8 * prev_output + 0.2 * raw_output;
  prev_output = filtered_output;

  return filtered_output;
}

void Car_tracking() {
  int ir1 = digitalRead(IR1);
  int ir2 = digitalRead(IR2);
  int ir3 = digitalRead(IR3);
  int ir4 = digitalRead(IR4);
  int ir5 = digitalRead(IR5);

  if (ir1 == 1 && ir2 == 1 && ir3 == 1 && ir4 == 1 && ir5 == 1) {
    //Move_Z = 0;
    Move_X = 0; 
  }
  else if (ir2 == 0 && ir4 == 1) {
    //Move_Z = 300;
    Move_X = -375; 
  } else if (ir2 == 1 && ir4 == 0) {
    //Move_Z = -300;
    Move_X = -375;  
  } else if (ir1 == 0 && ir4 == 1 && ir5 == 1) {
    //Move_Z = 1300;
    Move_X = -375;  
  } else if (ir5 == 0 && ir1 == 1 && ir2 == 1) {
    //Move_Z = -1300;
    Move_X = -375;  
  } else if (ir1 == 1 && ir2 == 1 && ir3 == 0 && ir4 == 1 && ir5 == 1) {
    Move_X = -550;  
  } else {
    Move_X = -375;  
  }
  
  leftencoder = Move_X ;
  righencoder = Move_X;
}


void Bluetooth() {
  if (Serial1.available()) {
    char inByte = Serial1.read(); 
    Serial.println(inByte);  
    switch (inByte) {
      case 'A':
        Serial.println("A");
        break;
      case 'M':
        mode = MANUAL;
        leftencoder = 0;
        righencoder = 0;
        Serial.println("M");
        break;
      case 'F':
        Move_X = -500;
        Move_Z = 0;
        break;
      case 'B':
        Move_X = 600;
        Move_Z = 0;
        break;
      case 'R':
        Move_X = 0;
        Move_Z = -1500;
        break;
      case 'L':
        Move_X = 0;
        Move_Z = 1500;
        break;
      case 'S':
        Move_X = 0;
        Move_Z = 0;
        break;
    }
    if (mode == MANUAL) {
      leftencoder = Move_X + Move_Z;
      righencoder = Move_X - Move_Z;
    }
  }
}

void loop() {
  READ_MPU();
  Bluetooth();

  if ((abs(pitch) > 40) || en == 0) {
    stopMotors();
    righencoder = 0;
    leftencoder = 0;
    en = 0;
  }

  if (abs(pitch) < 2.5) { en = 1;
   if (millis() - stableTime > 800) {
      readyToMove = true;
   }
  } else {
    stableTime = millis();
    readyToMove = false;
  } 

  if (en == 1) {
    if ((micros() - last_time) > 5000) {
      float dt = (float)(micros() - last_time) / 1000000.0;
      last_time = micros();

      theta = 0.55 * (leftencoder + righencoder);
      theta_1 = 0.55 * (leftencoder_1 + righencoder_1);
      phi = 0.25 * (leftencoder - righencoder);
      phi_1 = 0.25 * (leftencoder_1 - righencoder_1);
      thetadot = (theta - thetaold) / dt;
      phidot = (phi - phiold) / dt;
      thetaold = theta;
      phiold = phi;
      theta = constrain(theta, -900, 900);

      float phi_desired = IRTrackTurn * 0.01;  

      float x1[6] = { pitch, Gxrate, theta, thetadot, -(phi_desired + Move_Z * 0.01 ), -phidot };  
      float x2[6] = { pitch, Gxrate, theta, thetadot, (phi_desired + Move_Z * 0.01 ), phidot };  
      float output1 = 0;
      float output2 = 0;

      for (int i = 0; i < 6; i++) {
        output1 += K[i] * x1[i];
        output2 += K[i] * x2[i];
      }

      motorcontrol(output1, output2);

      Serial.print(pitch);
      Serial.print(",");
      Serial.print(output1);
      Serial.print(",");
      Serial.print(output2);
      Serial.print(",");
      Serial.print(phi_desired);
    }
    Serial.println();  
  }

  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= sendInterval) {
    String S = String(pitch) + "," + String(theta_1) + "," + String(phi_1);
    Serial1.println(S);
    lastSendTime = currentTime;
  }
}

void stopMotors() {
  analogWrite(leftpwm, 0);
  analogWrite(righpwm, 0);
  digitalWrite(leftmotor1, LOW);
  digitalWrite(leftmotor2, LOW);
  digitalWrite(righmotor1, LOW);
  digitalWrite(righmotor2, LOW);
}

void left_isr() {
  if (digitalRead(leftencoder_b)) {
    leftencoder--;
    leftencoder_1--;
  } else {
    leftencoder++;
    leftencoder_1++;
  }
}

void righ_isr() {
  if (digitalRead(righencoder_b)) {
    righencoder++;
    righencoder_1++;
  } else {
    righencoder--;
    righencoder_1--;
  }
}

void READ_MPU() {
  while (i2cRead(0x3B, i2cData, 14))
    ;
  AcX = ((i2cData[0] << 8) | i2cData[1]);
  AcZ = ((i2cData[4] << 8) | i2cData[5]);
  Gxro = -((i2cData[10] << 8) | i2cData[11]);
  Gxrate = (double)Gxro / 131.0;
  float dt = (float)(micros() - timer) / 1000000.0;
  timer = micros();
  pitch = kalmanX.getAngle((atan2(AcX, AcZ)) * RAD_TO_DEG, Gxrate, dt);
  pitch -= 1 + offset;
}

void motorcontrol(float output1, float output2) {
  long pwm_value1 = constrain(abs(output1), 0, GHPWM);
  long pwm_value2 = constrain(abs(output2), 0, GHPWM);
  if (output1 > 0) {
    leftmotor(pwm_value1, 1);
  } else {
    leftmotor(pwm_value1, 0);
  }
  if (output2 > 0) {
    righmotor(pwm_value2, 1);
  } else {
    righmotor(pwm_value2, 0);
  }
}

void leftmotor(uint8_t lpwm, int direct) {
  if (direct == 1) {
    digitalWrite(leftmotor1, HIGH);
    digitalWrite(leftmotor2, LOW);
    analogWrite(leftpwm, lpwm);
  } else {
    digitalWrite(leftmotor1, LOW);
    digitalWrite(leftmotor2, HIGH);
    analogWrite(leftpwm, lpwm);
  }
}

void righmotor(uint8_t rpwm, int direct) {
  if (direct == 1) {
    digitalWrite(righmotor1, HIGH);
    digitalWrite(righmotor2, LOW);
    analogWrite(righpwm, rpwm);
  } else {
    digitalWrite(righmotor1, LOW);
    digitalWrite(righmotor2, HIGH);
    analogWrite(righpwm, rpwm);
  }
}