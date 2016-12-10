/* MIT License
 *
 * Copyright (c) 2016 Matthew Barulic
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Wire.h>
#include <I2CEncoder.h>
#include <Servo.h>

Servo s1;
Servo s2;
Servo s3;
Servo s4;

const int upPos[] = {120, 120, 120, 130};
const int downPos[] = {77, 77, 75, 85};
int currentPos[] = {77, 77, 75, 85};

const int batteryPin = A0;
const double batteryScale = 0.012;
const double batteryBias = 1.98;

Servo fretMotor;

double fretSetPoint = 0.0;
const double fretPositions[] = {0, 0, -1.5, -3.5, -5.25, -6.75, -8.5, -9.75, -11.0, -12.25};
//const double fretPositions[] =   {0, 0,  0.5, 1.166, 1.75, 2.25,  2.83,  3.25,  3.66,  4.083};
const int fretMotorUpSpeed = 170;
const int fretMotorDownSpeed = 12;
const int fretMotorStopped = 180;

I2CEncoder encoder;

const int limitSwitchPin = 2;

Servo headServo;
int headPos;

void zeroFretMotor() {
  if(digitalRead(limitSwitchPin) == LOW) {
    fretMotor.write(fretMotorDownSpeed);
    delay(1000);
  }
  fretMotor.write(130);
  while(digitalRead(limitSwitchPin) == HIGH) {
  }
  fretMotor.write(fretMotorStopped);
  encoder.zero();
}

double errorHistory[] = {0, 0, 0, 0, 0};
const int errorHistorySize = 5;
int errorHistoryIndex = 0;

void updateFretMotor() {
  if(digitalRead(limitSwitchPin) == LOW) {
    encoder.zero();
  }
  
  double currentPosition = encoder.getPosition();

//  Serial.print(currentPosition);
//  Serial.print('\t');
//  Serial.print(fretSetPoint);
//  Serial.print('\t');

  double error = currentPosition - fretSetPoint;

  errorHistory[errorHistoryIndex] = error;
  errorHistoryIndex = (errorHistoryIndex + 1) % errorHistorySize;

  double integralError = 0;

  for(int i = 0; i < errorHistorySize; i++) {
    integralError += errorHistory[i];
  }

  static double P = -170;
  static double I = -10;

  double PWM = (P * error) + (I * integralError);

//  Serial.print(currentPosition);
//  Serial.print('\t');
//  Serial.print(fretSetPoint);
//  Serial.print('\t');
//  Serial.print(PWM);
//  Serial.print('\t');

  if(PWM > 0 && digitalRead(limitSwitchPin) == LOW) {
    fretMotor.write(fretMotorStopped);
//    Serial.println("SOFT STOPPED");
    return;
  }
  if( abs(PWM) < 12 ) {
//    Serial.println(fretMotorStopped);
    fretMotor.write(fretMotorStopped);
  } else {
    PWM += 90;
  
    PWM = min(fretMotorUpSpeed, max(fretMotorDownSpeed, PWM));

//    Serial.println(PWM);
    fretMotor.write(PWM);
  }
}

void writeServos() {
  s1.write(currentPos[0]);
  s2.write(currentPos[1]);
  s3.write(currentPos[2]);
  s4.write(currentPos[3]);
}

void setup() {
  Wire.begin();
  encoder.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);

  Serial.begin(115200);
  
  s1.attach( 6  );
  s2.attach( 9  );
  s3.attach( 10 );
  s4.attach( 11 );
  fretMotor.attach(5);
  headServo.attach(3);

  pinMode(batteryPin, INPUT);

  pinMode(limitSwitchPin, INPUT);
  digitalWrite(limitSwitchPin, HIGH);

  headPos = 60;
  headServo.write(headPos);

  writeServos();

  zeroFretMotor();
}

void loop() {
  if(Serial.available()) {
    auto command = Serial.readStringUntil('\n');
    switch(command.charAt(0)) {
      case 'b': {
        Serial.println( (analogRead(batteryPin) * batteryScale) + batteryBias );
        break;
      }
      case 's': {
        int s = command.charAt(1) - 49;
        if(currentPos[s] == downPos[s])
          currentPos[s] = upPos[s];
        else
          currentPos[s] = downPos[s];
        break;
      }
      case 'f': {
        command.remove(0,1);
        int f = command.toInt();
        fretSetPoint = fretPositions[f];
//          fretSetPoint = command.toFloat();
        break;
      }
      case 'e': {
        Serial.println(encoder.getPosition());
        break;
      }
      case 'z': {
        zeroFretMotor();
        break;
      }
      case 'g': {
        Serial.println(fretSetPoint);
        break;
      }
      case 'h': {
        if(headPos == 60)
          headPos = 45;
        else
          headPos = 60;
        headServo.write(headPos);
        break;
      }
      default:
        break;
    }
  }

  writeServos();
  updateFretMotor();
}
