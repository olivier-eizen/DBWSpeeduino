#include "ArduPID.h"
#include "Arduino.h"
#include "auxiliaries.h"
#include "decoders.h"
#include "globals.h"
#include "maths.h"
#include "pidautotuner.h"
#include "src/PID_v1/PID_v1.h"
#include "timers.h"

ArduPID dbwPID;
double PEDAL, PWM, TPS;
double Kp = configPage10.dbwKP;
double Ki = configPage10.dbwKI;
double Kd = configPage10.dbwKD;

// void initDbw() {
//   fan_pin_port = portOutputRegister(digitalPinToPort(pinFan));
//   fan_pin_mask = digitalPinToBitMask(pinFan);
//   FAN_OFF();  // Initialise program with the fan in the off state
//   BIT_CLEAR(currentStatus.status4, BIT_STATUS4_FAN);
//   currentStatus.fanDuty = 0;
// }

void actuateDBW() {
  if (PWM > 0) {
    analogWrite(configPage10.dbwThrotlePin1, abs(PWM));
    analogWrite(configPage10.dbwThrotlePin2, LOW);
  } else {
    analogWrite(configPage10.dbwThrotlePin1, LOW);
    analogWrite(configPage10.dbwThrotlePin2, abs(PWM));
  }
}

void dbw() {
  PEDAL = currentStatus.pedal;
  TPS = currentStatus.TPS;
  dbwPID.compute();
  if (PEDAL < 1 && TPS < 1) {
    analogWrite(configPage10.dbwThrotlePin1, LOW);
    analogWrite(configPage10.dbwThrotlePin2, LOW);
  } else {
    actuateDBW();
  }
}

// void calibrationAuto() {
//   dbwPID.begin(&TPS, &PWM, &PEDAL, config10., Ki, Kd);

//   PIDAutotuner tuner = PIDAutotuner();
//   tuner.setTargetInputValue(90);
//   tuner.setLoopInterval(loopInterval);
//   tuner.setOutputRange(-255, 255);
//   tuner.setZNMode(PIDAutotuner::znModeNoOvershoot);
//   tuner.startTuningLoop();
//   long microseconds;
//   while (!tuner.isFinished()) {
//     long prevMicroseconds = microseconds;
//     microseconds = micros();

//     readTB();
//     PWM = tuner.tunePID(TPS);
//     actuateDBW();

//     while (micros() - microseconds < loopInterval) delayMicroseconds(1);
//   }
//   Kp = tuner.getKp();
//   Ki = tuner.getKi();
//   Kd = tuner.getKd();
//   Serial.print("KP ");
//   Serial.print(Kp);
//   Serial.print(" KI ");
//   Serial.print(Ki);
//   Serial.print(" KD ");
//   Serial.println(Kd);
//   dbwPID.stop();
// }
