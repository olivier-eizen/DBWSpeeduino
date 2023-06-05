#include "ArduPID.h"
#include "Arduino.h"
#include "auxiliaries.h"
#include "decoders.h"
#include "globals.h"
#include "maths.h"
#include "pidautotuner.h"
#include "sensors.h"
#include "src/PID_v1/PID_v1.h"
#include "storage.h"
#include "timers.h"

ArduPID dbwPID;
double PEDAL, PWM, TPS;
unsigned long loopInterval = 1000;


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
  if (configPage10.dbwEnabled == 1) {
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
}

void readTB() {
  byte a = readTpsDBW();
  TPS = (double)a;
}

void dbwCalibrationPedalMin() {
  if (configPage10.dbwEnabled == 1 && currentStatus.RPM == 0) {
    configPage10.pedal1Min = fastMap1023toX(analogRead(configPage10.dbwPedalPin1), 255);
    configPage10.pedal2Min = fastMap1023toX(analogRead(configPage10.dbwPedalPin2), 255);
    writeConfig(10);
  }
}

void dbwCalibrationPedalMax() {
  if (configPage10.dbwEnabled == 1 && currentStatus.RPM == 0) {
    configPage10.pedal1Max = fastMap1023toX(analogRead(configPage10.dbwPedalPin1), 255);
    configPage10.pedal2Max = fastMap1023toX(analogRead(configPage10.dbwPedalPin2), 255);
    writeConfig(10);
  }
}

void dbwCalibrationTPS() {
  if (configPage10.dbwEnabled == 1 && currentStatus.RPM == 0) {
    // Maximum
    analogWrite(configPage10.dbwThrotlePin1, HIGH);
    analogWrite(configPage10.dbwThrotlePin2, HIGH);
    delay(250);
    configPage10.throttle1Max = fastMap1023toX(analogRead(configPage10.dbwThrotlePin1), 255);
    configPage10.throttle2Max = fastMap1023toX(analogRead(configPage10.dbwThrotlePin2), 255);
    // Minimum
    analogWrite(configPage10.dbwThrotlePin1, LOW);
    analogWrite(configPage10.dbwThrotlePin2, LOW);
    delay(250);
    configPage10.throttle1Min = fastMap1023toX(analogRead(configPage10.dbwThrotlePin1), 255);
    configPage10.throttle2Min = fastMap1023toX(analogRead(configPage10.dbwThrotlePin2), 255);
    writeConfig(10);
  }
}

void dbwCalibrationAuto() {
  if (configPage10.dbwEnabled == 1 && currentStatus.RPM == 0) {
    dbwPID.begin(&TPS, &PWM, &PEDAL, configPage10.dbwKP, configPage10.dbwKI, configPage10.dbwKD);
    PIDAutotuner tuner = PIDAutotuner();
    tuner.setTargetInputValue(90);
    tuner.setLoopInterval(loopInterval);
    tuner.setOutputRange(-255, 255);
    tuner.setZNMode(PIDAutotuner::znModeNoOvershoot);
    tuner.startTuningLoop();

    unsigned long microseconds;
    while (!tuner.isFinished()) {
      microseconds = micros();
      readTB();
      PWM = tuner.tunePID(TPS);
      actuateDBW();
      while (micros() - microseconds < loopInterval) delayMicroseconds(1);
    }

    configPage10.dbwKP = tuner.getKp();
    configPage10.dbwKI = tuner.getKi();
    configPage10.dbwKD = tuner.getKd();

    dbwPID.stop();
    writeConfig(10);
  }
}
