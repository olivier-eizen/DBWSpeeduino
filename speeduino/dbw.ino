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
#define frequencePWMde490hz 0b00000100
ArduPID dbwPID;
double PEDAL; 
double PWM; 
double TPS;
unsigned long loopInterval = 1000;

void initialiseDbw() {
  if (configPage10.dbwEnabled == 1) {
    pinMode(configPage10.dbw1Pin, OUTPUT);
    pinMode(configPage10.dbw2Pin, OUTPUT);
    
    dbwPID.setOutputLimits(-255, 255);
    dbwPID.setWindUpLimits(-20, 20);

    dbwPID.begin(&TPS, &PWM, &PEDAL, configPage10.dbwKP, configPage10.dbwKI, configPage10.dbwKD);
  }
}

void actuateDBW() {
  if (PWM > 0) {
    analogWrite(configPage10.dbw1Pin, abs(PWM));
    analogWrite(configPage10.dbw2Pin, 0);
  } else {
    analogWrite(configPage10.dbw1Pin, 0);
    analogWrite(configPage10.dbw2Pin, abs(PWM));
  }
}

void dbw() {
  if (configPage10.dbwEnabled == 1) {
    PEDAL = currentStatus.pedal;
    TPS = currentStatus.TPS;
    dbwPID.compute();
    if (PEDAL < 1) {
      analogWrite(configPage10.dbw1Pin, 0);
      analogWrite(configPage10.dbw2Pin, 0);
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
    analogWrite(configPage10.dbw1Pin, 255);
    analogWrite(configPage10.dbw2Pin, 0);
    delay(500);
    configPage10.throttle1Max = fastMap1023toX(analogRead(configPage10.dbwThrotlePin1), 255);
    configPage10.throttle2Max = fastMap1023toX(analogRead(configPage10.dbwThrotlePin2), 255);

    // Minimum
    analogWrite(configPage10.dbw1Pin, 0);
    analogWrite(configPage10.dbw2Pin, 0);
    delay(500);
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
    dbwPID.begin(&TPS, &PWM, &PEDAL, configPage10.dbwKP, configPage10.dbwKI, configPage10.dbwKD);
  }
}
