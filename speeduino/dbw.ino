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
// unsigned long loopInterval = 469;
unsigned long loopInterval = 1000;

void initialiseDbw() {
  if (configPage10.dbwEnabled == 1) {
    pinMode(configPage10.dbw1Pin, OUTPUT);
    pinMode(configPage10.dbw2Pin, OUTPUT);

    dbwPID.setOutputLimits(-255, 255);
    dbwPID.setWindUpLimits(-20, 20);

    PEDAL = map(currentStatus.pedal, 0, 200, 0, 100);
    TPS = map(currentStatus.TPS, 0, 200, 0, 100);

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
    PEDAL = map(currentStatus.pedal, 0, 200, 0, 100);
    TPS = map(currentStatus.TPS, 0, 200, 0, 100);
    dbwPID.compute();
    if (PEDAL < 1) {
      analogWrite(configPage10.dbw1Pin, 0);
      analogWrite(configPage10.dbw2Pin, 0);
    } else {
      actuateDBW();
    }
  }
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

  }
}

// DBW
void readPedal() {
  if (configPage10.dbwEnabled == true) {
    byte tempPEDAL1 = fastMap1023toX(analogRead(configPage10.dbwPedalPin1), 255);
    byte tempPEDAL2 = fastMap1023toX(analogRead(configPage10.dbwPedalPin2), 255);

    if (configPage10.pedal1Min < configPage10.pedal1Max) {
      if (tempPEDAL1 < configPage10.pedal1Min) {
        tempPEDAL1 = configPage10.pedal1Min;
      }
      if (tempPEDAL1 > configPage10.pedal1Max) {
        tempPEDAL1 = configPage10.pedal1Max;
      }
    } else {
      if (tempPEDAL1 > configPage10.pedal1Min) {
        tempPEDAL1 = configPage10.pedal1Min;
      }
      if (tempPEDAL1 < configPage10.pedal1Max) {
        tempPEDAL1 = configPage10.pedal1Max;
      }
    }

    if (configPage10.pedal2Min < configPage10.pedal2Max) {
      if (tempPEDAL2 < configPage10.pedal2Min) {
        tempPEDAL2 = configPage10.pedal2Min;
      }
      if (tempPEDAL2 > configPage10.pedal2Max) {
        tempPEDAL2 = configPage10.pedal2Max;
      }
    } else {
      if (tempPEDAL2 > configPage10.pedal2Min) {
        tempPEDAL2 = configPage10.pedal2Min;
      }
      if (tempPEDAL2 < configPage10.pedal2Max) {
        tempPEDAL2 = configPage10.pedal2Max;
      }
    }

    currentStatus.pedal1 = map(tempPEDAL1, configPage10.pedal1Min, configPage10.pedal1Max, 0, 200);
    currentStatus.pedal2 = map(tempPEDAL2, configPage10.pedal2Min, configPage10.pedal2Max, 0, 200);

    currentStatus.pedal = (currentStatus.pedal1 + currentStatus.pedal2) / 2;
  }
}

byte readTpsDBW() {
  byte tempTPS1 = fastMap1023toX(analogRead(configPage10.dbwThrotlePin1), 255);
  byte tempTPS2 = fastMap1023toX(analogRead(configPage10.dbwThrotlePin2), 255);

  if (configPage10.throttle1Min < configPage10.throttle1Max) {
    if (tempTPS1 < configPage10.throttle1Min) {
      tempTPS1 = configPage10.throttle1Min;
    }
    if (tempTPS1 > configPage10.throttle1Max) {
      tempTPS1 = configPage10.throttle1Max;
    }
  } else {
    if (tempTPS1 > configPage10.throttle1Min) {
      tempTPS1 = configPage10.throttle1Min;
    }
    if (tempTPS1 < configPage10.throttle1Max) {
      tempTPS1 = configPage10.throttle1Max;
    }
  }

  if (configPage10.throttle2Min < configPage10.throttle2Max) {
    if (tempTPS2 < configPage10.throttle2Min) {
      tempTPS2 = configPage10.throttle2Min;
    }
    if (tempTPS2 > configPage10.throttle2Max) {
      tempTPS2 = configPage10.throttle2Max;
    }
  } else {
    if (tempTPS2 > configPage10.throttle2Min) {
      tempTPS2 = configPage10.throttle2Min;
    }
    if (tempTPS2 < configPage10.throttle2Max) {
      tempTPS2 = configPage10.throttle2Max;
    }
  }

  currentStatus.tps1 = map(tempTPS1, configPage10.throttle1Min, configPage10.throttle1Max, 0, 200);
  currentStatus.tps2 = map(tempTPS2, configPage10.throttle2Min, configPage10.throttle2Max, 0, 200);

  return (currentStatus.tps1 + currentStatus.tps2) / 2;
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
      TPS = map(currentStatus.TPS, 0, 200, 0, 100);
      PWM = tuner.tunePID(TPS);
      actuateDBW();
      while (micros() - microseconds < loopInterval) delayMicroseconds(1);
    }

    configPage10.dbwKP = tuner.getKp();
    configPage10.dbwKI = tuner.getKi();
    configPage10.dbwKD = tuner.getKd();

    dbwPID.stop();
    // dbwPID.begin(&TPS, &PWM, &PEDAL, configPage10.dbwKP, configPage10.dbwKI, configPage10.dbwKD);
  }
}
