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

byte dbwCounter = 0;
ArduPID dbwPID;
PIDAutotuner tuner;
bool dbw_calibration_tps = false;
bool dbw_autotune_tps = false;
bool dbw_error = false;
// long configPage10.dbwSensorTolerance = 10;
long dbw_pwm_low_limit = -(dbw_pwm_max_count - 1);
long dbw_pwm_high_limit = +(dbw_pwm_max_count - 1);
long dbw_loop_speed = 0;
long dbw_loop_speed_cur = 0;
long dbw_precision = 10;
double DBW_TARGET, PWM, TPS;
double KP = 0, KI = 0, KD = 0;
double loopInterval = 2000;

void initialiseDbw() {
  if (configPage10.dbwEnabled == 1) {
    pinMode(configPage10.dbw1Pin, OUTPUT);
    pinMode(configPage10.dbw2Pin, OUTPUT);
    dbw_pwm_low_limit = -(dbw_pwm_max_count - 1);
    dbw_pwm_high_limit = +(dbw_pwm_max_count - 1);
    dbwPID.setOutputLimits(dbw_pwm_low_limit, dbw_pwm_high_limit);
    dbwPID.setWindUpLimits(-20, 20);

    startPid();
  }
}

ISR(TIMER1_COMPB_vect) {
  if (!dbw_calibration_tps) {
    unsigned long _pwm = constrain(abs(dbw_pwm_target_value), 1, dbw_pwm_high_limit);

    if (dbw_error) {
      SET_COMPARE(DBW_TIMER_COMPARE, DBW_TIMER_COUNTER + dbw_pwm_max_count);
      dbw_pwm_state = false;
      DBW1_PIN_LOW();
      DBW2_PIN_LOW();
    } else if (DBW_TARGET < configPage10.dbwFCT && !dbw_autotune_tps) {
      SET_COMPARE(DBW_TIMER_COMPARE, DBW_TIMER_COUNTER + dbw_pwm_max_count);
      dbw_pwm_state = false;
      DBW1_PIN_LOW();
      DBW2_PIN_LOW();
    } else if (DBW_TARGET > configPage10.dbwWOT && !dbw_autotune_tps) {
      DBW1_PIN_HIGH();
      DBW2_PIN_LOW();
      SET_COMPARE(DBW_TIMER_COMPARE, DBW_TIMER_COUNTER + dbw_pwm_max_count);
      dbw_pwm_state = false;
    } else if (dbw_pwm_target_value > 0) {
      posi(_pwm);
    } else {
      nega(_pwm);
    }
  }
}

void posi(unsigned long pwm) {
  if (dbw_pwm_state) {
    DBW1_PIN_LOW();  // Be sure both signal are off
    DBW2_PIN_LOW();  // Be sure both signal are off
    SET_COMPARE(DBW_TIMER_COMPARE, DBW_TIMER_COUNTER + (dbw_pwm_max_count - dbw_pwm_cur_value));
    dbw_pwm_state = false;
  } else {
    DBW1_PIN_HIGH();
    DBW2_PIN_LOW();  // Be sure signal is off
    SET_COMPARE(DBW_TIMER_COMPARE, DBW_TIMER_COUNTER + pwm);
    dbw_pwm_cur_value = pwm;
    dbw_pwm_state = true;
  }
}

void nega(unsigned long pwm) {
  if (dbw_pwm_state) {
    DBW1_PIN_LOW();  // Be sure both signal are off
    DBW2_PIN_LOW();  // Be sure both signal are off
    SET_COMPARE(DBW_TIMER_COMPARE, DBW_TIMER_COUNTER + (dbw_pwm_max_count - dbw_pwm_cur_value));
    dbw_pwm_state = false;
  } else {
    DBW1_PIN_LOW();  // Be sure signal is off
    DBW2_PIN_HIGH();
    SET_COMPARE(DBW_TIMER_COMPARE, DBW_TIMER_COUNTER + pwm);
    dbw_pwm_cur_value = pwm;
    dbw_pwm_state = true;
  }
}

void measureLoopTime() {
  dbw_loop_speed = micros() - dbw_loop_speed_cur;
  dbw_loop_speed_cur = micros();
}

void dbw() {
  if (configPage10.dbwEnabled == 1) {
    ENABLE_DBW_TIMER();
    readTpsDBW(false);  // smh it need to be there
    TPS = currentStatus.TPS;

    if (currentStatus.pedal < configPage10.dbwFCT && currentStatus.MAP < configPage10.dbwIdleTriggerMAP * 2 && currentStatus.RPM < configPage10.dbwIdleTriggerRPM) {
      DBW_TARGET = get3DTableValue(&dbwIdleTable, currentStatus.MAP, currentStatus.RPM);  // Idle
      BIT_SET(currentStatus.spark, BIT_SPARK_IDLE);
    } else {
      DBW_TARGET = get3DTableValue(&dbwTable, (currentStatus.pedal * 2), currentStatus.RPM);  // Driving
      BIT_CLEAR(currentStatus.spark, BIT_SPARK_IDLE);
    }

    if (tuner.isFinished() && dbw_autotune_tps) {
      saveGain();
      startPid();
      dbw_autotune_tps = false;
    }

    if (dbw_autotune_tps) {
      PWM = tuner.tunePID(TPS);  // Set PWM
    } else {
      dbwPID.compute();  // Set PWM
      measureLoopTime();
    }

    currentStatus.dbwDuty = map(PWM, 0, dbw_pwm_high_limit, 0, 200);
    dbw_pwm_target_value = PWM;
    dbwCounter++;
  }
}

void dbwCalibrationPedalMin() {
  if (configPage10.dbwEnabled == 1 && currentStatus.RPM == 0) {
    configPage10.pedal1Min = fastMap1023toX(analogRead(configPage10.dbwPedalPin1), 255);
    configPage10.pedal2Min = fastMap1023toX(analogRead(configPage10.dbwPedalPin2), 255);
    dbw_error = false;
  }
}

void dbwCalibrationPedalMax() {
  if (configPage10.dbwEnabled == 1 && currentStatus.RPM == 0) {
    configPage10.pedal1Max = fastMap1023toX(analogRead(configPage10.dbwPedalPin1), 255);
    configPage10.pedal2Max = fastMap1023toX(analogRead(configPage10.dbwPedalPin2), 255);
    dbw_error = false;
  }
}

void dbwCalibrationTPS() {
  if (configPage10.dbwEnabled == 1 && currentStatus.RPM == 0) {
    dbw_calibration_tps = true;
    // Maximum
    DBW1_PIN_HIGH();
    delay(500);
    configPage10.throttle1Max = fastMap1023toX(analogRead(configPage10.dbwThrotlePin1), 255);
    configPage10.throttle2Max = fastMap1023toX(analogRead(configPage10.dbwThrotlePin2), 255);

    // Minimum
    DBW1_PIN_LOW();
    delay(500);
    configPage10.throttle1Min = fastMap1023toX(analogRead(configPage10.dbwThrotlePin1), 255);
    configPage10.throttle2Min = fastMap1023toX(analogRead(configPage10.dbwThrotlePin2), 255);
    dbw_calibration_tps = false;
    dbw_error = false;
  }
}

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

    // if both Signal exceed 5% difference
    if (abs(currentStatus.pedal1 - currentStatus.pedal2) > (configPage10.dbwSensorTolerance)) {
      dbw_error = true;
    }

    currentStatus.pedal = (currentStatus.pedal1 + currentStatus.pedal2) / 2;
  }
}

void readTpsDBW(bool useFilter) {
  byte tempTPS1 = fastMap1023toX(analogRead(configPage10.dbwThrotlePin1), 255);
  byte tempTPS2 = fastMap1023toX(analogRead(configPage10.dbwThrotlePin2), 255);

  byte tempTPS = (tempTPS1 + tempTPS2) / 2;
  if (useFilter == true) {
    currentStatus.tpsADC = ADC_FILTER(tempTPS, configPage4.ADCFILTER_TPS, currentStatus.tpsADC);
  } else {
    currentStatus.tpsADC = tempTPS;
  }

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

  // if both Signal exceed 5% difference
  if (abs(currentStatus.tps1 - currentStatus.tps2) > (configPage10.dbwSensorTolerance)) {
    dbw_error = true;
  }

  currentStatus.TPS = (currentStatus.tps1 + currentStatus.tps2) / 2;
}

void dbwCalibrationAuto() {
  if (configPage10.dbwEnabled == 1 && currentStatus.RPM == 0) {
    dbw_autotune_tps = true;
    startPid();
    tuner = PIDAutotuner();
    tuner.setTargetInputValue(180);
    tuner.setLoopInterval(dbw_loop_speed);
    tuner.setOutputRange(1, dbw_pwm_high_limit);
    tuner.setZNMode(PIDAutotuner::znModeNoOvershoot);
    tuner.startTuningLoop();
  }
}

void saveGain() {
  configPage10.dbwKP = tuner.getKp() * dbw_precision;
  configPage10.dbwKI = tuner.getKi() * dbw_precision;
  configPage10.dbwKD = tuner.getKd() * dbw_precision;
  writeConfig(10);
  BIT_SET(currentStatus.status4, BIT_STATUS4_DBW_REFRESH);
}

void startPid() {
  dbwPID.stop();
  dbwPID.begin(&TPS, &PWM, &DBW_TARGET, configPage10.dbwKP / dbw_precision, configPage10.dbwKI / dbw_precision, configPage10.dbwKD / dbw_precision);
}