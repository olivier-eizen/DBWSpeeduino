#ifndef AUX_H
#define AUX_H

#include BOARD_H //Note that this is not a real file, it is defined in globals.h. 

void initialiseAuxPWM();
void boostControl();
void boostDisable();
void boostByGear();
void idleControl();
void initialiseFan();
void initialiseDbw();
void nitrousControl();
void fanControl();
// void vvtControl();
// void wmiControl();

#define SIMPLE_BOOST_P  1
#define SIMPLE_BOOST_I  1
#define SIMPLE_BOOST_D  1

#if(defined(CORE_TEENSY))
#define BOOST_PIN_LOW()     (digitalWrite(pinBoost, LOW))
#define BOOST_PIN_HIGH()    (digitalWrite(pinBoost, HIGH))
#define VVT1_PIN_LOW()      (digitalWrite(pinVVT_1, LOW))
#define VVT1_PIN_HIGH()     (digitalWrite(pinVVT_1, HIGH))
#define VVT2_PIN_LOW()      (digitalWrite(pinVVT_2, LOW))
#define VVT2_PIN_HIGH()     (digitalWrite(pinVVT_2, HIGH))
#define FAN_PIN_LOW()       (digitalWrite(pinFan, LOW))
#define FAN_PIN_HIGH()      (digitalWrite(pinFan, HIGH))
#define N2O_STAGE1_PIN_LOW()    (digitalWrite(configPage10.n2o_stage1_pin, LOW))
#define N2O_STAGE1_PIN_HIGH()   (digitalWrite(configPage10.n2o_stage1_pin, HIGH))
#define N2O_STAGE2_PIN_LOW()    (digitalWrite(configPage10.n2o_stage2_pin, LOW))
#define N2O_STAGE2_PIN_HIGH()   (digitalWrite(configPage10.n2o_stage2_pin, HIGH))
#define DBW1_PIN_LOW()    (digitalWrite(configPage10.dbw1Pin, LOW))
#define DBW1_PIN_HIGH()   (digitalWrite(configPage10.dbw2Pin, HIGH))
#define DBW2_PIN_LOW()    (digitalWrite(configPage10.dbw1Pin, LOW))
#define DBW2_PIN_HIGH()   (digitalWrite(configPage10.dbw2Pin, HIGH))
#else
#define BOOST_PIN_LOW()  *boost_pin_port &= ~(boost_pin_mask)
#define BOOST_PIN_HIGH() *boost_pin_port |= (boost_pin_mask)
// #define VVT1_PIN_LOW()    *vvt1_pin_port &= ~(vvt1_pin_mask)
// #define VVT1_PIN_HIGH()   *vvt1_pin_port |= (vvt1_pin_mask)
// #define VVT2_PIN_LOW()    *vvt2_pin_port &= ~(vvt2_pin_mask)
// #define VVT2_PIN_HIGH()   *vvt2_pin_port |= (vvt2_pin_mask)
#define FAN_PIN_LOW()    *fan_pin_port &= ~(fan_pin_mask)
#define FAN_PIN_HIGH()   *fan_pin_port |= (fan_pin_mask)
#define N2O_STAGE1_PIN_LOW()  *n2o_stage1_pin_port &= ~(n2o_stage1_pin_mask)
#define N2O_STAGE1_PIN_HIGH() *n2o_stage1_pin_port |= (n2o_stage1_pin_mask)
#define N2O_STAGE2_PIN_LOW()  *n2o_stage2_pin_port &= ~(n2o_stage2_pin_mask)
#define N2O_STAGE2_PIN_HIGH() *n2o_stage2_pin_port |= (n2o_stage2_pin_mask)
// DBW
#define DBW1_PIN_LOW()  *dbw1_pin_port &= ~(dbw1_pin_mask)
#define DBW1_PIN_HIGH() *dbw1_pin_port |= (dbw1_pin_mask)
#define DBW2_PIN_LOW()  *dbw2_pin_port &= ~(dbw2_pin_mask)
#define DBW2_PIN_HIGH() *dbw2_pin_port |= (dbw2_pin_mask)
#endif

#define READ_N2O_ARM_PIN()    ((*n2o_arming_pin_port & n2o_arming_pin_mask) ? true : false)
// #define VVT1_PIN_ON()     VVT1_PIN_HIGH();
// #define VVT1_PIN_OFF()    VVT1_PIN_LOW();
// #define VVT2_PIN_ON()     VVT2_PIN_HIGH();
// #define VVT2_PIN_OFF()    VVT2_PIN_LOW();

#define VVT_TIME_DELAY_MULTIPLIER  50

#define FAN_ON()         ((configPage6.fanInv) ? FAN_PIN_LOW() : FAN_PIN_HIGH())
#define FAN_OFF()        ((configPage6.fanInv) ? FAN_PIN_HIGH() : FAN_PIN_LOW())

#define WMI_TANK_IS_EMPTY() ((configPage10.wmiEmptyEnabled) ? ((configPage10.wmiEmptyPolarity) ? digitalRead(pinWMIEmpty) : !digitalRead(pinWMIEmpty)) : 1)

volatile PORT_TYPE *boost_pin_port;
volatile PINMASK_TYPE boost_pin_mask;
volatile PORT_TYPE *vvt1_pin_port;
volatile PINMASK_TYPE vvt1_pin_mask;
volatile PORT_TYPE *vvt2_pin_port;
volatile PINMASK_TYPE vvt2_pin_mask;
volatile PORT_TYPE *fan_pin_port;
volatile PINMASK_TYPE fan_pin_mask;

volatile PORT_TYPE *dbw1_pin_port;
volatile PINMASK_TYPE dbw1_pin_mask;
volatile PORT_TYPE *dbw2_pin_port;
volatile PINMASK_TYPE dbw2_pin_mask;

volatile PORT_TYPE *n2o_stage1_pin_port;
volatile PINMASK_TYPE n2o_stage1_pin_mask;
volatile PORT_TYPE *n2o_stage2_pin_port;
volatile PINMASK_TYPE n2o_stage2_pin_mask;
volatile PORT_TYPE *n2o_arming_pin_port;
volatile PINMASK_TYPE n2o_arming_pin_mask;

volatile bool boost_pwm_state;
unsigned int boost_pwm_max_count; //Used for variable PWM frequency
volatile unsigned int boost_pwm_cur_value;
long boost_pwm_target_value;
long boost_cl_target_boost;

volatile bool dbw_pwm_state;
unsigned int dbw_pwm_max_count;
volatile unsigned int dbw_pwm_cur_value;
long dbw_pwm_target_value;
long dbw_cl_target_boost;

volatile bool dbw2_pwm_state;
unsigned int dbw2_pwm_max_count;
volatile unsigned int dbw2_pwm_cur_value;
long dbw2_pwm_target_value;
long dbw2_cl_target_boost;

byte boostCounter;
byte vvtCounter;
#if defined(PWM_FAN_AVAILABLE)//PWM fan not available on Arduino MEGA
volatile bool fan_pwm_state;
unsigned int fan_pwm_max_count; //Used for variable PWM frequency
volatile unsigned int fan_pwm_cur_value;
long fan_pwm_value;
void fanInterrupt();
#endif
uint32_t vvtWarmTime;
bool vvtIsHot;
bool vvtTimeHold;

volatile bool vvt1_pwm_state;
volatile bool vvt2_pwm_state;
volatile bool vvt1_max_pwm;
volatile bool vvt2_max_pwm;
volatile char nextVVT;
unsigned int vvt_pwm_max_count; //Used for variable PWM frequency
volatile unsigned int vvt1_pwm_cur_value;
volatile unsigned int vvt2_pwm_cur_value;
long vvt1_pwm_value;
long vvt2_pwm_value;
long vvt_pid_target_angle;
long vvt2_pid_target_angle;
long vvt_pid_current_angle;
long vvt2_pid_current_angle;

void boostInterrupt();
// void vvtInterrupt();


#endif
