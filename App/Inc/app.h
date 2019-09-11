#ifndef __APP_H
#define __APP_H

/*NO Device mode*/
#define _NO_DEVICE 0

int appTask(void);
int appInit(void);

#define DD_NUM_OF_MD 6
#define DD_NUM_OF_AB 0

#define DD_NUM_OF_LD 1
#define DD_NUM_OF_SS 2
#define DD_USE_ENCODER1 1
#define DD_USE_ENCODER2 1
#define DD_NUM_OF_SV 0

#define DD_USE_RC 1

#include "DD_RC.h"
#include "DD_LD.h"
#include "DD_MD.h"
#include "DD_SV.h"
#include "DD_SS.h"

#define _ENCODER1_RESET_GPIOID GPIOAID
#define _ENCODER1_RESET_GPIOPIN GPIO_PIN_0
#define _IS_PRESSED_ENCODER1_RESET() ((MW_GPIORead(_ENCODER1_RESET_GPIOID,_ENCODER1_RESET_GPIOPIN)))

#define _ENCODER2_RESET_GPIOID GPIOAID
#define _ENCODER2_RESET_GPIOPIN GPIO_PIN_1
#define _IS_PRESSED_ENCODER2_RESET() ((MW_GPIORead(_ENCODER2_RESET_GPIOID,_ENCODER2_RESET_GPIOPIN)))

#define I2C_ENCODER_1 1
#define I2C_ENCODER_2 0

#define RIGHT_ENC 0
#define BACK_ENC 1
#define LEFT_ENC 2
#define FRONT_ENC 3

#define MECHA1_MD1 2
#define MECHA1_MD2 3
#define CENTRAL_THRESHOLD 4

#define VOLTAGE 12.0
#define VOLTAGE_ADJUST (12.0/VOLTAGE)

#define R_F_KUDO_ADJUST 1.0 //右前ステアの駆動モータ調整倍率
#define L_B_KUDO_ADJUST 0.90 //左後ろステア駆動モータ調整倍率

#define R_F_DEG_ADJUST 180
#define L_B_DEG_ADJUST 175

#define ARM_UP_MAXDUTY (5000*VOLTAGE_ADJUST)
#define ARM_SPIN_MAXDUTY (3000*VOLTAGE_ADJUST) //gyaku

#define STRAIGHT_MAX_DUTY 8000

#define SUS_LOW_DUTY 1000.0

#define ARM_UP_MD 4
#define ARM_SPIN_MD 5 

#define AB0 (1<<0)
#define AB1 (1<<1)

#define MD_GAIN ( DD_MD_MAX_DUTY / DD_RC_ANALOG_MAX )

/****以下追加分*****************/

#define MOVE_SAMPLE_VALUE 20 //←のポジションデータで自己位置推定


typedef enum{
  GET_ENCODER_VALUE = 0,
  RESET_ENCODER_VALUE = 1,
  GET_DIFF = 2,
}EncoderOperation_t;

typedef enum{
  MANUAL_SUSPENSION = 0,
  AUTO_TEST = 1,
  NO_OPERATION = 2,
  STOP_EVERYTHING = 3,
}TestMode_t;

typedef enum{
  PLUS_X = 0,
  MINUS_X = 1,
  PLUS_Y = 2,
  MINUS_Y = 3,
}MovingDestination_t;

typedef enum{
  PLUS_ACCELERATING = 0,
  CONSTANT_SPEED = 1,
  MINUS_ACCELERATING = 2,
  ARRIVED_TARGET = 3,
}MovingSituation_t;

typedef enum{
  NOW_POSITION_RIGHT = 0,
  NOW_POSITION_LEFT = 1,
  NOW_POSITION_CENTER = 2,
  NOW_POSITION_STOP = 3,
}NowPosition_t;

#endif


