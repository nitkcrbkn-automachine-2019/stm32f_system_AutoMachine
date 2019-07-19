#ifndef __APP_H
#define __APP_H

/*NO Device mode*/
#define _NO_DEVICE 0

int appTask(void);
int appInit(void);

#define DD_NUM_OF_MD 4
#define DD_NUM_OF_AB 0

#define DD_NUM_OF_LD 1
#define DD_NUM_OF_SS 0
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

#define MECHA1_MD1 2
#define MECHA1_MD2 3
#define CENTRAL_THRESHOLD 4

#define AB0 (1<<0)
#define AB1 (1<<1)

#define MD_GAIN ( DD_MD_MAX_DUTY / DD_RC_ANALOG_MAX )

/****以下追加分*****************/

typedef enum{
  GET_ENCODER_VALUE = 0,
  RESET_ENCODER_VALUE = 1,
}EncoderOperation_t;

#endif


