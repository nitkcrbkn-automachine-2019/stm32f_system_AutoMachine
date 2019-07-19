/* ===Kisarazu RBKN Library===
 *
 * autor          : Izutsu
 * version        : v0.01
 * last update    : 20190503
 *
 * **overview***
 * LDの通信プロトコルを定める。
 *
 * ・I2Cのみのサポート
 */

#ifndef __LD_H
#define __LD_H
#include <stdint.h>

typedef enum{
  D_LMOD_NONE = 0,
  D_LMOD_RAINBOW = 1,
  D_LMOD_RED = 2,
  D_LMOD_GREEN = 3,
  D_LMOD_BLUE = 4,
  D_LMOD_YELLOW = 5,
  D_LMOD_PURPLE = 6,
  D_LMOD_BLINK_RED = 7,
  D_LMOD_BLINK_GREEN = 8,
  D_LMOD_BLINK_BLUE = 9,
  D_LMOD_BLINK_YELLOW= 10,
  D_LMOD_BLINK_PURPLE = 11,
  D_LMOD_INCREMENT = 12,
  D_LMOD_BINARY_RED = 13,
  D_LMOD_BINARY_GREEN = 14,
  D_LMOD_BINARY_BLUE = 15,
  D_LMOD_DIMING_RAINBOW = 16,
  D_LMOD_DIMING_RED = 17,
  D_LMOD_DIMING_GREEN = 18,
  D_LMOD_DIMING_BLUE = 19,
  D_LMOD_DIMING_YELLOW = 20,
  D_LMOD_DIMING_PURPLE = 21,
}DD_LDMode_t;

typedef struct{
  uint8_t add;
  DD_LDMode_t mode[8];
}DD_LDHand_t;

/*
 * 8バイト送信。
 * [tape1_color[7:0](7:0)]....[tape8_color[7:0](7:0)]
 */
int DD_send2LD(DD_LDHand_t *dld);


/*
 * *LD handlerを表示。
 *
 * LD(Add:hex):[Tape_Number(1-8)],[COLOR]
 */
void DD_LDHandPrint(DD_LDHand_t *dld);

static char *blink_mode[] = {
  "NONE",
  "RAINBOW",
  "RED",
  "GREEN",
  "BLUE",
  "YELLOW",
  "PURPLE",
  "BLINK_RED",
  "BLINK_GREEN",
  "BLINK_BLUE",
  "BLINK_YELLOW",
  "BLINK_PURPLE",
  "INCREMENT",
  "BINARY_RED",
  "BINARY_GREEN",
  "BINARY_BLUE",
  "DIMING_RAINBOW",
  "DIMING_RED",
  "DIMING_GREEN",
  "DIMING_BLUE",
  "DIMING_YELLOW",
  "DIMING_PURPLE",
};

#endif
