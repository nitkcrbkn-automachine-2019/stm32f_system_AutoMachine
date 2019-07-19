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
#include <stdint.h>
#include "message.h"
#include "DD_LD.h"
#include "DD_Gene.h"

/*
 * 8バイト送信。
 * [tape1_color[7:0](7:0)]....[tape8_color[7:0](7:0)]
 */

int DD_send2LD(DD_LDHand_t *dld){
  const uint8_t sizeof_data = 8;
  /* Send data */
  return DD_I2CSend(dld->add, dld->mode, sizeof_data);
}

/*
 * *LD handlerを表示。
 *
 * LD(Add:hex):[Tape_Number(1-8)],[COLOR]
 */

void DD_LDHandPrint(DD_LDHand_t *dld){
  MW_printf("LD(%02x):", dld->add);
  for(int i=0;i<8;i++){
    if(i==0){
      MW_printf("[%d],[%16s]\n",i+1,blink_mode[(int)(dld->mode[i])]);
      continue;
    }
    MW_printf("       [%d],[%16s]\n",i+1,blink_mode[(int)(dld->mode[i])]);
  }
}

