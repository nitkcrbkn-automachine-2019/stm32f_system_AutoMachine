## MD
### プロコトル
| bit   |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
|:-----:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
|data[7]|T8_D7|T8_D6|T8_D5|T8_D4|T8_D3|T8_D2|T8_D1|T8_D0|
|data[6]|T7_D7|T7_D6|T7_D5|T7_D4|T7_D3|T7_D2|T7_D1|T7_D0|
|data[5]|T6_D7|T6_D6|T6_D5|T6_D4|T6_D3|T6_D2|T6_D1|T6_D0|
|data[4]|T5_D7|T5_D6|T5_D5|T5_D4|T5_D3|T5_D2|T5_D1|T5_D0|
|data[3]|T4_D7|T4_D6|T4_D5|T4_D4|T4_D3|T4_D2|T4_D1|T4_D0|
|data[2]|T3_D7|T3_D6|T3_D5|T3_D4|T3_D3|T3_D2|T3_D1|T3_D0|
|data[1]|T2_D7|T2_D6|T2_D5|T2_D4|T2_D3|T2_D2|T2_D1|T2_D0|
|data[0]|T1_D7|T1_D6|T1_D5|T1_D4|T1_D3|T1_D2|T1_D1|T1_D0|

T1_D0 == TapeLED 1 本目の 0 bit目のデータ 

表示形式は
```
LD(Add:hex):[Tape_num(1-8)],[Color:char]
```
です。

## 用意した型
enumはテープLEDの発色方式を示しています。
構造体は上から順にアドレス、先ほどのモード(8本分)となります。
```c
typedef enum{
  D_LMOD_NONE = 0,
  D_LMOD_RAINBOW = 1,
  D_LMOD_RED = 2,
      .
      .
      .
  D_LMOD_DIMING_BLUE = 19,
  D_LMOD_DIMING_YELLOW = 20,
  D_LMOD_DIMING_PURPLE = 21,
}DD_LDMode_t;

typedef struct{
  uint8_t add;
  DD_LDMode_t mode[8];
}DD_LDHand_t;
```

## 用意した関数
- int DD_send2LD(DD_LDHand_t *dld);

LDにデータを送信します。
- void DD_LDHandPrint(DD_LDHand_t *dld);

LDのハンドラを表示します。　
