#include "app.h"
#include "DD_Gene.h"
#include "DD_RCDefinition.h"
#include "SystemTaskManager.h"
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "MW_GPIO.h"
#include "MW_IWDG.h"
#include "message.h"
#include "MW_flash.h"
#include "constManager.h"
#include "trapezoid_ctrl.h"

/********/

static
int32_t I2C_Encoder(int32_t encoder_num, EncoderOperation_t operation);


/********/

static
int steering_spin_to_target(int target,int target_motor);
static
int steering_1_init(void);
static
int steering_2_init(void);
static
int get_diff(int target,int now_degree,int encoder_ppr);
static
int odmetry_position(double position[3]);
static
void fill_array(double array1[], double array2[], int size);
/*suspensionSystem*/
static
int suspensionSystem(void);
/*ABSystem*/
static 
int ABSystem(void);
static
int LEDSystem(void);
/*メモ
 *g_ab_h...ABのハンドラ
 *g_md_h...MDのハンドラ
 *
 *g_rc_data...RCのデータ
 */

int appInit(void){

  ad_init();

  /*GPIO の設定などでMW,GPIOではHALを叩く*/
  return EXIT_SUCCESS;
}

/*application tasks*/
int appTask(void){
  int ret=0;
  int target;

  static bool encoder1_reset = false;
  static bool encoder2_reset = false;
  static double position[3];
  
  if(__RC_ISPRESSED_R1(g_rc_data)&&__RC_ISPRESSED_R2(g_rc_data)&&
     __RC_ISPRESSED_L1(g_rc_data)&&__RC_ISPRESSED_L2(g_rc_data)){
    while(__RC_ISPRESSED_R1(g_rc_data)||__RC_ISPRESSED_R2(g_rc_data)||
	  __RC_ISPRESSED_L1(g_rc_data)||__RC_ISPRESSED_L2(g_rc_data))
        SY_wait(10);
    ad_main();
  }

  if(DD_RCGetRX(g_rc_data)<0){
    target = (int)(-(atan((double)DD_RCGetRY(g_rc_data)/(double)DD_RCGetRX(g_rc_data))*(180.0/3.141592)));
    if(target < 0){
      target = -target + 270;
    }else{
      target = 90-target + 180;
    }
    target = 360 - target;
  }else{
    target = (int)((atan((double)DD_RCGetRY(g_rc_data)/(double)DD_RCGetRX(g_rc_data))*(180.0/3.141592)));
    target += 90;
    target = 360 - target;
  }
  /* if(!encoder1_reset){ */
  /*   if(steering_1_init()==0){ */
  /*     encoder1_reset = true; */
  /*   } */
  /* } */
  if(!encoder2_reset){
    if(steering_2_init()==0){
      encoder2_reset = true;
    }
  }
  if(!encoder1_reset){
    if(steering_1_init()==0){
      encoder1_reset = true;
    }
  }
  if(encoder1_reset && encoder2_reset){
    ret = steering_spin_to_target(target,0);
  }
  if(ret){
    return ret;
  }


  ret = odmetry_position(position);
  if(ret){
    return ret;
  }
  /*それぞれの機構ごとに処理をする*/
  /*途中必ず定数回で終了すること。*/

  if(__RC_ISPRESSED_R1(g_rc_data)&&__RC_ISPRESSED_L1(g_rc_data)){
    I2C_Encoder(RIGHT_ENC, RESET_ENCODER_VALUE);
    I2C_Encoder(BACK_ENC, RESET_ENCODER_VALUE);
    I2C_Encoder(LEFT_ENC, RESET_ENCODER_VALUE);
    I2C_Encoder(FRONT_ENC, RESET_ENCODER_VALUE);
  }
  
  ret = suspensionSystem();
  if(ret){
    return ret;
  }

  ret = ABSystem();
  if(ret){
    return ret;
  }

  ret = LEDSystem();
  if(ret){
    return ret;
  }

  if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
    MW_printf("now_position[x][y][w] : [%10d][%10d][%10d]\n",(int)position[0],(int)position[1],(int)position[2]);

    MW_printf("Encoder_target[%3d]\n",target);
    if(encoder1_reset){
      MW_printf("encoder1_reset[true ]\n");
    }else{
      MW_printf("encoder1_reset[false]\n");
    }
    if(encoder2_reset){
      MW_printf("encoder2_reset[true ]\n");
    }else{
      MW_printf("encoder2_reset[false]\n");
    }
  }
  
  return EXIT_SUCCESS;
}

static
int steering_spin_to_target(int target,int target_motor){
  const int32_t encoder_ppr = (512)*4;
  const int div = (2048*4)/encoder_ppr;
  /* const int target_deg[9] = {2048,1024,512,256,128,64,32,26,21}; */
  /* const int target_duty[9]   = {8000,3000,1000,120,110,100,90,85,80}; */
  const int target_deg[2][9]  = {{2048/div,1024/div,512/div,256/div,128/div,64/div,32/div,26/div,21/div},
				 {2048/div,1024/div,512/div,256/div,128/div,64/div,32/div,26/div,21/div}};
  const int target_duty[2][9] = {{9000,5000,3000,1400,1300,1250,1200,1150,1000},
                                 {9000,5000,3000,1400,1300,1250,1200,1150,1000}};
  int32_t encoder;
  int32_t encoder_degree;
  int32_t target_degree;
  int spin_direction = 1;
  int diff_from_target;
  int duty;
  int i,j;
  int choose_motor[2] = {};

  if(target_motor == 0){
    choose_motor[0] = 0;
    choose_motor[1] = 2;
  }else if(target_motor == 1){
    choose_motor[0] = 0;
    choose_motor[1] = 1;
  }else if(target_motor == 2){
    choose_motor[0] = 1;
    choose_motor[1] = 2;
  }
  
  for(j=choose_motor[0];j<choose_motor[1];j++){
    switch(j){
    case 0:
      encoder = DD_encoder1Get_int32();
      break;
    case 1:
      encoder = DD_encoder2Get_int32();
      break;
    }
    encoder_degree = encoder % encoder_ppr;
    target_degree  = (int)(((double)encoder_ppr/360.0)*(double)target);
  
  
    if(encoder_degree < 0){
      encoder_degree = encoder_ppr - abs(encoder_degree);
    }
    diff_from_target = get_diff(target_degree,encoder_degree,encoder_ppr);
    for(i=0;i<9;i++){
      if(abs(diff_from_target)>=target_deg[j][i]){
	duty = target_duty[j][i];
	break;
      }
      if(i==8){
	duty = 0;
      }
    }
    if(diff_from_target > 0){
      g_md_h[j].duty = duty;
      if(duty==0){
	g_md_h[j].mode = D_MMOD_BRAKE;
      }else{
	g_md_h[j].mode = D_MMOD_FORWARD;
      }
    }else{
      g_md_h[j].duty = duty;
      if(duty==0){
	g_md_h[j].mode = D_MMOD_BRAKE;
      }else{
	g_md_h[j].mode = D_MMOD_BACKWARD;
      }
    }
    if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
      switch(j){
      case 0:
	MW_printf("E1_degree[%3d]\n",(int)((double)encoder_degree*(double)(360.0/(double)encoder_ppr)));
	break;
      case 1:
	MW_printf("E2_degree[%3d]\n",(int)((double)encoder_degree*(double)(360.0/(double)encoder_ppr)));
	break;
      }
    }
  }
}

static
int steering_1_init(void){
  if(_IS_PRESSED_ENCODER1_RESET()){
    g_md_h[0].duty = 0; 
    g_md_h[0].mode = D_MMOD_BRAKE;
    DD_encoder1reset();
    return 0;
  }else{
    g_md_h[0].duty = 900; 
    g_md_h[0].mode = D_MMOD_FORWARD;
  }
  return -1;
}

static
int steering_2_init(void){
  if(_IS_PRESSED_ENCODER2_RESET()){
    g_md_h[1].duty = 0; 
    g_md_h[1].mode = D_MMOD_BRAKE;
    DD_encoder2reset();
    return 0;
  }else{
    g_md_h[1].duty = 850; 
    g_md_h[1].mode = D_MMOD_FORWARD;
  }
  return -1;
}

static
int get_diff(int target,int now_degree,int encoder_ppr){
  int adjust = now_degree - encoder_ppr/2;
  int target_adjust = target - adjust;
  if(target_adjust>=encoder_ppr) target_adjust = target_adjust % encoder_ppr;
  if(target_adjust<0) target_adjust = encoder_ppr - abs(target_adjust);
  return target_adjust - encoder_ppr/2;
}

static int LEDSystem(void){
  static int color_num = 0;
  static bool c_up_flag = true;
  if(!__RC_ISPRESSED_UP(g_rc_data)){
    c_up_flag = true;
  }
  if(c_up_flag && __RC_ISPRESSED_UP(g_rc_data)){
    c_up_flag = false;
    g_ld_h[0].mode[4] = color_num;
    g_ld_h[0].mode[5] = color_num;
    g_ld_h[0].mode[6] = color_num;
    color_num++;
    if(color_num>=22){
      color_num = 0;
    }
    //g_led_mode = lmode_1;
  }
  if(__RC_ISPRESSED_DOWN(g_rc_data)){
    g_ld_h[0].mode[4] = D_LMOD_NONE;
    g_ld_h[0].mode[5] = D_LMOD_NONE;
    g_ld_h[0].mode[6] = D_LMOD_NONE;
    //g_led_mode = lmode_2;
  }
  
  return EXIT_SUCCESS;
}

static 
int ABSystem(void){

  /* g_ab_h[0].dat = 0x00; */
  /* if(__RC_ISPRESSED_CIRCLE(g_rc_data)){ */
  /*   g_ab_h[0].dat |= AB0; */
  /* } */
  /* if(__RC_ISPRESSED_CROSS(g_rc_data)){ */
  /*   g_ab_h[0].dat |= AB1; */
  /* } */

  return EXIT_SUCCESS;
}


/*プライベート 足回りシステム*/
static
int suspensionSystem(void){
  const int num_of_motor = 2;/*モータの個数*/
  int rc_analogdata;/*アナログデータ*/
  unsigned int idx;/*インデックス*/
  int i;

  /*for each motor*/
  for(i=0;i<num_of_motor;i++){
    /*それぞれの差分*/
    rc_analogdata = DD_RCGetLY(g_rc_data);
    switch(i){
    case 0:
      idx = MECHA1_MD1;
      break;
    case 1:
      idx = MECHA1_MD2;
      break;
    default:return EXIT_FAILURE;
    }
    
    /*これは中央か?*/
    if(abs(rc_analogdata)==0){
      g_md_h[idx].mode = D_MMOD_FREE;
      g_md_h[idx].duty = 0;
    }
    else{
      if(rc_analogdata > 0){
	/*前後の向き判定*/
	g_md_h[idx].mode = D_MMOD_FORWARD;
      }
      else{
	g_md_h[idx].mode = D_MMOD_BACKWARD;
      }
      /*絶対値を取りDutyに格納*/
      g_md_h[idx].duty = abs(rc_analogdata) * MD_GAIN;
    }
  }
  /* if(__RC_ISPRESSED_RIGHT(g_rc_data)){ */
  /*   g_md_h[0].mode = D_MMOD_FORWARD; */
  /*   //g_md_h[1].mode = D_MMOD_FORWARD; */
  /*   g_md_h[0].duty = 900; */
  /*   //g_md_h[1].duty = 80; */
  /* } */
  /* if(__RC_ISPRESSED_LEFT(g_rc_data)){ */
  /*   g_md_h[0].mode = D_MMOD_BACKWARD; */
  /*   //g_md_h[1].mode = D_MMOD_FORWARD; */
  /*   g_md_h[0].duty = 900; */
  /*   //g_md_h[1].duty = 80; */
  /* } */
  return EXIT_SUCCESS;
}

static
int odmetry_position(double position[3]){
  static bool matrix_init = false;
  const int encoder_ppr = (2048)*4;
  const int tire_diameter = 48;
  static double cal_matrix[3][4];
  static double recent_position[3] = {0.0, 0.0, 0.0};
  double temp_position[3],return_position[3],return_position_val[3],encoder_diff[4];
  int i,j;

  if(!matrix_init){
    cal_matrix[0][0] = -0.055815556002182089637945390335639840362915960837233;
    cal_matrix[0][1] = -0.5453070716931292888110482643773866605415027706796;
    cal_matrix[0][2] =  0.055815556002182089637945390335639840362915960837233;
    cal_matrix[0][3] =  0.4546929283068707118895173562261333945849722932039;

    cal_matrix[1][0] = -0.5000000000000000000000000000000000000000000000000;
    cal_matrix[1][1] =  0.0000000000000000000000000000000000000000000000000;
    cal_matrix[1][2] = -0.5000000000000000000000000000000000000000000000000;
    cal_matrix[1][3] =  0.0000000000000000000000000000000000000000000000000;

    cal_matrix[2][0] =  0.3014040024117832840449051078124551379597461885211;
    cal_matrix[2][1] =  0.2446581871428981595796606276378879669241149616699;
    cal_matrix[2][2] = -0.3014040024117832840449051078124551379597461885211;
    cal_matrix[2][3] =  0.2446581871428981595796606276378879669241149616699;

    matrix_init = true;
  }
  
  for(i=0;i<4;i++){
    encoder_diff[i] = (double)(I2C_Encoder(i,GET_DIFF));
  }

  for(i=0;i<3;i++){
    temp_position[i] = 0.0;
    for(j=0;j<4;j++){
      temp_position[i] += cal_matrix[i][j] * encoder_diff[j];
    }
    return_position[i] = recent_position[i] + temp_position[i];
  }
  
  fill_array(recent_position, return_position, 3);

  for(i=0;i<3;i++){
    return_position_val[i] = return_position[i] * (((double)(tire_diameter)*M_PI)/(double)(encoder_ppr));
  }
  fill_array(position, return_position_val, 3);

  return 0;
}

static
int32_t I2C_Encoder(int32_t encoder_num, EncoderOperation_t operation){
  int32_t value=0,temp_value=0,diff=0;
  static int32_t adjust[4] = {0,0,0,0};
  static int32_t recent_value[4] = {0,0,0,0};
  static int message_count = 0;
  
  switch(encoder_num){
  case 0:
    temp_value = g_ss_h[I2C_ENCODER_1].data[0] + (g_ss_h[I2C_ENCODER_1].data[1] << 8) + (g_ss_h[I2C_ENCODER_1].data[2] << 16) + (g_ss_h[I2C_ENCODER_1].data[3] << 24);
    value = temp_value + adjust[0];
    if(operation == GET_ENCODER_VALUE){
      break;
    }else if(operation == RESET_ENCODER_VALUE){
      adjust[0] = -temp_value;
      value = temp_value + adjust[0];
    }else if(operation == GET_DIFF){
      diff = value - recent_value[0];
    }
    break;
    
  case 1:
    temp_value = g_ss_h[I2C_ENCODER_1].data[4] + (g_ss_h[I2C_ENCODER_1].data[5] << 8) + (g_ss_h[I2C_ENCODER_1].data[6] << 16) + (g_ss_h[I2C_ENCODER_1].data[7] << 24);
    value = temp_value + adjust[1];
    if(operation == GET_ENCODER_VALUE){
      break;
    }else if(operation == RESET_ENCODER_VALUE){
      adjust[1] = -temp_value;
      value = temp_value + adjust[1];
    }else if(operation == GET_DIFF){
      diff = value - recent_value[1];
    }
    break;
    
  case 2:
    temp_value = g_ss_h[I2C_ENCODER_2].data[0] + (g_ss_h[I2C_ENCODER_2].data[1] << 8) + (g_ss_h[I2C_ENCODER_2].data[2] << 16) + (g_ss_h[I2C_ENCODER_2].data[3] << 24);
    value = temp_value + adjust[2];
    if(operation == GET_ENCODER_VALUE){
      break;
    }else if(operation == RESET_ENCODER_VALUE){
      adjust[2] = -temp_value;
      value = temp_value + adjust[2];
    }else if(operation == GET_DIFF){
      diff = value - recent_value[2];
    }
    break;
    
  case 3:
    temp_value = g_ss_h[I2C_ENCODER_2].data[4] + (g_ss_h[I2C_ENCODER_2].data[5] << 8) + (g_ss_h[I2C_ENCODER_2].data[6] << 16) + (g_ss_h[I2C_ENCODER_2].data[7] << 24);
    value = temp_value + adjust[3];
    if(operation == GET_ENCODER_VALUE){
      break;
    }else if(operation == RESET_ENCODER_VALUE){
      adjust[3] = -temp_value;
      value = temp_value + adjust[3];
    }else if(operation == GET_DIFF){
      diff = value - recent_value[3];
    }
    break;
  }

  recent_value[encoder_num] = value;
  if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
    if(operation != RESET_ENCODER_VALUE){
      if(message_count >= 3){
	MW_printf("<E0>[%10d] <E1>[%10d] <E2>[%10d] <E3>[%10d]\n",recent_value[0],recent_value[1],recent_value[2],recent_value[3]);
	message_count = 0;
      }else{
	message_count++;
      }
    }
  }
  if(operation == GET_DIFF){
    return diff;
  }
  return value;
}

static
void fill_array(double array1[], double array2[], int size){
  for(int i=0;i<size;i++){
    array1[i] = array2[i];
  }
}
