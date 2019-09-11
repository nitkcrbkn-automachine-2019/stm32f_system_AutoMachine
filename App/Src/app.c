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
int go_to_target(double zahyou_1[2], double zahyou_2[2], double max_duty, bool accelerating);
static
MovingSituation_t decide_straight_duty(double *return_duty, double zahyou_1[2], double zahyou_2[2], double position[MOVE_SAMPLE_VALUE][3], double max_duty, bool acceleration);
static
int decide_turn_duty(double *right_duty_adjust, double *left_duty_adjust,  double straight_duty, double zahyou_1[2], double zahyou_2[2], double position[MOVE_SAMPLE_VALUE][3], double max_duty, bool acceleration);
static
int steering_spin_to_target(int target,int target_motor);
static
int steering_1_init(void);
static
int steering_2_init(void);
static
int get_diff(int target,int now_degree,int encoder_ppr);
static
int odmetry_position(double position[3], int recet);
static
int odmetry_position_recent(double position[MOVE_SAMPLE_VALUE][3], int recet);
static
NowPosition_t get_deg_dis(double get_x[MOVE_SAMPLE_VALUE], double get_y[MOVE_SAMPLE_VALUE], double zahyou_1[2], double zahyou_2[2], double *return_degree, double *return_distance);
static
void fill_array(double array1[], double array2[], int size);

static char *testmode_name[] = {
  "MANUAL_SUSPENSION",
  "AUTO_TEST",
  "NO_OPERATION",
  "STOP_EVERYTHING",
};

/********/
/*suspensionSystem*/
static
int all_motor_stop(void);
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

  static bool test_flag = false;
  
  int mun_sus_target;
  static int mun_sus_recent_target = 0;
  static bool encoder1_reset = false;
  static bool encoder2_reset = false;

  static double position[3];

  static TestMode_t now_mode = STOP_EVERYTHING;
  
  static bool circle_flag,cross_flag,sqare_flag,triangle_flag;
  static bool up_flag,down_flag,right_flag,left_flag;
  static bool r1_flag,r2_flag,l1_flag,l2_flag;
  
  if(!__RC_ISPRESSED_CIRCLE(g_rc_data)) circle_flag = true;
  if(!__RC_ISPRESSED_CROSS(g_rc_data)) cross_flag = true;
  if(!__RC_ISPRESSED_SQARE(g_rc_data)) sqare_flag = true;
  if(!__RC_ISPRESSED_TRIANGLE(g_rc_data)) triangle_flag = true;
  if(!__RC_ISPRESSED_UP(g_rc_data)) up_flag = true;
  if(!__RC_ISPRESSED_DOWN(g_rc_data)) down_flag = true;
  if(!__RC_ISPRESSED_RIGHT(g_rc_data)) right_flag = true;
  if(!__RC_ISPRESSED_LEFT(g_rc_data)) left_flag = true;
  if(!__RC_ISPRESSED_R1(g_rc_data)) r1_flag = true;
  if(!__RC_ISPRESSED_R2(g_rc_data)) r2_flag = true;
  if(!__RC_ISPRESSED_L1(g_rc_data)) l1_flag = true;
  if(!__RC_ISPRESSED_L2(g_rc_data)) l2_flag = true;

  if(__RC_ISPRESSED_RIGHT(g_rc_data) && right_flag){
    if(now_mode == STOP_EVERYTHING){
      now_mode = MANUAL_SUSPENSION;
    }else{
      now_mode++;
    }    
    right_flag = false;
  }
  if(__RC_ISPRESSED_LEFT(g_rc_data) && left_flag){
    if(now_mode == MANUAL_SUSPENSION){
      now_mode = STOP_EVERYTHING;
    }else{
      now_mode--;
    }    
    left_flag = false;
  }
  
  /* if(__RC_ISPRESSED_R1(g_rc_data)&&__RC_ISPRESSED_R2(g_rc_data)&& */
  /*    __RC_ISPRESSED_L1(g_rc_data)&&__RC_ISPRESSED_L2(g_rc_data)){ */
  /*   while(__RC_ISPRESSED_R1(g_rc_data)||__RC_ISPRESSED_R2(g_rc_data)|| */
  /* 	  __RC_ISPRESSED_L1(g_rc_data)||__RC_ISPRESSED_L2(g_rc_data)) */
  /*       SY_wait(10); */
  /*   ad_main(); */
  /* } */

  switch(now_mode){
  case NO_OPERATION:////////////////////////////////////////////
    all_motor_stop();
    break;
  case STOP_EVERYTHING:
    all_motor_stop();
    break;//////////////////////////////////////////////////////
  case AUTO_TEST:///////////////////////////////////////////////
    if(__RC_ISPRESSED_CIRCLE(g_rc_data) && circle_flag){
      test_flag = true;
    }
    if(__RC_ISPRESSED_CROSS(g_rc_data)){
      test_flag = false;
    }
    if(!test_flag){
      all_motor_stop();
    }else{
      
    }
    break;//////////////////////////////////////////////////////
  case MANUAL_SUSPENSION:

    if(sqrt(abs(DD_RCGetRX(g_rc_data))*abs(DD_RCGetRX(g_rc_data))+abs(DD_RCGetRY(g_rc_data))*abs(DD_RCGetRY(g_rc_data))) < sqrt(45*45)){
      mun_sus_target = mun_sus_recent_target;
    }else{
      if(DD_RCGetRX(g_rc_data)<0){
	mun_sus_target = (int)(-(atan((double)DD_RCGetRY(g_rc_data)/(double)DD_RCGetRX(g_rc_data))*(180.0/3.141592)));
	if(mun_sus_target < 0){
	  mun_sus_target = -mun_sus_target + 270;
	}else{
	  mun_sus_target = 90-mun_sus_target + 180;
	}
	mun_sus_target = 360 - mun_sus_target;
      }else{
	mun_sus_target = (int)((atan((double)DD_RCGetRY(g_rc_data)/(double)DD_RCGetRX(g_rc_data))*(180.0/3.141592)));
	mun_sus_target += 90;
	mun_sus_target = 360 - mun_sus_target;
      }
    }
    
    mun_sus_recent_target = mun_sus_target;
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
      ret = steering_spin_to_target(mun_sus_target+R_F_DEG_ADJUST,1);
      ret = steering_spin_to_target(mun_sus_target+L_B_DEG_ADJUST,2);
    }

    ret = suspensionSystem();
    if(ret){
      return ret;
    }

    if(__RC_ISPRESSED_R1(g_rc_data)){//SPIN_M
      g_md_h[ARM_SPIN_MD].mode = D_MMOD_FORWARD;
      g_md_h[ARM_SPIN_MD].duty = ARM_SPIN_MAXDUTY;
    }else if(__RC_ISPRESSED_R2(g_rc_data)){
      g_md_h[ARM_SPIN_MD].mode = D_MMOD_BACKWARD;
      g_md_h[ARM_SPIN_MD].duty = ARM_SPIN_MAXDUTY;
    }else{
      g_md_h[ARM_SPIN_MD].mode = D_MMOD_BRAKE;
      g_md_h[ARM_SPIN_MD].duty = 0;
    }

    
    if(__RC_ISPRESSED_L1(g_rc_data)){//SPIN_M
      g_md_h[ARM_UP_MD].mode = D_MMOD_FORWARD;
      g_md_h[ARM_UP_MD].duty = ARM_UP_MAXDUTY;
    }else if(__RC_ISPRESSED_L2(g_rc_data)){
      g_md_h[ARM_UP_MD].mode = D_MMOD_BACKWARD;
      g_md_h[ARM_UP_MD].duty = ARM_UP_MAXDUTY;
    }else{
      g_md_h[ARM_UP_MD].mode = D_MMOD_BRAKE;
      g_md_h[ARM_UP_MD].duty = 0;
    }
    
    
    break; /////////////////////////////////////////////////
    
  }

  ret = odmetry_position(position,0);
  if(ret){
    return ret;
  }

  
  if(__RC_ISPRESSED_R1(g_rc_data)&&__RC_ISPRESSED_L1(g_rc_data)){
    I2C_Encoder(RIGHT_ENC, RESET_ENCODER_VALUE);
    I2C_Encoder(BACK_ENC, RESET_ENCODER_VALUE);
    I2C_Encoder(LEFT_ENC, RESET_ENCODER_VALUE);
    I2C_Encoder(FRONT_ENC, RESET_ENCODER_VALUE);
    ret = odmetry_position(position,1);
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
    MW_printf("mode : [%30s]\n",testmode_name[now_mode]);
    MW_printf("now_position[x][y][w] : [%10d][%10d][%10d]\n",(int)position[0],(int)position[1],(int)(position[2]*100.0));

    MW_printf("Encoder_target[%3d]\n",mun_sus_target);
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
MovingSituation_t decide_straight_duty(double *return_duty, double zahyou_1[2], double zahyou_2[2], double position[MOVE_SAMPLE_VALUE][3], double max_duty, bool acceleration){
  MovingDestination_t mode;
  MovingSituation_t situation;
  double distance_to_target;
  double distance_from_first;
  static double recent_zahyou_1[2]={0.0, 0.0},recent_zahyou_2[2]={0.0, 0.0};
  static double first_distance;
  static bool first_flag = true;

  if((recent_zahyou_1[0]!=zahyou_1[0]) || (recent_zahyou_1[1]!=zahyou_1[1]) || (recent_zahyou_2[0]!=zahyou_2[0]) || (recent_zahyou_2[1]!=zahyou_2[1])){
    first_flag = true;
    if(zahyou_2[0]-zahyou_1[0]==0.0){
      if(zahyou_2[1]-zahyou_1[1] >= 0.0){
	mode = PLUS_Y;
      }else{
	mode = MINUS_Y;
      }
    }else if(zahyou_2[1]-zahyou_1[1]==0.0){
      if(zahyou_2[0]-zahyou_1[0] >= 0.0){
	mode = PLUS_X;
      }else{
	mode = MINUS_X;
      }
    }
    recent_zahyou_1[0] = zahyou_1[0];
    recent_zahyou_1[1] = zahyou_1[1];
    recent_zahyou_2[0] = zahyou_2[0];
    recent_zahyou_2[1] = zahyou_2[1];
  }

  switch(mode){
  case PLUS_X:
    distance_to_target = zahyou_2[0] - position[0][0];
    break;
  case MINUS_X:
    distance_to_target = position[0][0] - zahyou_2[0];
    break;
  case PLUS_Y:
    distance_to_target = zahyou_2[1] - position[0][1];
    break;
  case MINUS_Y:
    distance_to_target = position[0][1] - zahyou_2[1];
    break;
  }

  if(first_flag){
    first_distance = distance_to_target;
    first_flag = false;
  }

  if(acceleration){
    distance_from_first = fabs(first_distance-distance_to_target);
    if(distance_to_target > 1000.0){
      if(distance_from_first <= 1000.0){
	*return_duty = max_duty * (distance_from_first/1000.0)*(distance_from_first/1000.0)*(distance_from_first/1000.0);
	if(*return_duty <= SUS_LOW_DUTY) *return_duty = SUS_LOW_DUTY;
	situation = PLUS_ACCELERATING;
      }else{
	*return_duty = max_duty;
	situation = CONSTANT_SPEED;
      }
    }else{
      *return_duty = max_duty * (distance_to_target/1000.0 - 1.0)*(distance_to_target/1000.0 - 1.0)*(distance_to_target/1000.0 - 1.0) + max_duty;
      if(*return_duty <= SUS_LOW_DUTY) *return_duty = SUS_LOW_DUTY;
      situation = MINUS_ACCELERATING;
    }
    if(distance_to_target <= 5.0){ //5mmくらい進むkana
      *return_duty = 0.0;
      situation = ARRIVED_TARGET;
    }
  }else{
    if(distance_to_target <= 5.0){
      *return_duty = 0.0;
      situation = ARRIVED_TARGET;
    }else{
      *return_duty = max_duty;
      situation = CONSTANT_SPEED;
    }
  }
  
  return situation;
}

static
int decide_turn_duty(double *right_duty_adjust, double *left_duty_adjust, double straight_duty, double zahyou_1[2], double zahyou_2[2], double position[MOVE_SAMPLE_VALUE][3], double max_duty, bool acceleration){
  MovingSituation_t situation;
  NowPosition_t now_position;
  double degree,distance;
  double get_x[MOVE_SAMPLE_VALUE],get_y[MOVE_SAMPLE_VALUE];
  int i;
  
  for(i=0; i<MOVE_SAMPLE_VALUE; i++){
    get_x[i] = position[i][0];
    get_x[i] = position[i][1];
  }
  now_position = get_deg_dis(get_x, get_y, zahyou_1, zahyou_2, &degree, &distance);
  
  return 0;
}

static
int steering_spin_to_target(int target,int target_motor){
  const int32_t encoder_ppr = (512)*4;
  const int div = (2048*4)/encoder_ppr;
  /* const int target_deg[9] = {2048,1024,512,256,128,64,32,26,21}; */
  /* const int target_duty[9]   = {8000,3000,1000,120,110,100,90,85,80}; */
  const int target_deg[2][9]  = {{2048/div,1024/div,512/div,256/div,128/div,64/div,32/div,26/div,21/div},
				 {2048/div,1024/div,512/div,256/div,128/div,64/div,32/div,26/div,21/div}};
  int target_duty[2][9] = {{9000,5000,3000,1400,1300,1250,1200,1150,1000},
			   {9000,5000,3000,1400,1300,1250,1200,1150,1000}};
  int32_t encoder;
  int32_t encoder_degree;
  int32_t target_degree;
  int spin_direction = 1;
  int diff_from_target;
  int duty;
  int i,j;
  int choose_motor[2] = {};

  for(i=0;i<9;i++){
    target_duty[0][i] *= VOLTAGE_ADJUST;
    target_duty[1][i] *= VOLTAGE_ADJUST;
  }
  
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
    g_md_h[0].duty = 900.0*VOLTAGE_ADJUST; 
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
    g_md_h[1].duty = 850.0*VOLTAGE_ADJUST; 
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
    g_ld_h[0].mode[7] = color_num;
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
    g_ld_h[0].mode[7] = D_LMOD_NONE;
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
  int duty;
  int rc_analogdata;/*アナログデータ*/
  int rc_analogdata_turn;/*アナログデータ*/
  unsigned int idx;/*インデックス*/
  int i;

  /*for each motor*/
  for(i=0;i<num_of_motor;i++){
    /*それぞれの差分*/
    rc_analogdata = DD_RCGetLY(g_rc_data);
    rc_analogdata_turn = DD_RCGetLX(g_rc_data);
    switch(i){
    case 0:
      idx = MECHA1_MD1;
      break;
    case 1:
      idx = MECHA1_MD2;
      rc_analogdata_turn = -rc_analogdata_turn;
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
	switch(i){
	case 0:
	  g_md_h[idx].mode = D_MMOD_FORWARD;
	  break;
	case 1:
	  g_md_h[idx].mode = D_MMOD_BACKWARD;
	  break;
	default:return EXIT_FAILURE;
	}
      }else{
	switch(i){
	case 0:
	  g_md_h[idx].mode = D_MMOD_BACKWARD;
	  break;
	case 1:
	  g_md_h[idx].mode = D_MMOD_FORWARD;
	  break;
	default:return EXIT_FAILURE;
	}
      }
      /*絶対値を取りDutyに格納*/
      duty = abs(rc_analogdata)*MD_GAIN + rc_analogdata_turn*MD_GAIN*0.4;
      if(duty >=  DD_MD_MAX_DUTY) duty = DD_MD_MAX_DUTY-1;
      if(duty < 0) duty = 0;
      switch(i){
      case 0:
	duty *= R_F_KUDO_ADJUST;
	break;
      case 1:
	duty *= L_B_KUDO_ADJUST;;
	break;
      }
      g_md_h[idx].duty = duty;
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
int odmetry_position_recent(double recent_position[MOVE_SAMPLE_VALUE][3], int recet){

  int i,j;
  double position[3] = {0.0, 0.0, 0.0};

  odmetry_position(position,0);
  
  if(recet == 1){
    for(i=0;i<MOVE_SAMPLE_VALUE;i++){
      recent_position[i][0] = 0.0;
      recent_position[i][1] = 0.0;
      recent_position[i][2] = 0.0;
    }
    return 0;
  }
  
  for(i=0;i<MOVE_SAMPLE_VALUE-1;i++){
    fill_array(recent_position[i+1],recent_position[i],3);
  }
  fill_array(recent_position[0],position,3);
  
  return 0;
}

static
int odmetry_position(double position[3], int recet){
  static bool matrix_init = false;
  const int encoder_ppr = (2048)*4;
  const int tire_diameter = 48;
  static double cal_matrix[3][4];
  static double recent_position[3] = {0.0, 0.0, 0.0};
  double temp_position[3],return_position[3],return_position_val[3],recent_position_rad,encoder_diff[4];
  int i,j;

  if(recet==1){
    for(i=0;i<3;i++){
      position[i] = 0.0;
      recent_position[i] = 0.0;
    }
    return 0;
  }
  
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
    //return_position[i] = recent_position[i] + temp_position[i];
  }
  return_position[2] = recent_position[2] + temp_position[2];
  
  /**********/
  recent_position_rad = recent_position[2] * (1.0/55296.0) * M_PI;
  recent_position_rad = fmod(recent_position_rad, 2.0*M_PI);
  if(fabs(recent_position_rad) >= M_PI){
    if(recent_position_rad >= 0.0){
      recent_position_rad -= 2.0*M_PI;
    }else{
      recent_position_rad += 2.0*M_PI;
    }
  }

  return_position[0] = recent_position[0] + temp_position[0]*cos(recent_position_rad) + temp_position[1]*sin(recent_position_rad);
  return_position[1] = recent_position[1] - temp_position[0]*sin(recent_position_rad) + temp_position[1]*cos(recent_position_rad);
  /**********/
  
  fill_array(recent_position, return_position, 3);

  for(i=0;i<2;i++){
    return_position_val[i] = return_position[i] * (((double)(tire_diameter)*M_PI)/(double)(encoder_ppr));
  }
  return_position_val[2] = return_position[2] * (5.0/1536.0);
  return_position_val[2] = fmod(return_position_val[2], 360.00000);
  if(fabs(return_position_val[2]) >= 180.000000000000000000000000000000000){
    if(return_position_val[2] >= 0.0){
      return_position_val[2] -= 360.000;
    }else{
      return_position_val[2] += 360.000;
    }
  }
  fill_array(position, return_position_val, 3);

  return 0;
}

static
NowPosition_t get_deg_dis(double get_x[MOVE_SAMPLE_VALUE], double get_y[MOVE_SAMPLE_VALUE], double get_zahyou_1[2], double get_zahyou_2[2], double *return_degree, double *return_distance){
  double x[MOVE_SAMPLE_VALUE],y[MOVE_SAMPLE_VALUE],temp_data[MOVE_SAMPLE_VALUE];
  double zahyou_1[2],zahyou_2[2],temp_zahyou;
  MovingDestination_t mode;
  NowPosition_t position;
  double ave_x=0.0, ave_y=0.0, ave_xx=0.0, ave_xy=0.0;
  bool x_move_flag = false,y_move_flag = false;
  int i;
  double a,b,c,a_dis,b_dis,c_dis,d,e,f; //ax+by+c=0 dx+ey+f=0
  double degree,distance,destination;

  fill_array(x,get_x,MOVE_SAMPLE_VALUE);
  fill_array(y,get_y,MOVE_SAMPLE_VALUE);
  fill_array(zahyou_1,get_zahyou_1,2);
  fill_array(zahyou_2,get_zahyou_2,2);
  
  if(zahyou_2[0]-zahyou_1[0]==0.0){
    if(zahyou_2[1]-zahyou_1[1] >= 0.0){
      mode = PLUS_Y;
    }else{
      mode = MINUS_Y;
    }
  }else if(zahyou_2[1]-zahyou_1[1]==0.0){
    if(zahyou_2[0]-zahyou_1[0] >= 0.0){
      mode = PLUS_X;
    }else{
      mode = MINUS_X;
    }
  }

  switch(mode){
  case PLUS_X:
    destination = 1.0;
    if(get_zahyou_1[1]-get_y[0] < 0.0){
      position = NOW_POSITION_LEFT;
    }else if(get_zahyou_1[1]-get_y[0] > 0.0){
      position = NOW_POSITION_RIGHT;
    }else{
      position = NOW_POSITION_CENTER;
    }
    break;
  case MINUS_X:
    destination = 1.0;
    if(get_zahyou_1[1]-get_y[0] < 0.0){
      position = NOW_POSITION_RIGHT;
    }else if(get_zahyou_1[1]-get_y[0] > 0.0){
      position = NOW_POSITION_LEFT;
    }else{
      position = NOW_POSITION_CENTER;
    }
    break;
  case PLUS_Y:
    destination = -1.0;
    if(get_zahyou_1[0]-get_x[0] < 0.0){
      position = NOW_POSITION_RIGHT;
    }else if(get_zahyou_1[0]-get_x[0] > 0.0){
      position = NOW_POSITION_LEFT;
    }else{
      position = NOW_POSITION_CENTER;
    }
    break;
  case MINUS_Y:
    destination = -1.0;
    if(get_zahyou_1[0]-get_x[0] < 0.0){
      position = NOW_POSITION_LEFT;
    }else if(get_zahyou_1[0]-get_x[0] > 0.0){
      position = NOW_POSITION_RIGHT;
    }else{
      position = NOW_POSITION_CENTER;
    }
    break;
  }

  if(mode == PLUS_Y || mode == MINUS_Y){
    temp_zahyou = zahyou_1[0];
    zahyou_1[0] = zahyou_1[1];
    zahyou_1[1] = -temp_zahyou;
    temp_zahyou = zahyou_2[0];
    zahyou_2[0] = zahyou_2[1];
    zahyou_2[1] = -temp_zahyou;
    for(i=0; i<MOVE_SAMPLE_VALUE; i++){
      temp_zahyou = x[i];
      x[i] = y[i];
      y[i] = -temp_zahyou;
    }
  }

  a = -(zahyou_2[1]-zahyou_1[1])/(zahyou_2[0]-zahyou_1[0]);
  b = 1.0;
  c = -(-a*zahyou_1[0] + zahyou_1[1]); 

  //% ./regression_line < datafile
  for(i=0; i<MOVE_SAMPLE_VALUE; i++){
    if(i>=1){
      if(x[i] != x[i-1]){
	x_move_flag = true;
      }
      if(y[i] != y[i-1]){
	y_move_flag = true;
      }
    }
    ave_x  += x[i];
    ave_y  += y[i];
    ave_xx += x[i]*x[i];
    ave_xy += x[i]*y[i];
  }

  if(mode != PLUS_Y && mode != MINUS_Y){
    a_dis = -(get_zahyou_2[1]-get_zahyou_1[1])/(get_zahyou_2[0]-get_zahyou_1[0]);
    b_dis = 1.0;
    c_dis = -(-a_dis*get_zahyou_1[0] + get_zahyou_1[1]);
  }else{
    a_dis = 1.0;
    b_dis = 0.0;
    c_dis = -get_zahyou_1[0];
  }
  
  distance = fabs(-a_dis*get_x[0] + b_dis*get_y[0] - c_dis) / sqrt(a_dis*a_dis+b_dis*b_dis);
  
  if(!x_move_flag && !y_move_flag){
    position = NOW_POSITION_STOP;
    *return_degree = 0;
    *return_distance = distance;
    return position;
  }else{
    ave_x  /= (double)MOVE_SAMPLE_VALUE;
    ave_y  /= (double)MOVE_SAMPLE_VALUE;
    ave_xx /= (double)MOVE_SAMPLE_VALUE;
    ave_xy /= (double)MOVE_SAMPLE_VALUE;

    //printf("%f %f %f %f\n",ave_x,ave_y,ave_xx,ave_xy);
    
    d = -(ave_xy - ave_x*ave_y)/(ave_xx - ave_x*ave_x);
    e = 1.0;
    f = -(ave_xx*ave_y - ave_xy*ave_x)/(ave_xx-ave_x*ave_x);
  }/* else if(!x_move_flag && y_move_flag){ */
  /*   d = 1.0; */
  /*   e = 0.0; */
  /*   f = 0.0; */
  /* } */
  //printf("f(x) = %f*x%+f\n", -d, -f);

  degree = acos(sqrt((a*d+b*e)*(a*d+b*e)/((a*a+b*b)*(d*d+e*e))));
  degree *= 180.0/M_PI * destination;

  switch(mode){
  case PLUS_X:
  case MINUS_X:
    if(d<0.0 && degree!=0.0){
      degree *= -1.0;
    }
    //distance = fabs(get_y[0]-get_zahyou_1[1]);
    break;
  case PLUS_Y:
  case MINUS_Y:
    if(d>0.0 && degree!=0.0){
      degree *= -1.0;
    }
    //distance = fabs(get_x[0]-get_zahyou_1[0]);
    break;
  }

  //printf("%f\n%f\n%f\n%f\n%f\n",a_dis,b_dis,c_dis,get_x[0],get_y[0]);

  *return_degree = degree;
  *return_distance = distance;
  
  return position;
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
int all_motor_stop(void){
  int i;
  for(i=0;i<DD_NUM_OF_MD;i++){
    g_md_h[i].duty = 0; 
    g_md_h[i].mode = D_MMOD_BRAKE;
  }
  return 0;
}

static
void fill_array(double array1[], double array2[], int size){
  for(int i=0;i<size;i++){
    array1[i] = array2[i];
  }
}
