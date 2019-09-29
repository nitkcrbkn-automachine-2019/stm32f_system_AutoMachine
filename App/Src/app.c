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
int I2C_Encoder(int encoder_num, EncoderOperation_t operation, int duty , bool *encoder_bug);


/********/
static
GetObject_t get_object(ArmMechaTarget_t end_position, int zeneba_destination, bool get_hasami, bool get_hasami_right, bool get_hasami_left, int recet);

static
MovingSituation_t go_to_target(double zahyou_1[2], double zahyou_2[2], double max_duty, bool acceleration, bool robo_destination);
/* static  */
/* MovingSituation_t go_to_target_2(double zahyou_1[2], double zahyou_2[2], double max_duty, bool acceleration, bool robo_destination); */
static
MovingSituation_t decide_straight_duty(double *return_duty, double zahyou_1[2], double zahyou_2[2], double position[3], double max_duty, bool acceleration, MovingDestination_t mode);
static
int decide_turn_duty(double *right_duty_adjust, double *left_duty_adjust,  double straight_duty, double zahyou_1[2], double zahyou_2[2], double position[3], MovingDestination_t mode);
static
int decide_turn_degree(double *right_duty_adjust, double *left_duty_adjust, double straight_duty, double zahyou_1[2], double zahyou_2[2], double position[MOVE_SAMPLE_VALUE][3], MovingDestination_t mode);
static
SteeringSituation_t steering_spin_to_target(int target,int target_motor);
static
int steering_1_init(void);
static
int steering_2_init(void);
static
int get_diff(int target,int now_degree,int encoder_ppr);
static
int odmetry_position(double position[3], int recet, bool adjust_flag, bool adjust_xyz[3], double adjust_data[3], bool cons_destination, MovingDestination_t destination);
/* static */
/* int odmetry_position_recent(double position[MOVE_SAMPLE_VALUE][3], int recet); */
static
NowPosition_t get_deg_dis(MovingDestination_t mode, double get_x[MOVE_SAMPLE_VALUE], double get_y[MOVE_SAMPLE_VALUE], double zahyou_1[2], double zahyou_2[2], double *return_degree, double *return_distance);
static
FirstUpMecha_t first_up_mecha_move(FirstUpMecha_t mode);
static
ZenebaMecha_t zeneba_mecha_move(int revolution, int recet);
static
ArmMecha_t arm_mecha_move(ArmMechaTarget_t target, int recet);
static
TurnSituation_t turn_robot(double position[3]);
static
void fill_array(double array1[], double array2[], int size);

static char *testmode_name[] = {
  "MANUAL_SUSPENSION",
  "AUTO_FIRSTMECHA_UP",
  "AUTO_FIRSTMECHA_DOWN",
  "AUTO_SHEETS",
  "AUTO_TOWEL_ALL",
  "AUTO_TOWEL_3",
  "AUTO_TOWEL_2",
  "AUTO_TOWEL_1",
  "AUTO_SHEETS_TOWEL",
  "AUTO_TEST",
  "NO_OPERATION",
  "STOP_EVERYTHING",
};

static char *moving_situation_name[] = {
  "PLUS_ACCELERATING",
  "CONSTANT_SPEED",
  "MINUS_ACCELERATING",
  "ARRIVED_TARGET",
  "SPIN_STEERING",
  "SPIN_END",
};

/********/
/*suspensionSystem*/
static
int all_motor_stop(void);
static
int sus_motor_stop(void);
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

  int i;

  static bool test_flag = false;
  
  int mun_sus_target;
  static int mun_sus_recent_target = 0;
  static bool encoder1_reset = false;
  static bool encoder2_reset = false;

  static double position[3] = {};
  static double recent_position[MOVE_SAMPLE_VALUE][3] = {};

  static TestMode_t now_mode = STOP_EVERYTHING;
  
  static bool circle_flag,cross_flag,sqare_flag,triangle_flag;
  static bool up_flag,down_flag,right_flag,left_flag;
  static bool r1_flag,r2_flag,l1_flag,l2_flag;
  static bool p_start_flag,p_right_flag,p_left_flag,p_recet_flag;

  static double target_zahyou_1[2],target_zahyou_2[2];

  static MovingSituation_t now_moving_situation = SPIN_STEERING;

  int right_degree,left_degree;

  static int moving_count = 0;

  bool odmetry_func[3];
  double odmetry_func_data[3];

  static int32_t destination_adjust_timecount = 0;
  static bool next_motion_recet_flag = true;
  static int next_motion_delay_count=0;

  static bool sheets_flag = false;
  static bool towel_1_flag = false;
  static bool towel_2_flag = false;
  static bool towel_3_flag = false;
  static bool towel_all_flag = false;
  static bool sheets_towel_flag = false;

  static ZenebaMecha_t zeneba_mecha_mode = ZENEBA_SPIN_NOW;

  static ArmMecha_t arm_mecha_mode = ARM_SPIN_NOW;
  static int arm_mecha_target = 0;
  
  static FirstUpMecha_t first_up_mecha_situ;
  static bool first_up_mecha_flag = false;
  static bool first_up_mecha_up = false;
  static bool first_up_mecha_down = false;

  /*Auto 追加分*/
  static bool is_pressed_start_flag = false;
  static bool is_pressed_recet_flag = true;
  static GameZone_t game_zone = BLUE_ZONE;

  static bool get_object_flag = false;
  static GetObject_t get_object_mode = GETTING_NOW;

  static int adjust_timecount = 0;

  static TurnSituation_t turn_situation = TURN_NOW;
  bool testes;
  /*************/
  
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

  if(!PANEL_START_SW()) p_start_flag = true;
  if(!PANEL_RIGHT_SW()) p_right_flag = true;
  if(!PANEL_LEFT_SW()) p_left_flag = true;
  if(!PANEL_RECET_SW()) p_recet_flag = true;
  
  if(PANEL_START_SW() && p_start_flag){
    is_pressed_start_flag = true;
    is_pressed_recet_flag = false;
    p_start_flag = false;
  }
  if(PANEL_RECET_SW() && p_recet_flag){
    is_pressed_start_flag = false;
    is_pressed_recet_flag = true;
    p_recet_flag = false;
  }
  if(PANEL_ZONE_SW()){
    game_zone = RED_ZONE;
  }else{
    game_zone = BLUE_ZONE;
  }
  
  if(__RC_ISPRESSED_UP(g_rc_data) && up_flag){
    arm_mecha_target++;
    if(arm_mecha_target >= 5){
      arm_mecha_target = 0;
    }
    up_flag = false;
  }
  
  if((__RC_ISPRESSED_RIGHT(g_rc_data) && right_flag) || (PANEL_RIGHT_SW() && p_right_flag)){
    if(now_mode == STOP_EVERYTHING){
      now_mode = MANUAL_SUSPENSION;
    }else{
      now_mode++;
    }
    is_pressed_start_flag = false;
    is_pressed_recet_flag = true;
    right_flag = false;
    p_right_flag = false;
  }
  if((__RC_ISPRESSED_LEFT(g_rc_data) && left_flag) || (PANEL_LEFT_SW() && p_left_flag)){
    if(now_mode == MANUAL_SUSPENSION){
      now_mode = STOP_EVERYTHING;
    }else{
      now_mode--;
    }
    is_pressed_start_flag = false;
    is_pressed_recet_flag = true;
    left_flag = false;
    p_left_flag = false;
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
    if(is_pressed_recet_flag){
      g_ld_h[0].mode[6] = D_LMOD_NONE;
      g_ld_h[0].mode[7] = D_LMOD_NONE;
    }
    all_motor_stop();
    break;
  case STOP_EVERYTHING:
    if(is_pressed_recet_flag){
      g_ld_h[0].mode[6] = D_LMOD_NONE;
      g_ld_h[0].mode[7] = D_LMOD_NONE;
    }
    all_motor_stop();
    break;//////////////////////////////////////////////////////
  case AUTO_FIRSTMECHA_UP:
    if(is_pressed_recet_flag){
      g_ld_h[0].mode[6] = D_LMOD_INCREMENT;
      g_ld_h[0].mode[7] = D_LMOD_BLINK_YELLOW;
      first_up_mecha_move(FIRST_UP_MECHA_STOP);
      first_up_mecha_situ  = FIRST_UP_MECHA_UP;
      first_up_mecha_flag = true;
    }
    if(is_pressed_start_flag){
      if(first_up_mecha_flag){
	first_up_mecha_situ = first_up_mecha_move(FIRST_UP_MECHA_UP);
	if(first_up_mecha_situ == FIRST_UP_MECHA_STOP){
	  first_up_mecha_flag = false;
	}
      }
    }
    break;///////////////////////////////////////////////////////
  case AUTO_FIRSTMECHA_DOWN:
    if(is_pressed_recet_flag){
      g_ld_h[0].mode[6] = D_LMOD_INCREMENT;
      g_ld_h[0].mode[7] = D_LMOD_BLINK_GREEN;
      first_up_mecha_move(FIRST_UP_MECHA_STOP);
      first_up_mecha_situ  = FIRST_UP_MECHA_DOWN;
      first_up_mecha_flag = true;
    }
    if(is_pressed_start_flag){
      if(first_up_mecha_flag){
	first_up_mecha_situ = first_up_mecha_move(FIRST_UP_MECHA_DOWN);
	if(first_up_mecha_situ == FIRST_UP_MECHA_STOP){
	  first_up_mecha_flag = false;
	}
      }
    }
    break;///////////////////////////////////////////////////////
  case AUTO_SHEETS:///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(is_pressed_recet_flag){
      g_ab_h[0].dat &= AB_RIGHT_OFF;
      g_ab_h[0].dat &= AB_LEFT_OFF;
      g_ab_h[0].dat &= AB_CENTER_OFF;
      g_ab_h[0].dat &= AB_UPMECHA_OFF;
      
      I2C_Encoder(RIGHT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(BACK_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(LEFT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(FRONT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      ret = odmetry_position(position,1,false,odmetry_func,position,false,PLUS_X);
      next_motion_delay_count=0;
      destination_adjust_timecount = 0;
      next_motion_recet_flag = true;
      arm_mecha_mode = ARM_SPIN_NOW;
      get_object_flag = false;

      all_motor_stop();
      moving_count = 0;
      arm_mecha_move(GET_CLIP, 1);

      g_ld_h[0].mode[6] = D_LMOD_RAINBOW;
      g_ld_h[0].mode[7] = D_LMOD_INCREMENT;
    }
    if(is_pressed_start_flag){
      switch(moving_count){
      case 0://シーツかけるところの基準線まで移動
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = 0.0;
  	  target_zahyou_1[1] = 0.0;
  	  target_zahyou_2[0] = 0.0;
  	  target_zahyou_2[1] = 5700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_GREEN;
  	  }
  	  next_motion_recet_flag = false;
  	}else{
        
  	  now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 4500.0, true, true);
  	  if(now_moving_situation == ARRIVED_TARGET){
  	    next_motion_recet_flag = true;
  	    moving_count++;
  	  }
  	}

	if(!get_object_flag){
	  get_object_mode = get_object(SET_UP_POSI, -1, false, false, false, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = true;
	  }
	}
  	break;
      case 1://角度あわせ
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = 0.0;
  	  target_zahyou_1[1] = position[1];//5700.0;
  	  target_zahyou_2[0] = 2000.0;
  	  target_zahyou_2[1] = position[1];//5700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_RED;
  	  }
  	  next_motion_recet_flag = false;
  	  destination_adjust_timecount = g_SY_system_counter;
  	}else{

  	  if((g_SY_system_counter-destination_adjust_timecount) < 2300){
  	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 2000.0, true, true);
  	    /* steering_spin_to_target(90+R_F_DEG_ADJUST,1); */
  	    /* steering_spin_to_target(90+L_B_DEG_ADJUST,2); */
  	    /* g_md_h[R_F_KUDO_MD].mode = D_MMOD_BACKWARD; */
  	    /* g_md_h[L_B_KUDO_MD].mode = D_MMOD_FORWARD; */
  	    /* g_md_h[R_F_KUDO_MD].duty = (int)round((2000)*L_B_KUDO_ADJUST); */
  	    /* g_md_h[L_B_KUDO_MD].duty = (int)round((2000)*R_F_KUDO_ADJUST); */
  	  }else if((g_SY_system_counter-destination_adjust_timecount) < 3300){
	    g_ab_h[0].dat |= AB_UPMECHA_ON; ///////////////////////////////////////////////
  	    steering_spin_to_target(120+R_F_DEG_ADJUST,1);
  	    steering_spin_to_target(120+L_B_DEG_ADJUST,2);
  	    g_md_h[R_F_KUDO_MD].mode = D_MMOD_BACKWARD;
  	    g_md_h[L_B_KUDO_MD].mode = D_MMOD_FORWARD;
  	    g_md_h[R_F_KUDO_MD].duty = (int)round((3500)*L_B_KUDO_ADJUST);
  	    g_md_h[L_B_KUDO_MD].duty = (int)round((3500)*R_F_KUDO_ADJUST);
  	  }else if((g_SY_system_counter-destination_adjust_timecount) < 4300){
  	    steering_spin_to_target(60+R_F_DEG_ADJUST,1);
  	    steering_spin_to_target(60+L_B_DEG_ADJUST,2);
  	    g_md_h[R_F_KUDO_MD].mode = D_MMOD_BACKWARD;
  	    g_md_h[L_B_KUDO_MD].mode = D_MMOD_FORWARD;
  	    g_md_h[R_F_KUDO_MD].duty = (int)round((3500)*L_B_KUDO_ADJUST);
  	    g_md_h[L_B_KUDO_MD].duty = (int)round((3500)*R_F_KUDO_ADJUST);
  	  }

  	  if((g_SY_system_counter-destination_adjust_timecount) >= 4300){
	    g_md_h[R_F_KUDO_MD].mode = D_MMOD_BRAKE;
  	    g_md_h[L_B_KUDO_MD].mode = D_MMOD_BRAKE;
  	    g_md_h[R_F_KUDO_MD].duty = 0;
  	    g_md_h[L_B_KUDO_MD].duty = 0;
  	    now_moving_situation = SPIN_STEERING;
  	    destination_adjust_timecount = 0;
  	    sus_motor_stop();
  	    odmetry_func[0] = true;
  	    odmetry_func[1] = true;
  	    odmetry_func[2] = true;
  	    odmetry_func_data[0] = 200.0;
  	    odmetry_func_data[1] = 5700.0;
  	    odmetry_func_data[2] = 0.0;
  	    odmetry_position(position,0,true,odmetry_func,odmetry_func_data,false,PLUS_X);
  	    next_motion_recet_flag = true;
  	    moving_count++;
  	  }
  	}
	if(!get_object_flag){
	  get_object_mode = get_object(SET_UP_POSI, -1, false, false, false, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = true;
	  }
	}
  	break;
      case 2://シーツポジションへ横移動
	if(!get_object_flag){
	  get_object_mode = get_object(SET_UP_POSI, -1, false, false, false, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = true;
	  }
	}
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  //g_ab_h[0].dat |= AB_UPMECHA_ON;
  	  target_zahyou_1[0] = 180.0;
  	  target_zahyou_1[1] = position[1];//5700.0;
  	  target_zahyou_2[0] = -1750.0;
  	  target_zahyou_2[1] = position[1];//5700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_BLUE;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{

	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 3000.0, true, true);
  	  }else{
	    if(get_object_flag){
	      next_motion_recet_flag = true;
	      moving_count++;
	      get_object_flag = false;
	    }
  	  }
  	}
      	break;
      case 3://upポジからreleaseポジへ下げる
	if(arm_mecha_mode != ARM_SPIN_END){
	  arm_mecha_mode = arm_mecha_move(SET_RELEASE_POSI, 0);
	}else{
	  next_motion_recet_flag = true;
	  moving_count++;
	  arm_mecha_mode = ARM_SPIN_NOW;
	}
	break;
      case 4://シーつポジションへ直進
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-1650.0;
  	  target_zahyou_1[1] = 5700.0;//position[1];
  	  target_zahyou_2[0] = position[0];//-1650.0;
  	  target_zahyou_2[1] = 6400.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_YELLOW;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 3000.0, true, true);
	    adjust_timecount = g_SY_system_counter;
  	  }else{
	    g_ab_h[0].dat |= AB_RIGHT_ON;
	    g_ab_h[0].dat |= AB_LEFT_ON;
	    if((g_SY_system_counter-adjust_timecount) > 500){
	      next_motion_recet_flag = true;
	      moving_count++;
	    }
  	  }
  	}
      	break;
      case 5://si-tuポジションから後退
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-1650.0;
  	  target_zahyou_1[1] = 6400.0;
  	  target_zahyou_2[0] = position[0];//-1650.0;
  	  target_zahyou_2[1] = 5700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_PURPLE;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 3000.0, true, true);
	    adjust_timecount = g_SY_system_counter;
  	  }else{
	    g_ab_h[0].dat |= AB_CENTER_ON;
	    if((g_SY_system_counter-adjust_timecount) > 500){
	      next_motion_recet_flag = true;
	      moving_count++;
	    }
  	  }
  	}
      	break;
      case 6://洗濯バサミゲット
	if(!get_object_flag){
	  get_object_mode = get_object(SET_RELEASE_POSI, -1, true, true, true, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = false;
	    next_motion_recet_flag = true;
	    moving_count++;
	    get_object_mode = GETTING_NOW;
	  }
	}
	break;
      case 7://洗濯バサミかけるポジションへ直進
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-1650.0;
  	  target_zahyou_1[1] = position[1];//5700.0;
  	  target_zahyou_2[0] = position[0];//-1650.0;
  	  target_zahyou_2[1] = 6300.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_BINARY_GREEN;
  	  }
  	  next_motion_recet_flag = false;
  	}else{
  	  now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 3000.0, true, true);
  	  if(now_moving_situation == ARRIVED_TARGET){
  	    next_motion_recet_flag = true;
  	    moving_count++;
  	  }
  	}
      	break;
      case 8://arm下げて洗濯バサミつける
	if(arm_mecha_mode != ARM_SPIN_END){
	  arm_mecha_mode = arm_mecha_move(SET_CLIP_POSI, 0);
	}else{
	  next_motion_recet_flag = true;
	  moving_count++;
	  arm_mecha_mode = ARM_SPIN_NOW;
	}
	break;
      case 9:
	if(next_motion_recet_flag){
	  adjust_timecount = g_SY_system_counter;
	  next_motion_recet_flag = false;
	}else{
	  if( (g_SY_system_counter-adjust_timecount) > 300){
	    g_md_h[ARM_SPIN_MD].mode = D_MMOD_BRAKE;
	    g_md_h[ARM_SPIN_MD].duty = 0;
	    next_motion_recet_flag = true;
	    moving_count++;
	  }else{
	    g_md_h[ARM_SPIN_MD].mode = D_MMOD_FORWARD;
	    g_md_h[ARM_SPIN_MD].duty = ARM_SPIN_MAXDUTY;
	  }
	}
	
	break;
      case 10://選択バサミ持ちながら横移動
  	if(next_motion_recet_flag){
  	  next_motion_delay_count++;
  	  if(next_motion_delay_count>=50){
  	    odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	    target_zahyou_1[0] = -1750.0;
  	    target_zahyou_1[1] = position[1]+50;//6300.0;
  	    target_zahyou_2[0] = -3450.0;
  	    target_zahyou_2[1] = position[1]+50;//6300.0;
  	    for(i=0; i<8; i++){
  	      g_ld_h[0].mode[i] = D_LMOD_BINARY_BLUE;
  	    }
  	    next_motion_delay_count = 0;
  	    next_motion_recet_flag = false;
  	  }
  	}else{
  	  now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 2000.0, false, true);
  	  if(now_moving_situation == ARRIVED_TARGET){
  	    next_motion_recet_flag = true;
  	    moving_count++;
  	  }
  	}
      	break;
      case 11:
	g_ab_h[0].dat |= AB_RIGHT_ON;
	g_ab_h[0].dat |= AB_LEFT_ON;
	g_ab_h[0].dat |= AB_CENTER_ON;
	if(arm_mecha_mode != ARM_SPIN_END){
	  arm_mecha_mode = arm_mecha_move(SET_RELEASE_POSI, 0);
	}else{
	  next_motion_recet_flag = true;
	  moving_count++;
	  arm_mecha_mode = ARM_SPIN_NOW;
	}
	break;
      }
    }
    break;
  case AUTO_TOWEL_ALL:///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(is_pressed_recet_flag){
      I2C_Encoder(RIGHT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(BACK_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(LEFT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(FRONT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      ret = odmetry_position(position,1,false,odmetry_func,position,false,PLUS_X);
      next_motion_delay_count=0;
      destination_adjust_timecount = 0;
      next_motion_recet_flag = true;
      arm_mecha_mode = ARM_SPIN_NOW;
      get_object_flag = false;

      all_motor_stop();
      moving_count = 0;
      arm_mecha_move(GET_CLIP, 1);
      
      g_ld_h[0].mode[6] = D_LMOD_RAINBOW;
      g_ld_h[0].mode[7] = D_LMOD_RAINBOW;
    }
    if(is_pressed_start_flag){
      switch(moving_count){
      case 0: //タオルかけるところの基準線まで移動
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = 0.0;
  	  target_zahyou_1[1] = 0.0;
  	  target_zahyou_2[0] = 0.0;
  	  target_zahyou_2[1] = 3700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_GREEN;
  	  }
  	  next_motion_recet_flag = false;
  	}else{
        
  	  now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 4500.0, true, true);
  	  if(now_moving_situation == ARRIVED_TARGET){
  	    next_motion_recet_flag = true;
  	    moving_count++;
  	  }
  	}

	if(!get_object_flag){
	  get_object_mode = get_object(SET_RELEASE_POSI, 1, false, false, false, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = true;
	  }
	}
  	break;
      case 1: //角度あわせ
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = 0.0;
  	  target_zahyou_1[1] = position[1];//5700.0;
  	  target_zahyou_2[0] = 2000.0;
  	  target_zahyou_2[1] = position[1];//5700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_RED;
  	  }
  	  next_motion_recet_flag = false;
  	  destination_adjust_timecount = g_SY_system_counter;
  	}else{

  	  if((g_SY_system_counter-destination_adjust_timecount) < 2300){
  	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 2000.0, true, true);
  	    /* steering_spin_to_target(90+R_F_DEG_ADJUST,1); */
  	    /* steering_spin_to_target(90+L_B_DEG_ADJUST,2); */
  	    /* g_md_h[R_F_KUDO_MD].mode = D_MMOD_BACKWARD; */
  	    /* g_md_h[L_B_KUDO_MD].mode = D_MMOD_FORWARD; */
  	    /* g_md_h[R_F_KUDO_MD].duty = (int)round((2000)*L_B_KUDO_ADJUST); */
  	    /* g_md_h[L_B_KUDO_MD].duty = (int)round((2000)*R_F_KUDO_ADJUST); */
  	  }else if((g_SY_system_counter-destination_adjust_timecount) < 3300){
  	    steering_spin_to_target(120+R_F_DEG_ADJUST,1);
  	    steering_spin_to_target(120+L_B_DEG_ADJUST,2);
  	    g_md_h[R_F_KUDO_MD].mode = D_MMOD_BACKWARD;
  	    g_md_h[L_B_KUDO_MD].mode = D_MMOD_FORWARD;
  	    g_md_h[R_F_KUDO_MD].duty = (int)round((3500)*L_B_KUDO_ADJUST);
  	    g_md_h[L_B_KUDO_MD].duty = (int)round((3500)*R_F_KUDO_ADJUST);
  	  }else if((g_SY_system_counter-destination_adjust_timecount) < 4300){
  	    steering_spin_to_target(60+R_F_DEG_ADJUST,1);
  	    steering_spin_to_target(60+L_B_DEG_ADJUST,2);
  	    g_md_h[R_F_KUDO_MD].mode = D_MMOD_BACKWARD;
  	    g_md_h[L_B_KUDO_MD].mode = D_MMOD_FORWARD;
  	    g_md_h[R_F_KUDO_MD].duty = (int)round((3500)*L_B_KUDO_ADJUST);
  	    g_md_h[L_B_KUDO_MD].duty = (int)round((3500)*R_F_KUDO_ADJUST);
  	  }

  	  if((g_SY_system_counter-destination_adjust_timecount) >= 4300){
	    g_md_h[R_F_KUDO_MD].mode = D_MMOD_BRAKE;
  	    g_md_h[L_B_KUDO_MD].mode = D_MMOD_BRAKE;
  	    g_md_h[R_F_KUDO_MD].duty = 0;
  	    g_md_h[L_B_KUDO_MD].duty = 0;
  	    now_moving_situation = SPIN_STEERING;
  	    destination_adjust_timecount = 0;
  	    sus_motor_stop();
  	    odmetry_func[0] = true;
  	    odmetry_func[1] = false;
  	    odmetry_func[2] = true;
  	    odmetry_func_data[0] = 200.0;
  	    odmetry_func_data[1] = 3700.0;
  	    odmetry_func_data[2] = 0.0;
  	    odmetry_position(position,0,true,odmetry_func,odmetry_func_data,false,PLUS_X);
  	    next_motion_recet_flag = true;
  	    moving_count++;
  	  }
  	}
	if(!get_object_flag){
	  get_object_mode = get_object(SET_RELEASE_POSI, 1, false, false, false, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = true;
	  }
	}
  	break;
      case 2:
	if(!get_object_flag){
	  get_object_mode = get_object(SET_RELEASE_POSI, 1, false, false, false, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = true;
	  }
	}
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  //g_ab_h[0].dat |= AB_UPMECHA_ON;
  	  target_zahyou_1[0] = 180.0;//230.0;
  	  target_zahyou_1[1] = position[1];//5700.0;
  	  target_zahyou_2[0] = -1750.0;
  	  target_zahyou_2[1] = position[1];//5700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_BLUE;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{

	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 4000.0, true, true);
  	  }else{
	    if(get_object_flag){
	      next_motion_recet_flag = true;
	      moving_count++;
	      get_object_flag = false;
	    }
  	  }
  	}
      	break;
      case 3: //一枚目ポジションへ直進
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-1750.0;
  	  target_zahyou_1[1] = 3700.0;//position[1];
  	  target_zahyou_2[0] = position[0];//-1750.0;
  	  target_zahyou_2[1] = 4400.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_YELLOW;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 3000.0, true, true);
	    adjust_timecount = g_SY_system_counter;
  	  }else{
	    g_ab_h[0].dat |= AB_RIGHT_ON;
	    g_ab_h[0].dat |= AB_LEFT_ON;
	    if((g_SY_system_counter-adjust_timecount) > 500){
	      next_motion_recet_flag = true;
	      moving_count++;
	    }
  	  }
  	}
      	break;
      case 4://一枚目ポジションから後退
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-1750.0;
  	  target_zahyou_1[1] = 4400.0;//position[1];
  	  target_zahyou_2[0] = position[0];//-1750.0;
  	  target_zahyou_2[1] = 3900.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_PURPLE;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 2000.0, true, true);
	    adjust_timecount = g_SY_system_counter;
  	  }else{
	    g_ab_h[0].dat |= AB_CENTER_ON;
	    if((g_SY_system_counter-adjust_timecount) > 500){
	      next_motion_recet_flag = true;
	      moving_count++;
	    }
  	  }
  	}
      	break;
      case 5://2枚目ポジションへ横移動
	if(!get_object_flag){
	  get_object_mode = get_object(SET_RELEASE_POSI, 1, false, false, false, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = true;
	  }
	}
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  //g_ab_h[0].dat |= AB_UPMECHA_ON;
  	  target_zahyou_1[0] = -1750.0;//position[0];
  	  target_zahyou_1[1] = position[1];//3700.0;
  	  target_zahyou_2[0] = -2450.0;//-2550.0
  	  target_zahyou_2[1] = position[1];//3700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_BLUE;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{

	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 4000.0, true, true);
  	  }else{
	    if(get_object_flag){
	      next_motion_recet_flag = true;
	      moving_count++;
	      get_object_flag = false;
	    }
  	  }
  	}
      	break;
      case 6://2枚目ポジションへ直進
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-2550.0;
  	  target_zahyou_1[1] = position[1];
  	  target_zahyou_2[0] = position[0];//-2550.0;
  	  target_zahyou_2[1] = 4400.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_YELLOW;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 3000.0, true, true);
	    adjust_timecount = g_SY_system_counter;
  	  }else{
	    g_ab_h[0].dat |= AB_RIGHT_ON;
	    g_ab_h[0].dat |= AB_LEFT_ON;
	    if((g_SY_system_counter-adjust_timecount) > 500){
	      next_motion_recet_flag = true;
	      moving_count++;
	    }
  	  }
  	}
      	break;
      case 7://2枚目ポジションから後退
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-2550.0;
  	  target_zahyou_1[1] = 4400.0;//position[1];
  	  target_zahyou_2[0] = position[0];//-2550.0;
  	  target_zahyou_2[1] = 3900.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_PURPLE;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 2000.0, true, true);
	    adjust_timecount = g_SY_system_counter;
  	  }else{
	    g_ab_h[0].dat |= AB_CENTER_ON;
	    if((g_SY_system_counter-adjust_timecount) > 500){
	      next_motion_recet_flag = true;
	      moving_count++;
	    }
  	  }
  	}
      	break;
      case 8://3枚目ポジションへ横移動
	if(!get_object_flag){
	  get_object_mode = get_object(SET_RELEASE_POSI, 1, false, false, false, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = true;
	  }
	}
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  //g_ab_h[0].dat |= AB_UPMECHA_ON;
  	  target_zahyou_1[0] = -2550.0;//position[0];
  	  target_zahyou_1[1] = position[1];//3700.0;
  	  target_zahyou_2[0] = -3150.0;//-3250.0;
  	  target_zahyou_2[1] = position[1];//3700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_BLUE;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 4000.0, true, true);
  	  }else{
	    if(get_object_flag){
	      next_motion_recet_flag = true;
	      moving_count++;
	      get_object_flag = false;
	    }
  	  }
  	}
      	break;
      case 9://3枚目ポジションへ直進
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-3250.0;
  	  target_zahyou_1[1] = position[1];
  	  target_zahyou_2[0] = position[0];//-3250.0;
  	  target_zahyou_2[1] = 4400.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_YELLOW;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 3000.0, true, true);
	    adjust_timecount = g_SY_system_counter;
  	  }else{
	    g_ab_h[0].dat |= AB_RIGHT_ON;
	    g_ab_h[0].dat |= AB_LEFT_ON;
	    if((g_SY_system_counter-adjust_timecount) > 500){
	      next_motion_recet_flag = true;
	      moving_count++;
	    }
  	  }
  	}
      	break;
      case 10://3枚目ポジションから後退
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-3250.0;
  	  target_zahyou_1[1] = 4400.0;//position[1];
  	  target_zahyou_2[0] = position[0];//-3250.0;
  	  target_zahyou_2[1] = 3900.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_PURPLE;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 2000.0, true, true);
	    adjust_timecount = g_SY_system_counter;
  	  }else{
	    g_ab_h[0].dat |= AB_CENTER_ON;
	    if((g_SY_system_counter-adjust_timecount) > 500){
	      next_motion_recet_flag = true;
	      moving_count++;
	    }
  	  }
  	}
      	break;
      case 11:
  	if(next_motion_recet_flag){
  	  next_motion_delay_count++;
  	  if(next_motion_delay_count>=100){
  	    odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	    target_zahyou_1[0] = -3300;//-3250.0;
  	    target_zahyou_1[1] = position[1];//6100.0;
  	    target_zahyou_2[0] = 0.0;
  	    target_zahyou_2[1] = position[1];//6100.0;
  	    for(i=0; i<8; i++){
  	      g_ld_h[0].mode[i] = D_LMOD_BINARY_BLUE;
  	    }
  	    next_motion_delay_count = 0;
  	    next_motion_recet_flag = false;
  	  }
  	}else{
  	  now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 4000.0, true, true);
  	  if(now_moving_situation == ARRIVED_TARGET){
  	    next_motion_recet_flag = true;
  	    moving_count++;
  	  }
  	}
      	break;
      case 12:
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-1650.0;
  	  target_zahyou_1[1] = 3900.0;//position[1];
  	  target_zahyou_2[0] = position[0];//-1650.0;
  	  target_zahyou_2[1] = 0.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_BINARY_GREEN;
  	  }
  	  next_motion_recet_flag = false;
  	}else{
  	  now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 4000.0, true, true);
  	  if(now_moving_situation == ARRIVED_TARGET){
  	    next_motion_recet_flag = true;
  	    moving_count++;
  	  }
  	}
      	break;
      }
    }
    break;
  case AUTO_TOWEL_3:///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(is_pressed_recet_flag){
      I2C_Encoder(RIGHT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(BACK_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(LEFT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(FRONT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      ret = odmetry_position(position,1,false,odmetry_func,position,false,PLUS_X);
      next_motion_delay_count=0;
      destination_adjust_timecount = 0;
      next_motion_recet_flag = true;
      arm_mecha_mode = ARM_SPIN_NOW;
      get_object_flag = false;

      all_motor_stop();
      moving_count = 0;

      g_ld_h[0].mode[6] = D_LMOD_RAINBOW;
      g_ld_h[0].mode[7] = D_LMOD_RED;
    }
    if(is_pressed_start_flag){
      
    }
    break;
  case AUTO_TOWEL_2:///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(is_pressed_recet_flag){
      I2C_Encoder(RIGHT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(BACK_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(LEFT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(FRONT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      ret = odmetry_position(position,1,false,odmetry_func,position,false,PLUS_X);
      next_motion_delay_count=0;
      destination_adjust_timecount = 0;
      next_motion_recet_flag = true;
      arm_mecha_mode = ARM_SPIN_NOW;
      get_object_flag = false;

      all_motor_stop();
      moving_count = 0;

      g_ld_h[0].mode[6] = D_LMOD_RAINBOW;
      g_ld_h[0].mode[7] = D_LMOD_YELLOW;
    }
    if(is_pressed_start_flag){
      
    }
    break;
  case AUTO_TOWEL_1:///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(is_pressed_recet_flag){
      I2C_Encoder(RIGHT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(BACK_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(LEFT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(FRONT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      ret = odmetry_position(position,1,false,odmetry_func,position,false,PLUS_X);
      next_motion_delay_count=0;
      destination_adjust_timecount = 0;
      next_motion_recet_flag = true;
      arm_mecha_mode = ARM_SPIN_NOW;
      get_object_flag = false;

      all_motor_stop();
      moving_count = 0;

      g_ld_h[0].mode[6] = D_LMOD_RAINBOW;
      g_ld_h[0].mode[7] = D_LMOD_BLUE;
    }
    if(is_pressed_start_flag){
      
    }
    break;
  case AUTO_SHEETS_TOWEL:///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(is_pressed_recet_flag){
      I2C_Encoder(RIGHT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(BACK_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(LEFT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(FRONT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      ret = odmetry_position(position,1,false,odmetry_func,position,false,PLUS_X);
      next_motion_delay_count=0;
      destination_adjust_timecount = 0;
      next_motion_recet_flag = true;
      arm_mecha_mode = ARM_SPIN_NOW;
      get_object_flag = false;

      all_motor_stop();
      moving_count = 0;

      g_ld_h[0].mode[6] = D_LMOD_INCREMENT;
      g_ld_h[0].mode[7] = D_LMOD_INCREMENT;
    }
    if(is_pressed_start_flag){
      
    }
    break;
  /* case AUTO_FIRSTMECHA_MOVE: */
  /*   if(__RC_ISPRESSED_CIRCLE(g_rc_data) && circle_flag){ */
  /*     first_up_mecha_flag = true; */
  /*     first_up_mecha_up = true; */
  /*     first_up_mecha_down = false; */
  /*   }else if(__RC_ISPRESSED_TRIANGLE(g_rc_data) && triangle_flag){ */
  /*     first_up_mecha_flag = true; */
  /*     first_up_mecha_up = false; */
  /*     first_up_mecha_down = true; */
  /*   } */
  /*   if(__RC_ISPRESSED_CROSS(g_rc_data)){ */
  /*     first_up_mecha_flag = false; */
  /*     first_up_mecha_up = false; */
  /*     first_up_mecha_down = false; */
  /*   } */
  /*   if(!first_up_mecha_flag){ */
  /*     first_up_mecha_flag = false; */
  /*     first_up_mecha_up = false; */
  /*     first_up_mecha_down = false; */
  /*   }else{ */
  /*     if(first_up_mecha_up){ */
  /* 	first_up_mecha_situ = first_up_mecha_move(FIRST_UP_MECHA_UP); */
  /*     }else if(first_up_mecha_down){ */
  /* 	first_up_mecha_situ = first_up_mecha_move(FIRST_UP_MECHA_DOWN); */
  /*     } */
  /*     if(first_up_mecha_situ == FIRST_UP_MECHA_STOP){ */
  /* 	first_up_mecha_flag = false; */
  /* 	first_up_mecha_up = false; */
  /* 	first_up_mecha_down = false; */
  /*     } */
  /*   } */
    
  /*   break;///////////////////////////////////////////////////////////////////////// */
  case AUTO_TEST:///////////////////////////////////////////////////////////////////////////////////////////////////////
    /* This is BLUE ZONE pro */

    if(is_pressed_recet_flag){
      g_ld_h[0].mode[6] = D_LMOD_BLINK_GREEN;
      g_ld_h[0].mode[7] = D_LMOD_BLINK_GREEN;
    }
    
    if(__RC_ISPRESSED_R1(g_rc_data)&&__RC_ISPRESSED_L1(g_rc_data)){
      I2C_Encoder(RIGHT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(BACK_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(LEFT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      I2C_Encoder(FRONT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      ret = odmetry_position(position,1,false,odmetry_func,position,false,PLUS_X);
      //ret = odmetry_position_recent(recent_position,1);
    }
    if(__RC_ISPRESSED_CIRCLE(g_rc_data) && circle_flag){
      sheets_flag = true;
      towel_1_flag = false;
      towel_2_flag = false;
      towel_3_flag = false;
      towel_all_flag = false;
      sheets_towel_flag = false;
    }else if(__RC_ISPRESSED_TRIANGLE(g_rc_data) && triangle_flag){
      sheets_flag = false;
      towel_1_flag = false;
      towel_2_flag = false;
      towel_3_flag = false;
      towel_all_flag = true;
      sheets_towel_flag = false;
    }
    if(__RC_ISPRESSED_CROSS(g_rc_data)){
      next_motion_delay_count=0;
      destination_adjust_timecount = 0;
      next_motion_recet_flag = true;
      arm_mecha_mode = ARM_SPIN_NOW;
      get_object_flag = false;

      sheets_flag = false;
      towel_1_flag = false;
      towel_2_flag = false;
      towel_3_flag = false;
      towel_all_flag = false;
      sheets_towel_flag = false;
    }

    if(!sheets_flag && !towel_1_flag && !towel_2_flag && !towel_3_flag && !towel_all_flag && !sheets_towel_flag){
      next_motion_delay_count=0;
      destination_adjust_timecount = 0;
      sheets_flag = false;
      next_motion_recet_flag = true;
      all_motor_stop();
      moving_count = 0;
    }
    
    if(!towel_all_flag){
    }else{
      switch(moving_count){
      case 0: //タオルかけるところの基準線まで移動
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = 0.0;
  	  target_zahyou_1[1] = 0.0;
  	  target_zahyou_2[0] = 0.0;
  	  target_zahyou_2[1] = 3700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_GREEN;
  	  }
  	  next_motion_recet_flag = false;
  	}else{
        
  	  now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 4500.0, true, true);
  	  if(now_moving_situation == ARRIVED_TARGET){
  	    next_motion_recet_flag = true;
  	    moving_count++;
  	  }
  	}

	if(!get_object_flag){
	  get_object_mode = get_object(SET_RELEASE_POSI, 1, false, false, false, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = true;
	  }
	}
  	break;
      case 1: //角度あわせ
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = 0.0;
  	  target_zahyou_1[1] = position[1];//5700.0;
  	  target_zahyou_2[0] = 2000.0;
  	  target_zahyou_2[1] = position[1];//5700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_RED;
  	  }
  	  next_motion_recet_flag = false;
  	  destination_adjust_timecount = g_SY_system_counter;
  	}else{

  	  if((g_SY_system_counter-destination_adjust_timecount) < 2300){
  	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 2000.0, true, true);
  	    /* steering_spin_to_target(90+R_F_DEG_ADJUST,1); */
  	    /* steering_spin_to_target(90+L_B_DEG_ADJUST,2); */
  	    /* g_md_h[R_F_KUDO_MD].mode = D_MMOD_BACKWARD; */
  	    /* g_md_h[L_B_KUDO_MD].mode = D_MMOD_FORWARD; */
  	    /* g_md_h[R_F_KUDO_MD].duty = (int)round((2000)*L_B_KUDO_ADJUST); */
  	    /* g_md_h[L_B_KUDO_MD].duty = (int)round((2000)*R_F_KUDO_ADJUST); */
  	  }else if((g_SY_system_counter-destination_adjust_timecount) < 3300){
  	    steering_spin_to_target(120+R_F_DEG_ADJUST,1);
  	    steering_spin_to_target(120+L_B_DEG_ADJUST,2);
  	    g_md_h[R_F_KUDO_MD].mode = D_MMOD_BACKWARD;
  	    g_md_h[L_B_KUDO_MD].mode = D_MMOD_FORWARD;
  	    g_md_h[R_F_KUDO_MD].duty = (int)round((3500)*L_B_KUDO_ADJUST);
  	    g_md_h[L_B_KUDO_MD].duty = (int)round((3500)*R_F_KUDO_ADJUST);
  	  }else if((g_SY_system_counter-destination_adjust_timecount) < 4300){
  	    steering_spin_to_target(60+R_F_DEG_ADJUST,1);
  	    steering_spin_to_target(60+L_B_DEG_ADJUST,2);
  	    g_md_h[R_F_KUDO_MD].mode = D_MMOD_BACKWARD;
  	    g_md_h[L_B_KUDO_MD].mode = D_MMOD_FORWARD;
  	    g_md_h[R_F_KUDO_MD].duty = (int)round((3500)*L_B_KUDO_ADJUST);
  	    g_md_h[L_B_KUDO_MD].duty = (int)round((3500)*R_F_KUDO_ADJUST);
  	  }

  	  if((g_SY_system_counter-destination_adjust_timecount) >= 4300){
	    g_md_h[R_F_KUDO_MD].mode = D_MMOD_BRAKE;
  	    g_md_h[L_B_KUDO_MD].mode = D_MMOD_BRAKE;
  	    g_md_h[R_F_KUDO_MD].duty = 0;
  	    g_md_h[L_B_KUDO_MD].duty = 0;
  	    now_moving_situation = SPIN_STEERING;
  	    destination_adjust_timecount = 0;
  	    sus_motor_stop();
  	    odmetry_func[0] = true;
  	    odmetry_func[1] = false;
  	    odmetry_func[2] = true;
  	    odmetry_func_data[0] = 200.0;
  	    odmetry_func_data[1] = 3700.0;
  	    odmetry_func_data[2] = 0.0;
  	    odmetry_position(position,0,true,odmetry_func,odmetry_func_data,false,PLUS_X);
  	    next_motion_recet_flag = true;
  	    moving_count++;
  	  }
  	}
	if(!get_object_flag){
	  get_object_mode = get_object(SET_RELEASE_POSI, 1, false, false, false, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = true;
	  }
	}
  	break;
      case 2:
	if(!get_object_flag){
	  get_object_mode = get_object(SET_RELEASE_POSI, 1, false, false, false, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = true;
	  }
	}
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  //g_ab_h[0].dat |= AB_UPMECHA_ON;
  	  target_zahyou_1[0] = 180.0;//230.0;
  	  target_zahyou_1[1] = position[1];//5700.0;
  	  target_zahyou_2[0] = -1750.0;
  	  target_zahyou_2[1] = position[1];//5700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_BLUE;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{

	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 4000.0, true, true);
  	  }else{
	    if(get_object_flag){
	      next_motion_recet_flag = true;
	      moving_count++;
	      get_object_flag = false;
	    }
  	  }
  	}
      	break;
      case 3: //一枚目ポジションへ直進
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-1750.0;
  	  target_zahyou_1[1] = 3700.0;//position[1];
  	  target_zahyou_2[0] = position[0];//-1750.0;
  	  target_zahyou_2[1] = 4400.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_YELLOW;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 3000.0, true, true);
	    adjust_timecount = g_SY_system_counter;
  	  }else{
	    g_ab_h[0].dat |= AB_RIGHT_ON;
	    g_ab_h[0].dat |= AB_LEFT_ON;
	    if((g_SY_system_counter-adjust_timecount) > 500){
	      next_motion_recet_flag = true;
	      moving_count++;
	    }
  	  }
  	}
      	break;
      case 4://一枚目ポジションから後退
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-1750.0;
  	  target_zahyou_1[1] = 4400.0;//position[1];
  	  target_zahyou_2[0] = position[0];//-1750.0;
  	  target_zahyou_2[1] = 3900.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_PURPLE;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 2000.0, true, true);
	    adjust_timecount = g_SY_system_counter;
  	  }else{
	    g_ab_h[0].dat |= AB_CENTER_ON;
	    if((g_SY_system_counter-adjust_timecount) > 500){
	      next_motion_recet_flag = true;
	      moving_count++;
	    }
  	  }
  	}
      	break;
      case 5://2枚目ポジションへ横移動
	if(!get_object_flag){
	  get_object_mode = get_object(SET_RELEASE_POSI, 1, false, false, false, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = true;
	  }
	}
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  //g_ab_h[0].dat |= AB_UPMECHA_ON;
  	  target_zahyou_1[0] = -1750.0;//position[0];
  	  target_zahyou_1[1] = position[1];//3700.0;
  	  target_zahyou_2[0] = -2450.0;//-2550.0
  	  target_zahyou_2[1] = position[1];//3700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_BLUE;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{

	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 4000.0, true, true);
  	  }else{
	    if(get_object_flag){
	      next_motion_recet_flag = true;
	      moving_count++;
	      get_object_flag = false;
	    }
  	  }
  	}
      	break;
      case 6://2枚目ポジションへ直進
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-2550.0;
  	  target_zahyou_1[1] = position[1];
  	  target_zahyou_2[0] = position[0];//-2550.0;
  	  target_zahyou_2[1] = 4400.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_YELLOW;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 3000.0, true, true);
	    adjust_timecount = g_SY_system_counter;
  	  }else{
	    g_ab_h[0].dat |= AB_RIGHT_ON;
	    g_ab_h[0].dat |= AB_LEFT_ON;
	    if((g_SY_system_counter-adjust_timecount) > 500){
	      next_motion_recet_flag = true;
	      moving_count++;
	    }
  	  }
  	}
      	break;
      case 7://2枚目ポジションから後退
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-2550.0;
  	  target_zahyou_1[1] = 4400.0;//position[1];
  	  target_zahyou_2[0] = position[0];//-2550.0;
  	  target_zahyou_2[1] = 3900.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_PURPLE;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 2000.0, true, true);
	    adjust_timecount = g_SY_system_counter;
  	  }else{
	    g_ab_h[0].dat |= AB_CENTER_ON;
	    if((g_SY_system_counter-adjust_timecount) > 500){
	      next_motion_recet_flag = true;
	      moving_count++;
	    }
  	  }
  	}
      	break;
      case 8://3枚目ポジションへ横移動
	if(!get_object_flag){
	  get_object_mode = get_object(SET_RELEASE_POSI, 1, false, false, false, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = true;
	  }
	}
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  //g_ab_h[0].dat |= AB_UPMECHA_ON;
  	  target_zahyou_1[0] = -2550.0;//position[0];
  	  target_zahyou_1[1] = position[1];//3700.0;
  	  target_zahyou_2[0] = -3150.0;//-3250.0;
  	  target_zahyou_2[1] = position[1];//3700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_BLUE;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 4000.0, true, true);
  	  }else{
	    if(get_object_flag){
	      next_motion_recet_flag = true;
	      moving_count++;
	      get_object_flag = false;
	    }
  	  }
  	}
      	break;
      case 9://3枚目ポジションへ直進
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-3250.0;
  	  target_zahyou_1[1] = position[1];
  	  target_zahyou_2[0] = position[0];//-3250.0;
  	  target_zahyou_2[1] = 4400.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_YELLOW;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 3000.0, true, true);
	    adjust_timecount = g_SY_system_counter;
  	  }else{
	    g_ab_h[0].dat |= AB_RIGHT_ON;
	    g_ab_h[0].dat |= AB_LEFT_ON;
	    if((g_SY_system_counter-adjust_timecount) > 500){
	      next_motion_recet_flag = true;
	      moving_count++;
	    }
  	  }
  	}
      	break;
      case 10://3枚目ポジションから後退
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-3250.0;
  	  target_zahyou_1[1] = 4400.0;//position[1];
  	  target_zahyou_2[0] = position[0];//-3250.0;
  	  target_zahyou_2[1] = 3900.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_PURPLE;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 2000.0, true, true);
	    adjust_timecount = g_SY_system_counter;
  	  }else{
	    g_ab_h[0].dat |= AB_CENTER_ON;
	    if((g_SY_system_counter-adjust_timecount) > 500){
	      next_motion_recet_flag = true;
	      moving_count++;
	    }
  	  }
  	}
      	break;
      case 11:
  	if(next_motion_recet_flag){
  	  next_motion_delay_count++;
  	  if(next_motion_delay_count>=100){
  	    odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	    target_zahyou_1[0] = -3300;//-3250.0;
  	    target_zahyou_1[1] = position[1];//6100.0;
  	    target_zahyou_2[0] = 0.0;
  	    target_zahyou_2[1] = position[1];//6100.0;
  	    for(i=0; i<8; i++){
  	      g_ld_h[0].mode[i] = D_LMOD_BINARY_BLUE;
  	    }
  	    next_motion_delay_count = 0;
  	    next_motion_recet_flag = false;
  	  }
  	}else{
  	  now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 4000.0, true, true);
  	  if(now_moving_situation == ARRIVED_TARGET){
  	    next_motion_recet_flag = true;
  	    moving_count++;
  	  }
  	}
      	break;
      case 12:
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-1650.0;
  	  target_zahyou_1[1] = 3900.0;//position[1];
  	  target_zahyou_2[0] = position[0];//-1650.0;
  	  target_zahyou_2[1] = 0.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_BINARY_GREEN;
  	  }
  	  next_motion_recet_flag = false;
  	}else{
  	  now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 4000.0, true, true);
  	  if(now_moving_situation == ARRIVED_TARGET){
  	    next_motion_recet_flag = true;
  	    moving_count++;
  	  }
  	}
      	break;
      }
    }
    
    //////*シーツ掛けます*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(!sheets_flag){
    }else{
      switch(moving_count){
      case 0://シーツかけるところの基準線まで移動
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = 0.0;
  	  target_zahyou_1[1] = 0.0;
  	  target_zahyou_2[0] = 0.0;
  	  target_zahyou_2[1] = 5700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_GREEN;
  	  }
  	  next_motion_recet_flag = false;
  	}else{
        
  	  now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 4500.0, true, true);
  	  if(now_moving_situation == ARRIVED_TARGET){
  	    next_motion_recet_flag = true;
  	    moving_count++;
  	  }
  	}

	if(!get_object_flag){
	  get_object_mode = get_object(SET_UP_POSI, -1, false, false, false, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = true;
	  }
	}
  	break;
      case 1://角度あわせ
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = 0.0;
  	  target_zahyou_1[1] = position[1];//5700.0;
  	  target_zahyou_2[0] = 2000.0;
  	  target_zahyou_2[1] = position[1];//5700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_RED;
  	  }
  	  next_motion_recet_flag = false;
  	  destination_adjust_timecount = g_SY_system_counter;
  	}else{

  	  if((g_SY_system_counter-destination_adjust_timecount) < 2300){
  	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 2000.0, true, true);
  	    /* steering_spin_to_target(90+R_F_DEG_ADJUST,1); */
  	    /* steering_spin_to_target(90+L_B_DEG_ADJUST,2); */
  	    /* g_md_h[R_F_KUDO_MD].mode = D_MMOD_BACKWARD; */
  	    /* g_md_h[L_B_KUDO_MD].mode = D_MMOD_FORWARD; */
  	    /* g_md_h[R_F_KUDO_MD].duty = (int)round((2000)*L_B_KUDO_ADJUST); */
  	    /* g_md_h[L_B_KUDO_MD].duty = (int)round((2000)*R_F_KUDO_ADJUST); */
  	  }else if((g_SY_system_counter-destination_adjust_timecount) < 3300){
	    g_ab_h[0].dat |= AB_UPMECHA_ON; ///////////////////////////////////////////////
  	    steering_spin_to_target(120+R_F_DEG_ADJUST,1);
  	    steering_spin_to_target(120+L_B_DEG_ADJUST,2);
  	    g_md_h[R_F_KUDO_MD].mode = D_MMOD_BACKWARD;
  	    g_md_h[L_B_KUDO_MD].mode = D_MMOD_FORWARD;
  	    g_md_h[R_F_KUDO_MD].duty = (int)round((3500)*L_B_KUDO_ADJUST);
  	    g_md_h[L_B_KUDO_MD].duty = (int)round((3500)*R_F_KUDO_ADJUST);
  	  }else if((g_SY_system_counter-destination_adjust_timecount) < 4300){
  	    steering_spin_to_target(60+R_F_DEG_ADJUST,1);
  	    steering_spin_to_target(60+L_B_DEG_ADJUST,2);
  	    g_md_h[R_F_KUDO_MD].mode = D_MMOD_BACKWARD;
  	    g_md_h[L_B_KUDO_MD].mode = D_MMOD_FORWARD;
  	    g_md_h[R_F_KUDO_MD].duty = (int)round((3500)*L_B_KUDO_ADJUST);
  	    g_md_h[L_B_KUDO_MD].duty = (int)round((3500)*R_F_KUDO_ADJUST);
  	  }

  	  if((g_SY_system_counter-destination_adjust_timecount) >= 4300){
	    g_md_h[R_F_KUDO_MD].mode = D_MMOD_BRAKE;
  	    g_md_h[L_B_KUDO_MD].mode = D_MMOD_BRAKE;
  	    g_md_h[R_F_KUDO_MD].duty = 0;
  	    g_md_h[L_B_KUDO_MD].duty = 0;
  	    now_moving_situation = SPIN_STEERING;
  	    destination_adjust_timecount = 0;
  	    sus_motor_stop();
  	    odmetry_func[0] = true;
  	    odmetry_func[1] = true;
  	    odmetry_func[2] = true;
  	    odmetry_func_data[0] = 200.0;
  	    odmetry_func_data[1] = 5700.0;
  	    odmetry_func_data[2] = 0.0;
  	    odmetry_position(position,0,true,odmetry_func,odmetry_func_data,false,PLUS_X);
  	    next_motion_recet_flag = true;
  	    moving_count++;
  	  }
  	}
	if(!get_object_flag){
	  get_object_mode = get_object(SET_UP_POSI, -1, false, false, false, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = true;
	  }
	}
  	break;
      case 2://シーツポジションへ横移動
	if(!get_object_flag){
	  get_object_mode = get_object(SET_UP_POSI, -1, false, false, false, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = true;
	  }
	}
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  //g_ab_h[0].dat |= AB_UPMECHA_ON;
  	  target_zahyou_1[0] = 180.0;
  	  target_zahyou_1[1] = position[1];//5700.0;
  	  target_zahyou_2[0] = -1750.0;
  	  target_zahyou_2[1] = position[1];//5700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_BLUE;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{

	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 3000.0, false, true);
  	  }else{
	    if(get_object_flag){
	      next_motion_recet_flag = true;
	      moving_count++;
	      get_object_flag = false;
	    }
  	  }
  	}
      	break;
      case 3://upポジからreleaseポジへ下げる
	if(arm_mecha_mode != ARM_SPIN_END){
	  arm_mecha_mode = arm_mecha_move(SET_RELEASE_POSI, 0);
	}else{
	  next_motion_recet_flag = true;
	  moving_count++;
	  arm_mecha_mode = ARM_SPIN_NOW;
	}
	break;
      case 4://シーつポジションへ直進
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-1650.0;
  	  target_zahyou_1[1] = position[1];//5700.0
  	  target_zahyou_2[0] = position[0];//-1650.0;
  	  target_zahyou_2[1] = 6400.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_YELLOW;
  	  }
  	  next_motion_recet_flag = false;
	  now_moving_situation = PLUS_ACCELERATING;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 3000.0, true, true);
	    adjust_timecount = g_SY_system_counter;
  	  }else{
	    g_ab_h[0].dat |= AB_RIGHT_ON;
	    g_ab_h[0].dat |= AB_LEFT_ON;
	    if((g_SY_system_counter-adjust_timecount) > 500){
	      next_motion_recet_flag = true;
	      moving_count++;
	    }
  	  }
  	}
      	break;
      case 5://si-tuポジションから後退
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-1650.0;
  	  target_zahyou_1[1] = 6400.0;
  	  target_zahyou_2[0] = position[0];//-1650.0;
  	  target_zahyou_2[1] = 5700.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_PURPLE;
  	  }
  	  next_motion_recet_flag = false;
  	}else{
	  if(now_moving_situation != ARRIVED_TARGET){
	    now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 3000.0, true, true);
	    adjust_timecount = g_SY_system_counter;
  	  }else{
	    g_ab_h[0].dat |= AB_CENTER_ON;
	    if((g_SY_system_counter-adjust_timecount) > 500){
	      next_motion_recet_flag = true;
	      moving_count++;
	    }
  	  }
  	}
      	break;
      case 6://洗濯バサミゲット
	if(!get_object_flag){
	  get_object_mode = get_object(GET_CLIP, -1, true, true, true, 0);
	  if(get_object_mode == GETTING_END){
	    get_object_flag = false;
	    next_motion_recet_flag = true;
	    moving_count++;
	  }
	}
	break;
      case 7://洗濯バサミかけるポジションへ直進
  	if(next_motion_recet_flag){
  	  odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	  target_zahyou_1[0] = position[0];//-1650.0;
  	  target_zahyou_1[1] = position[1];//5700.0;
  	  target_zahyou_2[0] = position[0];//-1650.0;
  	  target_zahyou_2[1] = 6300.0;
  	  for(i=0; i<8; i++){
  	    g_ld_h[0].mode[i] = D_LMOD_BINARY_GREEN;
  	  }
  	  next_motion_recet_flag = false;
  	}else{
  	  now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 3000.0, true, true);
  	  if(now_moving_situation == ARRIVED_TARGET){
  	    next_motion_recet_flag = true;
  	    moving_count++;
  	  }
  	}
      	break;
      case 8://arm下げて洗濯バサミつける
	if(arm_mecha_mode != ARM_SPIN_END){
	  arm_mecha_mode = arm_mecha_move(SET_CLIP_POSI, 0);
	}else{
	  next_motion_recet_flag = true;
	  moving_count++;
	  arm_mecha_mode = ARM_SPIN_NOW;
	}
	break;
      case 9://選択バサミ持ちながら横移動
  	if(next_motion_recet_flag){
  	  next_motion_delay_count++;
  	  if(next_motion_delay_count>=50){
  	    odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
  	    target_zahyou_1[0] = -1750.0;
  	    target_zahyou_1[1] = position[1]+50;//6300.0;
  	    target_zahyou_2[0] = -3450.0;
  	    target_zahyou_2[1] = position[1]+50;//6300.0;
  	    for(i=0; i<8; i++){
  	      g_ld_h[0].mode[i] = D_LMOD_BINARY_BLUE;
  	    }
  	    next_motion_delay_count = 0;
  	    next_motion_recet_flag = false;
  	  }
  	}else{
  	  now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 2000.0, false, true);
  	  if(now_moving_situation == ARRIVED_TARGET){
  	    next_motion_recet_flag = true;
  	    moving_count++;
  	  }
  	}
      	break;
      case 10:
	g_ab_h[0].dat |= AB_RIGHT_ON;
	g_ab_h[0].dat |= AB_LEFT_ON;
	g_ab_h[0].dat |= AB_CENTER_ON;
	if(arm_mecha_mode != ARM_SPIN_END){
	  arm_mecha_mode = arm_mecha_move(SET_RELEASE_POSI, 0);
	}else{
	  next_motion_recet_flag = true;
	  moving_count++;
	  arm_mecha_mode = ARM_SPIN_NOW;
	}
	break;
      }
      //now_moving_situation = go_to_target(target_zahyou_1, target_zahyou_2, 3500.0, true, true);
      //now_moving_situation = go_to_target_2(target_zahyou_1, target_zahyou_2, 3500.0, true, true);
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    break;/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  case MANUAL_SUSPENSION:

    if(is_pressed_recet_flag){
      g_ld_h[0].mode[6] = D_LMOD_BLINK_PURPLE;
      g_ld_h[0].mode[7] = D_LMOD_BLINK_PURPLE;
    }

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
      steering_spin_to_target(mun_sus_target+R_F_DEG_ADJUST,1);
      steering_spin_to_target(mun_sus_target+L_B_DEG_ADJUST,2);
    }
    
#if DD_USE_RC
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
      steering_spin_to_target(mun_sus_target+R_F_DEG_ADJUST,1);
      steering_spin_to_target(mun_sus_target+L_B_DEG_ADJUST,2);
    }

    ret = suspensionSystem();
    if(ret){
      return ret;
    }

    if(__RC_ISPRESSED_R1(g_rc_data) && __RC_ISPRESSED_R2(g_rc_data)){
      g_md_h[ARM_SPIN_MD].mode = D_MMOD_BRAKE;
      g_md_h[ARM_SPIN_MD].duty = 0;
    }else if(__RC_ISPRESSED_R1(g_rc_data)){//SPIN_M
      g_md_h[ARM_SPIN_MD].mode = D_MMOD_FORWARD;
      g_md_h[ARM_SPIN_MD].duty = ARM_SPIN_MAXDUTY;
    }else if(__RC_ISPRESSED_R2(g_rc_data)){
      g_md_h[ARM_SPIN_MD].mode = D_MMOD_BACKWARD;
      g_md_h[ARM_SPIN_MD].duty = ARM_SPIN_MAXDUTY;
    }else{
      g_md_h[ARM_SPIN_MD].mode = D_MMOD_BRAKE;
      g_md_h[ARM_SPIN_MD].duty = 0;
    }

    if(__RC_ISPRESSED_L1(g_rc_data) && __RC_ISPRESSED_L2(g_rc_data)){
      first_up_mecha_move(FIRST_UP_MECHA_STOP);
    }else if(__RC_ISPRESSED_L1(g_rc_data)){//SPIN_M
      first_up_mecha_move(FIRST_UP_MECHA_UP);
    }else if(__RC_ISPRESSED_L2(g_rc_data)){
      first_up_mecha_move(FIRST_UP_MECHA_DOWN);
    }else{
      first_up_mecha_move(FIRST_UP_MECHA_STOP);
    }

    if(__RC_ISPRESSED_R1(g_rc_data) && __RC_ISPRESSED_R2(g_rc_data)){
      if(__RC_ISPRESSED_SQARE(g_rc_data)){
  	g_md_h[ZENEBA_MD].mode = D_MMOD_FORWARD;
  	g_md_h[ZENEBA_MD].duty = ZENEBA_MAXDUTY;
      }else if(__RC_ISPRESSED_CROSS(g_rc_data)){
  	g_md_h[ZENEBA_MD].mode = D_MMOD_BACKWARD;
  	g_md_h[ZENEBA_MD].duty = ZENEBA_MAXDUTY;
      }else{
  	g_md_h[ZENEBA_MD].mode = D_MMOD_BRAKE;
  	g_md_h[ZENEBA_MD].duty = 0;
      }
    }else{
      g_md_h[ZENEBA_MD].mode = D_MMOD_BRAKE;
      g_md_h[ZENEBA_MD].duty = 0;
    }

    if(__RC_ISPRESSED_CIRCLE(g_rc_data)){
      if(zeneba_mecha_mode != ZENEBA_SPIN_END){
  	zeneba_mecha_mode = zeneba_mecha_move(1,0);
      }
    }else{
      zeneba_mecha_mode = ZENEBA_SPIN_NOW;
    }

    if(__RC_ISPRESSED_SQARE(g_rc_data) && !__RC_ISPRESSED_R1(g_rc_data)){
      if(turn_situation != TURN_END){
        turn_situation = turn_robot(position);
      }
    }else{
      turn_situation = TURN_NOW;
    }

    if(__RC_ISPRESSED_CROSS(g_rc_data)){
      if(get_object_mode != GETTING_END){
  	get_object_mode = get_object(SET_RELEASE_POSI, 1,false, false, false, 0);
      }
    }else{
      get_object_mode = GETTING_NOW;
      get_object(SET_RELEASE_POSI, 1, false, false, false, 1);
    }

    
    /* ret = LEDSystem(); */
    /* if(ret){ */
    /*   return ret; */
    /* } */

    if(!__RC_ISPRESSED_CROSS(g_rc_data) || __RC_ISPRESSED_L1(g_rc_data)){
      ret = ABSystem();
      if(ret){
      	return ret;
      }
    }

#endif
    
    break; /////////////////////////////////////////////////

  default:
    break;
  }

  if(now_mode == MANUAL_SUSPENSION){
    ret = odmetry_position(position,0,false,odmetry_func,position,false,PLUS_X);
    if(ret){
      return ret;
    }
  }

  
  if((__RC_ISPRESSED_R1(g_rc_data)&&__RC_ISPRESSED_L1(g_rc_data)) || PANEL_RECET_SW()){
    I2C_Encoder(RIGHT_ENC, RESET_ENCODER_VALUE, 0, &testes);
    I2C_Encoder(BACK_ENC, RESET_ENCODER_VALUE, 0, &testes);
    I2C_Encoder(LEFT_ENC, RESET_ENCODER_VALUE, 0, &testes);
    I2C_Encoder(FRONT_ENC, RESET_ENCODER_VALUE, 0, &testes);
    ret = odmetry_position(position,1,false,odmetry_func,position,false,PLUS_X);
    //ret = odmetry_position_recent(recent_position,1);
  }


  if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
    //MW_printf("SY : [%10d]\n",g_SY_system_counter);
    MW_printf("mode : [%30s]\n",testmode_name[now_mode]);
    if(game_zone==BLUE_ZONE){ MW_printf("ZONE[BLUE]\n"); }else{ MW_printf("ZONE[ RED]\n"); }
    //MW_printf("moving_situ : [%20s]\n",moving_situation_name[now_moving_situation]);
    //MW_printf("arm_mecha_target : [%1d]\n",arm_mecha_target);
    //MW_printf("Encoder_target[%3d]\n",mun_sus_target);
    
    if(_IS_PRESSED_FIRST_UP_LIMITSW()){ MW_printf("UpMecha_UP_SW[ ON]\n"); }else{ MW_printf("UpMecha_UP_SW[OFF]\n"); }
    if(_IS_PRESSED_FIRST_UNDER_LIMITSW()){ MW_printf("UpMecha_UNDER_SW[ ON]\n"); }else{ MW_printf("UpMecha_UNDER_SW[OFF]\n"); }
    if(_IS_PRESSED_ZENEBA_LIMITSW()){ MW_printf("ZENEBA_SW[ ON]\n"); }else{ MW_printf("ZENEBA_SW[OFF]\n"); }
    if(_IS_PRESSED_ARM_UNDER_LIMITSW()){ MW_printf("ARM_UNDER_SW[ ON]\n"); }else{ MW_printf("ARM_UNDER_SW[OFF]\n"); }
    if(_IS_PRESSED_ARM_UP_LIMITSW()){ MW_printf("ARM_UP_SW[ ON]\n"); }else{ MW_printf("ARM_UP_SW[OFF]\n"); }
    if(PANEL_START_SW()){ MW_printf("START[ ON] "); }else{ MW_printf("START[OFF] "); }
    if(PANEL_RIGHT_SW()){ MW_printf("RIGHT[ ON] "); }else{ MW_printf("RIGHT[OFF] "); }
    if(PANEL_LEFT_SW()){ MW_printf("LEFT[ ON] "); }else{ MW_printf("LEFT[OFF] "); }
    if(PANEL_RECET_SW()){ MW_printf("RECET[ ON] "); }else{ MW_printf("RECET[OFF] "); }
    
    /* if(encoder1_reset){ */
    /*   MW_printf("encoder1_reset[true ]\n"); */
    /* }else{ */
    /*   MW_printf("encoder1_reset[false]\n"); */
    /* } */
    /* if(encoder2_reset){ */
    /*   MW_printf("encoder2_reset[true ]\n"); */
    /* }else{ */
    /*   MW_printf("encoder2_reset[false]\n"); */
    /* } */
  }
  
  return EXIT_SUCCESS;
}

static
MovingSituation_t go_to_target(double zahyou_1[2], double zahyou_2[2], double max_duty, bool acceleration, bool robo_destination){
  static SteeringSituation_t steering_situation[2];
  MovingSituation_t situation;
  MovingSituation_t over_shoot_situation;
  static MovingDestination_t mode;
  MovingDestination_t over_shoot_mode;
  double straight_duty,right_duty_adjust,left_duty_adjust;
  static double right_duty = 0.0,left_duty = 0.0;
  static double recent_zahyou_1[2]={},recent_zahyou_2[2]={};
  static double position[3] = {};
  static bool first_flag = false;
  static bool over_shoot = false;
  bool odmetry_func[3];
  
  if((recent_zahyou_1[0]!=zahyou_1[0]) || (recent_zahyou_1[1]!=zahyou_1[1]) || (recent_zahyou_2[0]!=zahyou_2[0]) || (recent_zahyou_2[1]!=zahyou_2[1])){
    over_shoot = false;
    first_flag = true;
    recent_zahyou_1[0] = zahyou_1[0];
    recent_zahyou_1[1] = zahyou_1[1];
    recent_zahyou_2[0] = zahyou_2[0];
    recent_zahyou_2[1] = zahyou_2[1];
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
    right_duty = 0.0;
    left_duty = 0.0;
  }

  if(first_flag){
    switch(mode){
    case PLUS_Y:
      if(!steering_situation[0]){
	steering_situation[0] = steering_spin_to_target(0+R_F_DEG_ADJUST,1);//-1
      }
      if(!steering_situation[1]){
	steering_situation[1] = steering_spin_to_target(0+L_B_DEG_ADJUST,2);//1
      }
      break;
    case MINUS_Y:
      if(!steering_situation[0]){
	steering_situation[0] = steering_spin_to_target(0+R_F_DEG_ADJUST,1);
      }
      if(!steering_situation[1]){
	steering_situation[1] = steering_spin_to_target(0+L_B_DEG_ADJUST,2);
      }
      break;
    case PLUS_X:
      if(!steering_situation[0]){
	steering_situation[0] = steering_spin_to_target(90+R_F_DEG_ADJUST,1);
      }
      if(!steering_situation[1]){
	steering_situation[1] = steering_spin_to_target(90+L_B_DEG_ADJUST,2);
      }
      break;
    case MINUS_X:
      if(!steering_situation[0]){
	steering_situation[0] = steering_spin_to_target(90+R_F_DEG_ADJUST,1);//92
      }
      if(!steering_situation[1]){
	steering_situation[1] = steering_spin_to_target(90+L_B_DEG_ADJUST,2);//86
      }
      break;
    }
    if(steering_situation[0]==STEERING_STOP && steering_situation[1]==STEERING_STOP){
      first_flag = false;
      steering_situation[0] = false;
      steering_situation[1] = false;
      situation = SPIN_END;
    }
    situation = SPIN_STEERING;
  }else{
    switch(mode){
    case PLUS_Y:
      steering_spin_to_target(-1+R_F_DEG_ADJUST,1);
      steering_spin_to_target(1+L_B_DEG_ADJUST,2);
      break;
    case MINUS_Y:
      steering_spin_to_target(0+R_F_DEG_ADJUST,1);
      steering_spin_to_target(0+L_B_DEG_ADJUST,2);
      break;
    case PLUS_X:
      steering_spin_to_target(90+R_F_DEG_ADJUST,1);
      steering_spin_to_target(90+L_B_DEG_ADJUST,2);
      break;
    case MINUS_X:
      steering_spin_to_target(90+R_F_DEG_ADJUST,1);
      steering_spin_to_target(90+L_B_DEG_ADJUST,2);
      break;
    }
    
    odmetry_position(position,0,false,odmetry_func,position,true,mode);
    /* if(situation == ARRIVED_TARGET){ */
      
    /*   switch(mode){ */
    /*   case PLUS_Y: */
    /* 	if(fabs(position[0][1]-zahyou_2[1]) > MOVE_ACCEPTABLE_WIDTH){ */
    /* 	  over_shoot = true; */
    /* 	  over_shoot_mode = MINUS_Y; */
    /* 	} */
    /* 	break; */
    /*   case MINUS_Y: */
    /* 	if(fabs(position[0][1]-zahyou_2[1]) > MOVE_ACCEPTABLE_WIDTH){ */
    /* 	  over_shoot = true; */
    /* 	  over_shoot_mode = PLUS_Y; */
    /* 	} */
    /* 	break; */
    /*   case PLUS_X: */
    /* 	if(fabs(position[0][0]-zahyou_2[0]) > MOVE_ACCEPTABLE_WIDTH){ */
    /* 	  over_shoot = true; */
    /* 	  over_shoot_mode = MINUS_X; */
    /* 	} */
    /* 	break; */
    /*   case MINUS_X: */
    /* 	if(fabs(position[0][0]-zahyou_2[0]) > MOVE_ACCEPTABLE_WIDTH){ */
    /* 	  over_shoot = true; */
    /* 	  over_shoot_mode = PLUS_X; */
    /* 	} */
    /* 	break; */
    /*   } */
      
    /*   if(over_shoot){ */

    /* 	straight_duty = SUS_LOW_DUTY; */
	
    /* 	right_duty = straight_duty;  */
    /* 	left_duty = straight_duty;  */
    /* 	if(right_duty < 0.0) right_duty = 0.0; */
    /* 	if(left_duty < 0.0) left_duty = 0.0; */
    /* 	if(right_duty > 9999.0) right_duty = 9999.0; */
    /* 	if(left_duty > 9999.0) left_duty = 9999.0; */

    /* 	switch(over_shoot_mode){ */
    /* 	case PLUS_Y: */
    /* 	case MINUS_X: */
    /* 	  g_md_h[R_F_KUDO_MD].mode = D_MMOD_FORWARD; */
    /* 	  g_md_h[L_B_KUDO_MD].mode = D_MMOD_BACKWARD; */
    /* 	  g_md_h[R_F_KUDO_MD].duty = (int)round((right_duty)*R_F_KUDO_ADJUST); */
    /* 	  g_md_h[L_B_KUDO_MD].duty = (int)round((left_duty)*L_B_KUDO_ADJUST); */
    /* 	  break; */
    /* 	case PLUS_X: */
    /* 	case MINUS_Y: */
    /* 	  g_md_h[R_F_KUDO_MD].mode = D_MMOD_BACKWARD; */
    /* 	  g_md_h[L_B_KUDO_MD].mode = D_MMOD_FORWARD; */
    /* 	  g_md_h[R_F_KUDO_MD].duty = (int)round((left_duty)*L_B_KUDO_ADJUST); */
    /* 	  g_md_h[L_B_KUDO_MD].duty = (int)round((right_duty)*R_F_KUDO_ADJUST); */
    /* 	} */

    /* 	switch(over_shoot_mode){ */
    /* 	case PLUS_Y: */
    /* 	case MINUS_Y: */
    /* 	  if(fabs(position[0][1]-zahyou_2[1]) > MOVE_ACCEPTABLE_WIDTH){ */
    /* 	    over_shoot_situation = OVER_SHOOT; */
    /* 	  }else{ */
    /* 	    over_shoot_situation = ARRIVED_TARGET; */
    /* 	  } */
    /* 	  break; */
    /* 	case PLUS_X: */
    /* 	case MINUS_X: */
    /* 	  if(fabs(position[0][0]-zahyou_2[0]) > MOVE_ACCEPTABLE_WIDTH){ */
    /* 	    over_shoot_situation = OVER_SHOOT; */
    /* 	  }else{ */
    /* 	    over_shoot_situation = ARRIVED_TARGET; */
    /* 	  } */
    /* 	  break; */
    /* 	} */
	
    /* 	if(over_shoot_situation == ARRIVED_TARGET){ */
    /* 	  situation = ARRIVED_TARGET; */
    /* 	  over_shoot = false; */
    /* 	  g_md_h[R_F_KUDO_MD].duty = 0; */
    /* 	  g_md_h[L_B_KUDO_MD].duty = 0; */
    /* 	  g_md_h[R_F_KUDO_MD].mode = D_MMOD_BRAKE; */
    /* 	  g_md_h[L_B_KUDO_MD].mode = D_MMOD_BRAKE; */
    /* 	}else{ */
    /* 	  over_shoot_situation = OVER_SHOOT; */
    /* 	  return over_shoot_situation; */
    /* 	} */
    /*   }else{ */
    /* 	g_md_h[R_F_KUDO_MD].duty = 0; */
    /* 	g_md_h[L_B_KUDO_MD].duty = 0; */
    /* 	g_md_h[R_F_KUDO_MD].mode = D_MMOD_BRAKE; */
    /* 	g_md_h[L_B_KUDO_MD].mode = D_MMOD_BRAKE; */
    /*   } */
    /* }else{ */
    /* if(!over_shoot){ */
    situation = decide_straight_duty(&straight_duty, zahyou_1, zahyou_2, position, max_duty, acceleration, mode);
    decide_turn_duty(&right_duty_adjust, &left_duty_adjust, straight_duty, zahyou_1, zahyou_2, position, mode);
    
    right_duty = straight_duty + right_duty_adjust;
    left_duty = straight_duty + left_duty_adjust;
    if(right_duty < 0.0) right_duty = 0.0;
    if(left_duty < 0.0) left_duty = 0.0;
    if(right_duty > 9999.0) right_duty = 9999.0;
    if(left_duty > 9999.0) left_duty = 9999.0;
    
    
    switch(mode){
    case PLUS_Y:
    case MINUS_X:
      g_md_h[R_F_KUDO_MD].mode = D_MMOD_FORWARD;
      g_md_h[L_B_KUDO_MD].mode = D_MMOD_BACKWARD;
      g_md_h[R_F_KUDO_MD].duty = (int)round((right_duty)*R_F_KUDO_ADJUST);
      g_md_h[L_B_KUDO_MD].duty = (int)round((left_duty)*L_B_KUDO_ADJUST);
      break;
    case PLUS_X:
    case MINUS_Y:
      g_md_h[R_F_KUDO_MD].mode = D_MMOD_BACKWARD;
      g_md_h[L_B_KUDO_MD].mode = D_MMOD_FORWARD;
      g_md_h[R_F_KUDO_MD].duty = (int)round((left_duty)*L_B_KUDO_ADJUST);
      g_md_h[L_B_KUDO_MD].duty = (int)round((right_duty)*R_F_KUDO_ADJUST);
      break;
    }
    /* } */

    if(situation == ARRIVED_TARGET ){ /* || over_shoot){ */
      
      /* switch(mode){ */
      /* case PLUS_Y: */
      /* 	if(fabs(position[0][1]-zahyou_2[1]) > MOVE_ACCEPTABLE_WIDTH){ */
      /* 	  over_shoot = true; */
      /* 	  over_shoot_mode = MINUS_Y; */
      /* 	} */
      /* 	break; */
      /* case MINUS_Y: */
      /* 	if(fabs(position[0][1]-zahyou_2[1]) > MOVE_ACCEPTABLE_WIDTH){ */
      /* 	  over_shoot = true; */
      /* 	  over_shoot_mode = PLUS_Y; */
      /* 	} */
      /* 	break; */
      /* case PLUS_X: */
      /* 	if(fabs(position[0][0]-zahyou_2[0]) > MOVE_ACCEPTABLE_WIDTH){ */
      /* 	  over_shoot = true; */
      /* 	  over_shoot_mode = MINUS_X; */
      /* 	} */
      /* 	break; */
      /* case MINUS_X: */
      /* 	if(fabs(position[0][0]-zahyou_2[0]) > MOVE_ACCEPTABLE_WIDTH){ */
      /* 	  over_shoot = true; */
      /* 	  over_shoot_mode = PLUS_X; */
      /* 	} */
      /* 	break; */
      /* } */
      
      /* if(over_shoot){ */

      /* 	straight_duty = SUS_LOW_DUTY; */
	
      /* 	right_duty = straight_duty;  */
      /* 	left_duty = straight_duty;  */
      /* 	if(right_duty < 0.0) right_duty = 0.0; */
      /* 	if(left_duty < 0.0) left_duty = 0.0; */
      /* 	if(right_duty > 9999.0) right_duty = 9999.0; */
      /* 	if(left_duty > 9999.0) left_duty = 9999.0; */

      /* 	switch(over_shoot_mode){ */
      /* 	case PLUS_Y: */
      /* 	case MINUS_X: */
      /* 	  g_md_h[R_F_KUDO_MD].mode = D_MMOD_FORWARD; */
      /* 	  g_md_h[L_B_KUDO_MD].mode = D_MMOD_BACKWARD; */
      /* 	  g_md_h[R_F_KUDO_MD].duty = (int)round((right_duty)*R_F_KUDO_ADJUST); */
      /* 	  g_md_h[L_B_KUDO_MD].duty = (int)round((left_duty)*L_B_KUDO_ADJUST); */
      /* 	  break; */
      /* 	case PLUS_X: */
      /* 	case MINUS_Y: */
      /* 	  g_md_h[R_F_KUDO_MD].mode = D_MMOD_BACKWARD; */
      /* 	  g_md_h[L_B_KUDO_MD].mode = D_MMOD_FORWARD; */
      /* 	  g_md_h[R_F_KUDO_MD].duty = (int)round((left_duty)*L_B_KUDO_ADJUST); */
      /* 	  g_md_h[L_B_KUDO_MD].duty = (int)round((right_duty)*R_F_KUDO_ADJUST); */
      /* 	  break; */
      /* 	} */

      /* 	switch(over_shoot_mode){ */
      /* 	case PLUS_Y: */
      /* 	case MINUS_Y: */
      /* 	  if(fabs(position[0][1]-zahyou_2[1]) > MOVE_ACCEPTABLE_WIDTH){ */
      /* 	    over_shoot_situation = OVER_SHOOT; */
      /* 	  }else{ */
      /* 	    over_shoot_situation = ARRIVED_TARGET; */
      /* 	  } */
      /* 	  break; */
      /* 	case PLUS_X: */
      /* 	case MINUS_X: */
      /* 	  if(fabs(position[0][0]-zahyou_2[0]) > MOVE_ACCEPTABLE_WIDTH){ */
      /* 	    over_shoot_situation = OVER_SHOOT; */
      /* 	  }else{ */
      /* 	    over_shoot_situation = ARRIVED_TARGET; */
      /* 	  } */
      /* 	  break; */
      /* 	} */
	
      /* 	if(over_shoot_situation == ARRIVED_TARGET){ */
      /* 	  situation = ARRIVED_TARGET; */
      /* 	  over_shoot = false; */
      /* 	  g_md_h[R_F_KUDO_MD].duty = 0; */
      /* 	  g_md_h[L_B_KUDO_MD].duty = 0; */
      /* 	  g_md_h[R_F_KUDO_MD].mode = D_MMOD_BRAKE; */
      /* 	  g_md_h[L_B_KUDO_MD].mode = D_MMOD_BRAKE; */
      /* 	}else{ */
      /* 	  over_shoot_situation = OVER_SHOOT; */
      /* 	  return over_shoot_situation; */
      /* 	} */
      /* }else{ */
      g_md_h[R_F_KUDO_MD].duty = 0;
      g_md_h[L_B_KUDO_MD].duty = 0;
      g_md_h[R_F_KUDO_MD].mode = D_MMOD_BRAKE;
      g_md_h[L_B_KUDO_MD].mode = D_MMOD_BRAKE;
      /* } */
    }
      
    /* if(situation == ARRIVED_TARGET){ */
    /*   over_shoot_situation = OVER_SHOOT; */
    /*   return over_shoot_situation; */
    /* } */
    /* } */
  }
  
  return situation;
}

/* static  */
/* MovingSituation_t go_to_target_2(double zahyou_1[2], double zahyou_2[2], double max_duty, bool acceleration, bool robo_destination){ */
/*   static SteeringSituation_t steering_situation[2] = {false,false}; */
/*   MovingSituation_t situation; */
/*   static MovingDestination_t mode; */
/*   static double straight_duty,right_degree_adjust,left_degree_adjust,right_duty,left_duty; */
/*   static double recent_zahyou_1[2]={},recent_zahyou_2[2]={}; */
/*   static double position[MOVE_SAMPLE_VALUE][3] = {}; */
/*   static first_flag = false; */
/*   static int deg_adjust_temp; */
/*   static bool over_shoot = false; */
  
/*   if((recent_zahyou_1[0]!=zahyou_1[0]) || (recent_zahyou_1[1]!=zahyou_1[1]) || (recent_zahyou_2[0]!=zahyou_2[0]) || (recent_zahyou_2[1]!=zahyou_2[1])){ */
/*     first_flag = true; */
/*     recent_zahyou_1[0] = zahyou_1[0]; */
/*     recent_zahyou_1[1] = zahyou_1[1]; */
/*     recent_zahyou_2[0] = zahyou_2[0]; */
/*     recent_zahyou_2[1] = zahyou_2[1]; */
/*     if(zahyou_2[0]-zahyou_1[0]==0.0){ */
/*       if(zahyou_2[1]-zahyou_1[1] >= 0.0){ */
/* 	mode = PLUS_Y; */
/*       }else{ */
/* 	mode = MINUS_Y; */
/*       } */
/*     }else if(zahyou_2[1]-zahyou_1[1]==0.0){ */
/*       if(zahyou_2[0]-zahyou_1[0] >= 0.0){ */
/* 	mode = PLUS_X; */
/*       }else{ */
/* 	mode = MINUS_X; */
/*       } */
/*     } */
/*   } */

/*   if(first_flag){ */
/*     switch(mode){ */
/*     case PLUS_Y: */
/*       if(!steering_situation[0]){ */
/* 	steering_situation[0] = steering_spin_to_target(0+R_F_DEG_ADJUST,1);//-1 */
/*       } */
/*       if(!steering_situation[1]){ */
/* 	steering_situation[1] = steering_spin_to_target(0+L_B_DEG_ADJUST,2);//1 */
/*       } */
/*       break; */
/*     case MINUS_Y: */
/*       if(!steering_situation[0]){ */
/* 	steering_situation[0] = steering_spin_to_target(0+R_F_DEG_ADJUST,1); */
/*       } */
/*       if(!steering_situation[1]){ */
/* 	steering_situation[1] = steering_spin_to_target(0+L_B_DEG_ADJUST,2); */
/*       } */
/*       break; */
/*     case PLUS_X: */
/*       if(!steering_situation[0]){ */
/* 	steering_situation[0] = steering_spin_to_target(90+R_F_DEG_ADJUST,1); */
/*       } */
/*       if(!steering_situation[1]){ */
/* 	steering_situation[1] = steering_spin_to_target(90+L_B_DEG_ADJUST,2); */
/*       } */
/*       break; */
/*     case MINUS_X: */
/*       if(!steering_situation[0]){ */
/* 	steering_situation[0] = steering_spin_to_target(90+R_F_DEG_ADJUST,1);//92 */
/*       } */
/*       if(!steering_situation[1]){ */
/* 	steering_situation[1] = steering_spin_to_target(90+L_B_DEG_ADJUST,2);//86 */
/*       } */
/*       break; */
/*     } */
/*     if(steering_situation[0]==STEERING_STOP && steering_situation[1]==STEERING_STOP){ */
/*       first_flag = false; */
/*       steering_situation[0] = false; */
/*       steering_situation[1] = false; */
/*       situation = SPIN_END; */
/*     } */
/*     situation = SPIN_STEERING; */
/*   }else{ */
/*     odmetry_position_recent(position, 0); */
/*     situation = decide_straight_duty(&straight_duty, zahyou_1, zahyou_2, position, max_duty, acceleration, mode); */
/*     decide_turn_degree(&right_degree_adjust, &left_degree_adjust, straight_duty, zahyou_1, zahyou_2, position, mode); */

/*     if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){ */
/*       MW_printf("R_DEG_ADJUST:[%4d] L_DEG_ADJUST:[%4d]\n",(int)round(right_degree_adjust),(int)round(left_degree_adjust)); */
/*     } */
    
/*     switch(mode){ */
/*     case PLUS_Y: */
/*       steering_situation[0] = steering_spin_to_target((int)round(right_degree_adjust)+(int)R_F_DEG_ADJUST,1); */
/*       steering_situation[1] = steering_spin_to_target((int)round(left_degree_adjust)+(int)L_B_DEG_ADJUST,2); */
/*       break; */
/*     case MINUS_Y: */
/*       steering_situation[0] = steering_spin_to_target((int)round(left_degree_adjust)+R_F_DEG_ADJUST,1); */
/*       steering_situation[1] = steering_spin_to_target((int)round(right_degree_adjust)+L_B_DEG_ADJUST,2); */
/*       break; */
/*     case PLUS_X: */
/*       steering_situation[0] = steering_spin_to_target((int)round(-left_degree_adjust)+90+R_F_DEG_ADJUST,1); */
/*       steering_situation[1] = steering_spin_to_target((int)round(-right_degree_adjust)+90+L_B_DEG_ADJUST,2); */
/*       break; */
/*     case MINUS_X: */
/*       steering_situation[0] = steering_spin_to_target((int)round(-right_degree_adjust)+90+R_F_DEG_ADJUST,1); */
/*       steering_situation[1] = steering_spin_to_target((int)round(-left_degree_adjust)+90+L_B_DEG_ADJUST,2); */
/*       break; */
/*     } */
    
/*     if(situation == ARRIVED_TARGET){ */
/*       g_md_h[R_F_KUDO_MD].duty = 0; */
/*       g_md_h[L_B_KUDO_MD].duty = 0; */
/*       g_md_h[R_F_KUDO_MD].mode = D_MMOD_BRAKE; */
/*       g_md_h[L_B_KUDO_MD].mode = D_MMOD_BRAKE; */
/*     }else{ */
/* 	right_duty = straight_duty; */
/* 	left_duty = straight_duty; */
/* 	if(right_duty < 0.0) right_duty = 0.0; */
/* 	if(left_duty < 0.0) left_duty = 0.0; */
/* 	if(right_duty > 9999.0) right_duty = 9999.0; */
/* 	if(left_duty > 9999.0) left_duty = 9999.0; */
/*       switch(mode){ */
/*       case PLUS_Y: */
/*       case MINUS_X: */
/* 	g_md_h[R_F_KUDO_MD].mode = D_MMOD_FORWARD; */
/* 	g_md_h[L_B_KUDO_MD].mode = D_MMOD_BACKWARD; */
/* 	break; */
/*       case PLUS_X: */
/*       case MINUS_Y: */
/* 	g_md_h[R_F_KUDO_MD].mode = D_MMOD_BACKWARD; */
/* 	g_md_h[L_B_KUDO_MD].mode = D_MMOD_FORWARD; */
/* 	break; */
/*       } */
/*       g_md_h[R_F_KUDO_MD].duty = (int)round((right_duty)*R_F_KUDO_ADJUST); */
/*       g_md_h[L_B_KUDO_MD].duty = (int)round((left_duty)*L_B_KUDO_ADJUST); */
/*     } */
/*   } */
  
/*   return situation; */
/* } */

static
MovingSituation_t decide_straight_duty(double *return_duty, double zahyou_1[2], double zahyou_2[2], double position[3], double max_duty, bool acceleration, MovingDestination_t mode){
  MovingSituation_t situation;
  double distance_to_target;
  static double distance_from_first;
  static double recent_zahyou_1[2]={0.0, 0.0},recent_zahyou_2[2]={0.0, 0.0};
  static double first_distance;
  static bool first_flag = true;

  if((recent_zahyou_1[0]!=zahyou_1[0]) || (recent_zahyou_1[1]!=zahyou_1[1]) || (recent_zahyou_2[0]!=zahyou_2[0]) || (recent_zahyou_2[1]!=zahyou_2[1])){
    first_flag = true;
    recent_zahyou_1[0] = zahyou_1[0];
    recent_zahyou_1[1] = zahyou_1[1];
    recent_zahyou_2[0] = zahyou_2[0];
    recent_zahyou_2[1] = zahyou_2[1];
  }

  switch(mode){
  case PLUS_X:
    distance_to_target = zahyou_2[0] - position[0];
    break;
  case MINUS_X:
    distance_to_target = position[0] - zahyou_2[0];
    break;
  case PLUS_Y:
    distance_to_target = zahyou_2[1] - position[1];
    break;
  case MINUS_Y:
    distance_to_target = position[1] - zahyou_2[1];
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
	*return_duty = SUS_LOW_DUTY  - max_duty * (distance_from_first/1000.0 - 1.0)*(distance_from_first/1000.0 - 1.0) + max_duty;
	/* *return_duty = SUS_LOW_DUTY + max_duty * (distance_from_first/1000.0)*(distance_from_first/1000.0)*(distance_from_first/1000.0); */
	/* *return_duty = max_duty * (distance_from_first/1000.0)*(distance_from_first/1000.0)*(distance_from_first/1000.0); */
	if(*return_duty <= SUS_LOW_DUTY) *return_duty = SUS_LOW_DUTY;
	if(*return_duty > max_duty) *return_duty = max_duty;
	situation = PLUS_ACCELERATING;
      }else{
	*return_duty = max_duty;
	situation = CONSTANT_SPEED;
      }
    }else{
      *return_duty = -max_duty * (distance_to_target/1000.0 - 1.15)*(distance_to_target/1000.0 - 1.15) + max_duty;
      /* *return_duty = -max_duty * (distance_to_target/1000.0 - 1.0)*(distance_to_target/1000.0 - 1.0) + max_duty; */
      /* *return_duty = max_duty * (distance_to_target/1000.0 - 1.0)*(distance_to_target/1000.0 - 1.0)*(distance_to_target/1000.0 - 1.0) + max_duty; */
      if(*return_duty <= SUS_LOW_DUTY) *return_duty = SUS_LOW_DUTY;
      if(*return_duty > max_duty) *return_duty = max_duty;
      situation = MINUS_ACCELERATING;
    }
    if(max_duty >= 4000.0){
      if(distance_to_target <= 80.0){ //5mmくらい進むkana
	*return_duty = 0.0;
	situation = ARRIVED_TARGET;
      }
    }else{
      if(distance_to_target <= 50.0){ //5mmくらい進むkana
	*return_duty = 0.0;
	situation = ARRIVED_TARGET;
      }
    }
  }else{
    if(distance_to_target <= 10.0){
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
int decide_turn_duty(double *right_duty_adjust, double *left_duty_adjust, double straight_duty, double zahyou_1[2], double zahyou_2[2], double position[3], MovingDestination_t mode){
  MovingSituation_t situation;
  NowPosition_t now_position;
  double degree,distance;
  //double get_x[MOVE_SAMPLE_VALUE],get_y[MOVE_SAMPLE_VALUE];
  double right_adjust_value=0.0,left_adjust_value=0.0;
  int i;
  double a_dis,b_dis,c_dis;
  
  /* for(i=0; i<MOVE_SAMPLE_VALUE; i++){ */
  /*   get_x[i] = position[i][0]; */
  /*   get_y[i] = position[i][1]; */
  /* } */
  //now_position = get_deg_dis(mode, get_x, get_y, zahyou_1, zahyou_2, &degree, &distance);
  
  if(mode != PLUS_Y && mode != MINUS_Y){
    /* a_dis = -(get_zahyou_2[1]-get_zahyou_1[1])/(get_zahyou_2[0]-get_zahyou_1[0]); */
    /* b_dis = 1.0; */
    /* c_dis = -(-a_dis*get_zahyou_1[0] + get_zahyou_1[1]); */
    distance = fabs(zahyou_1[1] - position[1]); 
  }else{
    a_dis = 1.0;
    b_dis = 0.0;
    c_dis = -zahyou_1[0];
    distance = fabs(-a_dis*position[0] + b_dis*position[1] - c_dis) / sqrt(a_dis*a_dis+b_dis*b_dis);
  }
  degree = position[2];
  
  if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
    MW_printf("degree:[%4d] distance:[%4d]\n",(int)round(degree),(int)round(distance));
  }

  switch(mode){
  case PLUS_X:
    if(zahyou_1[1]-position[1] < 0.0){
      now_position = NOW_POSITION_LEFT;
    }else if(zahyou_1[1]-position[1] > 0.0){
      now_position = NOW_POSITION_RIGHT;
    }else{
      now_position = NOW_POSITION_CENTER;
    }
    break;
  case MINUS_X:
    if(zahyou_1[1]-position[1] < 0.0){
      now_position = NOW_POSITION_RIGHT;
    }else if(zahyou_1[1]-position[1] > 0.0){
      now_position = NOW_POSITION_LEFT;
    }else{
      now_position = NOW_POSITION_CENTER;
    }
    break;
  case PLUS_Y:
    if(zahyou_1[0]-position[0] < 0.0){
      now_position = NOW_POSITION_RIGHT;
    }else if(zahyou_1[0]-position[0] > 0.0){
      now_position = NOW_POSITION_LEFT;
    }else{
      now_position = NOW_POSITION_CENTER;
    }
    break;
  case MINUS_Y:
    if(zahyou_1[0]-position[0] < 0.0){
      now_position = NOW_POSITION_LEFT;
    }else if(zahyou_1[0]-position[0] > 0.0){
      now_position = NOW_POSITION_RIGHT;
    }else{
      now_position = NOW_POSITION_CENTER;
    }
    break;
  }
  
  if(distance > MOVE_ACCEPTABLE_WIDTH){
    switch(now_position){
    case NOW_POSITION_RIGHT:
      if(distance*2.0 <= 100.0){
	right_adjust_value = 150.0;
      }else if(distance*2.0 >= 600.0){
	right_adjust_value = 600.0;
      }else{
	right_adjust_value = distance * 3.0;
      }
      if(degree > 0.0){
	right_adjust_value += degree * 125.0;
	//left_adjust_value  -= degree * 200.0;
      }
      if(degree < 0.0){
	right_adjust_value -= degree * 125.0;
	//left_adjust_value  -= degree * 200.0;
      }
      break;
    case NOW_POSITION_LEFT:
      if(distance*2.0 <= 100.0){
	left_adjust_value = 150.0;
      }else if(distance*2.0 >= 600.0){
	left_adjust_value = 600.0;
      }else{
	left_adjust_value = distance * 3.0;
      }
      if(degree < 0.0){
	left_adjust_value += fabs(degree * 125.0);
	/* right_adjust_value -= fabs(degree * 200.0); */
      }
      if(degree > 0.0){
	left_adjust_value -= fabs(degree * 125.0);
	/* right_adjust_value -= fabs(degree * 200.0); */
      }
      break;
    case NOW_POSITION_CENTER:
      if(degree > 0.0){
	right_adjust_value = degree*150.0 + 75.0;
	//left_adjust_value  = -(degree*100.0 + 100.0);
      }else if(degree < 0.0){
	left_adjust_value = fabs(degree*150.0 + 75.0);
	//right_adjust_value = -fabs(degree*200.0 + 200.0);
      } 
      break;
    }
  }else{
    if(degree > 0.0){
      right_adjust_value = degree*150.0 + 75.0;
      //left_adjust_value  = -(degree*100.0 + 100.0);
    }else if(degree < 0.0){
      left_adjust_value = fabs(degree*150.0 + 75.0);
      //right_adjust_value = -fabs(degree*200.0 + 200.0);
    }
  }

  if(fabs(right_adjust_value) > 850.0){
    if(right_adjust_value > 0.0){
      right_adjust_value = 850.0;
    }else{
      right_adjust_value = -850.0;
    }
  }
  if(fabs(left_adjust_value) > 850.0){
    if(left_adjust_value > 0.0){
      left_adjust_value = 850.0;
    }else{
      left_adjust_value = -850.0;
    }
  }

  *right_duty_adjust = (straight_duty/1000.0) * right_adjust_value; 
  *left_duty_adjust = (straight_duty/1000.0) * left_adjust_value; 

  if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
    MW_printf("degree:[%4d] distance:[%4d] now_posi[%1d]\n",(int)round(degree),(int)round(distance),now_position);
  }
  
  return 0;
}

static
int decide_turn_degree(double *right_degree_adjust, double *left_degree_adjust, double straight_duty, double zahyou_1[2], double zahyou_2[2], double position[MOVE_SAMPLE_VALUE][3], MovingDestination_t mode){
  MovingSituation_t situation;
  NowPosition_t now_position;
  static double degree,distance;
  static double get_x[MOVE_SAMPLE_VALUE],get_y[MOVE_SAMPLE_VALUE];
  static double right_adjust_value=0.0,left_adjust_value=0.0;
  int i;

  right_adjust_value = 0.0;
  left_adjust_value = 0.0;
  
  for(i=0; i<MOVE_SAMPLE_VALUE; i++){
    get_x[i] = position[i][0];
    get_y[i] = position[i][1];
  }
  now_position = get_deg_dis(mode, get_x, get_y, zahyou_1, zahyou_2, &degree, &distance);

  degree = position[0][2];
  
  switch(mode){
  case PLUS_X:
    if(zahyou_1[1]-get_y[0] < 0.0){
      now_position = NOW_POSITION_LEFT;
    }else if(zahyou_1[1]-get_y[0] > 0.0){
      now_position = NOW_POSITION_RIGHT;
    }else{
      now_position = NOW_POSITION_CENTER;
    }
    break;
  case MINUS_X:
    if(zahyou_1[1]-get_y[0] < 0.0){
      now_position = NOW_POSITION_RIGHT;
    }else if(zahyou_1[1]-get_y[0] > 0.0){
      now_position = NOW_POSITION_LEFT;
    }else{
      now_position = NOW_POSITION_CENTER;
    }
    break;
  case PLUS_Y:
    if(zahyou_1[0]-get_x[0] < 0.0){
      now_position = NOW_POSITION_RIGHT;
    }else if(zahyou_1[0]-get_x[0] > 0.0){
      now_position = NOW_POSITION_LEFT;
    }else{
      now_position = NOW_POSITION_CENTER;
    }
    break;
  case MINUS_Y:
    if(zahyou_1[0]-get_x[0] < 0.0){
      now_position = NOW_POSITION_LEFT;
    }else if(zahyou_1[0]-get_x[0] > 0.0){
      now_position = NOW_POSITION_RIGHT;
    }else{
      now_position = NOW_POSITION_CENTER;
    }
    break;
  }
  
  if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
    MW_printf("degree:[%4d] distance:[%4d] now_posi[%1d]\n",(int)round(degree),(int)round(distance),now_position);
  }
  
  if(distance > (double)MOVE_ACCEPTABLE_WIDTH){
    switch(now_position){
    case NOW_POSITION_RIGHT:
      right_adjust_value = 10.0;
      if(distance <= 50.0){
      	right_adjust_value = 5.0;
      }else if(distance >= 250.0){
      	right_adjust_value = 25.0;
      }else{
      	right_adjust_value = distance / (10.0);
      }
      break;
    case NOW_POSITION_LEFT:
      left_adjust_value = 10.0;
      if(distance <= 50.0){
      	left_adjust_value = 5.0;
      }else if(distance*2.0 >= 250.0){
      	left_adjust_value = 25.0;
      }else{
      	left_adjust_value = distance / (10.0);
      }
      break;
    }
    /* if(now_position == 0){ */
    /*   right_adjust_value = distance; */
    /*   left_adjust_value = 0.0; */
    /* }else if(now_position == 1){ */
    /*   right_adjust_value = 0.0; */
    /*   left_adjust_value = distance; */
    /* }else if(now_position == 2){ */
    /*   right_adjust_value = 0.0; */
    /*   left_adjust_value = 0.0; */
    /* }else { */
    /*   right_adjust_value = 0.0; */
    /*   left_adjust_value = 20.0; */
    /* } */
    if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
      MW_printf("distance [out]\n");
    }
  }else{
    if(degree > 0.0){
      right_adjust_value = degree*3.0;
    }else if(degree < 0.0){
      left_adjust_value = fabs(degree*3.0);
    }
    if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
      MW_printf("distance [ in]\n");
    }
  }

  if(fabs(right_adjust_value) > 25.0){
    if(right_adjust_value > 0.0){
      right_adjust_value = 25.0;
    }else{
      right_adjust_value = -25.0;
    }
  }
  if(fabs(left_adjust_value) > 25.0){
    if(left_adjust_value > 0.0){
      left_adjust_value = 25.0;
    }else{
      left_adjust_value = -25.0;
    }
  }

  *right_degree_adjust = right_adjust_value; 
  *left_degree_adjust = left_adjust_value; 
  
  return 0;
}

static
TurnSituation_t turn_robot(double position[3]){
  TurnSituation_t return_value;
  static SteeringSituation_t steering_situation[2];
  static bool init_flag = false;

  if(!init_flag){
    if(!steering_situation[0]){
      steering_situation[0] = steering_spin_to_target(45+R_F_DEG_ADJUST,1);//-1
    }
    if(!steering_situation[1]){
      steering_situation[1] = steering_spin_to_target(45+L_B_DEG_ADJUST,2);//1
    }
    if(steering_situation[0]==STEERING_STOP && steering_situation[1]==STEERING_STOP){
      init_flag = true;
      steering_situation[0] = STEERING_NOW;
      steering_situation[1] = STEERING_NOW;
    }
    return_value = TURN_NOW;
  }else{
    steering_spin_to_target(45+R_F_DEG_ADJUST,1);
    steering_spin_to_target(45+L_B_DEG_ADJUST,2);
    if(fabs(position[2]) <= 178.0){
      g_md_h[R_F_KUDO_MD].mode = D_MMOD_FORWARD;
      g_md_h[L_B_KUDO_MD].mode = D_MMOD_BACKWARD;
      g_md_h[R_F_KUDO_MD].duty = (int)(round(SUS_LOW_DUTY*R_F_KUDO_ADJUST));
      g_md_h[L_B_KUDO_MD].duty = (int)(round(SUS_LOW_DUTY*L_B_KUDO_ADJUST));
      return_value = TURN_NOW;
    }else{
      g_md_h[R_F_KUDO_MD].mode = D_MMOD_BRAKE;
      g_md_h[L_B_KUDO_MD].mode = D_MMOD_BRAKE;
      g_md_h[R_F_KUDO_MD].duty = 0;
      g_md_h[L_B_KUDO_MD].duty = 0;
      return_value = TURN_END;
      init_flag = false;
    }
  }
  
  return return_value;
}

static
SteeringSituation_t steering_spin_to_target(int target,int target_motor){
  const int32_t encoder_ppr = (512)*4;
  const int div = (2048*4)/encoder_ppr;
  /* const int target_deg[9] = {2048,1024,512,256,128,64,32,26,21}; */
  /* const int target_duty[9]   = {8000,3000,1000,120,110,100,90,85,80}; */
  const int target_deg[2][9]  = {{2048/div,1024/div,512/div,256/div,128/div,64/div,32/div,26/div,21/div},
				 {2048/div,1024/div,512/div,256/div,128/div,64/div,32/div,26/div,21/div}};
  double target_duty[2][9] = {{9000.0,5000.0,3000.0,1400.0,1250.0,1150.0,1100.0,1000.0,800.0},
			   {9000.0,5000.0,3000.0,1400.0,1250.0,1150.0,1100.0,1000.0,800.0}};
  /* double target_duty[2][9] = {{9000.0,5000.0,3000.0,1400.0,1250.0,1150.0,1100.0,1000.0,800.0}, */
  /* 			   {9000.0,5000.0,3000.0,1400.0,1250.0,1150.0,1100.0,1000.0,800.0}}; */
  /* int target_duty[2][9] = {{9000,5000,3000,1400,1300,1250,1200,1150,1000}, */
  /* 			   {9000,5000,3000,1400,1300,1250,1200,1150,1000}}; */
  int32_t encoder;
  int32_t encoder_degree;
  int32_t target_degree;
  int spin_direction = 1;
  int diff_from_target;
  double duty;
  int i,j;
  int choose_motor[2] = {};
  SteeringSituation_t situation;
  static bool motor_set[2] = {false, false};
  
  for(i=0;i<9;i++){
    target_duty[0][i] *= R_F_DEG_DUTY_ADJUST;
    target_duty[1][i] *= L_B_DEG_DUTY_ADJUST;
    if(i>=3){
      target_duty[0][i] = R_F_DEG_LOW_DUTY;
      target_duty[1][i] = L_B_DEG_LOW_DUTY;
    }
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
    target_degree  = (int)round(((double)encoder_ppr/360.0)*(double)target);
  
  
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
	duty = 0.0;
      }
    }
    if(diff_from_target > 0){
      g_md_h[j].duty = (int)round(duty);
      if(duty==0.0){
	g_md_h[j].mode = D_MMOD_BRAKE;
	motor_set[j] = true;
      }else{
	g_md_h[j].mode = D_MMOD_FORWARD;
	motor_set[j] = false;
      }
    }else{
      g_md_h[j].duty = (int)round(duty);
      if(duty==0.0){
	g_md_h[j].mode = D_MMOD_BRAKE;
	motor_set[j] = true;
      }else{
	g_md_h[j].mode = D_MMOD_BACKWARD;
	motor_set[j] = false;
      }
    }
    if(target_motor == 0){
      if(motor_set[0] && motor_set[1]){
	situation = STEERING_STOP;
      }else{
	situation = STEERING_NOW;
      }
    }else if(target_motor == 1){
      if(motor_set[0]){
	situation = STEERING_STOP;
      }else{
	situation = STEERING_NOW;
      }
    }else if(target_motor == 2){
      if(motor_set[1]){
	situation = STEERING_STOP;
      }else{
	situation = STEERING_NOW;
      }
    }
    if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
      switch(j){
      case 0:
	MW_printf("E1:degree[%3d] target[%3d]\n",(int)round((double)encoder_degree*(double)(360.0/(double)encoder_ppr)),target);
	break;
      case 1:
	MW_printf("E2:degree[%3d] target[%3d]\n",(int)round((double)encoder_degree*(double)(360.0/(double)encoder_ppr)),target);
	break;
      }
    }
  }
  return situation;
}

static
int steering_1_init(void){
  if(_IS_PRESSED_ENCODER1_RESET()){
    g_md_h[0].duty = 0; 
    g_md_h[0].mode = D_MMOD_BRAKE;
    DD_encoder1reset();
    return 0;
  }else{
    g_md_h[0].duty = R_F_DEG_INIT_LOW_DUTY; 
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
    g_md_h[1].duty = L_B_DEG_INIT_LOW_DUTY; 
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

static
GetObject_t get_object(ArmMechaTarget_t end_position, int zeneba_destination, bool get_hasami, bool get_hasami_right, bool get_hasami_left, int recet){
  GetObject_t return_value;
  static ArmMecha_t arm_mecha_mode = ARM_SPIN_NOW;
  static ZenebaMecha_t zeneba_mecha_mode = ZENEBA_SPIN_NOW;
  static int mode_count = 0;

  if(recet == 1){
    arm_mecha_mode = ARM_SPIN_NOW;
    zeneba_mecha_mode = ZENEBA_SPIN_NOW;
    arm_mecha_move(GET_CLIP, 1);
    mode_count = 0;
    return GETTING_RECET;
  }
  
  switch(mode_count){
  case 0:
    g_ab_h[0].dat |= AB_RIGHT_ON;
    g_ab_h[0].dat |= AB_LEFT_ON;
    g_ab_h[0].dat |= AB_CENTER_ON;
    mode_count++;
    return_value = GETTING_NOW;
    break;
      
  case 1:
    if(arm_mecha_mode != ARM_SPIN_END){
      if(get_hasami){
	arm_mecha_mode = arm_mecha_move(GET_CLIP, 0);
      }else{
	arm_mecha_mode = arm_mecha_move(GET_LAUNDRY, 0);
      }
    }else{
      if(get_hasami){
	if(get_hasami_right) g_ab_h[0].dat &= AB_RIGHT_OFF;
	if(get_hasami_left) g_ab_h[0].dat &= AB_LEFT_OFF;
      }else{
	g_ab_h[0].dat &= AB_RIGHT_OFF;
	g_ab_h[0].dat &= AB_LEFT_OFF;
	g_ab_h[0].dat &= AB_CENTER_OFF;
      }
      arm_mecha_mode = ARM_SPIN_NOW;
      mode_count++;
    }
    return_value = GETTING_NOW;
    break;
      
  case 2:
    if(arm_mecha_mode != ARM_SPIN_END){
      arm_mecha_mode = arm_mecha_move(end_position, 0);
    }else{
      arm_mecha_mode = ARM_SPIN_NOW;
      mode_count++;
    }
    return_value = GETTING_NOW;
    break;
    
  case 3:
    if((zeneba_mecha_mode != ZENEBA_SPIN_END) && (zeneba_destination != 0)){
      zeneba_mecha_mode = zeneba_mecha_move(zeneba_destination,0);
      return_value = GETTING_NOW;
    }else{
      if(get_hasami){
	if(get_hasami_right) g_ab_h[0].dat &= AB_RIGHT_OFF;
	if(get_hasami_left) g_ab_h[0].dat &= AB_LEFT_OFF;
      }else{
	g_ab_h[0].dat &= AB_RIGHT_OFF;
	g_ab_h[0].dat &= AB_LEFT_OFF;
	g_ab_h[0].dat &= AB_CENTER_OFF;
      }
      zeneba_mecha_mode = ZENEBA_SPIN_NOW;
      arm_mecha_mode = ARM_SPIN_NOW;
      return_value = GETTING_END;
      //arm_mecha_move(GET_CLIP, 1);
      mode_count = 0;
    }
    break;
  }//switch(mode_count) end

  return return_value;
}

static
ArmMecha_t arm_mecha_move(ArmMechaTarget_t target, int recet){
  static ArmMechaTarget_t now_position = SET_RELEASE_POSI;
  static bool time_recet_flag = true;
  static int time_count = 0;
  static bool through_sensor = false;
  ArmMecha_t return_value;

  if(recet == 1){
    now_position = SET_RELEASE_POSI;
    time_recet_flag = true;
    time_count = 0;
    through_sensor = false;
    return ARM_RECET;
  }
  
  switch(now_position){
  case GET_LAUNDRY:
    switch(target){
    case SET_RELEASE_POSI:
      if(time_recet_flag){
	time_count = g_SY_system_counter;
	time_recet_flag = false;
      }
      if(g_SY_system_counter-time_count > 3500){
	g_md_h[ARM_SPIN_MD].mode = D_MMOD_BRAKE;
	g_md_h[ARM_SPIN_MD].duty = 0;
	return_value = ARM_SPIN_END;
	now_position = SET_RELEASE_POSI;
	time_recet_flag = true;
      }else{
	g_md_h[ARM_SPIN_MD].mode = D_MMOD_FORWARD;
	g_md_h[ARM_SPIN_MD].duty = ARM_SPIN_MAXDUTY;
	return_value = ARM_SPIN_NOW;
      }
      break;
    case SET_UP_POSI:
      if(time_recet_flag){
	time_count = g_SY_system_counter;
	time_recet_flag = false;
      }
      if(g_SY_system_counter-time_count > 7000){
	g_md_h[ARM_SPIN_MD].mode = D_MMOD_BRAKE;
	g_md_h[ARM_SPIN_MD].duty = 0;
	return_value = ARM_SPIN_END;
	now_position = SET_UP_POSI;
	time_recet_flag = true;
      }else{
	g_md_h[ARM_SPIN_MD].mode = D_MMOD_FORWARD;
	g_md_h[ARM_SPIN_MD].duty = ARM_SPIN_MAXDUTY;
	return_value = ARM_SPIN_NOW;
      }
      break;
    default:
      break;
    }
    break;
    
  case GET_CLIP:
    switch(target){
    case SET_RELEASE_POSI:
      if(time_recet_flag){
	time_count = g_SY_system_counter;
	time_recet_flag = false;
      }
      if(g_SY_system_counter-time_count > 4100){
	g_md_h[ARM_SPIN_MD].mode = D_MMOD_BRAKE;
	g_md_h[ARM_SPIN_MD].duty = 0;
	return_value = ARM_SPIN_END;
	now_position = SET_RELEASE_POSI;
	time_recet_flag = true;
      }else{
	g_md_h[ARM_SPIN_MD].mode = D_MMOD_FORWARD;
	g_md_h[ARM_SPIN_MD].duty = ARM_SPIN_MAXDUTY;
	return_value = ARM_SPIN_NOW;
      }
      break;
    default:
      break;
    }
    break;
  case SET_RELEASE_POSI:
    switch(target){
    case SET_CLIP_POSI:
      if(!_IS_PRESSED_ARM_UP_LIMITSW()){
	g_md_h[ARM_SPIN_MD].mode = D_MMOD_BACKWARD;
	g_md_h[ARM_SPIN_MD].duty = ARM_SPIN_MAXDUTY;
	return_value = ARM_SPIN_NOW;
      }else{
	g_md_h[ARM_SPIN_MD].mode = D_MMOD_BRAKE;
	g_md_h[ARM_SPIN_MD].duty = 0;
	return_value = ARM_SPIN_END;	
	now_position = SET_CLIP_POSI;
      }
      break;
    case GET_CLIP:
      if(!_IS_PRESSED_ARM_UNDER_LIMITSW()){
	g_md_h[ARM_SPIN_MD].mode = D_MMOD_BACKWARD;
	g_md_h[ARM_SPIN_MD].duty = ARM_SPIN_MAXDUTY;
	return_value = ARM_SPIN_NOW;
      }else{
	g_md_h[ARM_SPIN_MD].mode = D_MMOD_BRAKE;
	g_md_h[ARM_SPIN_MD].duty = 0;
	return_value = ARM_SPIN_END;	
	now_position = GET_CLIP;
      }
      break;
    case GET_LAUNDRY:
      if(!through_sensor){
	if(!_IS_PRESSED_ARM_UNDER_LIMITSW()){
	  g_md_h[ARM_SPIN_MD].mode = D_MMOD_BACKWARD;
	  g_md_h[ARM_SPIN_MD].duty = ARM_SPIN_MAXDUTY;
	  return_value = ARM_SPIN_NOW;
	}else{
	  through_sensor = true;
	  g_md_h[ARM_SPIN_MD].mode = D_MMOD_BACKWARD;
	  g_md_h[ARM_SPIN_MD].duty = ARM_SPIN_MAXDUTY;
	  return_value = ARM_SPIN_NOW;
	  /* g_md_h[ARM_SPIN_MD].mode = D_MMOD_BRAKE; */
	  /* g_md_h[ARM_SPIN_MD].duty = 0; */
	  /* return_value = ARM_SPIN_END;	 */
	  /* now_position = GET_LAUNDRY; */
	}
      }else{
	if(time_recet_flag){
	  time_count = g_SY_system_counter;
	  time_recet_flag = false;
	}
	if(g_SY_system_counter-time_count > 80){
	  g_md_h[ARM_SPIN_MD].mode = D_MMOD_BRAKE;
	  g_md_h[ARM_SPIN_MD].duty = 0;
	  return_value = ARM_SPIN_END;
	  now_position = GET_LAUNDRY;
	  time_recet_flag = true;
	  through_sensor = false;
	}else{
	  g_md_h[ARM_SPIN_MD].mode = D_MMOD_BACKWARD;
	  g_md_h[ARM_SPIN_MD].duty = ARM_SPIN_MAXDUTY;
	  return_value = ARM_SPIN_NOW;
	}
      }
      break;
    default:
      break;
    }
    break;
    
  case SET_CLIP_POSI:
    switch(target){
    case SET_RELEASE_POSI:
      if(time_recet_flag){
	time_count = g_SY_system_counter;
	time_recet_flag = false;
      }
      if(g_SY_system_counter-time_count > 1500){
	g_md_h[ARM_SPIN_MD].mode = D_MMOD_BRAKE;
	g_md_h[ARM_SPIN_MD].duty = 0;
	return_value = ARM_SPIN_END;
	now_position = SET_RELEASE_POSI;
	time_recet_flag = true;
      }else{
	g_md_h[ARM_SPIN_MD].mode = D_MMOD_FORWARD;
	g_md_h[ARM_SPIN_MD].duty = ARM_SPIN_MAXDUTY;
	return_value = ARM_SPIN_NOW;
      }
      break;
    default:
      break;
    }
    break;
    
  case SET_UP_POSI:
    switch(target){
    case SET_RELEASE_POSI:
      if(time_recet_flag){
	time_count = g_SY_system_counter;
	time_recet_flag = false;
      }
      if(g_SY_system_counter-time_count > 2500){
	g_md_h[ARM_SPIN_MD].mode = D_MMOD_BRAKE;
	g_md_h[ARM_SPIN_MD].duty = 0;
	return_value = ARM_SPIN_END;
	now_position = SET_RELEASE_POSI;
	time_recet_flag = true;
      }else{
	g_md_h[ARM_SPIN_MD].mode = D_MMOD_BACKWARD;
	g_md_h[ARM_SPIN_MD].duty = ARM_SPIN_MAXDUTY;
	return_value = ARM_SPIN_NOW;
      }
      break;
    default:
      break;
    }
    break;
  }//switch(now_position) end

  return return_value;
}

static
ZenebaMecha_t zeneba_mecha_move(int revolution, int recet){
  static int revo_count = 0;
  ZenebaMecha_t return_value;
  static bool not_react = false;
  
  if(recet == 1){
    revo_count = 0;
    return_value = ZENEBA_RECET;
    return return_value;
  }

  if(!_IS_PRESSED_ZENEBA_LIMITSW()){
    not_react = true;
  }

  if(not_react){
    if(_IS_PRESSED_ZENEBA_LIMITSW()){
      revo_count++;
      not_react = false;
    }
  }

  if(revo_count == abs(revolution)){
    g_md_h[ZENEBA_MD].mode = D_MMOD_BRAKE;
    g_md_h[ZENEBA_MD].duty = 0;
    return_value = ZENEBA_SPIN_END;
    revo_count = 0;
    not_react = false;
  }else{
    if(revolution > 0){
      g_md_h[ZENEBA_MD].mode = D_MMOD_BACKWARD;
    }else{
      g_md_h[ZENEBA_MD].mode = D_MMOD_FORWARD;
    }
    g_md_h[ZENEBA_MD].duty = ZENEBA_MAXDUTY;
    return_value = ZENEBA_SPIN_NOW;
  }
  
  return return_value;
}

static
FirstUpMecha_t first_up_mecha_move(FirstUpMecha_t mode){
  //static bool up_sw_flag=false,under_sw_flag=false;
  FirstUpMecha_t return_mode;
  switch(mode){
  case FIRST_UP_MECHA_UP:
    if(!_IS_PRESSED_FIRST_UP_LIMITSW()){
      g_md_h[ARM_UP_MD].mode = D_MMOD_FORWARD;
      g_md_h[ARM_UP_MD].duty = ARM_UP_MAXDUTY;
      return_mode = FIRST_UP_MECHA_UP;
    }else{
      g_md_h[ARM_UP_MD].mode = D_MMOD_BRAKE;
      g_md_h[ARM_UP_MD].duty = 0;
      return_mode = FIRST_UP_MECHA_STOP;
    }
    break;
  case FIRST_UP_MECHA_DOWN:
    if(!_IS_PRESSED_FIRST_UNDER_LIMITSW()){
      g_md_h[ARM_UP_MD].mode = D_MMOD_BACKWARD;
      g_md_h[ARM_UP_MD].duty = ARM_UP_MAXDUTY;
      return_mode = FIRST_UP_MECHA_DOWN;
    }else{
      g_md_h[ARM_UP_MD].mode = D_MMOD_BRAKE;
      g_md_h[ARM_UP_MD].duty = 0;
      return_mode = FIRST_UP_MECHA_STOP;
    }
    break;
  case FIRST_UP_MECHA_STOP:
    g_md_h[ARM_UP_MD].mode = D_MMOD_BRAKE;
    g_md_h[ARM_UP_MD].duty = 0;
    return_mode = FIRST_UP_MECHA_STOP;
    break;
  }
  return return_mode;
}

static int LEDSystem(void){
  static int color_num = 12;
  static bool c_up_flag = true;
  if(!__RC_ISPRESSED_UP(g_rc_data)){
    c_up_flag = true;
  }
  if(c_up_flag && __RC_ISPRESSED_UP(g_rc_data)){
    c_up_flag = false;
    color_num++;
    if(color_num>=22){
      color_num = 0;
    }
    //g_led_mode = lmode_1;
  }
  g_ld_h[0].mode[4] = color_num;
  g_ld_h[0].mode[5] = color_num;
  g_ld_h[0].mode[6] = color_num;
  g_ld_h[0].mode[7] = color_num;
  if(__RC_ISPRESSED_DOWN(g_rc_data)){
    color_num = 0;
    //g_led_mode = lmode_2;
  }
  
  return EXIT_SUCCESS;
}

static 
int ABSystem(void){

  if(__RC_ISPRESSED_L2(g_rc_data) && __RC_ISPRESSED_L1(g_rc_data)){
    if(__RC_ISPRESSED_TRIANGLE(g_rc_data)){
      g_ab_h[0].dat |= AB_UPMECHA_ON;
    }
    if(__RC_ISPRESSED_CIRCLE(g_rc_data)){
      g_ab_h[0].dat |= AB_RIGHT_ON;
    }else{
      g_ab_h[0].dat &= AB_RIGHT_OFF;
    }
    if(__RC_ISPRESSED_CROSS(g_rc_data)){
      g_ab_h[0].dat |= AB_CENTER_ON;
    }else{
      g_ab_h[0].dat &= AB_CENTER_OFF;
    }
    if(__RC_ISPRESSED_SQARE(g_rc_data)){
      g_ab_h[0].dat |= AB_LEFT_ON;
    }else{
      g_ab_h[0].dat &= AB_LEFT_OFF;
    }
  }else{
    g_ab_h[0].dat &= AB_UPMECHA_OFF;
    g_ab_h[0].dat &= AB_CENTER_OFF;
    g_ab_h[0].dat &= AB_RIGHT_OFF;
    g_ab_h[0].dat &= AB_LEFT_OFF;
  }

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
      idx = R_F_KUDO_MD;
      break;
    case 1:
      idx = L_B_KUDO_MD;
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

/* static */
/* int odmetry_position_recent(double recent_position[MOVE_SAMPLE_VALUE][3], int recet){ */

/*   int i,j; */
/*   double position[3] = {0.0, 0.0, 0.0}; */
/*   double to_func[3] = {0.0, 0.0, 0.0}; */
/*   bool to_func_flag[3]; */
  
  
/*   if(recet == 1){ */
/*     for(i=0;i<MOVE_SAMPLE_VALUE;i++){ */
/*       recent_position[i][0] = 0.0; */
/*       recent_position[i][1] = 0.0; */
/*       recent_position[i][2] = 0.0; */
/*     } */
/*     return 0; */
/*   } */

/*   odmetry_position(position,0,false,to_func_flag,to_func,false,PLUS_X); */
/*   for(i=0;i<MOVE_SAMPLE_VALUE-1;i++){ */
/*     fill_array(recent_position[i+1],recent_position[i],3); */
/*   } */
/*   fill_array(recent_position[0],position,3); */
  
/*   return 0; */
/* } */

static
int odmetry_position(double position[3], int recet, bool adjust_flag, bool adjust_xyz[3], double adjust_data[3], bool cons_destination, MovingDestination_t destination){
  static bool matrix_init = false;
  const int encoder_ppr = (2048)*4;
  const int tire_diameter = 48;
  static double cal_matrix[3][4];
  static double recent_position[3] = {0.0, 0.0, 0.0};
  double temp_position[3],return_position[3],return_position_val[3],recent_position_rad;//encoder_diff[4];
  int32_t encoder_diff[4];
  int i,j;
  int32_t bug_duty = 0;
  static bool encoder_bug_right = false;
  static bool encoder_bug_back = false;
  static bool encoder_bug_left = false;
  static bool encoder_bug_front = false;
  static bool encoder_bug = false;
  bool testes;

  if(recet==1){
    for(i=0;i<3;i++){
      position[i] = 0.0;
      recent_position[i] = 0.0;
    }
    return 0;
  }

  if(adjust_flag){
    for(i=0; i<3; i++){
      if(adjust_xyz[i]){
	position[i] = adjust_data[i];
	if(i==0 || i==1){
	  recent_position[i] = adjust_data[i] * (((double)(encoder_ppr))/((double)(tire_diameter)*M_PI));
	}else{
	  recent_position[i] = adjust_data[i];
	}
      }
    }
    return 0;
  }
  
  if(!matrix_init){
    /* cal_matrix[0][0] = -0.05581555600218208963794; */
    /* cal_matrix[0][1] = -0.54530707169312928881104; */
    /* cal_matrix[0][2] =  0.05581555600218208963794; */
    /* cal_matrix[0][3] =  0.45469292830687071188951; */
    
    /* cal_matrix[1][0] = -0.50000000000000000000000; */
    /* cal_matrix[1][1] =  0.00000000000000000000000; */
    /* cal_matrix[1][2] = -0.00000000000000000000000; */
    /* cal_matrix[1][3] =  0.00000000000000000000000; */

    /* cal_matrix[2][0] =  0.30140400241178328404490; */
    /* cal_matrix[2][1] =  0.24465818714289815957966; */
    /* cal_matrix[2][2] = -0.30140400241178328404490; */
    /* cal_matrix[2][3] =  0.24465818714289815957966; */

    /* cal_matrix[0][0] = -0.055815556002182089637945390335639840362915960837233; */
    /* cal_matrix[0][1] = -0.5453070716931292888110482643773866605415027706796; */
    /* cal_matrix[0][2] =  0.055815556002182089637945390335639840362915960837233; */
    /* cal_matrix[0][3] =  0.4546929283068707118895173562261333945849722932039; */

    cal_matrix[0][0] = -0.0558155560021820896379453903356398403629159608372;
    cal_matrix[0][1] = -0.5453070716931292888110482643773866605415027706796;
    cal_matrix[0][2] =  0.0558155560021820896379453903356398403629159608372;
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

  if(g_md_h[L_B_KUDO_MD].duty==0 && g_md_h[R_F_KUDO_MD].duty==0){
    bug_duty = 0;
  }else{
    bug_duty = (g_md_h[L_B_KUDO_MD].duty+g_md_h[R_F_KUDO_MD].duty) / 2.0;
  }
  
  if(!cons_destination){
    for(i=0;i<4;i++){
      encoder_diff[i] = I2C_Encoder(i,GET_DIFF, bug_duty, &encoder_bug);
    }
    if(encoder_bug){
      for(i=0; i<4; i++){
	encoder_diff[i] = 0;
      }
      encoder_bug = false;
    }
  }else{
    switch(destination){
    case PLUS_Y:
    case MINUS_Y:
      encoder_diff[0] = I2C_Encoder(0,GET_DIFF, bug_duty, &encoder_bug_right);
      encoder_diff[1] = I2C_Encoder(1,GET_DIFF, 0, &encoder_bug_back);
      encoder_diff[2] = I2C_Encoder(2,GET_DIFF, bug_duty, &encoder_bug_left);
      encoder_diff[3] = I2C_Encoder(3,GET_DIFF, 0, &encoder_bug_front);
      if(encoder_bug_back || encoder_bug_front){
	encoder_diff[1] = 0;
	encoder_diff[3] = 0;
      }
      if(encoder_bug_right && !encoder_bug_left){
	encoder_diff[0] = encoder_diff[2];
      }else if(!encoder_bug_right && encoder_bug_left){
	encoder_diff[2] = encoder_diff[0];
      }else if(encoder_bug_right && encoder_bug_left){
	encoder_diff[0] = 0;
	encoder_diff[2] = 0;
      }
      if(encoder_bug_back) I2C_Encoder(BACK_ENC, RESET_ENCODER_VALUE, 0, &testes);
      if(encoder_bug_front) I2C_Encoder(FRONT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      if(encoder_bug_right) I2C_Encoder(RIGHT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      if(encoder_bug_left) I2C_Encoder(LEFT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      
      break;
    case PLUS_X:
    case MINUS_X:
      encoder_diff[0] = I2C_Encoder(0,GET_DIFF, 0, &encoder_bug_right);
      encoder_diff[1] = I2C_Encoder(1,GET_DIFF, bug_duty, &encoder_bug_back);
      encoder_diff[2] = I2C_Encoder(2,GET_DIFF, 0, &encoder_bug_left);
      encoder_diff[3] = I2C_Encoder(3,GET_DIFF, bug_duty, &encoder_bug_front);
      if(encoder_bug_left || encoder_bug_right){
	encoder_diff[0] = 0;
	encoder_diff[2] = 0;
      }
      if(encoder_bug_front && !encoder_bug_back){
	encoder_diff[3] = -encoder_diff[1];
      }else if(!encoder_bug_front && encoder_bug_back){
	encoder_diff[1] = -encoder_diff[3];
      }else if(encoder_bug_front && encoder_bug_back){
	encoder_diff[1] = 0;
	encoder_diff[3] = 0;
      }
      if(encoder_bug_back) I2C_Encoder(BACK_ENC, RESET_ENCODER_VALUE, 0, &testes);
      if(encoder_bug_front) I2C_Encoder(FRONT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      if(encoder_bug_right) I2C_Encoder(RIGHT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      if(encoder_bug_left) I2C_Encoder(LEFT_ENC, RESET_ENCODER_VALUE, 0, &testes);
      
      break;
    }
  }

  for(i=0;i<3;i++){
    temp_position[i] = 0.0;
    for(j=0;j<4;j++){
      temp_position[i] += cal_matrix[i][j] * (double)(encoder_diff[j]);
    }
    //return_position[i] = recent_position[i] + temp_position[i];
  }
  return_position[2] = recent_position[2] + temp_position[2];
  
  /**********/
  recent_position_rad = recent_position[2] * (1.0/55296.0) * (M_PI);
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
  if(fabs(return_position_val[2]) >= 180.0000000000000000000){
    if(return_position_val[2] >= 0.0){
      return_position_val[2] -= 360.000;
    }else{
      return_position_val[2] += 360.000;
    }
  }
  fill_array(position, return_position_val, 3);

  if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
    MW_printf("now_position[x][y][w] : [%10d][%10d][%10d]\n",(int)position[0],(int)position[1],(int)(position[2]*100.0));
  }
  
  return 0;
}

static
NowPosition_t get_deg_dis(MovingDestination_t mode, double get_x[MOVE_SAMPLE_VALUE], double get_y[MOVE_SAMPLE_VALUE], double get_zahyou_1[2], double get_zahyou_2[2], double *return_degree, double *return_distance){
  double x[MOVE_SAMPLE_VALUE],y[MOVE_SAMPLE_VALUE],temp_data[MOVE_SAMPLE_VALUE];
  double zahyou_1[2],zahyou_2[2],temp_zahyou;
  static NowPosition_t position;
  double ave_x=0.0, ave_y=0.0, ave_xx=0.0, ave_xy=0.0;
  bool x_move_flag = false,y_move_flag = false;
  int i;
  double a,b,c,a_dis,b_dis,c_dis,d,e,f; //ax+by+c=0 dx+ey+f=0
  double degree,distance,destination;

  fill_array(x,get_x,MOVE_SAMPLE_VALUE);
  fill_array(y,get_y,MOVE_SAMPLE_VALUE);
  fill_array(zahyou_1,get_zahyou_1,2);
  fill_array(zahyou_2,get_zahyou_2,2);
  
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
    /* a_dis = -(get_zahyou_2[1]-get_zahyou_1[1])/(get_zahyou_2[0]-get_zahyou_1[0]); */
    /* b_dis = 1.0; */
    /* c_dis = -(-a_dis*get_zahyou_1[0] + get_zahyou_1[1]); */
    distance = fabs(get_zahyou_1[1] - get_y[0]); 
  }else{
    a_dis = 1.0;
    b_dis = 0.0;
    c_dis = -get_zahyou_1[0];
    distance = fabs(-a_dis*get_x[0] + b_dis*get_y[0] - c_dis) / sqrt(a_dis*a_dis+b_dis*b_dis);
  }
  
  if(!x_move_flag && !y_move_flag){
    position = NOW_POSITION_STOP;
    *return_degree = 0.0;
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
int I2C_Encoder(int encoder_num, EncoderOperation_t operation, int duty, bool *encoder_bug){
  int32_t value=0,temp_value=0,diff=0;
  static int32_t adjust[4] = {0,0,0,0};
  static int32_t recent_value[4] = {};
  static int32_t recent_diff[4] = {};
  static int32_t recent_time[4] = {};
  static int message_count = 0;
  int i;
  int bug_compare_value;
  
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

  if(operation == GET_DIFF){
    if(duty >= 1500){
      bug_compare_value = (int)( (((double)(duty)/6.5/*6.0*/)*(double)(g_SY_system_counter - recent_time[encoder_num])) );
    }else{
      bug_compare_value = 300*(g_SY_system_counter - recent_time[encoder_num]);
    }
    if(abs(diff) == 0){
      recent_value[encoder_num] = value;
    }else if(abs(diff) > bug_compare_value){//&& !first_flag){
      diff = 0;//recent_diff[encoder_num];
      //recent_diff[encoder_num] = diff;
      recent_value[encoder_num] = 0;
      for(i=0; i<8; i++){
	g_ld_h[0].mode[i] = D_LMOD_BLINK_BLUE;
      }
      encoder_bug = true;
    }else{
      recent_diff[encoder_num] = diff;
      recent_value[encoder_num] = value;
    }
  }else{
    recent_value[encoder_num] = value;
  }

  //if(first_flag) first_flag = false;
  recent_time[encoder_num] = g_SY_system_counter;
  if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
    if(operation != RESET_ENCODER_VALUE){
      if(message_count >= 3){
  	MW_printf("<E0>[%10d] <E1>[%10d] <E2>[%10d] <E3>[%10d] bug[%d]\n",recent_value[0],recent_value[1],recent_value[2],recent_value[3],bug_compare_value);
  	message_count = 0;
      }else{
  	message_count++;
      }
    }
  }
  if(operation == GET_DIFF){
    if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
      MW_printf("<E%1d_diff>[%d][%d]\n",encoder_num,diff,value);
    }
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
int sus_motor_stop(void){
  
  g_md_h[R_F_KUDO_MD].duty = 0; 
  g_md_h[R_F_KUDO_MD].mode = D_MMOD_BRAKE;
  g_md_h[L_B_KUDO_MD].duty = 0; 
  g_md_h[L_B_KUDO_MD].mode = D_MMOD_BRAKE;
  
  return 0;
}

static
void fill_array(double array1[], double array2[], int size){
  int i;
  for(i=0;i<size;i++){
    array1[i] = array2[i];
  }
}

