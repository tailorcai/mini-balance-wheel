#include <Arduino.h>

#include <WiFi.h>
#include <String>
#include "flex_log.h"
Flex_Log& _logger = Flex_Log::instance();

#include <ESP32Encoder.h> // https://github.com/madhephaestus/ESP32Encoder.git 
// #include "mcpwm_motor.h"    // for 9910
// #include <TB6612_ESP32.h> // for 6612
#include "at8236_esp32.h" 

#include "mpu6050_dmp.h"
#include "common.h"
#include "motor_adj.h"
#include "pid.h"

#include <SPI.h>
#include <Wire.h>


 // 管脚顺序要匹配PID
MotorWithAdj motorL(19,21,1,5000,8,1, 0);
MotorWithAdj motorR(23,22,1,5000,8,2, 0);
ESP32Encoder encoderL, encoderR;

#define LED_PIN       2
int8_t PIN_ENCODER_L[] = {17,16};
int8_t PIN_ENCODER_R[] = {2,4};

// BEGIN-BUTTON
#include <OneButton.h>
#define BUTTON_PIN 35

// 切换运行/暂停模式,或者由于倾斜角度过大，自动进入暂停模式
OneButton pauseBtn = OneButton(
  BUTTON_PIN,  // Input pin for the button
  true,        // Button is active LOW
  true         // Enable internal pull-up resistor
);

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
volatile u_long mpuIntTimestamp = 0;
void dmpDataReady() {
    mpuInterrupt = true;
    mpuIntTimestamp = micros();
}


int MotorRun = 0;
ERROR_TYPES error_mode;
void on_error(ERROR_TYPES mode) {
  MotorRun = 0;
  error_mode = mode;
  Serial.println("onerror: " + String(mode));
}

TaskHandle_t th_p[1];

MPU6050_Entity  entity_MPU6050;
V_PID v_pid(30,30.0/200);
B_PID b_pid(180,4.0,0.0);
Turn_PID turn_pid(0,0);


int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
uint8_t Flag_Target,yanshi_count,yanshi_flag;

// =========================
int Encoder_Value_Left,Encoder_Value_Right;             //左右编码器的脉冲计数
int Moto1_PWM,Moto2_PWM;                            //电机PWM变量 应是Motor的 向Moto致敬	

// =====================


int encoder_read( ESP32Encoder& encoder ) {
  int n = encoder.getCount();
  encoder.clearCount();
  return n;
}


/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
#define PWM_MAX 7200
void Set_Pwm(int moto1,int moto2)
{
  if( MotorRun ) {
    // motorsCtrl.updateMotorSpeed(0, moto1*100.0 / PWM_MAX ); 
    motorL.drive( moto1*255/PWM_MAX );
    // motorsCtrl.updateMotorSpeed(1, -moto2*100.0 / PWM_MAX ); 
    motorR.drive( moto2*255/PWM_MAX );
  }
  else {
    // motorsCtrl.updateMotorSpeed(0,0);
    // motorsCtrl.updateMotorSpeed(1,0);
    motorL.brake();
    motorR.brake();
  }
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：1：异常  0：正常
**************************************************************************/
uint8_t Turn_Off(float angle)
{
	    uint8_t temp;
			if(angle<-40||angle>40)
			{	                                                 //===倾角大于40度关闭电机
        temp=1;                                            //===Flag_Stop置1关闭电机
      }
			else
        temp=0;
      return temp;			
}

/**************************************************************************
函数功能：限制PWM赋值
入口参数：无
返回  值：无
**************************************************************************/
#define MIN_PWM 400

int Xianfu_Pwm(int pwm)
{	
	  int Amplitude=6900;    //===PWM满幅是7200 限制在6900
    if( pwm < 0 ) pwm -= MIN_PWM;
    if(pwm<-Amplitude) pwm=-Amplitude;
    if( pwm > 0) pwm +=MIN_PWM;
		if(pwm>Amplitude)  pwm=Amplitude;	
    return pwm;
}

float fRound(float x) {
  return round(x * 100)/100.0;
}

ERROR_TYPES balance_main() {
  static bool do_it = false;
  entity_MPU6050.get_Angle();
  // do_it = !do_it;
  // if( !do_it ) {
  //   return;
  // }
  
  // reader encoder
  Encoder_Value_Left = encoder_read( encoderL );
  Encoder_Value_Right = encoder_read( encoderR );

  // Serial.println("Encoders");
  // Serial.print( Encoder_Value_Left );Serial.print( '\t' );Serial.println( Encoder_Value_Right );

  Balance_Pwm =b_pid.vertical(entity_MPU6050.Angle_Balance, entity_MPU6050.Gyro_Balance);                   //===平衡PID控制	
		  Velocity_Pwm=0;// v_pid.velocity(Encoder_Value_Left,Encoder_Value_Right);                  //===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
      Turn_Pwm    = 0; //turn_pid.turn(Encoder_Value_Left,Encoder_Value_Right, entity_MPU6050.Gyro_Turn);
 	    Moto1_PWM=Balance_Pwm+Velocity_Pwm - Turn_Pwm;                                     //===计算左轮电机最终PWM
 	  	Moto2_PWM=Balance_Pwm+Velocity_Pwm + Turn_Pwm;                                     //===计算右轮电机最终PWM
   		Moto1_PWM = Xianfu_Pwm(Moto1_PWM);                                                       //===PWM限幅
      Moto2_PWM = Xianfu_Pwm(Moto2_PWM);
  if(Turn_Off(entity_MPU6050.Angle_Balance)==0) {                                     //===如果不存在异常
      Set_Pwm(Moto1_PWM,Moto2_PWM);       
      return NO_ERROR;
      }                                        //===赋值给PWM寄存器  
  else {
      // on_error( ERROR_FLOP );
      return ERROR_FLOP;
  }

}

void handlePauseClick() {
  MotorRun = 1 - MotorRun;
  if( !MotorRun )
    error_mode = ERROR_PAUSE;
  else {
    error_mode = NO_ERROR;
    v_pid.reset();
    b_pid.reset();
    // 需要清除编码器历史数据
    encoderL.clearCount();
    encoderR.clearCount();
  }
  Serial.println("PauseButton clicked");
}

void monitor_report() {
  // static float angle, gyro;
  // if( angle == fRound(entity_MPU6050.Angle_Balance) && gyro == fRound(entity_MPU6050.Gyro_Balance)) 
  //   return;
  char buf[200];
  sprintf(buf, "chs: %4.2f,%4.2f,%d,%d,%d,%d,%d\n",entity_MPU6050.Angle_Balance,entity_MPU6050.Gyro_Balance,Encoder_Value_Left, Encoder_Value_Right, Balance_Pwm, Velocity_Pwm,Turn_Pwm);
  _logger.log(buf);
  // angle = fRound(entity_MPU6050.Angle_Balance);
  // gyro = fRound(entity_MPU6050.Gyro_Balance);
}

void loop() {
  static int _cnt = 0;
  _cnt ++;

  pauseBtn.tick();

  if( !MotorRun ) {
    motorR.brake();
    motorL.brake();

    digitalWrite( LED_PIN, !digitalRead(LED_PIN));
    delay(200);
    return;
  } else {
    digitalWrite( LED_PIN, 0 );
  }

  // if programming failed, don't try to do anything
  if (!entity_MPU6050.dmpReady) {
    on_error( ERROR_6050 );
  };
  
  if ( mpuInterrupt ) {
    mpuInterrupt = false;
    ERROR_TYPES err;
    if( (err = balance_main()) == NO_ERROR ) 
      monitor_report();
    else 
      on_error( err );
  }
  else {
    delay(1);
  }
}

struct CMD_MAP{
  const char* name;
  Configurable* obj;
};
CMD_MAP cmds_table[] = { { "bpid", &b_pid },
                         { "vpid", &v_pid}, 
                         { "tpid", &turn_pid} };

void cmdCallback(void*cmd) {
  // Serial.print( "cmd: ");
  // Serial.println( (const char*) cmd );

  const char * scmd = (const char*)cmd;
  int nCmd = -1;
  for( int i=0;i<sizeof(cmds_table) / sizeof(CMD_MAP);i++) {
    const char* name = cmds_table[i].name;
    int len = strlen(name);
    if( strncmp( scmd , name,len) == 0 ) {
      nCmd = i;
      scmd += len + 1;
      bool r = cmds_table[i].obj->set_config( scmd );
      if( !r ) {
        _logger.debug( "Command error occurs");
      }
      else {
        _logger.debug( "Command success");
      }
      _logger.debug( cmds_table[i].obj->get_config() );
      return;
    }
  }

  _logger.debug("command invalid");

}
void HostTask(void *args) {
    _logger.run( cmdCallback );
}

void setup() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(115200);

  WiFi.begin("LINJIA","050208linjia");
  
  while( WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi begin ..");
    delay(200);
  }

  if( _logger.begin( host_ip )) {
    xTaskCreatePinnedToCore(HostTask, "HostTask", 4096, NULL, 3, &th_p[0], 0); 
  }

  pauseBtn.attachClick(handlePauseClick);
  pinMode( BUTTON_PIN, INPUT_PULLUP); // no need ? as the button class has done this?

  _logger.debug("clear encoder");
  // setup_display();

  encoderL.attachFullQuad( PIN_ENCODER_L[0], PIN_ENCODER_L[1]);
  encoderL.setCount(0);

  encoderR.attachFullQuad( PIN_ENCODER_R[0], PIN_ENCODER_R[1]);
  encoderR.setCount(0);

  pinMode(INTERRUPT_PIN, INPUT);
  _logger.debug("init MPU6050");
  entity_MPU6050.initialize( INTERRUPT_PIN , dmpDataReady );

    // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  on_error( ERROR_PAUSE );
}