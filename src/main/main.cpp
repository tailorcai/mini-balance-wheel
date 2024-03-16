#include <Arduino.h>

#include <WiFi.h>
#include <String>
#include "flex_log.h"
Flex_Log& _logger = Flex_Log::instance();

#include <ESP32Encoder.h> // https://github.com/madhephaestus/ESP32Encoder.git 
// #include "mcpwm_motor.h"    // for 9910
#include <TB6612_ESP32.h> // for 6612

#include "mpu6050_dmp.h"
#include "common.h"

#include <SPI.h>
#include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
// //BEGIN DISPLAY==================
// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels
// // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// #define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// void setup_display() {
//   if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
//     Serial.println(F("SSD1306 allocation failed"));
//     for(;;); // Don't proceed, loop forever
//   }
// }

// void drawStr(int line, const char* name, int t) {
//   char buf[100];

//   display.setCursor(0, line*10);
//   display.write( name );
//   display.setCursor(50,line*10);
//   if (t) display.write( t>0?"+":"-" );
//   display.setCursor(70,line*10);
//   display.write( itoa(t,buf,10) );
// }
//END
// McpwmMotor  motorsCtrl;

Motor motorL(13,14,15,1,2,5000,8,1,  0);
Motor motorR(26,27,5,1,2,5000,8,2,  5);
ESP32Encoder encoderL, encoderR;



#define LED_PIN       2
int8_t PIN_ENCODER_L[] = {25,32};
int8_t PIN_ENCODER_R[] = {35,34};
// int8_t PIN_MOTOR_L[] = {13,14};  // #12 is special pin
// int8_t PIN_MOTOR_R[] = {27,26};

//BEGIN-BUTTON
// #include <OneButton.h>
// #define BUTTON_PIN 21

// OneButton btn = OneButton(
//   BUTTON_PIN,  // Input pin for the button
//   true,        // Button is active LOW
//   true         // Enable internal pull-up resistor
// );

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
volatile u_long mpuIntTimestamp = 0;
void dmpDataReady() {
    mpuInterrupt = true;
    mpuIntTimestamp = micros();
}


int MotorRun = 1;
void handleClick() {
  MotorRun = 1 - MotorRun;
  Serial.println("Button clicked");
}

TaskHandle_t th_p[1];

void cmdCallback(void*cmd) {
    Serial.print( "cmd: ");
    Serial.println( (const char*) cmd );
}
void HostTask(void *args) {
    _logger.run( cmdCallback );
}


MPU6050_Entity  entity_MPU6050;

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
  // btn.attachClick(handleClick);
  // pinMode( BUTTON_PIN, INPUT_PULLUP);

  // setup_display();

  encoderL.attachFullQuad( PIN_ENCODER_L[0], PIN_ENCODER_L[1]);
  encoderL.setCount(0);

  encoderR.attachFullQuad( PIN_ENCODER_R[0], PIN_ENCODER_R[1]);
  encoderR.setCount(0);

  // motorsCtrl.attachMotor(0, PIN_MOTOR_L[0], PIN_MOTOR_L[1]);
  // motorsCtrl.attachMotor(1, PIN_MOTOR_R[0], PIN_MOTOR_R[1]);

  pinMode(INTERRUPT_PIN, INPUT);
  entity_MPU6050.initialize( INTERRUPT_PIN , dmpDataReady );
  // motorsCtrl.updateMotorSpeed( 0, 0 );
  // motorsCtrl.updateMotorSpeed( 1, 0 );



    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

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
函数功能：直立PD控制
入口参数：角度、角速度
返回  值：直立控制PWM
作    者：平衡小车之家
**************************************************************************/
#define ZHONGZHI -1
int balance_vertical(float Angle,float Gyro)
{  
   float Bias,kp=100,kd=0.4;    // 100,0.4
	 int balance;
	 Bias=Angle-ZHONGZHI;       //===求出平衡的角度中值 和机械相关
	 balance=kp*Bias+Gyro*kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}

/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修Target_Velocity，比如，改成60就比较慢了
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
作    者：平衡小车之家
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
	  static float Encoder_Integral=0;
	  float kp=50,ki=kp/200;
	  //=============速度PI控制器=======================//	
		Encoder_Least =(Encoder_Value_Left+Encoder_Value_Right)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
		Encoder *= 0.7;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.3;	                                    //===一阶低通滤波器    
    // #if 0
		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>15000)  	Encoder_Integral=15000;             //===积分限幅
		if(Encoder_Integral<-15000) 	Encoder_Integral=-15000;            //===积分限幅	
    // #endif
		Velocity=Encoder*kp+Encoder_Integral*ki;                          //===速度控制	
		return Velocity;
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
void rpt_proc() {
  static float angle, gyro;
  if( angle == fRound(entity_MPU6050.Angle_Balance) && gyro == fRound(entity_MPU6050.Gyro_Balance)) 
    return;
  char buf[200];
  sprintf(buf, "chs: %4.2f,%4.2f,%d\n",entity_MPU6050.Angle_Balance,entity_MPU6050.Gyro_Balance,Balance_Pwm);
  _logger.log(buf);
  angle = fRound(entity_MPU6050.Angle_Balance);
  gyro = fRound(entity_MPU6050.Gyro_Balance);
}

void balance_main() {
  static bool do_it = false;
  entity_MPU6050.get_Angle();
  // do_it = !do_it;
  // if( !do_it ) {
  //   return;
  // }
  
  // reader encoder
  Encoder_Value_Left = -encoder_read( encoderL );
  Encoder_Value_Right = encoder_read( encoderR );

  // Serial.println("Encoders");
  // Serial.print( Encoder_Value_Left );Serial.print( '\t' );Serial.println( Encoder_Value_Right );

  Balance_Pwm =balance_vertical(entity_MPU6050.Angle_Balance, entity_MPU6050.Gyro_Balance);                   //===平衡PID控制	
		  Velocity_Pwm=0;// velocity(Encoder_Value_Left,Encoder_Value_Right);                  //===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
 	    Moto1_PWM=Balance_Pwm+Velocity_Pwm;                                     //===计算左轮电机最终PWM
 	  	Moto2_PWM=Balance_Pwm+Velocity_Pwm;                                     //===计算右轮电机最终PWM
   		Moto1_PWM = Xianfu_Pwm(Moto1_PWM);                                                       //===PWM限幅
      Moto2_PWM = Xianfu_Pwm(Moto2_PWM);
      if(Turn_Off(entity_MPU6050.Angle_Balance)==0)                                      //===如果不存在异常
 			  Set_Pwm(Moto1_PWM,Moto2_PWM);                                               //===赋值给PWM寄存器  
      else {
        motorR.brake();
        motorL.brake();
      }

  rpt_proc();
}

// void oled_display() {
//   display.clearDisplay();
//   display.setTextSize(1);      // Normal 1:1 pixel scale
//   display.setTextColor(WHITE); // Draw white text
//   display.setCursor(0, 0);     // Start at top-left corner
//   display.cp437(true);  

//   display.setCursor(20,0);
//   display.write("Balanced Wheel");
 
//   display.setTextColor(WHITE); // Draw white text

//   drawStr(1, "Left_V:", Encoder_Value_Left); 		
//   drawStr(2, "Right_V:", Encoder_Value_Right); 		
//   drawStr(3, "PWM      :", Moto1_PWM); 	
//   drawStr(4, "B_PWM      :", Balance_Pwm); 	
//   drawStr(5, "Angle_B:", Angle_Balance);
//   drawStr(6, "Gyro_B:", Gyro_Balance);
//   display.display();
// }

void loop() {
  // btn.tick();
  static int _cnt = 0;
  _cnt = (_cnt+1) % 10 ;

  // if programming failed, don't try to do anything
    if (!entity_MPU6050.dmpReady) return;
    if ( !mpuInterrupt) return;
    mpuInterrupt = false;
    // Serial.println( mpuIntTimestamp );

  // put your main code here, to run repeatedly:
  // u_long t = micros();
  balance_main();
  // u_long dt = micros() - t;
  // Serial.print("balance_main cost:");
  // Serial.println( dt );

  Serial.print(".");
  if( _cnt == 1) {
    Serial.println(".");

    // u_long t = micros();
    // oled_display();
    // u_long dt = micros() - t;
    // Serial.print("oled_display cost:");
    // Serial.println( dt );
  }
}

