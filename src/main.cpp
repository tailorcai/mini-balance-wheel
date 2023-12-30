#include <Arduino.h>

#include <ESP32Encoder.h> // https://github.com/madhephaestus/ESP32Encoder.git 
#include "mcpwm_motor.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//BEGIN DISPLAY==================
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup_display() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
}

void drawStr(int line, const char* name, int t) {
  char buf[100];

  display.setCursor(0, line*10);
  display.write( name );
  display.setCursor(50,line*10);
  if (t) display.write( t>0?"+":"-" );
  display.setCursor(70,line*10);
  display.write( itoa(t,buf,10) );
}
//END
McpwmMotor  motorsCtrl;
ESP32Encoder encoderL, encoderR;
//BEGIN==========MPU6050==================================================
  bool dmpReady = false;  // set true if DMP init was successful
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer

  // orientation/motion vars
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float euler[3];         // [psi, theta, phi]    Euler angle container
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

  // packet structure for InvenSense teapot demo
  uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
  MPU6050 mpu;
//END==========MPU6050==================================================

#define INTERRUPT_PIN 4
#define LED_PIN       2
int8_t PIN_ENCODER_L[] = {33,32};
int8_t PIN_ENCODER_R[] = {35,34};
int8_t PIN_MOTOR_L[] = {13,14};  // #12 is special pin
int8_t PIN_MOTOR_R[] = {27,26};

//BEGIN-BUTTON
#include <OneButton.h>

#define BUTTON_PIN 23
OneButton btn = OneButton(
  BUTTON_PIN,  // Input pin for the button
  true,        // Button is active LOW
  true         // Enable internal pull-up resistor
);

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

int MotorRun = 0;
void handleClick() {
  MotorRun = 1 - MotorRun;
  Serial.println("Button clicked");
}

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(115200);

  btn.attachClick(handleClick);
  // pinMode( BUTTON_PIN, INPUT_PULLUP);

  setup_display();

  encoderL.attachFullQuad( PIN_ENCODER_L[0], PIN_ENCODER_L[1]);
  encoderL.setCount(0);

  encoderR.attachFullQuad( PIN_ENCODER_R[0], PIN_ENCODER_R[1]);
  encoderR.setCount(0);

  motorsCtrl.attachMotor(0, PIN_MOTOR_L[0], PIN_MOTOR_L[1]);
  motorsCtrl.attachMotor(1, PIN_MOTOR_R[0], PIN_MOTOR_R[1]);

  mpu.initialize();
  motorsCtrl.updateMotorSpeed( 0, 0 );
  motorsCtrl.updateMotorSpeed( 1, 0 );

  pinMode(INTERRUPT_PIN, INPUT);
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    // mpu.setXGyroOffset(220);
    // mpu.setYGyroOffset(76);
    // mpu.setZGyroOffset(-85);
    // mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
uint8_t Flag_Target,yanshi_count,yanshi_flag;

// =========================
int Encoder_Value_Left,Encoder_Value_Right;             //左右编码器的脉冲计数
int Moto1_PWM,Moto2_PWM;                            //电机PWM变量 应是Motor的 向Moto致敬	
float Angle_Balance,Gyro_Balance;           //平衡倾角 平衡陀螺仪 转向陀螺仪
// =====================
void get_Angle() {
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        //#ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            // Serial.print("quat\t");
            // Serial.print(q.w);
            // Serial.print("\t");
            // Serial.print(q.x);
            // Serial.print("\t");
            // Serial.print(q.y);
            // Serial.print("\t");
            // Serial.println(q.z);

            // mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            // Serial.print("ypr\t");
            // Serial.print(ypr[0] * 180/M_PI);
            // Serial.print("\t");
            // Serial.print(ypr[1] * 180/M_PI);
            // Serial.print("\t");
            // Serial.println(ypr[2] * 180/M_PI);        

            Angle_Balance = ypr[1] * 180/M_PI;    
            VectorInt16 gyro;
            mpu.dmpGetGyro(&gyro, fifoBuffer);
            Gyro_Balance = gyro.y;

            // Serial.print( Angle_Balance );
            // Serial.print( "\t" );
            // Serial.println( Gyro_Balance );

      }

}

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
#define ZHONGZHI 1
int balance_vertical(float Angle,float Gyro)
{  
   float Bias,kp=100,kd=0.40;
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
    #if 0
		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>15000)  	Encoder_Integral=15000;             //===积分限幅
		if(Encoder_Integral<-15000) 	Encoder_Integral=-15000;            //===积分限幅	
    #endif
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
    motorsCtrl.updateMotorSpeed(0, moto1*100.0 / PWM_MAX ); 
    motorsCtrl.updateMotorSpeed(1, -moto2*100.0 / PWM_MAX ); 
  }
  else {
    motorsCtrl.updateMotorSpeed(0,0);
    motorsCtrl.updateMotorSpeed(1,0);
  }
}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
int Xianfu_Pwm(int pwm)
{	
	  int Amplitude=6900;    //===PWM满幅是7200 限制在6900
    if( pwm < 0 ) pwm -= 3000;
    if(pwm<-Amplitude) pwm=-Amplitude;
    if( pwm > 0) pwm +=3000;
		if(pwm>Amplitude)  pwm=Amplitude;	
    return pwm;
}

void balance_main() {
  static bool do_it = false;

  get_Angle();
  do_it = !do_it;
  if( !do_it ) {
    return;
  }
  // reader encoder
  Encoder_Value_Left = -encoder_read( encoderL );
  Encoder_Value_Right = encoder_read( encoderR );

  // Serial.println("Encoders");
  // Serial.print( Encoder_Value_Left );Serial.print( '\t' );Serial.println( Encoder_Value_Right );

  Balance_Pwm =balance_vertical(Angle_Balance,Gyro_Balance);                   //===平衡PID控制	
		  Velocity_Pwm=0;// velocity(Encoder_Value_Left,Encoder_Value_Right);                  //===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
 	    Moto1_PWM=Balance_Pwm+Velocity_Pwm;                                     //===计算左轮电机最终PWM
 	  	Moto2_PWM=Balance_Pwm+Velocity_Pwm;                                     //===计算右轮电机最终PWM
   		Moto1_PWM = Xianfu_Pwm(Moto1_PWM);                                                       //===PWM限幅
      Moto2_PWM = Xianfu_Pwm(Moto2_PWM);
      // if(Turn_Off(Angle_Balance)==0)                                      //===如果不存在异常
 			Set_Pwm(Moto1_PWM,Moto2_PWM);                                               //===赋值给PWM寄存器  
      // Serial.print("B_PWM:");Serial.println(Balance_Pwm);
      // Serial.print("Velocity_Pwm:");Serial.println(Velocity_Pwm);
}

void oled_display() {
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);  

  display.setCursor(20,0);
  display.write("Balanced Wheel");
 
  display.setTextColor(WHITE); // Draw white text

  drawStr(1, "Left_V:", Encoder_Value_Left); 		
  drawStr(2, "Right_V:", Encoder_Value_Right); 		
  drawStr(3, "PWM      :", Moto1_PWM); 	
  drawStr(4, "B_PWM      :", Balance_Pwm); 	
  drawStr(5, "Angle_B:", Angle_Balance);
  drawStr(6, "Gyro_B:", Gyro_Balance);
  display.display();
}
void loop() {
  btn.tick();
  static int _cnt = 0;
  _cnt = (_cnt+1) % 10 ;
  
  // if programming failed, don't try to do anything
    if (!dmpReady) return;
    if ( !mpuInterrupt) return;
    mpuInterrupt = false;

  // put your main code here, to run repeatedly:
  balance_main();

  if( _cnt == 1)
    oled_display();
}

