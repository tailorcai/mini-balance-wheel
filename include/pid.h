#pragma once
struct B_PID {
  float kp;// = 300;
  float kd;// = 1.0;
  float ZHONGZHI; //=0
  B_PID(float _kp, float _ki, float _c): kp(_kp), kd(_ki), ZHONGZHI(_c) {}
  void reset() {}
  int vertical(float Angle,float Gyro){
    float Bias;
	  int balance;
	  Bias=Angle-ZHONGZHI;              //===求出平衡的角度中值 和机械相关
	  balance=kp*Bias+Gyro*kd;    //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	  return balance;
  }
};

struct V_PID{
  float kp;// = 50;
  float ki;// = 50/200;
  float Encoder;
  float Movement;
  float Encoder_Integral;
  float Speed;

  V_PID(float _kp, float _ki): kp(_kp), ki(_ki),Encoder(0),Encoder_Integral(0),Movement(0),Speed(0) {}
  void reset() { Encoder_Integral = 0; Encoder = 0; }
  int velocity(int encoder_left,int encoder_right)
  {  
      float Velocity,Encoder_Least;
      
      //=============速度PI控制器=======================//	
      Encoder_Least =(encoder_left+encoder_right)-Speed;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
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
};
