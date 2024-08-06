#pragma once

class Configurable {
public:
  const char** params;
  Configurable(const char**p):params(p) {}
  bool set_config(const char* input){
    int idx=0;
    for( const char** cmd = params; *cmd != NULL; cmd ++, idx++) {
      if( strncmp( input, *cmd, strlen(*cmd) ) == 0) {
        float v;
        input += strlen(*cmd)+1;
        if( 1 == sscanf( input, "%f", &v )) {  // skip a 'SPACE'
          return set_value( idx, v );
          // return true;
        }
        return false; 
      }
    }
    return false;
  }
  virtual String get_config()=0;
protected:
  virtual bool set_value( int idx , float) = 0;
};

// balance PID
const char* bpid_params[] = { "kp","kd","zz",NULL };

class B_PID : public Configurable {
public:
  float kp;// = 300;
  float kd;// = 1.0;
  float ZHONGZHI; //=0
  B_PID(float _kp, float _ki, float _c):Configurable(bpid_params), kp(_kp), kd(_ki), ZHONGZHI(_c) {}
  void reset() {}
  int vertical(float Angle,float Gyro){
    float Bias;
	  int balance;
	  Bias=Angle-ZHONGZHI;              //===求出平衡的角度中值 和机械相关
	  balance=kp*Bias+Gyro*kd;    //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	  return balance;
  }
  String get_config() {
    return "BPID: kp="+String(kp) + ",kd="+String(kd)+",zz="+String(ZHONGZHI);
  }

protected:
  bool set_value(int idx, float v) {
    switch(idx) {
      case 0: kp = v; return true;
      case 1: kd = v; return true;
      case 2: ZHONGZHI = v; return true;
    }
    return false;
  }

};

// velocity PID with speed
const char* vpid_params[] = { "kp","ki","move",NULL };
class V_PID : public Configurable{
public:
  float kp;// = 50;
  float ki;// = 50/200;
  float Encoder;
  float Encoder_Integral;
  float Movement;

  V_PID(float _kp, float _ki): Configurable(vpid_params), 
    kp(_kp), ki(_ki),Encoder(0),Encoder_Integral(0),Movement(0) {}
  void reset() { Encoder_Integral = 0; Encoder = 0; }
  int velocity(int encoder_left,int encoder_right)
  {  
      float Velocity,Encoder_Least;
      
      //=============速度PI控制器=======================//	
      Encoder_Least =(encoder_left+encoder_right)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
      Encoder *= 0.7;		                                                //===一阶低通滤波器       
      Encoder += Encoder_Least*0.3;	                                    //===一阶低通滤波器    
      // #if 0
      Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
      Encoder_Integral=Encoder_Integral - Movement;                       //===接收遥控器数据，控制前进后退
      if(Encoder_Integral>15000)  	Encoder_Integral=15000;             //===积分限幅
      if(Encoder_Integral<-15000) 	Encoder_Integral=-15000;            //===积分限幅	
      // #endif
      Velocity=Encoder*kp+Encoder_Integral*ki;                          //===速度控制	
      return Velocity;
  }
  String get_config() {
    return "VPID: kp="+ String(kp) + ",ki="+ki+",move="+Movement;
  }

protected:
  bool set_value( int idx , float v) {
    switch( idx ) {
      case 0: kp = v;return true;
    case 1:
      ki = v; return true;
    case 2:
      Movement = v;return true;
    }
    return false;
  }
};

const char* tpid_params[] = { "kp","kd","turn",NULL };
class Turn_PID : public Configurable{
public:
  int TurnMode; // 0 = no, 1 = left, -1 = right
  float Kp,Kd; // 42.0, 0
  int Turn_Count;
  float Turn_Target;
  float Encoder_temp;
  float Turn_Convert;
  Turn_PID(int kp, int kd) :Configurable(tpid_params), 
      TurnMode(0),Kp(kp), Kd(kd),Turn_Count(0),Turn_Target(0),Encoder_temp(0),Turn_Convert(0.7) {}
  int Turn_Amplitude() {
    return 50/1;
  }

  // Turn_Pwm    =turn(Encoder_Left,Encoder_Right,Gyro_Turn); 
  int turn(int encoder_left,int encoder_right,float gyro)//转向控制
  {
      float turn_Amplitude=Turn_Amplitude();
      //=============遥控左右旋转部分=======================//
      if(TurnMode)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
      {
        if(++Turn_Count==1)
          Encoder_temp=abs(encoder_left+encoder_right);
        Turn_Convert=50/Encoder_temp;
        if(Turn_Convert<0.4)Turn_Convert=0.4;
        if(Turn_Convert>1)Turn_Convert=1;
      }	
      else
      {
        Turn_Convert=0.7;
        Turn_Count=0;
        Encoder_temp=0;
      }		
      if(1==TurnMode)	           Turn_Target-=Turn_Convert;        //===接收转向遥控数据
      else if(-1==TurnMode)	     Turn_Target+=Turn_Convert;        //===接收转向遥控数据
      else Turn_Target=0;                                            //===接收转向遥控数据
      if(Turn_Target>turn_Amplitude)  Turn_Target=turn_Amplitude;    //===转向速度限幅
      if(Turn_Target<-turn_Amplitude) Turn_Target=-turn_Amplitude;   //===转向速度限幅

      // !!!TODO
      // if(Flag_Qian==1||Flag_Hou==1)  Kd=0.6;                         //===接收转向遥控数据直立行走的时候增加陀螺仪就纠正    
      // else Kd=0;                                   
      //=============转向PD控制器=======================//
      float Turn=Turn_Target*Kp+gyro*Kd;                 //===结合Z轴陀螺仪进行PD控制
      return Turn;
  }
  String get_config() {
    return "TPID: kp="+ String(Kp) + ",kd="+Kd+",turn="+TurnMode;
  }  
protected:
  bool set_value( int idx , float v) {
    switch(idx) {
    case 0:
      Kp = v; return true;
    case 1:
      Kd = v; return true;
    case 2:
      TurnMode = (int) v;
      return true;
    }  
    return false;
  }

};
