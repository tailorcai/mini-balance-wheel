class MotorWithAdj : public Motor_8236 {
  int percentAdjust;
  public:
    MotorWithAdj(int In1pin, int In2pin, int offset, int freq, int resolution, int channel_pin, int perAdj):
      Motor_8236(In1pin,In2pin,offset,freq,resolution,channel_pin ), percentAdjust(perAdj) {}
    void drive(int speed) {
      speed = max( -255, min( 255, int( round( speed * (1.0+percentAdjust)))));
      Motor_8236::drive( speed );
    }
};