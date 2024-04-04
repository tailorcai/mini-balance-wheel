class MotorWithAdj : public Motor {
  int percentAdjust;
  public:
    MotorWithAdj(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin, int freq, int resolution, int channel_pin, int perAdj):
      Motor(In1pin,In2pin,PWMpin,offset,STBYpin,freq,resolution,channel_pin ), percentAdjust(perAdj) {}
    void drive(int speed) {
      speed = max( -255, min( 255, int( round( speed * (1.0+percentAdjust)))));
      Motor::drive( speed );
    }
};