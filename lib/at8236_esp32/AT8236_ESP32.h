/******************************************************************************

******************************************************************************/

#include <Arduino.h>
#include <driver/ledc.h>
//used in some functions so you don't have to send a speed
#define DEFAULTSPEED 255



class Motor_8236
{
  public:
    // Constructor. Mainly sets up pins.
    Motor_8236(int In1pin, int In2pin, int offset,int freq, int resolution, int channel_pin);

    // Drive in direction given by sign, at speed given by magnitude of the
	//parameter.
    void drive(int speed);

	// drive(), but with a delay(duration)
    void drive(int speed, int duration);

	//currently not implemented
    //void stop();           // Stop motors, but allow them to coast to a halt.
    //void coast();          // Stop motors, but allow them to coast to a halt.

	//Stops motor by setting both input pins high
    void brake();

	//set the chip to standby mode.  The drive function takes it out of standby
	//(forward, back, left, and right all call drive)
	void standby();

  private:
    //variables for the 2 inputs, PWM input, Offset value, and the Standby pin
	int In1, In2, Offset,Standby,Channel;

	//private functions that spin the motor CC and CCW
	void fwd(int speed);
	void rev(int speed);


};
