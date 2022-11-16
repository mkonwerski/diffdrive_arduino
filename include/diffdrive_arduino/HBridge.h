#ifndef HBRIDGE_H
#define HBRIDGE_H

#include <pigpiod_if2.h>

class HBridge
{
private:
  int pi_; // pigpio id

  // INPUT pin numbers for motor driver
  int pin_in1_;
  int pin_in2_;
  int pin_pwm_;

public:
  HBridge()
  { }

  ~HBridge()
  {
    set_mode(pi_, pin_in1_, PI_INPUT);
    set_mode(pi_, pin_in2_, PI_INPUT);
    if (pin_pwm_ >= 0)
    {
      set_mode(pi_, pin_pwm_, PI_INPUT);
    }
  }

  void setPin(int pi, int pin_in1, int pin_in2)
  {
    pi_ = pi;
    pin_in1_ = pin_in1;
    pin_in2_ = pin_in2;

    set_mode(pi_, pin_in1_, PI_OUTPUT);
    set_mode(pi_, pin_in2_, PI_OUTPUT);
  }

  void drive(int power) const
  {
    if (power > 0)
    {
      set_PWM_dutycycle(pi_, pin_in1_, power);
      gpio_write(pi_, pin_in2_, 0);
    }
    else
    {
      power = -power;
      gpio_write(pi_, pin_in1_, 0);
      set_PWM_dutycycle(pi_, pin_in2_, power);
    }
  }
};

#endif