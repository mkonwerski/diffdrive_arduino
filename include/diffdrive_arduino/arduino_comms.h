#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

#include <cstring>
#include <string>

class ArduinoComms
{

public:

  ArduinoComms()
  {  }

  void setup(); //done
  void sendEmptyMsg(); //done
  void readEncoderValues(int val_1, int val_2); //done
  void setMotorValues(int val_1, int val_2); //done
  void setPidValues(float k_p, float k_d, float k_i, float k_o);

  bool connected() const { return 1; }

  void sendMsg(int val_1, int chanel, int value); //done


};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H