#ifndef MYBLUETOOTH_H_
#define MYBLUETOOTH_H_

#include <Arduino.h>
#include <String.h>
#include <BluetoothSerial.h>

extern BluetoothSerial SerialBT;

/*蓝牙发送浮点数据，转为字符串发送*/
void sendFloatBT(float data);

/*蓝牙发送字符串*/
void sendStringBT(char *sdata);


#endif /* MYBLUETOOTH_H_ */
