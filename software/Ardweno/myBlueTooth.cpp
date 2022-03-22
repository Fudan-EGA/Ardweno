#include "myBlueTooth.h"

BluetoothSerial SerialBT;


/*蓝牙发送浮点数据，转为字符串发送*/
void sendFloatBT(float data)
{
  int i;
  char dstr[20];
  sprintf(dstr, "%.2f", data);  //浮点数转字符串
  for(i=0;i<strlen(dstr);i++)
  {  
    SerialBT.write(dstr[i]);
  }
}

/*蓝牙发送字符串*/
void sendStringBT(char *sdata) 
{
  int i;
  for(i=0;i<strlen(sdata);i++)
  {  
    SerialBT.write(sdata[i]);
  }
}
