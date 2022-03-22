#include "pid.h"
#include "myBlueTooth.h"
#include "moto.h"

/*************蓝牙参数配置**********************/
#define Master 0    //主从机模式选择 1主机 0从机（未使用到）
void Bluetooth_Event(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);  //蓝牙事件回调函数
void btReceive(int rec);  //蓝牙接收信息处理
uint8_t address[6]={0x78,0x21,0x84,0x7D,0xA6,0xEA}; //从机MAC地址 不同的蓝牙地址不同 需要自己修改（未使用到）

char recBuffBT_M[100];
int i_buff_M=0;
int i_rec=0;
char receive_A1[20];

uint16_t control_flag=0;
/***********控制状态宏定义*************/
#define STOP        0x00
#define START       0x01
#define LEFT        0x02
#define RIGHT       0x03
#define SPEEDUP     0x04
#define SPEEDDOWN   0x05
#define FORWARD     B00100000
#define BACK        0x07
#define BALLANCE    B10000000
#define CHANGE      B01000000
#define NEXTSTAT    0xff 
/**********************************/

/*************PID参数定义***********/
PIDs pid_ang, pid_spdL, pid_spdR;

PID_Param pid_ang_param = {300.0, 6.0, 280.0, 200};
PID_Param pid_spd_paramL = {20.0, 5.0, 0.0, 100.0};
PID_Param pid_spd_paramR = {20.0, 5.0, 0.0, 100.0};



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialBT.register_callback(Bluetooth_Event); //设置事件回调函数 连接 断开 发送 接收
  SerialBT.begin("Leansbot"); //Bluetooth device name    
  Serial.println("The device started, now you can pair it with bluetooth!");


  
}

void loop() {
  // put your main code here, to run repeatedly:
  

}





/*******************    蓝牙事件回调函数   ******************/

void Bluetooth_Event(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)  //蓝牙事件回调函数
{
    if(event == ESP_SPP_OPEN_EVT || event == ESP_SPP_SRV_OPEN_EVT) //蓝牙连接成功标志 
    {                                                              //蓝牙主机和从机模式对应的标志不同，前面的是主机模式的，后面是从机模式
        sendStringBT("connection successful!\r\n");
    }
    else if(event == ESP_SPP_CLOSE_EVT)     //蓝牙断开连接标志
    {
        sendStringBT("disconnect successful!\r\n");
    }
    else if(event == ESP_SPP_DATA_IND_EVT)  //数据接收标志
    {
        while(SerialBT.available())
        {
            recBuffBT_M[i_buff_M]=SerialBT.read();

            switch(recBuffBT_M[i_buff_M]){
                  case 'T': control_flag=STOP;break; //停止
                  case 'F': control_flag=FORWARD;break;
                  case 'B': control_flag=BACK;break;
                  case 'M': control_flag=BALLANCE;break;
                  default:break;
                }

            /*角度环PID蓝牙调参
            if((i_buff_M>=7)&&((recBuffBT_M[i_buff_M-7]=='P')||(recBuffBT_M[i_buff_M-7]=='I')||(recBuffBT_M[i_buff_M-7]=='D'))){
              switch(recBuffBT_M[i_buff_M-7]){    //pid参数传递,"P1000.00"
                case 'P':
                  ANG_P_DATA = (float)((recBuffBT_M[i_buff_M-6]-'0')*1000.0+(recBuffBT_M[i_buff_M-5]-'0')*100.0+(recBuffBT_M[i_buff_M-4]-'0')*10.0+(recBuffBT_M[i_buff_M-3]-'0')+(recBuffBT_M[i_buff_M-1]-'0')*0.1+(recBuffBT_M[i_buff_M]-'0')*0.01);
                  break;
                case 'I':
                  ANG_I_DATA = (float)((recBuffBT_M[i_buff_M-6]-'0')*1000.0+(recBuffBT_M[i_buff_M-5]-'0')*100.0+(recBuffBT_M[i_buff_M-4]-'0')*10.0+(recBuffBT_M[i_buff_M-3]-'0')+(recBuffBT_M[i_buff_M-1]-'0')*0.1+(recBuffBT_M[i_buff_M]-'0')*0.01);
                  break;
                case 'D':
                  ANG_D_DATA = (float)((recBuffBT_M[i_buff_M-6]-'0')*1000.0+(recBuffBT_M[i_buff_M-5]-'0')*100.0+(recBuffBT_M[i_buff_M-4]-'0')*10.0+(recBuffBT_M[i_buff_M-3]-'0')+(recBuffBT_M[i_buff_M-1]-'0')*0.1+(recBuffBT_M[i_buff_M]-'0')*0.01);
                  break;
              }
              */

                //速度环PID调参
            if((i_buff_M>=7)&&((recBuffBT_M[i_buff_M-7]=='P')||(recBuffBT_M[i_buff_M-7]=='I')||(recBuffBT_M[i_buff_M-7]=='D')||(recBuffBT_M[i_buff_M-7]=='S'))){
              switch(recBuffBT_M[i_buff_M-7]){    //pid参数传递,"P1000.00"
                case 'P':
                  SPD_P_DATA = (float)((recBuffBT_M[i_buff_M-6]-'0')*1000.0+(recBuffBT_M[i_buff_M-5]-'0')*100.0+(recBuffBT_M[i_buff_M-4]-'0')*10.0+(recBuffBT_M[i_buff_M-3]-'0')+(recBuffBT_M[i_buff_M-1]-'0')*0.1+(recBuffBT_M[i_buff_M]-'0')*0.01);
                  break;
                case 'I':
                  SPD_I_DATA = (float)((recBuffBT_M[i_buff_M-6]-'0')*1000.0+(recBuffBT_M[i_buff_M-5]-'0')*100.0+(recBuffBT_M[i_buff_M-4]-'0')*10.0+(recBuffBT_M[i_buff_M-3]-'0')+(recBuffBT_M[i_buff_M-1]-'0')*0.1+(recBuffBT_M[i_buff_M]-'0')*0.01);
                  break;
                case 'D':
                  SPD_D_DATA = (float)((recBuffBT_M[i_buff_M-6]-'0')*1000.0+(recBuffBT_M[i_buff_M-5]-'0')*100.0+(recBuffBT_M[i_buff_M-4]-'0')*10.0+(recBuffBT_M[i_buff_M-3]-'0')+(recBuffBT_M[i_buff_M-1]-'0')*0.1+(recBuffBT_M[i_buff_M]-'0')*0.01);
                  break;
                case 'S':
                  TARGET_SPEED = (float)((recBuffBT_M[i_buff_M-6]-'0')*1000.0+(recBuffBT_M[i_buff_M-5]-'0')*100.0+(recBuffBT_M[i_buff_M-4]-'0')*10.0+(recBuffBT_M[i_buff_M-3]-'0')+(recBuffBT_M[i_buff_M-1]-'0')*0.1+(recBuffBT_M[i_buff_M]-'0')*0.01);
                  control_flag|=FORWARD;
                  break;
              }

              sendStringBT("Set successful!\r\n");
              sendStringBT(recBuffBT_M);
              sendStringBT("\r\n");

              i_buff_M=0;
              control_flag|=CHANGE;
            }
          else ++i_buff_M;
        }
    }
    else if(event == ESP_SPP_WRITE_EVT)     //数据发送标志
    {
        Serial.write("  send complete! \r\n");
    }
}
