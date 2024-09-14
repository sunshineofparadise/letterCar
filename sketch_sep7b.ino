

//  AT8236带稳压模块 ------ 电机编码器
//      AO1        ------  电机-
//      5V         ------  编码器5V
//      E1B        ------  A相
//      E1A        ------  B相
//      GND        ------  GND
//      AO2        ------  电机+
//
//      BO1        ------  电机-
//      5V         ------  编码器5V
//      E2B        ------  A相
//      E2A        ------  B相
//      GND        ------  GND
//      BO2        ------  电机+

/*-------加载各种库文件-------*/
#include <Wire.h>            //I2C通信库，单片机与陀螺仪通信
// #include <SoftwareSerial.h>  //软串口通信库，单片机与蓝牙通信
#include <MPU6050_tockn.h>
#include <Arduino.h>
#include "esp_system.h"
#include "esp_task_wdt.h"
#include <Ticker.h>

/*-------预定义Arduino与各模块连接的引脚-------*/
#define TX_Pin 14          //定义蓝牙串口TX针脚A0
#define RX_Pin 15          //定义蓝牙串口RX针脚A1
#define OUT_Pin 32        //定义超声波ECHO针脚
#define TRIG_Pin A3        //定义超声波TRIG针脚
#define SDA_Pin A4         //定义陀螺仪SDA针脚库默认
#define SCL_Pin A5         //定义陀螺仪SCL针脚库默认
#define Left_AIN1 33        //定义左轮TB6612引脚A1
#define Left_AIN2 25        //定义左轮TB6612引脚A2
#define Left_EnCoderA 18    //定义左轮编码器A相引脚
#define Right_BIN1 26      //定义右轮TB6612引脚B1
#define Right_BIN2 27       //定义右轮TB6612引脚B2
#define Right_EnCoderA 23  //定义右轮编码器A相引脚
#define Right_EnCoderB 23  //定义右轮编码器B相引脚
#define Stby_Pin 7         //定义TB6612引脚STBY
#define LStar_PWM 5        //测试出左电机起转PWM补偿值
#define RStar_PWM 8        //测试出右电机起转PWM补偿值
#define Drive_ANG 1.2      //定义小车前进后退角度数值，数值越大冲的越快
#define Voltage 15 //模拟引脚读取电源电压

#define ZHONGZHI 0.9 // 小车的机械中值  DIFFERENCE
/////////编码器引脚////////
#define ENCODER_L 8  //编码器采集引脚 每路2个 共4个
#define DIRECTION_L 4
#define ENCODER_R 7
#define DIRECTION_R 2
float Balance_Kp = 29, Balance_Kd = 0.9, Velocity_Kp = 2, Velocity_Ki = 0.01;
int Flag_Qian, Flag_Hou, Flag_Left, Flag_Right; //遥控相关变量
int Balance_Pwm, Velocity_Pwm, Turn_Pwm;   //直立 速度 转向环的PWM
volatile long Velocity_L, Velocity_R = 0;   //左右轮编码器数据
int Velocity_Left, Velocity_Right = 0;     //左右轮速度
const int pwmFrequency = 2000;  // PWM 频率（单位 Hz）
const int pwmResolution = 8;   // 分辨率（0-255，对应 8 位）
const int Left_AIN1_Channel = 0;
const int Left_AIN2_Channel = 1;  // LEDC 通道 (ESP32 支持 0-15)
const int Right_BIN1_Channel = 2;
const int Right_BIN2_Channel = 3;

int LEDLight = 0;

int times = 0;
int Velocity_Count=0;

Ticker timer;

/*-------实例化蓝牙串口和陀螺仪模块-------*/
// SoftwareSerial Bluetooth(RX_Pin, TX_Pin);  //初始化软通信串口RX, TX,对应蓝牙串口对象
MPU6050 Mpu6050(Wire);                     //实例化一个MPU6050对象，对象名称为mpu6050

/*-------定义调试和预设变量-------*/
float Balance_ANG = 0.81;                         //测试出的静态机械平衡角度。
float ANG_Kp = 18, ANG_Ki = 0.9, ANG_Kd = 0.8;           //反复调试得到角度环的Kp Ki Kd的值
float SPD_Kp = 20 , SPD_Ki = 0.1;                          //反复调试得到速度环的Kp Ki Kd的值  通常SPD_Ki = SPD_Kp / 200
float Turn_Kp = 0.6, Turn_SPD;                             //转向环Kp值通常为0.6

/*-------定义平衡车控制变量-------*/
float ANG_DIF_Val, ANG_INTG_Val;  //平衡车需要保持的角度，存在的角度偏差，偏差积分变量
float AngleX, AngleY, GyroX, GyroZ, AccZ;                   //Mpu6050输出的角度值为浮点数，两位有效小数
int delayShow=0;
float Spd_Last;                                             //上一次的速度数值
long  SPD_INTG_ValA, SPD_INTG_ValB;                         //定义速度积分变量
int   SPD_A = 0, SPD_B = 0, Mark_A = 0, Mark_B = 0;         //定义速度脉冲数和
int   ANG_PWM, SPD_PWM, Turn_PWM, TOT_PWM;                  //定义角度PWM、速度PWM、转向PWM和合计PWM

float distance;
long duration;

void setup() {
  Serial.begin(115200);    //打开串行通信,波特率大小影响print数量
  // Bluetooth.begin(9600);  //蓝牙串口初始化
  Motor_begin();          //电机马达初始化
  // 配置 PWM 通道、频率和分辨率
  ledcSetup(Left_AIN1_Channel, pwmFrequency, pwmResolution);
  // 将 PWM 通道附加到指定的 GPIO 引脚
  ledcAttachPin(Left_AIN1, Left_AIN1_Channel);
  ledcSetup(Left_AIN2_Channel, pwmFrequency, pwmResolution);
  ledcAttachPin(Left_AIN2, Left_AIN2_Channel);
  ledcSetup(Right_BIN1_Channel, pwmFrequency, pwmResolution);
  ledcAttachPin(Right_BIN1, Right_BIN1_Channel);
  ledcSetup(Right_BIN2_Channel, pwmFrequency, pwmResolution);
  ledcAttachPin(Right_BIN2, Right_BIN2_Channel);

  MPU6050_begin();        //陀螺仪初始化
  Serial.println("I'm Ready！");
  // 设置定时器中断，每 10 毫秒触发一次
  timer.attach_ms(10, control);
  // // Bluetooth.println("OK");  //输出OK到手机APP调试窗口
}

void loop() {
  EnCode_Count();
  // Bluetooth_DEBUG();    //手机蓝牙调试控制

}
void control() {
  // 读取US-016的OUT引脚
  distance = digitalRead(OUT_Pin);
  if (distance == LOW) {
    // 当OUT引脚为低电平，表示检测到障碍物
    digitalWrite(19, HIGH);
  } else {
    digitalWrite(19, LOW);
  }
  AnglePID_PWMCount();  //角度环PWM计算
  SpeedPID_PWMCount(SPD_A, SPD_B);                         //速度环PWM计算
  // SPD_PWM = velocity(SPD_A, SPD_B);                         //速度环PWM计算
  View_PWM();
  SPD_A = SPD_B = 0;                                       //计数器清零重新计数
  TurnPID_PWMCount();
  TOT_PWM = ANG_PWM - SPD_PWM;                             //串联角度环和速度环
  // TOT_PWM = ANG_PWM - 0;                             //串联角度环和速度环
  Car_DRV(TOT_PWM);                                        //小车运动执行

  delayShow++;
  // if (delayShow > 99) {
  //   if (LEDLight == 0) {
  //     digitalWrite(19, HIGH);
  //     LEDLight = 1;
  //   } else {
  //     digitalWrite(19, LOW);
  //     LEDLight = 0;
  //   }
  //   delayShow = 0;
  // }
}
void View_PWM() {
  if (++Velocity_Count < 100)
  {
    return;
  }
  Velocity_Count = 0;
  //  Serial.print("前方距离:"); Serial.print(distance);  Serial.print(',');        
   Serial.print("角度X:"); Serial.print(AngleX);  Serial.print(',');        
  Serial.print("直立环:"); Serial.print(ANG_PWM); Serial.print(',');
  Serial.print("速度环:"); Serial.print(-SPD_PWM); Serial.print(',');
  Serial.print("综合值:"); Serial.print(TOT_PWM); Serial.println(',');
  // Serial.print("理想值:"); Serial.println(0);
}

/*-------定义陀螺仪初始化程序-------*/
void MPU6050_begin() {
  Wire.begin();
  Mpu6050.begin();
  Mpu6050.calcGyroOffsets(true);  //测试陀螺仪补偿值，耗时3秒
  // // Mpu6050.setGyroOffsets(-0.5, -4.2, -0.03);               //后期可直接设置不用每次都测试
}

/*-------定义马达初始化程序-------*/
void Motor_begin() {
  /*-------定义A马达驱动引脚-------*/
  // pinMode(Left_AIN1, OUTPUT);
  // pinMode(Left_AIN2, OUTPUT);
  pinMode(Left_EnCoderA, INPUT);
  pinMode(19, OUTPUT);
  /*-------定义B马达驱动引脚-------*/
  // pinMode(Right_BIN1, OUTPUT);
  // pinMode(Right_BIN2, OUTPUT);
  pinMode(Right_EnCoderA, INPUT);
  // pinMode(Right_EnCoderB, OUTPUT);
  // pinMode(Stby_Pin, OUTPUT);
  //编码器引脚
  pinMode(Voltage,INPUT); //初始化作为输入端
  // pinMode(ENCODER_L, INPUT);  
  // pinMode(DIRECTION_L, INPUT);
  // pinMode(ENCODER_R, INPUT);
  // pinMode(DIRECTION_R, INPUT);
  //超声波引脚
  pinMode(OUT_Pin, INPUT);
}

/*-------定义小车控制程序-------*/
void Car_DRV(int Pwm) {  //两个电机转动方向镜像对称，小车向一个方向行驶
  int Left_PWM, Right_PWM;
  /*-------补偿两轮启动PWM-------*/
  if (Pwm > 0) {
    Left_PWM = Pwm + LStar_PWM;
    Right_PWM = Pwm + RStar_PWM;
  }
  if (Pwm < 0) {
    Left_PWM = Pwm - LStar_PWM;
    Right_PWM = Pwm - RStar_PWM;
  }
  if (Pwm == 0) {
    Left_PWM = 0;
    Right_PWM = 0;
  }


  /*-------控制电机转动-------*/
  Motor_DRV(Left_AIN1_Channel, Left_AIN2_Channel, Left_PWM);       //输入左侧电机3个针脚和PWM数值
  Motor_DRV(Right_BIN1_Channel, Right_BIN2_Channel, -Right_PWM);  //输入右侧电机3个针脚和PWM数值
}

/*-------定义电机转动程序-------*/
void Motor_DRV(int Pin1, int Pin2, int PwmVal) {
  PwmVal = constrain(PwmVal, -255, 255);  //限定Pwm区间在-255~255
  // digitalWrite(Stby_Pin, HIGH);
  // analogWrite(PinPwm, abs(PwmVal));  //设置输出的PWM数值
  if (PwmVal == 0) {                 //如果PWM为0则输出针脚为低电压，马达停止转动
    ledcWrite(Pin1, 0);
    ledcWrite(Pin2, 0);
  }
  if (PwmVal > 0) {  //如果PWM大于0则电机顺时针转动
    ledcWrite(Pin2, 0);
    ledcWrite(Pin1, PwmVal);
  }
  if (PwmVal < 0) {  //如果PWM小于0则电机逆时针转动
    ledcWrite(Pin2, -PwmVal);
    ledcWrite(Pin1, 0);
  }
}

/*-------定义蓝牙串口调试和控制程序-------*/
void Bluetooth_DEBUG() {         //根据手机端发送来的串口数据控制数值增减
  // while (Bluetooth.available())  //当有数据时开始执行
  // {
  //   char Receive_Char = Bluetooth.read();  //读取蓝牙端发送的数据
  //   switch (Receive_Char) {
  //     /*-------机械平衡点调整-------*/
  //     case '1': ZHONGZHI += 0.1; break;  //调节物理平衡点,w前倾，s后仰
  //     case '2': ZHONGZHI -= 0.1; break;
  //     /*-------角度环调试-------*/
  //     case '3': ANG_Kp += 0.1; break;   //调节直立环 比例Kp项-
  //     case '4': ANG_Kp -= 0.1; break;   //调节直立环 比例Kp项+
  //     case '5': ANG_Ki += 0.01; break;  //调节直立环 积分项Ki-
  //     case '6': ANG_Ki -= 0.01; break;  //调节直立环 积分项Ki+
  //     case '7': ANG_Kd += 0.01; break;  //调节直立环 微分项Kd-
  //     case '8':
  //       ANG_Kd -= 0.01;
  //       break;  //调节直立环 微分项Kd+
  //     /*------控制小车前后运行------------------------*/
  //     case 'w': ZHONGZHI = Balance_ANG - Drive_ANG; break;
  //     case 'b': ZHONGZHI = Balance_ANG + Drive_ANG; break;
  //     case 's': ZHONGZHI = Balance_ANG; break;
  //   }
  //   /*-------调试时PID极性限制-------*/
  //   if (ANG_Kp < 0) ANG_Kp = 0;
  //   if (ANG_Ki < 0) ANG_Ki = 0;
  //   if (ANG_Kd < 0) ANG_Kd = 0;

  //   /*-------串口打印输出显示-------*/
  //   Bluetooth.print("ZHONGZHI: ");
  //   Bluetooth.println(ZHONGZHI);
  //   Bluetooth.print("ANG_Kp:");
  //   Bluetooth.print(ANG_Kp);
  //   Bluetooth.print("  ANG_Ki:");
  //   Bluetooth.print(ANG_Ki);
  //   Bluetooth.print("  ANG_Kd:");
  //   Bluetooth.println(ANG_Kd);
  //   delay(1);
  // }
}
/**************************************************************************
函数功能：直立PD控制  作者：平衡小车之家
入口参数：角度、角速度
返回  值：直立控制PWM
**************************************************************************/
int balance(float Angle, float Gyro)
{
  float Bias;
  int balance;
  Bias = Angle - ZHONGZHI;                         //===求出平衡的角度中值 和机械相关
  balance = Balance_Kp * Bias + Gyro * Balance_Kd; //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
  return balance;
}
/**************************************************************************
函数功能：速度PI控制 作者：平衡小车之家
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
**************************************************************************/
int velocity(int encoder_left, int encoder_right)
{
  static float Velocity, Encoder_Least, Encoder, Movement=0;
  static float Encoder_Integral, Target_Velocity;
  float kp = 2, ki = kp / 200;    //PI参数
  // if       ( Flag_Qian == 1)Movement = 600;
  // else   if ( Flag_Hou == 1)Movement = -600;
  // else    //这里是停止的时候反转，让小车尽快停下来
  // {
  //   Movement = 0;
  //   if (Encoder_Integral > 300)   Encoder_Integral -= 200;
  //   if (Encoder_Integral < -300)  Encoder_Integral += 200;
  // }
  //=============速度PI控制器=======================//
  Encoder_Least = (encoder_left + encoder_right) - 0;               //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零）
  Encoder *= 0.7;                                                   //===一阶低通滤波器
  Encoder += Encoder_Least * 0.3;                                   //===一阶低通滤波器
  Encoder_Integral += Encoder;                                      //===积分出位移 积分时间：40ms
  Encoder_Integral = Encoder_Integral - Movement;                   //===接收遥控器数据，控制前进后退
  if (Encoder_Integral > 21000)    Encoder_Integral = 21000;        //===积分限幅
  if (Encoder_Integral < -21000) Encoder_Integral = -21000;         //===积分限幅
  Velocity = Encoder * Velocity_Kp + Encoder_Integral * Velocity_Ki;                  //===速度控制
  // if (Turn_Off(KalFilter.angle, Battery_Voltage) == 1 || Flag_Stop == 1)    Encoder_Integral = 0;//小车停止的时候积分清零
  return Velocity;
}
/*-------定义角度环PID调试程序输出电机电压PWM数值-------*/
/* 用陀螺仪返回的数据计算直立PID的PWM
   前倾陀螺仪X轴为正，后仰陀螺仪X轴为负
   车子前后移动保持平衡状态
*/
void AnglePID_PWMCount() {       //计算电机转动需要的PWM数值
  Mpu6050.update(); 
  AngleX = Mpu6050.getAngleX();  //获取陀螺仪X方向角度
  AngleY = Mpu6050.getAngleY();                                     //获取陀螺仪Y方向角度
  GyroX = Mpu6050.getGyroX();     // 获取陀螺仪X方向角速度
  // ANG_PWM = balance(AngleX,GyroX);
  if ((abs(AngleX) <= 45) && (abs(AngleY) < 5)) {                            //如果小车前后倾斜角度小于75°,并且左右倾角小于10°则运行
    ANG_DIF_Val = AngleX - ZHONGZHI;                                        //计算小车偏转角度与静态平衡角度的差值。
    ANG_INTG_Val += ANG_DIF_Val;                                              //计算角度偏差的积分，INTG_Val为全局变量，一直积累
    ANG_INTG_Val = constrain(ANG_INTG_Val, -800, 800);                        //限定误差积分的最大和最小值
    ANG_PWM = ANG_Kp * ANG_DIF_Val + ANG_Ki * ANG_INTG_Val + ANG_Kd * GyroX;  //通过调节PID计算角度环PWM数值
  } else ANG_PWM = 0;
}
/*-------定义速度环PID调试程序输出电机电压PWM数值-------*/
/* 通过电机转动算出速度环PID的PWM
   前进左轮A速度环为正，右纶B速度环为负
   用于辅助小车尽快平衡
*/
void SpeedPID_PWMCount(int Spd_LA, int Spd_RB) {             //车轮位移PID计算PWM，通过电机编码器返回数据计算
  float Spd_Val = (Spd_LA + Spd_RB) / 2;                     //消除两轮不一致的误差
  Spd_Val = 0.3 * Spd_Val + 0.7 * Spd_Last ;                 //★增加速度环一阶滤波器，让数值平缓过渡
  Spd_Last = Spd_Val;
  SPD_INTG_ValA += Spd_LA;
  SPD_INTG_ValB += Spd_RB;
  SPD_INTG_ValA = constrain(SPD_INTG_ValA, -1000, 1000);       //限定误差积分的最大和最小值
  SPD_INTG_ValB = constrain(SPD_INTG_ValB, -1000, 1000);       //限定误差积分的最大和最小值
  long SPD_INTG_Val = (SPD_INTG_ValA + SPD_INTG_ValB) / 2;
  SPD_INTG_Val = constrain(SPD_INTG_Val, -2000, 2000);
  if ((abs(AngleX) <= 45) && (abs(AngleY) < 5)) {
    SPD_PWM = SPD_Kp * Spd_Val + SPD_Ki * SPD_INTG_Val;        //通过调节PI计算速度环PWM数值
  }
  else SPD_PWM = 0;
  if (ANG_PWM < 0) SPD_PWM = -SPD_PWM;                         //★解决单A相测速方向问题
}
/*-------定义转向环PID调试程序输出电机电压PWM数值-------*/
/* 通过陀螺仪Z轴加速度计算小车转向PWM
*/
void TurnPID_PWMCount()                                    //转向PMW计算
{
  GyroZ = Mpu6050.getGyroZ();                              //获取陀螺仪Z轴角速度
  AccZ = Mpu6050.getAccZ();                                //获取陀螺仪Z轴角加速度
  Turn_PWM = Turn_Kp * (Turn_SPD - GyroZ) - Turn_Kp * AccZ ;
}
/*-------定义编码器计数程序-------*/
void EnCode_Count() {
  if (digitalRead(Left_EnCoderA) == 1 && Mark_A == 0) {      //计算霍尔编码器电机速度脉冲数值
    SPD_A++; Mark_A = 1;
  }
  if (digitalRead(Left_EnCoderA) == 0 && Mark_A == 1) {
    SPD_A++; Mark_A = 0;
  }
  if (digitalRead(Right_EnCoderA) == 1 && Mark_B == 0) {
    SPD_B++; Mark_B = 1;
  }
  if (digitalRead(Right_EnCoderA) == 0 && Mark_B == 1) {
    SPD_B++; Mark_B = 0;
  }
}