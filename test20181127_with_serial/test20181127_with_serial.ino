#include <Arduino.h>
#include "Wire.h"
#include <KalmanFilter.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <EEPROM.h>
#include <TB6612FNG.h>
#include "I2Cdev.h"
//#include <MsTimer2.h>               //定时器库的 头文件
// 36、39IO口不能用
// TB6612FNG 3+3+1 PIN
#define ENCODER_L 4 //编码器采集引脚 每路2个 共4个   
#define DIRECTION_L 5  //change 左B
#define ENCODER_R 34
#define DIRECTION_R 35//change 右B
#define STANDBY 27
#define AIN1 12
#define AIN2 14
#define PWMA 13
#define BIN1 26
#define BIN2 25
#define PWMB 15     //Wrover是15，DevKitC是33 
MPU6050 Mpu6050 ;
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AX = 0, AY = 0, AZ = 0, Tmp, GX = 0, GY = 0, GZ = 0;
int Velocity_Left, Velocity_Right = 0;     //左右轮速度
int flag = 1;
float Velocity_Pwm, Turn_Pwm;   //直立 速度 转向环的PWM
float Balance_Pwm, testpwm;
float Balance_Kp = 13.84, Balance_Kd = -0.04, Velocity_Kp = 1.365, Velocity_Ki = 0.006825;
float Motor1, Motor2;      //电机叠加之后的PWM
volatile long Velocity_L, Velocity_R = 0;   //左右轮编码器数据
float ZHONGZHI  = 0;//小车的机械中值
unsigned long millis_time;
Tb6612fng motors(STANDBY, AIN1, AIN2, PWMA, BIN1, BIN2, PWMB);
// 中断控制
hw_timer_t * timer = NULL;      // timer指针

char get_command;
int Flag_Qian = 0, Flag_Hou = 0, Flag_Stop = 0;

/*************
   ISR中断服务程序
 *************/
/*void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    interruptCounter++;
    portEXIT_CRITICAL_ISR(&timerMux);
    // Give a semaphore that we can check in the loop
    xSemaphoreGiveFromISR(timerSemaphore, NULL);
    // It is safe to use digitalRead/Write here if you want to toggle an output
  }*/

/***************************************************************************
   卡尔曼滤波参数初始化
 ***************************************************************************/
float angle, angle6;
float Gyro_x, Gyro_y, Gyro_z;
float accelz = 0;
float angle_err, q_bias;
float Pdot[4] = { 0, 0, 0, 0};
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
float angle_dot;
float Angle;
float angleAx;
/*
  void get_mpu6050_data()
  {

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AX = "); Serial.print(AX);
  Serial.print(" | AY = "); Serial.print(AY);
  Serial.print(" | AZ = "); Serial.print(AZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53); //equation for temperature in degrees C from datasheet
  Serial.print(" | GX = "); Serial.print(GX);
  Serial.print(" | GY = "); Serial.print(GY);
  Serial.print(" | GZ = "); Serial.println(GZ);
  delay (333);
  }
*/

/*****************************************************************/
/*******************卡尔曼相关函数 begin**************************/
void Kalman_Filter(double angle_m, double gyro_m, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0)
{
  // Serial.printf("kalmanxxxxxxx");
  //q_bias=0;
  angle += (gyro_m - q_bias) * dt;
  angle_err = angle_m - angle;
  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err; //最优角度
  q_bias += K_1 * angle_err;
  angle_dot = gyro_m - q_bias; //最优角速度
}

void Angletest(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0, float K1)
{
  //平衡参数
  Angle = atan2(ay , az) * 57.3;           //角度计算公式,Angle:一阶互补滤波计算出的小车最终倾斜角度
  Gyro_x = (gx - 128.1) / 131;              //角度转换
  Kalman_Filter(Angle, Gyro_x, dt, Q_angle, Q_gyro, R_angle, C_0);          //卡曼滤波
  //旋转角度Z轴参数
  if (gz > 32768) gz -= 65536;              //强制转换2g  1g
  Gyro_z = -gz / 131;                      //Z轴参数转换
  accelz = az / 16.4;
  angleAx = atan2(ax, az) * 180 / PI; //计算与x轴夹角
  Gyro_y = -gy / 131.00; //计算角速度
  Angle = Yiorderfilter(angleAx, Gyro_y, dt, K1); //一阶滤波
}

float Yiorderfilter(float angle_m, float gyro_m, float dt, float K1)
{
  return angle6 = K1 * angle_m + (1 - K1) * (angle6 + gyro_m * dt);
}

/*******************卡尔曼相关函数 end**************************/
/***************************************************************/

void kalmanfilter()
{
  if (Mpu6050.testConnection()) {
    Serial.println("MPU6050 OPEN!!!!YEAH!!!");
  }
  else {
    Serial.println("MPU6050 bad!! 5555555!!!");
  }
  /* Serial.print("AX = "); Serial.print(AX);
    Serial.print(" | AY = "); Serial.print(AY);
    Serial.print(" | AZ = "); Serial.print(AZ);
    Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53); //equation for temperature in degrees C from datasheet
    Serial.print(" | GX = "); Serial.print(GX);
    Serial.print(" | GY = "); Serial.print(GY);
    Serial.print(" | GZ = "); Serial.println(GZ);*/
  float dt = 0.005; //注意：dt的取值为滤波器采样时间 5ms
  float Q_angle = 0.001, Q_GYro = 0.005;
  float R_angle = 0.5 , C_0 = 1;
  float K1 = 0.05; // 对加速度计取值的权重
  Angletest(AX, AY, AZ, GX, GY, GZ, dt, Q_angle, Q_GYro, R_angle, C_0, K1);        //通过卡尔曼滤波获取角度
  //angle = KalFilter.angle;//Angle是一个用于显示的整形变量
  Serial.printf("Angle = %f \n", angle);
}

/**************************************************************************
  函数功能：外部中断读取左编码器数据，具有二倍频功能 注意外部中断是跳变沿触发
  入口参数：无
  返回  值：无
**************************************************************************/
void IRAM_ATTR READ_ENCODER_L() {
  if (digitalRead(ENCODER_L) == LOW) {     //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L--;  //根据另外一相电平判定方向
    else      Velocity_L++;
  }
  else {     //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L++; //根据另外一相电平判定方向
    else     Velocity_L--;
  }
}
/**************************************************************************
  函数功能：外部中断读取右编码器数据，具有二倍频功能 注意外部中断是跳变沿触发
  入口参数：无
  返回  值：无
**************************************************************************/
void IRAM_ATTR READ_ENCODER_R() {
  if (digitalRead(ENCODER_R) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R--;//根据另外一相电平判定方向
    else      Velocity_R++;
  }
  else {   //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R++; //根据另外一相电平判定方向
    else     Velocity_R--;
  }
}

/***************************************************************************
  函数功能：直立PD控制
  入口参数：角度、角速度
  返回值：直立控制PWM
**************************************************************************/
float balance(float angle, float Gyro)
{
  float Bias;
  float balance_x;
  Bias = angle + 9.3;   //===求出平衡的角度中值 和机械相关
  if (Bias>0) Bias+=2.4;
  //if (Bias<0) Bias-=1.5;
  Serial.print("angle = "); Serial.println(angle);
  Serial.print("Bias = "); Serial.println(Bias);
  balance_x = Balance_Kp * Bias + Gyro * Balance_Kd; //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
  // Serial.printf("balance = %f \n", balance_x);
  return balance_x;
}

/**************************************************************************
  函数功能：限制PWM赋值
  入口参数：无
  返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{
  int Amplitude = 250;  //===PWM满幅是255 限制在250
  //if(Flag_Qian==1)  Motor2-=DIFFERENCE;  //DIFFERENCE是一个衡量平衡小车电机和机械安装差异的一个变量。直接作用于输出，让小车具有更好的一致性。
  //if(Flag_Hou==1)   Motor2-=DIFFERENCE-2;
  if (Motor1 < -Amplitude) Motor1 = -Amplitude;
  if (Motor1 > Amplitude)  Motor1 = Amplitude;
  if (Motor2 < -Amplitude) Motor2 = -Amplitude;
  if (Motor2 > Amplitude)  Motor2 = Amplitude;
  Motor1 /= 255;
  Motor2 /= 255;
}

/*************************************************************************/
//速度环
/*************************************************************************/
float velocity(int encoder_left, int encoder_right)
{
  static float Velocity, Encoder_Least, Encoder, Movement;
  static float Encoder_Integral, Target_Velocity;
 // float kp = 2, ki = kp / 200;    //PI参数
 // if       ( Flag_Qian == 1)Movement = 600;
 // else   if ( Flag_Hou == 1)Movement = -600;
 // else    //这里是停止的时候反转，让小车尽快停下来
  //{
  //  Movement = 0;
  //  if (Encoder_Integral > 300)   Encoder_Integral -= 200;
  //  if (Encoder_Integral < -300)  Encoder_Integral += 200;
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
  //  if (Turn_Off(KalFilter.angle, Battery_Voltage) == 1 || Flag_Stop == 1)    Encoder_Integral = 0;//小车停止的时候积分清零
  return Velocity;
}




void control()
{
  static int Velocity_Count, Turn_Count, Encoder_Count;
  // motors.enable(false);
  //  delay(30);
  Mpu6050.getMotion6(&AX, &AY, &AZ, &GX, &GY, &GZ);


   Serial.print("Velocity_Left=  "); Serial.println(Velocity_Left);
   Serial.print("Velocity_Right=  "); Serial.println(Velocity_Right);
  kalmanfilter();
  Balance_Pwm = balance(angle, Gyro_x);
  //testpwm = float(255.0 / (255.0 / Balance_Kp));

  if (++Velocity_Count >= 0) //速度控制，控制周期40ms
  {
    Velocity_Left = Velocity_L;    Velocity_L = 0;  //读取左轮编码器数据，并清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
    Velocity_Right = Velocity_R;    Velocity_R = 0; //读取右轮编码器数据，并清零
    Serial.print("Max = ");Serial.println(Velocity_Left+Velocity_Right);
   // Serial.print("Velocity_Right = ");Serial.println(Velocity_Right);
    Velocity_Pwm = velocity(Velocity_Left, Velocity_Right);//速度PI控制，控制周期40ms
    Velocity_Count = 0;
  }
  
  Motor1 = Balance_Pwm + Velocity_Pwm; // + Turn_Pwm;  //直立速度转向环的叠加
  Motor2 = Balance_Pwm + Velocity_Pwm; // - Turn_Pwm; //直立速度转向环的叠加
  Xianfu_Pwm();//限幅
  //  Serial.printf("bbcc %f,\n", Motor1);
  flag = 1;
  //  if (angle < -20 || angle > 20) flag=0;//角度太大，别开了
  if (flag == 0)  {
    motors.brake(); // 角度过大保护
  }
  else
  {
    //   motors.enable(true);
   // Serial.print("Motor1=  "); Serial.println(Motor1);
   // Serial.print("Motor2=  "); Serial.println(Motor2);
    motors.drive(Motor1, Motor2, 3, false);
  }//正常
}

void get_serial_command()
{
  get_command = Serial.read();
  if (get_command == 'q')
  {
    Flag_Qian = 1;
    Flag_Hou = 0;
    Flag_Stop = 0;
  }
  if (get_command == 'h')
  {
    Flag_Qian = 0;
    Flag_Hou = 1;
    Flag_Stop = 0;
  }
  if (get_command == 's')
  {
    Flag_Qian = 0;
    Flag_Hou = 0;
    Flag_Stop = 1;
  }
  if (get_command == '1')
  { Balance_Kp += 0.2;
    //Serial.print(" Kp = "); Serial.println(Balance_Kp);
  }
  if (get_command == '0')
  {
    Balance_Kp -= 0.2;
    Serial.print(" Kp = ");
    Serial.println(Balance_Kp);
  }
}

void setup()
{
  // TODO 对编码器的理解
  pinMode(ENCODER_L, INPUT);       //编码器引脚
  pinMode(DIRECTION_L, INPUT);       //编码器引脚
  pinMode(ENCODER_R, INPUT);       //编码器引脚
  pinMode(DIRECTION_R, INPUT);       //编码器引脚
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x80);     // set to 0x80 (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(300);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(300);
  Serial.begin(115200);
  Mpu6050.initialize();     //初始化MPU6050
  delay(1500);              //延时等待初始化完成
  motors.begin();
  delay(20);
  // 定时中断与外部中断
  //  timerSemaphore = xSemaphoreCreateBinary();
  //  timer = timerBegin(1, 80, true);//预分配系数80，80M/80=1M,每秒钟递减1M次
  //timerAttachInterrupt(timer, &get_mpu6050_data, true);
  // timerAlarmWrite(timer, 100000, true);//5k/1M = 5ms
  // timerAlarmEnable(timer);
  if (Mpu6050.testConnection()) {
    Serial.println("MPU6050 OPEN!!!!YEAH!!!");
  }
  else {
    Serial.println("MPU6050 bad!! 5555555!!!");
  }
  attachInterrupt(ENCODER_L, READ_ENCODER_L, CHANGE);
  attachInterrupt(ENCODER_R, READ_ENCODER_R, CHANGE);
}

void loop()
{
  sei();//全局中断开启
  get_serial_command();
  control();
  //Serial.print(" 【Kp】 = "); Serial.println(Balance_Kp);
  //Serial.print("AIN1: ");Serial.print(AIN1);
  // Serial.print("AIN2: ");Serial.print(AIN2);
  //Serial.print("BIN1: ");Serial.print(BIN1);
  //  Serial.print("BIN2: ");Serial.println(BIN2);
  // delay(20);
}
