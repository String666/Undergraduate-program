#define CARSPEED 100
#define CARSPEED1 120
#define CARSPEED2 100
#define CARSPEED3 255
#define CARSPEED4 150
#include <Servo.h>
#include <Wire.h>
#include <time.h>
#include "Adafruit_TCS34725.h"
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
//左2轮控制引脚接线
const int l_ain1 = 2;
const int l_ain2 = 3;//左前轮引脚
const int l_bin1 = 4;
const int l_bin2 = 5;//左后轮引脚
//右2轮控制引脚接线
const int r_ain1 = 6;
const int r_ain2 = 7;//右前轮引脚
const int r_bin1 = 8;
const int r_bin2 = 9;//右后轮引脚
//中轮控制引脚接线
const int rz_ain1 = 10;
const int rz_ain2 = 11;//中右轮引脚
const int lz_bin1 = 12;
const int lz_bin2 = 13;//中左轮引脚
//灰度
int L_Pin = A1;   // 左边灰度传感器连接到模拟引脚A1
int R_Pin = A0;   // 右边灰度传感器连接到模拟引脚A0
int L_val = 0;  //定义变量存以储读值
int R_val = 0;  //定义变量存以储读值
//计数
int i = 0;
int a = 0; //储存车读取黑线的状态
int m = 0;//储存色温数值
int r = 0;
int g = 0;
int b = 0;
int c = 0; //储存识别到的颜色
int j[4] = {0, 0, 0, 0};
int y=0;
int x=0;
int lx=0;
int w=0;
int p=0;
int q=0;
int n=0;
long countTimer1;
long countTimer2;
long countTimer3;
long countTimer4;
long countTimer5;
//舵机
Servo servo;
void setup() {
  pinMode(r_ain1, OUTPUT);
  pinMode(r_ain2, OUTPUT);
  pinMode(r_bin1, OUTPUT);
  pinMode(r_bin2, OUTPUT);
  pinMode(l_ain1, OUTPUT);
  pinMode(l_ain2, OUTPUT);
  pinMode(l_bin1, OUTPUT);
  pinMode(l_bin2, OUTPUT);
  pinMode(rz_ain1, OUTPUT);
  pinMode(rz_ain2, OUTPUT);
  pinMode(lz_bin1, OUTPUT);
  pinMode(lz_bin2, OUTPUT);
  pinMode(R_Pin, INPUT);
  pinMode(L_Pin, INPUT);
  servo.attach(A3);//舵机脚位
  servo.write(0);//舵机初始化
  Serial.begin(9600);
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
}
void ZHI(){
  analogWrite(r_ain1, 190);
  analogWrite(r_ain2, 0);
  analogWrite(r_bin1, 190);
  analogWrite(r_bin2, 0);
  analogWrite(rz_ain1, 190);
  analogWrite(rz_ain2, 0);
  analogWrite(l_ain1, 190);
  analogWrite(l_ain2, 0);
  analogWrite(l_bin1, 190);
  analogWrite(l_bin2, 0);
  analogWrite(lz_bin1, 190);
  analogWrite(lz_bin2, 0);
}
void ZHII(){
  analogWrite(r_ain1, 110);
  analogWrite(r_ain2, 0);
  analogWrite(r_bin1, 110);
  analogWrite(r_bin2, 0);
  analogWrite(rz_ain1, 110);
  analogWrite(rz_ain2, 0);
  analogWrite(l_ain1, 150);
  analogWrite(l_ain2, 0);
  analogWrite(l_bin1, 150);
  analogWrite(l_bin2, 0);
  analogWrite(lz_bin1, 150);
  analogWrite(lz_bin2, 0);
}
void ZHIIIII(){
  analogWrite(r_ain1, 110);
  analogWrite(r_ain2, 0);
  analogWrite(r_bin1, 110);
  analogWrite(r_bin2, 0);
  analogWrite(rz_ain1, 110);
  analogWrite(rz_ain2, 0);
  analogWrite(l_ain1, 110);
  analogWrite(l_ain2, 0);
  analogWrite(l_bin1, 110);
  analogWrite(l_bin2, 0);
  analogWrite(lz_bin1, 110);
  analogWrite(lz_bin2, 0);
}
void R()  {
  analogWrite(r_ain1, 0);
  analogWrite(r_ain2, 180);
  analogWrite(r_bin1, 0);
  analogWrite(r_bin2, 180);
  analogWrite(rz_ain1, 0);
  analogWrite(rz_ain2, 180);//右轮
  analogWrite(l_ain1, 200);
  analogWrite(l_ain2, 0);
  analogWrite(l_bin1, 200);
  analogWrite(l_bin2, 0);
  analogWrite(lz_bin1, 200);
  analogWrite(lz_bin2, 0);//左轮正转
}
void L()  {
  analogWrite(r_ain1, 255);
  analogWrite(r_ain2, 0);
  analogWrite(r_bin1, 255);
  analogWrite(r_bin2, 0);
  analogWrite(rz_ain1, 255);
  analogWrite(rz_ain2, 0);//右轮
  analogWrite(l_ain1, 0 );
  analogWrite(l_ain2, 200);
  analogWrite(l_bin1, 0);
  analogWrite(l_bin2, 220);
  analogWrite(lz_bin1, 0);
  analogWrite(lz_bin2, 220);//左轮
}
void ting() {
  analogWrite(r_ain1, 0);
  analogWrite(r_ain2, 0);
  analogWrite(r_bin1, 0);
  analogWrite(r_bin2, 0);
  analogWrite(rz_ain1, 0);
  analogWrite(rz_ain2, 0);
  analogWrite(l_ain1, 0);
  analogWrite(l_ain2, 0);
  analogWrite(l_bin1, 0);
  analogWrite(l_bin2, 0);
  analogWrite(lz_bin1, 0);
  analogWrite(lz_bin2, 0);
}
int xunji() {
  L_val = analogRead(L_Pin);   // 读取左边传感器的电位信号，越黑，值越小
  R_val = analogRead(R_Pin);   // 读取右边传感器的电位信号，越黑，值越小
  Serial.print("L_val=");
  Serial.print(L_val);
  Serial.print("   R_val=");
  Serial.println(R_val);
  if ((L_val > 160) && (R_val > 160)) //直行
  {
    if(n == 0)
    {
    ZHI();
    }
    else if(n == 1)
    {
    ZHIIIII();
   
    }
    Serial.println("直行");
    if(x==1)
    {y++;
    x=0;}
    if(p==0){
     countTimer3=millis();
     p=1;}
     if(lx==1)
    {w++;
    lx=0;}
    if(q==0){
     countTimer3=millis();
     q=1;}
  }
  else if ((L_val < 160) && (R_val < 160)) //直行,且遇黑线
  { ZHI();
    if(n==1){
    ZHIIIII();}
    Serial.println("直行遇黑线或者爬楼梯");
  }
  else if ((L_val > 160) && (R_val < 160)) //右转
  { x = 1;
    R();
    Serial.println("右转");
    if(p == 1) {
      countTimer4 = millis();
      Serial.println(countTimer4-countTimer3);
      p = 0;
    }
  }
  else if ((L_val < 160) && (R_val > 160)) //左转
  { lx=1;
    L();
    Serial.println("左转");}
  if(q == 1) {
      countTimer5 = millis();
      Serial.println(countTimer5-countTimer3);
      q = 0;
    }
    if(millis()<=15000){
     y = 0;
    w=0;
    q=0;
    p = 0; }
  else if(((countTimer4-countTimer3) >=400)||((w-y)>3)){
    y = 0;
    w=0;
    q=0;
    p = 0;
  }
 if(y>5){
    n=1;}
}
void zhaqiqiu() {
  int t = 0;
  int h = 0;
  do {
    ting();
    h = panduan();
    j[t] = h;
  } while (h == 0);//识别气球颜色
    t++;
  countTimer1 = millis();
  do {
    xunji();
  countTimer2 = millis();
  }
  while ((countTimer2 - countTimer1) <= 1000);//走第一段距离
  do {
    ting();
    h = panduan();
    j[t] = h;
  } while (h == 0);//识别气球颜色
  if (j[t] == j[0])
    { 
      ZHII();
      delay(300);
        ting();
       for (int v = 0; v <= 1; v++) {
        servo.write(180);
        delay(3000);
        servo.write(0);
        delay(3000);
      }
      countTimer1 = millis();
  do {
    xunji();
  countTimer2 = millis();
  }
  while ((countTimer2 - countTimer1) <= 800000);
   n=0;
    }
    else{
      countTimer1 = millis();
      t++;
  do {
    xunji();
  countTimer2 = millis();
  }while ((countTimer2 - countTimer1) <= 900);}//走第二段距离
  do {
    ting();
    h = panduan();
    j[t] = h;
  } while (h == 0);
  if (j[t] == j[0])
    { 
      ZHII();
      delay(300);
        ting();
       for (int v = 0; v <= 1; v++) {
        servo.write(180);
        delay(3000);
        servo.write(0);
        delay(3000);
      }
      countTimer1 = millis();
  do {
    xunji();
  countTimer2 = millis();
  }
  while ((countTimer2 - countTimer1) <= 800000);
   n=0;
    }
    else{
      countTimer1 = millis();
      t++;
  do {
    xunji();
  countTimer2 = millis();
  }while ((countTimer2 - countTimer1) <= 900);}//第三段距离   
  ZHII();
      delay(300);
        ting();
       for (int v = 0; v <= 1; v++) {
        servo.write(180);
        delay(3000);
        servo.write(0);
        delay(3000);
      }
      countTimer1 = millis();
  do {
    xunji();
  countTimer2 = millis();
  }
  while ((countTimer2 - countTimer1) <= 800000);
   n=0;
    
   //走终点}
  
}
int shibie() {
  uint16_t r, g, b, c, colorTemp, lux;
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);
  Serial.println(colorTemp);
  return colorTemp;
}
int panduan() {
  int y[4] = {0, 0, 0, 0};
  int x = 0;
  for (int b = 1; b <= 10; b++)
  { m = shibie();
    delay(10);
    Serial.println(m);
    if (m <= 4300) //识别为红色
    {
      y[0]++;
    }
    else if ((m > 4300) && (m <= 5500)) //识别为绿色
    {
      y[1]++;
    }
    else if ((m > 5500) && (m <= 10000)) //识别为蓝色
    {
      y[2]++;
    }
    /*else
    {
      y[3]++;
    }*/
  }
  Serial.println(y[0]);
  Serial.println(y[1]);
  Serial.println(y[2]);
  Serial.println(y[3]);
  for (int c = 0; c <= 3; c++) {
    if ((y[c] >= y[0]) && (y[c] >= y[1]) && (y[c] >= y[2]) && (y[c] >= y[3]))
    {
      x = c + 1;
    }
    else {
      x = x;
    }
  }
  Serial.println(x);
  return x;
}
void loop() {
xunji();

  // n=1;
  if ((n==1)&&(L_val < 140) && (R_val < 140)) {
    //countTimer1 = millis();
    countTimer1 = millis();
  do {
    xunji();
  countTimer2 = millis();
  }
  while ((countTimer2 - countTimer1) <= 200);
    ting();
    delay(200);
    zhaqiqiu();}
  
}
