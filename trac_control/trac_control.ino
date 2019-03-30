#include <Servo.h>

// loop
String command;

// Trace tracking
int a_in[5];
// bool a_bin[5];
// true : black
// false : white
const int TCRT_LED = 22;
const int threshold_color = 150;
int waitingCommand = 1;
int waitCount = 0;
int nowCommand = 1;
const int threshold_count = 10;

// Distance measuring
const int TRIG_PIN = 53;
const int ECHO_PIN = 52;
long duration, cm;
int waitingPause = 0;
const int threshold_pause = 10;
const long threshold_dist = 45;


//Motor
const int in1 = 6;
const int in2 = 7;
const int ena = 8;
int m_speed = 30; // 0 ~ 255
//Motor

//Servo
Servo servo1;
int pos = 90;
//Servo

void setup() {
  Serial.begin(9600);
  
  // Trace tracking
  pinMode(TCRT_LED, OUTPUT);
  
  //Motor
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);

  //Servo
  servo1.attach(2);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available())
  {
    command = Serial.readStringUntil('\n');
    if(command == "Trace")
    {
      Trace_Tracking();
      Serial.println("Tracing Mode");
    }
  }
}

// Trace tracking main function
void Trace_Tracking()
{
  /*if(DIST_PAUSE(GET_DISTANCE()) == true){
    BRAKE();
    return;
  }*/
  
  digitalWrite(TCRT_LED, HIGH);    // Turning ON LED
  delayMicroseconds(500);  //wait
  READ_TRACE();
  int now_C = DECISION(GET_COMMAND());
  if(now_C == 0){
    // Brake
    BRAKE();
  }
  else if(now_C == 1){
    // Go straight
    FORWARD();
  }
  else if(now_C == 2){
    // Turn left
    turn(75, 200);
  }
  else if(now_C == 3){
    // Turn right
    turn(105, 200);
  }
}

// Trace tracking
void READ_TRACE(){
  a_in[0] = analogRead(A1);
  a_in[1] = analogRead(A2);
  a_in[2] = analogRead(A3);
  a_in[3] = analogRead(A4);
  a_in[4] = analogRead(A5);
}

int GET_COMMAND(){
  int num = 0;
  for(int i = 0;i < 5; ++i){
    if(a_in[i] > threshold_color){
      // Black
      // a_bin[i] = true;
      num |= (1 << (4 - i));
    } else {
      // White
      // a_bin[i] = false;
    }
  }

  if(num == 4 || num == 14){
    // Go straight
    return 1;
  }
  else if(num == 16 || num == 8 || num == 24 || num == 12 || num == 28){
    // Turn right
    return 2;
  }
  else if(num == 1 || num == 2 || num == 3 || num == 6 || num == 7){
    // Turn left
    return 3;
  }
  else {
    // Stop
    return 0;
  }
}

int DECISION(int newCommand){
  if(newCommand != nowCommand && newCommand != waitingCommand){
    waitingCommand = newCommand;
    waitCount = 0;
    return nowCommand;
  }
  if(newCommand != nowCommand && newCommand == waitingCommand){
    if(waitCount >= threshold_count){
      waitCount = 0;
      nowCommand = newCommand;
      return nowCommand;
    }
    waitCount++;
    return nowCommand;
  }
  return nowCommand;
}

// Distance Measurement
long GET_DISTANCE(){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);     // 給 Trig 高電位，持續 10微秒
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(ECHO_PIN, INPUT);             // 讀取 echo 的電位
  duration = pulseIn(ECHO_PIN, HIGH);   // 收到高電位時的時間
  if(duration > 6000){
    return 100;
  }
  cm = (duration / 2) / 28.84;         // 將時間換算成距離 cm
  return cm;
}

bool DIST_PAUSE(long _dist){
  if(_dist < threshold_dist){
    if(waitingPause > threshold_count){
      return true;
    }
    waitingPause++;
  } else {
    waitingPause = 0;
  }
  return false;
}

//Motor
void STOP()
{
  digitalWrite(ena,0);
}

void FORWARD()
{
  digitalWrite(ena,m_speed);
  digitalWrite(in1,0);
  digitalWrite(in2,1);
}

void BACKWARD()
{
  digitalWrite(ena,m_speed);
  digitalWrite(in1,1);
  digitalWrite(in2,0);
}

void BRAKE()
{
  digitalWrite(ena,m_speed);
  digitalWrite(in1,0);
  digitalWrite(in2,0);
}
void change_speed(int s)
{
  m_speed = s;
}
//Motor

//Servo
void turn(int angle, int ms)
{
  if(angle < 30 || angle > 150)
      return;
  servo1.write(angle);
  pos = angle;
  delay(ms);
}

void straight()
{
  servo1.write(90);
  pos = 90;
}

void left(int angle, int ms)
{
  if(angle < 90)
  {
      for(;pos > angle;pos-=1)
      {
        servo1.write(pos);
      }
      delay(ms);
  }
}

void right(int angle, int ms)
{
  if(angle > 90)
  {
      for(;pos < angle;pos+=1)
      {
        servo1.write(pos);
      }
      delay(ms);
  }
}
//Servo
