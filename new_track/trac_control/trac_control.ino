//////// version lvrbc1 0331 1221 without blynk(commented)

#include <Servo.h>
#include "Cooler.h"
#include <SoftwareSerial.h>
/*SoftwareSerial DebugSerial(2,3);

#define BLINK_PRINT DebugSerial;
#include <BlynkSimpleSerialBLE.h>
*/

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
const int threshold_count = 1;

// Distance measuring
const int TRIG_PIN = 53;
const int ECHO_PIN = 52;
long duration, cm;
int waitingPause = 0;
const int threshold_pause = 10;
const long threshold_dist = 45;


//Motor
const int in1 = 4;
const int in2 = 5;
const int ena = 8;
int m_speed = 200; // 0 ~ 255
//Motor

//driving component
  //minimun unit of time that make turn
const int   TIME_DELTA = 5; // unit in ms
const int   TIME_OUT = 5;   //how many max step in one iteration of follow
const float   MINI_DIST = 3.0;

/////for cooler

// GPS
TinyGPS gps;


// Master Enable
bool enabled = false;

//WidgetTerminal terminal(V3);

// Serial components
SoftwareSerial bluetoothSerial(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
SoftwareSerial nss(GPS_TX_PIN, 255);            // TXD to digital pin 6

/* Compass */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

/////////////

//Servo
Servo servo1;
int pos = 90;
//Servo

//char auth[] = "d505dc30336d4b64958172eb095d617a";

void setup() {
  Serial.begin(9600);
  
  // Trace tracking
  pinMode(TCRT_LED, OUTPUT);
  
  //Motor
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);

  //Servo
  servo1.attach(2);
  // Compass
  setupCompass();
  //GPS
  nss.begin(9600);
  //Bluetooth
  bluetoothSerial.begin(9600);
  //For using Blynk , auth need to be define in CoolerDefinitions
  //Serial.println("Start blynk");
  //Blynk.begin(bluetoothSerial, auth);
  //Blynk.begin(SerialBLE,auth);
  //Serial.println("Blynk start!");
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("In loop!");
  //Blynk.run();
  if(Serial.available())
  {
    Serial.println("OK!");
    command = Serial.readStringUntil('\n');
    if(command == "Trace")
    {
      Trace_Tracking();
      Serial.println("Tracing Mode");
    }
    else if( command == "Follow" ){
      //Blynk.run();

      //these are test session when blynk not availible
      command = Serial.readStringUntil('\n');
      BlynkParam fake(&command , command.length());
      GpsParam gps( fake );
      GeoLoc fakeloc;
      fakeloc.lat = gps.getLat();
      fakeloc.lon = gps.getLon();
      driveTo( fakeloc , TIME_OUT );
    }
    else if(command = "RC")
    {
        while(1)
        {
            String dir = Serial.readStringUntil('\n');
            if(dir == "w")
                FORWARD();
            else if(dir == "a")
                turn(45,1000);
            else if(dir == "d")
                turn(135,1000);
            else if(dir == "s")
                BRAKE();
        }
    }
  }
  //Trace_Tracking();
  //FORWARD();
  //turn(30,1000);
  /*servo1.write(45);
  delay(500);
  servo1.write(135);
  delay(500);*/
}
void RC()
{
    Serial.println("RC");
}

// Trace tracking main function
void Trace_Tracking()
{
  if(DIST_PAUSE(GET_DISTANCE()) == true){
    BRAKE();
    Serial.println("BRAKE!!!");
    return;
  }
  
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
  Serial.println(a_in[0]);
  Serial.println(a_in[1]);
  Serial.println(a_in[2]);
  Serial.println(a_in[3]);
  Serial.println(a_in[4]);
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
  else if(num == 16 || num == 8 || num == 24 || num == 12 || num == 28 || num == 20){
    // Turn lefts
    return 2;
  }
  else if(num == 1 || num == 2 || num == 3 || num == 6 || num == 7){
    // Turn right
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
  Serial.println(cm);
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

//blynk function
//for follow
BLYNK_WRITE(v0){
    GpsParam gps(param);
    nss.listen();
    GeoLoc coolerLoc = checkGPS();
    bluetoothSerial.listen();
    
    GeoLoc phoneLoc;
    do{
      phoneLoc;
      phoneLoc.lat = gps.getLat();
      phoneLoc.lon = gps.getLon();

      Serial.println("Received remote GPS: ");
      Serial.print(gps.getLat(), 7); Serial.print(", "); Serial.println(gps.getLon(), 7);


      driveTo( phoneLoc, TIME_OUT );
      nss.listen();
      coolerLoc = checkGPS();
      bluetoothSerial.listen();
    }while( geoDistance( coolerLoc , phoneLoc ) < MINI_DIST );
    STOP();
}

//for RC
BLYNK_WRITE(v1)
{
    if(param.asInt() == 1)
    {
        FORWARD();
    }
}

BLYNK_WRITE(v2)
{
    if(param.asInt() == 1)
    {
        left(30,1000);
    }
}

BLYNK_WRITE(v3)
{
    if(param.asInt() == 1)
    {
        right(30,1000);
    }
}

//with waypoint Going funtions
void driveTo(struct GeoLoc &loc, int timeout) {
  nss.listen();
  GeoLoc coolerLoc = checkGPS();
  bluetoothSerial.listen();

  if (coolerLoc.lat != 0 && coolerLoc.lon != 0 && enabled) {
    float d = 0;
    nss.listen();
    coolerLoc = checkGPS();
    bluetoothSerial.listen();
    
    d = geoDistance(coolerLoc, loc);
    float t = geoBearing(coolerLoc, loc) - geoHeading();
    
    Serial.print("Distance: ");
    Serial.println(geoDistance(coolerLoc, loc));
  
    Serial.print("Bearing: ");
    Serial.println(geoBearing(coolerLoc, loc));

    Serial.print("heading: ");
    Serial.println(geoHeading());
    
    drive(d, t, TIME_DELTA);
  }
}
void drive(int distance = 0.0 , float tur = 0.0 , int interval = TIME_DELTA ) {
  float t = tur;
  while (t < -180) t += 360;
  while (t >  180) t -= 360;

  t+=90;
  if( t>150 )
    t = 150;
  else if( t< 30 )
    t = 30;
  Serial.print("turn: ");
  Serial.println(t);
  Serial.print("original: ");
  Serial.println(tur);

  turn( t , interval );
  
}
//////////Below should be in  cooler.cpp but i am stupid sor.

GeoLoc checkGPS() {
  Serial.println("Reading onboard GPS: ");
  bool newdata = false;
  unsigned long start = millis();
  while (millis() - start < GPS_UPDATE_INTERVAL) {
    if (feedgps())
      newdata = true;
  }
  if (newdata) {
    return gpsdump(gps);
  }

  GeoLoc coolerLoc;
  coolerLoc.lat = 0.0;
  coolerLoc.lon = 0.0;
  
  return coolerLoc;
}

// Get and process GPS data
GeoLoc gpsdump(TinyGPS &gps) {
  float flat, flon;
  unsigned long age;
  
  gps.f_get_position(&flat, &flon, &age);

  GeoLoc coolerLoc;
  coolerLoc.lat = flat;
  coolerLoc.lon = flon;

  Serial.print(coolerLoc.lat, 7); Serial.print(", "); Serial.println(coolerLoc.lon, 7);

  return coolerLoc;
}

// Feed data as it becomes available 
bool feedgps() {
  while (nss.available()) {
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}
/*
// Lid Hook
BLYNK_WRITE(V0) {
  switch (lidState) {
    case OPENED:
      setServo(SERVO_LID_CLOSE);
      lidState = CLOSED;
      break;
    case CLOSED:
      setServo(SERVO_LID_OPEN);
      lidState = OPENED;
      break;
  }
}
// Killswitch Hook
BLYNK_WRITE(V1) {
  enabled = !enabled;
  
  //Stop the wheels
  stop();
}

// GPS Streaming Hook
BLYNK_WRITE(V2) {
  GpsParam gps(param);
  
  Serial.println("Received remote GPS: ");
  
  // Print 7 decimal places for Lat
  Serial.print(gps.getLat(), 7); Serial.print(", "); Serial.println(gps.getLon(), 7);

  GeoLoc phoneLoc;
  phoneLoc.lat = gps.getLat();
  phoneLoc.lon = gps.getLon();

  driveTo(phoneLoc, GPS_STREAM_TIMEOUT);
}

// Terminal Hook
BLYNK_WRITE(V3) {
  Serial.print("Received Text: ");
  Serial.println(param.asStr());

  String rawInput(param.asStr());
  int colonIndex;
  int commaIndex;
  
  do {
    commaIndex = rawInput.indexOf(',');
    colonIndex = rawInput.indexOf(':');
    
    if (commaIndex != -1) {
      String latStr = rawInput.substring(0, commaIndex);
      String lonStr = rawInput.substring(commaIndex+1);

      if (colonIndex != -1) {
         lonStr = rawInput.substring(commaIndex+1, colonIndex);
      }
    
      float lat = latStr.toFloat();
      float lon = lonStr.toFloat();
    
      if (lat != 0 && lon != 0) {
        GeoLoc waypoint;
        waypoint.lat = lat;
        waypoint.lon = lon;
    
        Serial.print("Waypoint found: "); Serial.print(lat); Serial.println(lon);
        driveTo(waypoint, GPS_WAYPOINT_TIMEOUT);
      }
    }
    
    rawInput = rawInput.substring(colonIndex + 1);
    
  } while (colonIndex != -1);
}
*/

void displayCompassDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

float geoBearing(struct GeoLoc &a, struct GeoLoc &b) {
  float y = sin(b.lon-a.lon) * cos(b.lat);
  float x = cos(a.lat)*sin(b.lat) - sin(a.lat)*cos(b.lat)*cos(b.lon-a.lon);
  return atan2(y, x) * RADTODEG;
}

float geoDistance(struct GeoLoc &a, struct GeoLoc &b) {
  const float R = 6371000; // km
  float p1 = a.lat * DEGTORAD;
  float p2 = b.lat * DEGTORAD;
  float dp = (b.lat-a.lat) * DEGTORAD;
  float dl = (b.lon-a.lon) * DEGTORAD;

  float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return R * y;
}

float geoHeading() {
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // Offset
  heading -= DECLINATION_ANGLE;
  heading -= COMPASS_OFFSET;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 

  // Map to -180 - 180
  while (headingDegrees < -180) headingDegrees += 360;
  while (headingDegrees >  180) headingDegrees -= 360;

  return headingDegrees;
}
/*
void setServo(int pos) {
  lidServo.attach(SERVO_PIN);
  lidServo.write(pos);
  delay(2000);
  lidServo.detach();
}

void setSpeedMotorA(int speed) {
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, HIGH);
  
  // set speed to 200 out of possible range 0~255
  analogWrite(MOTOR_A_EN_PIN, speed + MOTOR_A_OFFSET);
}

void setSpeedMotorB(int speed) {
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, HIGH);
  
  // set speed to 200 out of possible range 0~255
  analogWrite(MOTOR_B_EN_PIN, speed + MOTOR_B_OFFSET);
}

void setSpeed(int speed)
{
  // this function will run the motors in both directions at a fixed speed
  // turn on motor A
  setSpeedMotorA(speed);

  // turn on motor B
  setSpeedMotorB(speed);
}

void stop() {
  // now turn off motors
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, LOW);  
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, LOW);
}
*/




void setupCompass() {
   /* Initialise the compass */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displayCompassDetails();
}
