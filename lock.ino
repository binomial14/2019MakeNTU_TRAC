#include<Servo.h>

bool isOpen = false;

const int LED_GREEN = 7;
const int LED_RED = 8;
const int BUTTON_PIN[4] = {3, 4, 5, 6};
const int SET_PIN = 2;
int button_on[4];
int PW = 1234;

int nowIN = 0;
bool isZero = true;

Servo servo_door;
const int SERVO_PIN = 9;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  for(int i = 0; i < 4; ++i){
    pinMode(BUTTON_PIN[i], INPUT);
  }
  pinMode(SET_PIN, INPUT);
  servo_door.attach(SERVO_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(nowIN);
  Serial.print('\t');
  
  if(!isOpen){
    /*digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);*/
    
    delayMicroseconds(500);
    int set_b = GET_SET_B();
    int res = GET_BUTTON();
    
    if(set_b == 1){
      nowIN = 0;
      isZero = true;
      return;
    }

    if(res == 0){
      isZero = true;
      return;
    }
    if(isZero == false){
      return;
    }
    isZero = false;
    nowIN = nowIN * 10 + res;
    
    if(nowIN > 1000){
      CHECK_PW();
    }
  } else {
    digitalWrite(LED_GREEN, HIGH);
    delayMicroseconds(500);
    int set_b = GET_SET_B();
    Serial.print("Open");
    Serial.print('\n');
    if(set_b == 1){
      // Close door
      digitalWrite(LED_GREEN, LOW);
      isOpen = false;
      SPIN_CLOSE();
      delayMicroseconds(2000);
    }
  }
  
  
}

int GET_BUTTON(){
  int cnt = 0;
  int ans = 0;
  for(int i = 0; i < 4; ++i){
    button_on[i] = digitalRead(BUTTON_PIN[i]);
    if(button_on[i] == 1){
      cnt++;
      ans = i + 1;
    }
    Serial.print(button_on[i]);
    Serial.print('\t');
  }
  Serial.print('\n');

  if(cnt == 0 || cnt > 1){
    return 0;
  }
  return ans;
}

int GET_SET_B(){
  return digitalRead(SET_PIN);
}

void CHECK_PW(){
  if(PW == nowIN){
    nowIN = 0;
    isOpen = true;
    SPIN_OPEN();
  } else {
    nowIN = 0;
    digitalWrite(LED_RED, HIGH);
    delayMicroseconds(500);
    digitalWrite(LED_RED, LOW);
  }
}

void SPIN_OPEN(){
  servo_door.write(50);
  delayMicroseconds(500);
}

void SPIN_CLOSE(){
  servo_door.write(160);
  delayMicroseconds(500);
}
