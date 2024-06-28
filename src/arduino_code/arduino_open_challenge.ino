#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define INTERRUPT_PIN 2
#define meas 20
MPU6050 mpu;
#include <Servo.h>
Servo myservo;
#include "Adafruit_APDS9960.h"
Adafruit_APDS9960 apds;


long delay_prev = 0;
int mindel = 1000, maxdel = 0;

uint16_t r, g, b, c;
int s = 0;
int first_line = 1;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

#define mPin1 8
#define mPin2 11
#define srvPin 9
#define btnPin 12

//long duration;
int distance;
int minDis, readA0, readA1, readA2;
int readA[25], temp;
int i, j;
int a0, a1, a2;
int disL = 45;
int parkL = 110, parkH = 160;
int cen = 96;
int rot = 35;
int rotc = 3;
int hsp = 170, lsp = 170, ssp = 255, initsp = 125; //normal hsp=130 lsp=100
int maxd;
int mmax = 3;
int disLf = 100, disRt = 100;
long dis = 500, prevdis = 500, nowdis;
long prev_wall;
//float mo;
long prev = 0;
int dir = 1;
int turn = 0;
float ang, nang, diff;
int turnn = 0, trnm;
int turnDir = 0;
int clr = 0;
long cprev, ultraprev = 0, turnprev;
int ultra_delay = 30, queue = 1;
float add_angle = 0;
int protect = 0;
long prot_pr = 0;
long sum;


void setup() {

  pinMode(btnPin, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Serial.begin(115200);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  Serial.println(devStatus);



  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  myservo.attach(srvPin);

  if (!apds.begin())
  {
    Serial.println("failed to initialize device! Please check your wiring.");
  }
  else Serial.println("Device initialized!");
  apds.enableColor(true);
  
  pinMode(mPin1, OUTPUT);
  pinMode(mPin2, OUTPUT);

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  myservo.write(cen);
  digitalWrite(13, LOW);


  while (digitalRead(btnPin) == HIGH)
  {
    delay(10);
  }
  analogWrite(mPin2, hsp);
  digitalWrite(mPin1, LOW);
  delay(300);
  analogWrite(mPin2, initsp);


  nang = add_angle;
  delay_prev = millis();
  ultraprev = millis();
  turnprev = millis();
}


//================ LOOP =========================
void loop() {

  //checks for the first line color, runs only until finds the first line
  if (first_line == 1 && apds.colorDataReady())
  {
    apds.getColorData(&r, &g, &b, &c);
    if (c < 28)
    {
      if (r > g && r > b && first_line == 1)
      {
        prev = millis();
        turnDir = 2; //2=turn right
        first_line = 0;
      }
      else if (b > g && b > r && first_line == 1)
      {
        prev = millis();
        s++;
        turnDir = 1; //1=turn left
        first_line = 0;
      }
    }
  }
  

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Read from the gyroscope/accelerometer sensor
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    ang = ypr[0] * 180 / M_PI;    
    //    Serial.println(ang);
  }

  if (millis() - ultraprev > ultra_delay)
  {
    dis = analogRead(A0);
    Serial.print(" dis= "); Serial.print(dis);
   
    disLf = analogRead(A1);
    disRt = analogRead(A2);
    ultraprev = millis();
  }
 


  if (turn == 0 && disLf > 300)
  {
    turn = 3;
    myservo.write(cen + 15);
    prev_wall = millis();
  }

  if (turn == 0  && disRt > 300)
  {
    turn = 3;
    myservo.write(cen - 15);
    prev_wall = millis();
  }

  if (turn == 3 && millis() - prev_wall > 600)
  {
    turn = 0;
    myservo.write(cen);
  }



  //turn=0 initial state(just go straight)
  //turn=1 turn on a corner state
  //turn=2 goes stright but need a small correction steering
  //turn=3 goes straight but one of the corner distance sensors detect a nearby wall. Correction steering need
  if ((turn == 0 || turn == 2) && dis > 350 && dis < 400 && millis() - turnprev > 1500)
  {
    prev = millis();
    turn = 1;
    digitalWrite(13, HIGH);

    trnm = turnn % 4;

    if (turnDir == 1) //turn left
    {
      add_angle = add_angle - 1;
      if (trnm == 0) nang = 90 + add_angle;
      else if (trnm == 1) nang = 180 + add_angle;
      else if (trnm == 2) nang = -90 + add_angle;
      else if (trnm == 3) nang = add_angle;
      myservo.write(cen - rot);
    }
    else if (turnDir == 2) //turn right
    {
      add_angle = add_angle + 1;
      if (trnm == 0) nang = -90;
      else if (trnm == 1) nang = 180 + add_angle;
      else if (trnm == 2) nang = 90 + add_angle;
      else if (trnm == 3) nang = add_angle;
      myservo.write(cen + rot);
    }

    analogWrite(mPin2, hsp);
  }

  if (trnm == 1) //convert negative angle to a positive one
  {
    if (ang <= -135)
      ang = ang + 360;
  }


  diff = abs(nang - ang);
  if (turn == 0 && diff > 1) //automatic direction correction
  {
    turn = 2;
    rotc = 2 * diff + 3;
    if (rotc > rot)
      rotc = rot;
    if (nang > ang)
    {
      cprev = millis();
      myservo.write(cen - rotc);
    }
    else
    {
      cprev = millis();
      myservo.write(cen + rotc);
    }
  }

  if (turn == 2 && millis() - cprev > 200)
  {
    myservo.write(cen);
    turn = 0;
  }

  if (turn == 1 && abs(nang - ang) < 15) //ready to go straight
  {
    myservo.write(cen);
    analogWrite(mPin2, lsp);
    turn = 0;
    digitalWrite(13, LOW);
    turnn += 1; //number of turns counter
    turnprev = millis();
  }

  prot_pr = millis();
  diff = abs(nang - ang);

  //after 12 turns enter into parking mode
  if (turnn == 12)     // && diff < 2)
  {
    analogWrite(mPin2, initsp);
    
    while (millis() - prot_pr < 3000)
    {
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Read from the gyroscope/accelerometer sensor
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        ang = ypr[0] * 180 / M_PI;
      }
      diff = abs(nang - ang);
      rotc = 3 * diff;
      if (rotc > rot)
        rotc = rot;
      if (nang > ang)
      {
        myservo.write(cen - rotc);
      }
      else
      {
        myservo.write(cen + rotc);
      }
    }
    digitalWrite(mPin1, HIGH);
    digitalWrite(mPin2, HIGH);
    prot_pr = millis();

    while (true)
    { 
      sum = 0;
      for (i = 0; i < 30; i++)
      {
        sum += analogRead(A0);
        delay(20);
      }
      distance = sum / 30;
	  //0.67v=100cm  //~0.45v=150cm
      if (distance < 130)//move forward
        {
          digitalWrite(mPin1, LOW);
          digitalWrite(mPin2, HIGH);
          delay(150);
          digitalWrite(mPin1, LOW);
          digitalWrite(mPin2, LOW);
          delay(500);
        }
      else if (distance > 110)//move backwards
      {
        digitalWrite(mPin1, HIGH);
        digitalWrite(mPin2, LOW);
        delay(300);
        digitalWrite(mPin1, LOW);
        digitalWrite(mPin2, LOW);
        delay(200);
      }
      else      
      {
        digitalWrite(13, HIGH);
        digitalWrite(2, LOW);
        while (true);
      }
    }

  }

  delay_prev = millis();
}
