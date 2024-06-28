#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#include <Servo.h>
Servo myservo;
#include "Adafruit_APDS9960.h"
Adafruit_APDS9960 apds;
#define srvPin 9
#define btnPin 12
#define mPin1 8
#define mPin2 11
#define white_color 28

#define INTERRUPT_PIN 2
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

String fir_obs = "";
String sec_obs = "";
String th_obs = "";
String start_obs = "";
long uturn_prev;
String prev_big = "";
int uturn = 0, uturn_ok = 0;
int turnn = 0;
int must_do_uturn = 0;
int disLf, disRt;
int s = 0;
long prev = 5000;
int first_line = 1;
String first_color = "";
String color = "white";
String rec = "";
int ang, direction_angle, angle_distance ;
int in = 0; // stores incoming data from serial port
int dir = 0;
long ultraprev = 0;
long duration = 0;
long a0;
int cen = 101;
int rot = 45;
int hsp = 170, lsp = 130;
long color_delay;
int turnDir;
long stop_prev = 0;
int stop_flag = 0;
int ultra_delay = 15;
uint16_t r, g, b, c;
int d[8], maxd;
int mmax = 3;
long second_color = 0;
bool stop_first = false;
float nang = 0, add_angle = 0, diff;
int trnm;
bool obstacle;
long next_big_obstacle;
int reverse_uturn = 0;
int stop_mode = 0;

void setup() {
  Serial.begin(115200);
  pinMode(btnPin, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  if (!apds.begin())
  {
    Serial.println("failed to initialize device! Please check your wiring.");
  }
  else Serial.println("Device initialized!");
  apds.enableColor(true);

  myservo.attach(srvPin);

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




  pinMode(mPin1, OUTPUT);
  pinMode(mPin2, OUTPUT);


  myservo.write(cen);
  digitalWrite(13, LOW);

  while (digitalRead(btnPin) == HIGH)
  {
    delay(10);
  }

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  analogWrite(mPin2, 0);

  /*for (int i = 0; i < 8; i++)
    {
    d[i] = 100;
    }*/
}


//================ LOOP =========================
void loop() {

  //read accelerometer sensor
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Read from the gyroscope/accelerometer sensor
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    direction_angle = ypr[0] * 180 / M_PI;
    //Serial.print("(arduino) angle= "); Serial.println(direction_angle);
    //    Serial.print("ypr\t");
    //    Serial.println(ang);
  }  


  //read bottom color sensor
  if (apds.colorDataReady())
  {
    apds.getColorData(&r, &g, &b, &c);
    if (c < white_color && millis() - color_delay > 500)
    {

      if (r > g && r > b && first_line == 1)
      {
        prev = millis();

        trnm = turnn % 4;
        add_angle = add_angle + 1;
        if (trnm == 0) nang = -90;
        else if (trnm == 1) nang = 180 + add_angle;
        else if (trnm == 2) nang = 90 + add_angle;
        else if (trnm == 3) nang = add_angle;

        s++;
        turnn++;

        Serial.print("(arduino) s= "); Serial.println(s);
        Serial.print("(arduino) first_color= "); Serial.println("orange");
        Serial.print("(arduino) nang= "); Serial.println(nang);
        first_color = "orange";
        color = "orange";
        first_line = 0;
      }
      else if (b > g && b > r && first_line == 1)
      {
        prev = millis();

        trnm = turnn % 4;
        if (trnm == 0) nang = 90 + add_angle;
        else if (trnm == 1) nang = 180 + add_angle;
        else if (trnm == 2) nang = -90 + add_angle;
        else if (trnm == 3) nang = add_angle;

        s++;
        turnn++;

        Serial.print("(arduino) s= "); Serial.println(s);
        Serial.print("(arduino) first_color= "); Serial.println("blue");
        Serial.print("(arduino) nang= "); Serial.println(nang);
        first_color = "blue";
        color = "blue";
        first_line = 0;
      }
      else if (r > g && r > b && millis() - prev > 4000 && first_line == 0 && first_color == "orange")
      {
        trnm = turnn % 4;
        add_angle = add_angle + 1;
        if (trnm == 0) nang = -90;
        else if (trnm == 1) nang = 180 + add_angle;
        else if (trnm == 2) nang = 90 + add_angle;
        else if (trnm == 3) nang = add_angle;

        s++;
        if (s == 8)
          fir_obs = prev_big;
        if (s == 12)
          prev_big = "";
        turnn++;

        Serial.print("(arduino) s= "); Serial.println(s);
        Serial.print("(arduino) nang= "); Serial.println(nang);

        if (s == 8 || s == 4)
        {
          duration += millis() - prev;
        }
        prev = millis();
        color = "orange";
      }
      else if (b > g && b > r && millis() - prev > 4000 && first_line == 0 && first_color == "blue")
      {


        trnm = turnn % 4;
        if (trnm == 0) nang = 90 + add_angle;
        else if (trnm == 1) nang = 180 + add_angle;
        else if (trnm == 2) nang = -90 + add_angle;
        else if (trnm == 3) nang = add_angle;

        s++;
        if (s == 8)
          fir_obs = prev_big;
        if (s == 12)
          prev_big = "";
        turnn++;

        Serial.print("(arduino) s= "); Serial.println(s);
        Serial.print("(arduino) nang= "); Serial.println(nang);

        if (s == 8 || s == 4)
        {
          duration += millis() - prev;
        }
        prev = millis();
        color = "blue";
      }
      else
      {
        color = "white";
      }
    }
  }

  if (trnm == 1) //convert negative angle to a positive one
  {
    if (direction_angle <= -135)
      direction_angle = direction_angle + 360;
  }


  //============= uturn code ===============
  if ( s == 9 && uturn == 0 && uturn_ok == 0)
  {
    uturn_ok = 1;
    Serial.print(fir_obs); Serial.print("-"); Serial.print(sec_obs); Serial.print("-");
    Serial.print(th_obs); Serial.print("-"); Serial.print(start_obs); Serial.println("");
    if (fir_obs != "" && sec_obs != "" && th_obs != "")
    {
      if (sec_obs == "RED")
      {
        must_do_uturn = 1;

      }
    }
    else if (th_obs == "")
      if (start_obs != "" && fir_obs == "RED")
      {
        must_do_uturn = 1;

      }
    if (start_obs == "" && sec_obs == "RED")
    {
      must_do_uturn = 1;

    }

    if (must_do_uturn == 1)
    {
      Serial.println("must_do_uturn");
      if (th_obs == "GREEN" || (th_obs == "" && start_obs == "GREEN"))
        reverse_uturn = 1;
      turnn -= 2;
      uturn = 1;
      if (s == 9)
        s = 8;
    
      if (th_obs == "GREEN")
        myservo.write(cen + 40);
      else
        myservo.write(cen - 40); 
      analogWrite(mPin2, hsp);
      
      if (first_color == "orange")
        first_color = "blue";
      else if (first_color == "blue")
        first_color = "orange";
    }
  }

  if (uturn == 1 && reverse_uturn == 0 && abs(direction_angle - 90) < 10)
  {
    uturn = 0;
    
    myservo.write(cen);
    digitalWrite(mPin1, HIGH);
    digitalWrite(mPin2, HIGH);
    uturn = 2;
    myservo.write(cen + 35);
    uturn_prev = millis();
    
    analogWrite(mPin2, 0);
    analogWrite(mPin1, lsp);
  }
  else if (uturn == 1 && reverse_uturn == 1 && abs(-90 - direction_angle) < 10)
  {
    uturn = 0;
    
    myservo.write(cen);
    digitalWrite(mPin1, HIGH);
    digitalWrite(mPin2, HIGH);
    uturn = 2;
    myservo.write(cen - 35); 
    uturn_prev = millis();
    
    analogWrite(mPin2, 0);
    analogWrite(mPin1, lsp);
  }



  if (uturn == 2 && reverse_uturn == 0 && dir == 0)
  {
    analogWrite(mPin2, 0);
    analogWrite(mPin1, lsp);
  }
  else if (uturn == 2 && dir != 0 && reverse_uturn == 0 &&  millis() - uturn_prev > 500)
  {
    uturn = 0;
    analogWrite(mPin1, 0);
    analogWrite(mPin2, hsp);
    nang = 180;
  }
  else if (uturn == 2 && reverse_uturn == 1 &&  millis() - uturn_prev > 500 && abs(direction_angle - 180) < 10)
  {
    uturn = 0;
    analogWrite(mPin1, 0);
    analogWrite(mPin2, hsp);
    nang = 180;
  }


  if (s == 12 && stop_flag == 0)
  {
    stop_flag = 1;
    if (must_do_uturn == 0) //no uturn
      if (th_obs == "")
        if (start_obs == "")
          stop_mode = 1; //stop after 1st obstacle
        else
          stop_mode = 2; //stop after find big obstacle and do back movement
      else
        stop_mode = 1; //stop after 1st obstacle
    else //uturn done
      if (start_obs != "")
        stop_mode = 1; //stop after 1st obstacle
      else
        stop_mode = 2; //stop after find big obstacle and do back movement
  }

  if (stop_flag == 1 && prev_big != "" && stop_mode == 1)
  {
    prev = millis();
    stop_flag = 2;
  }
  else if (stop_flag == 1 && prev_big != "" && stop_mode == 2)
  {
    digitalWrite(mPin1, HIGH);
    digitalWrite(mPin2, HIGH);
    analogWrite(mPin2, 0);
    analogWrite(mPin1, lsp);
    myservo.write(cen);
    prev = millis();
    stop_flag = 3;
  }

  if (stop_flag == 2 && millis() - prev > 1500)
  {
    digitalWrite(mPin1, HIGH);
    digitalWrite(mPin2, HIGH);
    while (true)
    {
      delay(5);
    }
  }
  else if (stop_flag == 3 && millis() - prev > 1500)
  {
    digitalWrite(mPin1, HIGH);
    digitalWrite(mPin2, HIGH);
    while (true)
    {
      delay(5);
    }
  }



  if (digitalRead(btnPin) == LOW)
  {
    analogWrite(mPin2, lsp);
    color_delay = millis();
  }

  //read messages from serial
  if (Serial.available() > 0)
  {
    // read the incoming byte:
    in = Serial.read();
    if (in == 71 || in == 82)
    {
      if (in == 82)
      {
        if (s == 8 && sec_obs == "")
        {
          sec_obs = "RED";
          next_big_obstacle = millis();
        }
        if (s == 8 && sec_obs != "" && th_obs == "" && millis() - next_big_obstacle > 1000)
        {
          th_obs = "GREEN";
        }
        prev_big = "RED";
        if (s == 0)
          start_obs = "RED";
        Serial.println("RED!!!");
      }
      else
      {
        if (s == 8 && sec_obs == "")
        {
          sec_obs = "GREEN";
          next_big_obstacle = millis();
        }
        if (s == 8 && sec_obs != "" && th_obs == "" && millis() - next_big_obstacle > 1000)
        {
          th_obs = "GREEN";
        }
        prev_big = "GREEN";
        if (s == 0)
          start_obs = "GREEN";
        Serial.println("GREEN!!!");
      }
    }
    else if (in != 115)
    {
      rec = rec + (char)in;
    }
    else
    {

      if (rec[0] == char(108)) dir = -1;
      else if (rec[0] == char(114)) dir = 1;
      else if (rec[0] == char(110)) dir = 0;

      rec.remove(0, 1);
      ang = rec.toInt();

      rec = "";
    }
  }

  //read from corner distance sensors
  if (uturn == 0)
  {
    //8cm = 1.58v = 323 arduino read value
    //4cm = 2.7v = 553 arduino read value
    disLf = analogRead(A1);
    disRt = analogRead(A2);
    if (disLf > 300)
    {
      angle_distance = (disLf - 300) / 10 + 35;
      if (angle_distance > 50)
        angle_distance = 50;
      myservo.write(cen + angle_distance);
      digitalWrite(13, HIGH);
      analogWrite(mPin2, hsp);
    }
    else if (disRt > 300)
    {
      angle_distance = (disRt - 300) / 10 + 35;
      if (angle_distance > 50)
        angle_distance = 50;
      myservo.write(cen - angle_distance);
      digitalWrite(13, HIGH);
      analogWrite(mPin2, hsp);
    }
    else
    {
      digitalWrite(13, LOW);
      analogWrite(mPin2, lsp);
      if (dir == -1)
      {
        myservo.write(cen - ang);
        if (abs(ang) > 10)
          analogWrite(mPin2, hsp);
      }
      else if (dir == 1)
      {
        myservo.write(cen + ang);
        if (abs(ang) > 10)
          analogWrite(mPin2, hsp);
      }
      else if (dir == 0)
      {
        if (disLf <= 300 && disRt <= 300)
        {
          diff = abs(direction_angle - nang); 
          if (diff > 30)
            diff = 30;
          if (diff < -30)
            diff = -30;
          
          if (nang > direction_angle)
          {
            myservo.write(cen - diff);
          }
          else
          {
            myservo.write(cen + diff);
          }
        }
        
        else if (first_color == "blue")
        {
          myservo.write(cen - rot);
        }
        else if (first_color == "orange")
        {
          myservo.write(cen + rot);
        }
      }
    }
  }

}
