#include <tcs3200.h>
#include "Wire.h"
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 18  // use pin 18 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

float angle=abs(ypr[0]);                          // variable for angle DATA - Z Axis-

// DC motor parameters and pins
byte dcmotfpin=4;                     // output pin for FORWARD DC driving motor 
byte dcmotbpin=5;                     // output pin for BACKWARD DC driving motor 
int speedo=200;                       // speed for driving dc motor

// stearing motor parameters and pins
//byte stearRmotpin=4;                  
//byte stearLmotpin =5;               
Servo servo;
int pos = 90 ;                       // servo motor intial angle position in forward position
byte servopin = 7;                    // output pwm pin for servo stearing motor


// color sensor parameters and pins
tcs3200 rtcs(42, 40, 44, 46, 50);    // pins for RIGHT color sensor  (S0, S1, S2, S3, output pin)  
tcs3200 ltcs(32, 34, 26, 28, 30);    // pins for LEFT color sensor   (S0, S1, S2, S3, output pin)  

int red, green, blue, white;         // variables for RGB and clear color DATA

int ominred = 70, omingreen = 40, ominblue = 50, ominwhite = 120;  // parameter for minimum RGB values in orange color
int omaxred = 90, omaxgreen = 70, omaxblue = 90, omaxwhite = 200;  // parameter for maximum RGB values in orange color

int bminred = 34, bmingreen = 40, bminblue = 50, bminwhite = 142;  // parameter for minimum RGB values in blue color
int bmaxred = 55, bmaxgreen = 83, bmaxblue = 100, bmaxwhite = 166;  // parameter for maximum RGB values in blue color

//bool cw=false,ccw=false, intcolor=true;

int cornerlinecount=1;                // count corner line for achivie 3 laps   if counter= 12 then 3 laps              
bool center=false;
bool enLcolor = true, enRcolor = true ;


// start button parameters and pins
byte startpin=45;                     // input pin for start push button                ////////////////////////////تعديل البننات///////////////////////////////////////////////////////////////
byte startsignal;                     // DATA from button - HIGH or LOW-

// ultrasonic sensor parameters and pins
#define fecho 48                      // input echo pin for FORWARD ultrasonic
#define ftrig 46                      // output trig pin for FORWARD ultrasonic

#define recho 8                     // input echo pin for RIGHT ultrasonic
#define rtrig 9                     // output trig pin for RIGHT ultrasonic

#define lecho 3                      // input echo pin for LEFT ultrasonic
#define ltrig 2                      // output trig pin for LEFT ultrasonic

#define becho 10 //31                     // input echo pin for BACKRIGHT ultrasonic
#define btrig 11 //30                      // output trig pin for BACKRIGHT ultrasonic

#define aecho 50 //53                      // input echo pin for BACKLEFT ultrasonic
#define atrig 52 //52                      // output trig pin for BACKLEFT ultrasonic

char pioutput;
int fdistance,rdistance,ldistance,adistance,bdistance;
int intfdistance,intrdistance,intldistance,intadistance,intbdistance;
bool intdistance=true;
long duration;

int lap = 1;

int uldiff=15; 
int minf=intfdistance-uldiff;
int maxf=intfdistance+uldiff;
int minr=intrdistance-uldiff;
int maxr=intrdistance+uldiff;
int minl=intldistance-uldiff;
int maxl=intldistance+uldiff;

bool restartLoop= true;

void setup() 
{
  Serial.begin(115200);
  Wire.begin();
  delay(1000);
  pinMode(ftrig, OUTPUT); 
  pinMode(fecho, INPUT);
  pinMode(rtrig, OUTPUT); 
  pinMode(recho, INPUT);
  pinMode(ltrig, OUTPUT); 
  pinMode(lecho, INPUT);
  pinMode(atrig, OUTPUT); 
  pinMode(aecho, INPUT);
  pinMode(btrig, OUTPUT); 
  pinMode(becho, INPUT);
  pinMode(dcmotfpin, OUTPUT);
  pinMode(dcmotbpin, OUTPUT);
  digitalWrite(dcmotfpin,LOW);
  digitalWrite(dcmotbpin,LOW);
  intdistance = true;
  readdistance(ftrig, fecho, 'F');
  delay(50);
  readdistance(rtrig, recho, 'R');
  delay(50);
  readdistance(ltrig, lecho, 'L');
  Serial.print("intfdis= ");
  Serial.println(intfdistance);
  delay(500);
  Serial.print("intrdis= ");
  Serial.println(intrdistance);
  delay(500);
  Serial.print("intldis= ");
  Serial.println(intldistance);
  servo.attach(servopin);
  servo.write(pos);
  while(true)
  {
    startsignal=digitalRead(startpin);
    if(startsignal == HIGH)
    {
      break;
    }
  }
}



void loop()
{
  //straight();
  forward(150);
  delay(10);
  readdistance(ftrig, fecho, 'F');
  delay(10);
  readdistance(rtrig, recho, 'R');
  if(Serial.available()>0)
    {
      pioutput = Serial.read();
      Serial.println("in serial loop");
      /*if(Serial.available() < 0)
      {
        Serial.println(pioutput);
        break;
      }*/
    }
    A_OR_B();
    F_OR_E();
    C_OR_D();
    //G_OR_H();
  if( (fdistance < 90)  &&  (rdistance > 150) ) // بتتعير حسب الرجعة اذا بلوصت
  {
    //santara_cw();
    right();
   
  }
  readdistance(ftrig, fecho, 'F');
  delay(10);
  readdistance(ltrig, lecho, 'L');
  
 /* if( (fdistance < 65)  &&  (ldistance > 150) )
  {
    ccw();
  }*/
}
void G_OR_H()
{
  while(true)
  {
    if(pioutput == 'G' || pioutput == 'H')
    {
      while( Serial.available() > 0)
      {
        pioutput = Serial.read();
        if(Serial.available() < 0)
        {
          break;
        } 
      }
      stopdriving();
      straight();
      backward(200);
      delay(1000);
      stopdriving();
      delay(100);
      forward(200);
    }
  }
  pioutput='@';
}
void C_OR_D()
{
  if((pioutput == 'C') || (pioutput == 'D')) // C = red pillars on the left // D = green pillars on the right.
  {
    straight();
    pioutput='@';
    return;
  }
}
void F_OR_E()
{
  if(pioutput == 'F' || pioutput == 'E') // F = green pillars on the left // green pillars on the middle.
  {
    left();
   while( Serial.available() > 0)
    {
      pioutput = Serial.read();
      if(Serial.available() < 0)
      {
        break;
      }
    }
    if(pioutput == 'D')               // D = green pillars on the right.
    {
      straight();
      readdistance(rtrig, recho, 'R');
      if(rdistance < 15)
      {
        delay(100);
        right();
        readdistance(btrig, becho, 'M');
        if(bdistance < 15)
        {
          right();
          delay(100);
          straight();
          //break;
        }
      }
    }
  }
  pioutput = '@';
  return;
}
void A_OR_B()
{
  if(pioutput == 'A' || pioutput == 'B') // A = red pillars on the right // B = red pillars on the middle.
  {
    Serial.println("in A OR B fn");
    right();
    //delay(1000);
    while(true)
    {
        if( Serial.available() > 0)
        {
          pioutput = Serial.read();
         
        }
        while(pioutput == 'C')               // C = red pillars on the left.
        {
          straight();
          readdistance(ltrig, lecho, 'L');
          Serial.println(ldistance);
          if(ldistance < 7)
          {
            while(true)
            {
              delay(500); //   تتعير على حسب المدة الي بدو الروبوت يقعدها ليعدي عن المكعب ويقرء الاترا 
              left();
              readdistance(btrig, becho, 'N');
              Serial.println(bdistance);

              if(bdistance < 7)
              {
                //left();
                delay(100);
                straight();
                
                break;
              }
            }
            break;
          }
          pioutput= '@';
      }
    }
  }
  return;
}
void santara_cw()
{
  stopdriving();
  delay(100);
  left();
  backward(255);
  delay(500);// بتعدل على حسب المدة الي بحتاجها الروبوت عشان يصير قريب على الجدار
  straight();
  stopdriving();
  delay(100);
  forward(255);

  
  return; // تاكد من صحتها
}
void cw()
{
  lap++;
  Serial.print("lap = "); 
  Serial.println(lap);
  right();
  delay(600);
  while(true)
  {
    
    
    if(abs(ldistance-adistance) < 3)
    {
      straight();
      delay(3);
      forward(255);
      break;
    }
    //delay(15);
    if(lap == 13)
    {
      while(true)
      {
        straight();
        delay(10);
        forward(200);
        readul();
        //if(minf < intfdistance && intfdistance < maxf && minr < intrdistance && intrdistance < maxr && minl < intldistance && intldistance < maxl )
        if((minf < intfdistance < maxf) && (minr < intrdistance < maxr) && (minl < intldistance < maxl))
        {
          while(true)
          {
            //delay(100);
            stopdriving();
            Serial.println("Mission complete !");
          }
        }
      }
    }
  }
  return;
}

void ccw()
{
  lap++;
  left();
  
  
  
    //delay(1000);                                                        ////////////////تعديل الوقت ما قبل القرائة//////////////////////////////////////
    
    
    

    if(abs(rdistance-bdistance) < 3)
    {
      straight();
      delay(3);
      forward(255);
      //break;
    }
    //delay(15);
    if(lap == 13)
    {
      while(true)
      {
        readdistance(rtrig, recho, 'R');
        //delay(10);
        readdistance(btrig, becho, 'B');
    
        if(abs(rdistance-bdistance) < 3)
        {
          straight();
          delay(10);
          forward(200);
          readul();
          //if(minf < intfdistance && intfdistance < maxf && minr < intrdistance && intrdistance < maxr && minl < intldistance && intldistance < maxl )
          if((minf < intfdistance < maxf) && (minr < intrdistance < maxr) && (minl < intldistance < maxl))
          {
            while(true)
            {
              //delay(100);
              stopdriving();
              Serial.println("Mission complete !");
            }
          }
        }
        
      }
    }
     return;
  
 
}

void readdistance(int trigPin, int echoPin, char u)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  if(u=='F')
  {
    fdistance = duration * 0.034 / 2;
    if(intdistance == true)
    {
      intfdistance = duration * 0.034 / 2;
     // intdistance = false;
    }
  }
  if(u=='R')
  {
    rdistance = duration * 0.034 / 2 ;
    if(intdistance == true)
    {
      intrdistance = duration * 0.034 / 2;
     // intdistance = false;
    }
  }
  if(u=='L')
  {
    ldistance = duration * 0.034 / 2 ;
    if(intdistance == true)
    {
      intldistance = duration * 0.034 / 2;
     // intdistance = false;
    }
  }
  if(u=='A') // bl
  {
    adistance = duration * 0.034 / 2;
    if( intdistance == true)
    {
      intadistance = duration * 0.034 / 2;
      //intdistance = false;
    }
  }
  if(u=='B') // br
  {
    bdistance = duration * 0.034 / 2;
    if( intdistance == true)
    {
      intbdistance = duration * 0.034 / 2;
      intdistance = false;
    }
  }
}

void readul()
{
  readdistance(ftrig, fecho, 'F');
  delay(10);
  readdistance(rtrig, recho, 'R');
  delay(10);
  readdistance(ltrig, lecho, 'L');
}

void straight()
{
  pos= 90;
  servo.write(pos);
}

void right()
{
  pos=140;
  servo.write(pos);
}

void left()
{
  pos=50;
  servo.write(pos);
}

void stopdriving()
{
  digitalWrite(dcmotfpin,LOW);
  digitalWrite(dcmotbpin,LOW);
}

void backward(int speedo)
{
  analogWrite(dcmotbpin,speedo);
  digitalWrite(dcmotfpin,LOW);
}

void forward(int speedo)
{
  //digitalWrite(dcmotfpin,HIGH); // moving FORWARD
  analogWrite(dcmotfpin,speedo);
  digitalWrite(dcmotbpin,LOW);
  //delay(500);
  //digitalWrite(dcmotfpin,LOW);
  //digitalWrite(dcmotbpin,LOW);
}

