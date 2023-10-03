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
byte dcmotfpin=2;                     // output pin for FORWARD DC driving motor 
byte dcmotbpin=4;                     // output pin for BACKWARD DC driving motor 
int speedo=200;                       // speed for driving dc motor

// stearing motor parameters and pins
//byte stearRmotpin=4;                  
//byte stearLmotpin =5;               
Servo servo;
int pos = 90 ;                       // servo motor intial angle position in forward position
byte servopin = 5;                    // output pwm pin for servo stearing motor


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
byte startpin=45;                     // input pin for start push button                
byte startsignal;                     // DATA from button - HIGH or LOW-

// ultrasonic sensor parameters and pins
#define fecho 46                      // input echo pin for FORWARD ultrasonic
#define ftrig 48                      // output trig pin for FORWARD ultrasonic

#define recho 9                     // input echo pin for RIGHT ultrasonic
#define rtrig 8                     // output trig pin for RIGHT ultrasonic

#define lecho 6                      // input echo pin for LEFT ultrasonic
#define ltrig 7                      // output trig pin for LEFT ultrasonic

#define becho 10 //31                     // input echo pin for BACKRIGHT ultrasonic
#define btrig 11 //30                      // output trig pin for BACKRIGHT ultrasonic

#define aecho 50 //53                      // input echo pin for BACKLEFT ultrasonic
#define atrig 52 //52                      // output trig pin for BACKLEFT ultrasonic


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

/*void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
}*/

void setup() 
{
  Serial.begin(115200);
  Wire.begin();
  //mpusetup();                              //gyro had some issues ... 
  delay(1000);
  //pinMode(24,OUTPUT);                      // left color sensor enable
  //pinMode(48,OUTPUT);                      // right color sensor enable
  //digitalWrite(48,LOW);
  //digitalWrite(24,LOW);
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
  //readgyro();
  //Serial.println(angle);
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
  straight();
  forward(255);
  readdistance(rtrig, recho, 'R');
  delay(10);
  readdistance(ftrig, fecho, 'F');
  //Serial.print("r = "); 
  //Serial.println(rdistance);
  if( (fdistance < 60)  &&  (rdistance > 150) )
  {
    lap++;
    Serial.print("lap = "); 
    Serial.println(lap);
    forward(220);
    right();
    delay(600);
    while(true)
    {
      delay(5);                                                        //delay is changeable.
      digitalWrite(ltrig, LOW);
      delayMicroseconds(2);
      digitalWrite(ltrig, HIGH);
      delayMicroseconds(10);
      digitalWrite(ltrig, LOW);
      duration = pulseIn(lecho, HIGH);
      ldistance = duration * 0.034 / 2;

      digitalWrite(atrig, LOW);
      delayMicroseconds(2);
      digitalWrite(atrig, HIGH);
      delayMicroseconds(10);
      digitalWrite(atrig, LOW);
      duration = pulseIn(aecho, HIGH);
      adistance = duration * 0.034 / 2;

      
                              //fallior old tests:
                              //gyro test ...
                                        //readdistance(ltrig, lecho, 'L');
                                        //delay(10);
                                        //readdistance(atrig, aecho, 'A');
                                        //(0.00 < angle < 0.02) ||
                                        //|| (3.06 < angle < 3.09) || ( -3.09 < angle < -3.06 ) || ( -1.57 < angle < -1.54)
                                        //|| (angle > 2.93) 
                                        //|| ((angle > -3.00 && (angle < -2.94)) || ((angle > -1.54) && (angle < -1.51)) || ((angle < 0.10) && (angle > 0.5)))
                                        //if( (ldistance < 65)  &&  (adistance > 150) )
      
      
      //Serial.print("ldis= ");
      //Serial.print("adis= ");
      //Serial.println(adistance);
      //Serial.println(abs(ldistance-adistance));



      if(abs(ldistance-adistance) < 6)
      {
        straight();
        delay(3);
        forward(255);
        //restartLoop=true;
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
          //readul();
          Serial.print(" F = ");
          Serial.print(fdistance);
          Serial.print(" , R = ");
          Serial.print(rdistance);
          Serial.print(" , L = ");
          Serial.println(ldistance);
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
      Serial.println(ldistance);
    }
  }
  readdistance(ftrig, fecho, 'F');
  delay(10);
  readdistance(ltrig, lecho, 'L');
  if( (fdistance < 65)  &&  (ldistance > 150) )
  {
    lap++;
    Serial.print("lap = "); 
    Serial.println(lap);
    forward(255);
    left();
    delay(1000);
    while(true)
    {
      //delay(1000);                                                       
      readdistance(rtrig, recho, 'R');
      //delay(10);
      readdistance(btrig, becho, 'B');

                                //fallior old tests:
                                ////gyro test ...
                                      //(0.00 < angle < 0.02) ||
                                      //|| (3.06 < angle < 3.09) || ( -3.09 < angle < -3.06 ) || ( -1.57 < angle < -1.54)
                                      //|| (angle > 2.93) 
                                      //|| ((angle > -3.00 && (angle < -2.94)) || ((angle > -1.54) && (angle < -1.51)) || ((angle < 0.10) && (angle > 0.5)))
                                      //if( (ldistance < 65)  &&  (adistance > 150) )


      Serial.print("rdis= ");
      Serial.println(rdistance);
      Serial.print("bdis= ");
      Serial.println(bdistance);
      Serial.println(abs(rdistance-bdistance));
      if(abs(rdistance-bdistance) < 6)
      {
        straight();
        delay(3);
        forward(255);
        //restartLoop=true;
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
          //readul();
          Serial.print(" F = ");
          Serial.print(fdistance);
          Serial.print(" , R = ");
          Serial.print(rdistance);
          Serial.print(" , L = ");
          Serial.println(ldistance);
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
  }
}


void readgyro()
{
  // if programming failed, don't try to do anything
    //delay(1);
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
            
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("ypr\t");
            //Serial.println(ypr[0] * 180/M_PI);
            angle=ypr[0] * 180/M_PI;
            //angle = (90*angle)/1.54;
            /*Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);*/
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
            
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
            
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
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

void mpusetup()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


   // Serial.begin(115200);
    //while (!Serial); 

    
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
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
    } else {

        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}
