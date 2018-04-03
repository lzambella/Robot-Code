#include "I2Cdev.h"
#include "Wire.h"
#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE 
#include "MPU6050_6Axis_MotionApps20.h"
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 3
#define LED_PIN 12
#define THRESHOLD_ANGLE 45 // The angle our bot will gnerally rotate at

MPU6050 mpu;

int silentTest=0;
byte IN1 = 6;
byte IN2 = 7;
byte IN3 = 5;                        
byte IN4 = 4;
byte backTicks=0;                          
byte rWheelSpeedMax = 150; //prevent burnout
byte lWheelSpeedMax = 100;
byte rWheelSpeed = rWheelSpeedMax;
byte lWheelSpeed = lWheelSpeedMax;
byte boundsThresh = 20;
byte frontPin = 2; //front pin
byte frontLED = 22; // on: robot is turning

const int lEnable = 12;
const int rEnable = 13;

const int button = 23;
bool enabled = false;

int errorDistance=0;

// Gyroscope variables

// For change in angle measurement
double angleInitial = -1; // snapshot of stored angle
double angleCurrent = 0; //Current angle read from the gyroscope
double angleFinal = 0; // Our angle we want the robot to turn at, gets set in the turn method

int SWITCHSTATE = 0; // checks whether the switch has been pressed
bool STATE = false;
bool rotating = false; // If the robot is currently rotating

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
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void gyroscopeSetup(){
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

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
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

void setup()
{
    Serial.begin(115200);
    gyroscopeSetup();
    pinMode(IN2, OUTPUT);  
    pinMode(IN4, OUTPUT);
    pinMode(8, OUTPUT); // Sets the trigPin as an Output
    pinMode(9, INPUT); // Sets the echoPin as an Input
    pinMode(10, OUTPUT); // Sets the trigPin as an Output
    pinMode(11, INPUT); // Sets the echoPin as an Input
    pinMode(frontLED, OUTPUT);
    pinMode(button, INPUT);
    
    pinMode(24, OUTPUT); // stop LED
    digitalWrite(24, HIGH); // enable it
}

 /*
  * 
  * 
  *  Main Program Loop
  * 
  */
void loop()
{
  while (!enabled) {
    if (digitalRead(button) == HIGH) {
      enabled = true;
      digitalWrite(24, LOW); // Disable the red led
    }
  }
  int value;
  //delay(30);
  backTicks+=1;
  int frontDist = getDistance(frontPin);
  int leftDistance = getLeftDistance();
  int rightDistance = getRightDistance();
  errorDistance = leftDistance-rightDistance;
  /*
  Serial.print(String(leftDistance) + " " + frontDist + " " + String(rightDistance));
  Serial.println();
  Serial.print("Error distance: " + String(errorDistance));
  Serial.println();
  Serial.print("Back ticks: " + String(backTicks));
  Serial.println();
  */

        // if programming failed, don't try to do anything
    if (!dmpReady) return;

    
    do {
      // other program behavior stuff here
      // Any code not related to the gyroscope should be placed here
      
      //if (rotating) Serial.println("Currently Rotating!");
      if (rotating && abs(angleCurrent - angleInitial) >= abs(angleFinal)) { 
          // if we reached our threshold angle
          moveForward();
          Serial.println("Sucessfully rotated!");
          rotating = false;
      }
      else if (!rotating) {  
        if(backTicks >= 80)
        {
          backTicks=0;
          moveBackward();
          //delay(10);
          if(leftDistance > rightDistance)
            turn(-1, 45);
          else turn(1, 45);
          //delay(10);
          stopMovement();
        }     
        if(frontDist <= 9)
        {
          
          digitalWrite(frontLED,HIGH);
          if(rightDistance <= 7)
          {
            if(frontDist <= 4 && rightDistance <= 4)
                moveBackward();
            else
                turn(-1, 45);        
          }
          else
          {
            turn(1, 45);
          }
          //delay(100);
          //moveForward();
          //delay(5);
        }
        else if(leftDistance < boundsThresh && rightDistance < boundsThresh) //if we're within wall range
        {
          /*if(errorDistance > 2) //if its far on the right side, turn left a bit
          {
            lWheelSpeed = lWheelSpeedMax*0.8;
            rWheelSpeed = rWheelSpeedMax;
            Serial.println("Lean left");
          }
          else if(errorDistance < -2)
          {
            rWheelSpeed = rWheelSpeedMax*0.8;
            lWheelSpeed = lWheelSpeedMax;
            Serial.println("Lean right");
          }
          else
          {
            lWheelSpeed = lWheelSpeedMax;
            rWheelSpeed = rWheelSpeedMax;
            Serial.println("Normal");
          }*/
          if(leftDistance <= 4)
          {
            turn(1, 45);
            //delay(10);
          }
          else if(rightDistance <= 4)
          {
            turn(-1, 45);
            //delay(10);
          }
          moveForward();
        }
        else
        {
          lWheelSpeed = lWheelSpeedMax;
          rWheelSpeed = rWheelSpeedMax;
          digitalWrite(frontLED,LOW);
          moveForward();
        }
      }
    } while (!mpuInterrupt && fifoCount < packetSize);
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            Serial.print(rotating);
            Serial.print("\t");
            Serial.print(angleInitial);
            Serial.print("\t");
            Serial.println(angleFinal);
            angleCurrent = ypr[0] * 180/M_PI;
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
void moveForward()
{
    if(silentTest)
      return;
    digitalWrite(IN1,LOW);   
    digitalWrite(IN2,HIGH); 

    digitalWrite(IN3,LOW);   
    digitalWrite(IN4,HIGH);

    analogWrite(lEnable, lWheelSpeed);
    analogWrite(rEnable, rWheelSpeed);
}
void moveBackward()
{
    if(silentTest)
      return;
    digitalWrite(IN1,LOW);   
    digitalWrite(IN2,HIGH);
    
    digitalWrite(IN3,LOW);   
    digitalWrite(IN4,HIGH);

    analogWrite(lEnable, lWheelSpeed);
    analogWrite(rEnable, rWheelSpeed);
}
void stopMovement()
{
    digitalWrite(IN1,LOW);   
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);   
    digitalWrite(IN4,LOW);
}
void turn(int direction)
{
  //0 = forward, -1= left, 1=right
  if(silentTest)
    return;
  if(direction == -1)
  {
    digitalWrite(IN1,LOW);   
    digitalWrite(IN2,HIGH); 
    digitalWrite(IN3,HIGH);   
    digitalWrite(IN4,LOW);
    analogWrite(lEnable, lWheelSpeed);
    analogWrite(rEnable, rWheelSpeed);       
    Serial.println("Turning left");
    //delay(5);
  }
  if(direction == 1)
  {
    digitalWrite(IN1,HIGH);   
    digitalWrite(IN2,LOW); 
    digitalWrite(IN3,LOW);   
    digitalWrite(IN4,HIGH);   
    analogWrite(lEnable, lWheelSpeed);
    analogWrite(rEnable, rWheelSpeed);
    Serial.println("Turning right");
  }
}

void turn(int direction, int degrees) {
  Serial.println("Rotating using gyrocope");
  angleFinal = degrees;
  angleInitial = angleCurrent; //snapshot the current angle
  rotating = true;
  digitalWrite(LED_PIN, LOW);
  // Set the motors
  turn(direction); 
  digitalWrite(LED_PIN, HIGH);
}
int getLeftDistance()
{
  long duration;
  int distance;
  digitalWrite(8, LOW);
  //delayMicroseconds(2);
  digitalWrite(8, HIGH);
  //delayMicroseconds(2);
  digitalWrite(8, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(9, HIGH);
  // Calculating the distance
  distance= duration*0.034/2;
  return distance;
 
}
 
int getRightDistance()
{
  long duration;
  int distance;
  digitalWrite(10, LOW);
  delayMicroseconds(2);
  digitalWrite(10, HIGH);
  delayMicroseconds(2);
  digitalWrite(10, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(11, HIGH);
  // Calculating the distance
  distance= duration*0.034/2;
  return distance;
 
}
long getDistance(int pingPin)
{
  long duration, inches, cm;
  if(pingPin == 2)
  {
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin, LOW);
 
    pinMode(pingPin, INPUT);
    duration = pulseIn(pingPin, HIGH);
 
    inches = microsecondsToInches(duration);
    cm = microsecondsToCentimeters(duration);
    return inches;
  }
 
}
// Reads and stores the angle from the gyroscope
void readAngle() {

}
 
//Library
long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}
 
long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}
