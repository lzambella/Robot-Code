#include "I2Cdev.h"
#include "Wire.h"
#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE 
#include "MPU6050_6Axis_MotionApps20.h"
#include "LinkedList\LinkedList.h" // store every single turn
#define OUTPUT_READABLE_YAWPITCHROLL // Output our angles to the serial monitor
#define INTERRUPT_PIN 3 // IIC interupt pin
#define LED_PIN 12
#define THRESHOLD_ANGLE 45 // The angle our bot will gnerally rotate at (deprecated)
#define TURN_LED 27 // LED that enables when the robot is in the turning state

MPU6050 mpu;

byte LEFT = 0;
byte RIGHT = 1;
byte FORWARD = 2;
byte INVERT = 3;
// 50 = 1600us
// 100 = 600us
int silentTest=0;byte IN1 = 6; // L Motor 
byte IN2 = 7;
byte IN3 = 5; // R Motor?            
byte IN4 = 4;
byte backTicks=0;                          
byte rWheelSpeedMax = 150; //prevent burnout
byte lWheelSpeedMax = 150; // stupid motors arent matched
byte rWheelSpeed = rWheelSpeedMax; // This is redundant
byte lWheelSpeed = lWheelSpeedMax;
byte boundsThresh = 20;
byte frontPin = 2; //front pin
byte frontLED = 22; // on: robot is turninggal


const int rEnable = 12; //pwm for left motor
const int lEnable = 13; //pwm for right motor

const int button = 23; // Button to enable the robot
bool enabled = false; // whether the robot should be running full code

int errorDistance=0;

// Gyroscope variables

// For change in angle measurement
double angleInitial = -1; // snapshot of stored angle
double angleCurrent = 0; //Current angle read from the gyroscope
double angleFinal = 0; // Our angle we want the robot to turn at, gets set in the turn method
double angleStarting = 0; // current angle after a successful turn

int SWITCHSTATE = 0; // checks whether the switch has been pressed (deprecated)
bool STATE = false; // No idea
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

int frontDist;
int leftDist;
int rightDist;

LinkedList<byte> turns = LinkedList<byte>(); // Store all the turns the robot makes
LinkedList<double> angles = LinkedList<double>(); // Store the current angle every program loop


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
	pinMode(TURN_LED, OUTPUT);
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
  /*
  Serial.print(String(leftDistance) + " " + frontDist + " " + String(rightDistance));
  Serial.println();
  Serial.print("Error distance: " + String(errorDistance));
  Serial.println();
  Serial.print("Back ticks: " + String(backTicks));
  Serial.println();
  */
	frontDist = getDistance(frontPin);
	leftDist = getLeftDistance();
	rightDist = getRightDistance();
	errorDistance = leftDist - rightDist;
        // if programming failed, don't try to do anything
    if (!dmpReady) return;

    
    do {
		backTicks++;
		// other program behavior stuff here
		// Any code not related to the gyroscope should be placed here
		if (backTicks >= 2000)
	{
			backTicks = 0;
			moveBackward();
			delay(500);
			if (leftDist > rightDist)
				turn(-1, 45);
			else turn(1, 45);
			delay(500);
			moveForward();
		}
		else if (rotating && backTicks >= 2000) { 
			rotating = false;
				backTicks = 0;
				moveBackward();
				delay(500);
				if (leftDist > rightDist)
					turn(-1, 45);
				else turn(1, 45);
				delay(500);
				moveForward();
		}
      else if (rotating && abs(angleCurrent - angleInitial) >= abs(angleFinal)) { 

          // if we reached our threshold angle
          moveForward();
          Serial.println("Sucessfully rotated!");
          rotating = false;
		  digitalWrite(TURN_LED, LOW);
		  angleStarting = angleCurrent;
		  backTicks++;
      }
      else if (!rotating) {  // if we are supposed to be moving forward

		// Ineffecient code to prevent stopping in a wall use accelerometer instead
  
		  /*
		   * Turning logic
		   */
		  backTicks++;
		if (frontDist <= 10 && rightDist >= 5 && leftDist >= 5) { // Prioritize turning left at a fork
			 turn(LEFT, 45);
			 turns.add(LEFT);
		}
		else if (frontDist <= 10 && leftDist <= 5) { // left corner
			turn(RIGHT, 45);
			turns.add(RIGHT);
		}
		else if (frontDist <= 10 && rightDist <= 5) { //right corner
			turn(LEFT, 45);
			turns.add(LEFT);
		}
		else if (frontDist <= 10 && rightDist <= 5 && leftDist <= 5) { // dead end
			turn(LEFT, 180);
			turns.add(INVERT);
		}
        else
        {
          lWheelSpeed = lWheelSpeedMax;
          rWheelSpeed = rWheelSpeedMax;
          digitalWrite(frontLED,LOW);
          moveForward();
        }

		/*
		Check if the robot starts drifting from a straight path
		note: what happens when the current angle is near the rollover (-179 and 179)?? just detect from the walls instead?
		
		if (average() <= angleCurrent - 15)
			turn(RIGHT, average() - angleCurrent);
		else if (average() >= angleCurrent + 15)
			turn(LEFT, angleCurrent - average());
		*/

		// Flush the linked lists to prevent memory leaks
		if (turns.size() > 100) turns.clear();
		if (angles.size() > 100) angles.clear();
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
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		angleCurrent = ypr[0] * 180 / M_PI;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
			Serial.print("\t");
            Serial.print(rotating);
            Serial.print("\t");
            Serial.print(angleInitial);
            Serial.print("\t");
            Serial.println(angleFinal);
        #endif
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
    digitalWrite(IN1,HIGH);   
    digitalWrite(IN2,LOW);
    
    digitalWrite(IN3,HIGH);   
    digitalWrite(IN4,LOW);

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
///Find the average of the angles stored
double average() {
	double sum = 0;
	for (int i = 0; i < angles.size(); i++)
		sum += angles.get(i);
	return sum / angles.size();
}
void turn(byte direction)
{
	digitalWrite(TURN_LED, HIGH);
  //0 = forward, -1= left, 1=right
  if(silentTest)
    return;
  if(direction == LEFT)
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
  if(direction == RIGHT)
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

void turn(byte direction, int degrees) {
  Serial.println("Rotating using gyrocope");
  angleFinal = degrees;
  angleInitial = angleCurrent; //snapshot the current angle
  rotating = true;
  // Set the motors
  turn(direction); 
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
