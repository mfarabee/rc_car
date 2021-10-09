#include <SoftwareSerial.h>


/*RC_CAR Written by Mike Farabee
 * 
 * The following are the instructions for programming the HC-05 bluetooth module:
 * https://alselectro.wordpress.com/2014/10/21/bluetooth-hc05-how-to-pair-two-modules/
 1) upload empty sketch to arduino
 2) unplug and repower arduino
 3) RX -> pin0 (voltage divider) (RX)  <-- These are reversed from normal connection 
 4) TX -> pin1 (TX)                    <--
 5) EN pin to 3.3v
 6) Serial monitor = both LR&CR 38400 baud
 7) AT (case sensitive on some hc-05's, use uppercase)
 8) Change serial monitor baud rate to 38400,  change no line editing to both NL&CR
 9) AT commands
    AT  < check connection
    AT+UART    <- baudrate
    AT+RESET
    AT+NAME    <- need power to enable when hiting enter
    AT+ROLE    <- 0=slave 1=master
    AT+PSWD
    AT+ORGL    <- restore to original state
    AT+CMODE   <- 0=bound to addr, 1= connect to any
  My settings:
    AT+UART=38400,1,0
    AT+ROLE=0
    AT+NAME=RC1   <- RC1, RC2, RC3...
 */
 
// Pins for optional digital out
#define option1Pin 4  
#define option2Pin 5

// Pins for Ultrasonic sensor
#define echoPin 15  // A1
#define trigPin 14  // A0

// PWM pins 5,6 = ~980HZ (available for use)

// PWM pins 9,10, 3,11 = ~ 490HZ
// Left motor pins
#define leftMotorPWM 9
#define leftMotorA   8
#define leftMotorB   7

// Right motor pins
#define rightMotorPWM   10
#define rightMotorA     11
#define rightMotorB     12

#define STOP_DISTANCE 18.0

// PWM goes from 0-255, I can only go as low as 70 before dc motors stops
#define MAX_SPEED 255
#define MIN_SPEED 70   // This is the speed the car stalls 
#define MAX_RUN_TIME 15000

enum CarDirection {Stop,Forward,Reverse,Left,Right};
CarDirection CurrentDirection = Stop;

unsigned long RunTimer=millis();
int FoundObstacle = 0;

String Data;

// Using SoftwareSerial, so that I do not have to worry about disconnecting the serial pins when uploading code
//connect TX of HC-05 to pin2
// connect RX of HC-05 to pin3 via divider:  Arduino(pin3)----1k----RX----2k----GND
SoftwareSerial MyPhone(2, 3); // RX | TX


/****************************************
 *             FindDistance
 * Sends pulse to sensor and measures return signal delay.
 * Returns distance in inches
***************************************/
float findDistance(int trigger,int echo){
  static unsigned long distanceTimer=0;
  float tmp = -1.0;
  unsigned long x;
  if(millis()-distanceTimer >50){ // only check for obsticle every 50 milliseconds
    digitalWrite(trigger,LOW);
    delayMicroseconds(2);
    digitalWrite(trigger,HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger,LOW);
    x=pulseIn(echo,HIGH);
    tmp=(float)x/150.0;
    distanceTimer=millis();
  }
  return(tmp);
}

/****************************************
 *             calculateSpeed
 * Converts speed percentave (0-100) to PWM value
******************************************/
int calculateSpeed(int speedPercent){
  int result;
  if(speedPercent <= 4){  // Allow 4 percent slop for zero
    result=0;
  } else if(speedPercent > 4) {
     result=map(speedPercent,4,100,MIN_SPEED,MAX_SPEED);
  }
  return(result);
}

/****************************************
 *             motorRight
 * moves right motors
 * positive speed goes forward, negative speed goes reverse
******************************************/
void motorRight(int motorSpeed){
  analogWrite(rightMotorPWM,abs(motorSpeed));
  if(motorSpeed>0){ // Forward
    digitalWrite(rightMotorA,HIGH); 
    digitalWrite(rightMotorB,LOW);
  }else if(motorSpeed<0){ // Reverse
    digitalWrite(rightMotorA,LOW);
    digitalWrite(rightMotorB,HIGH);
  }else{ // motorSpeed = 0
    digitalWrite(rightMotorA,LOW);
    digitalWrite(rightMotorB,LOW);
  }
}

/****************************************
 *             motorLeft
 * moves left motors
 * positive speed goes forward, negative speed goes reverse
******************************************/
void motorLeft(int motorSpeed){
  analogWrite(leftMotorPWM,abs(motorSpeed));
  if(motorSpeed>0){ // Forward
    digitalWrite(leftMotorA,HIGH);
    digitalWrite(leftMotorB,LOW);
  }else if(motorSpeed<0){ // Reverse
    digitalWrite(leftMotorA,LOW);
    digitalWrite(leftMotorB,HIGH);
  }else{ // motorSpeed = 0
    digitalWrite(leftMotorA,LOW);
    digitalWrite(leftMotorB,LOW);
  }

}

/****************************************
 *         driveStop
 * stops both motors by sending zero speed to both left and right motors
******************************************/
void driveStop(){
  //Serial.println("STOP");
  motorLeft(0);
  motorRight(0);
  CurrentDirection=Stop;  
}

/****************************************
 *         driveTurn
 * turns car by moving one side forward (positive speed) 
 * and the other reverse (negative speed).
******************************************/
void driveTurn(float dDirection, int speedPercent){
  int motorSpeed;
  if(CurrentDirection==Forward && CurrentDirection==Reverse){ // If forward or reverse, stop before turning
     driveStop();
  }
  motorSpeed = calculateSpeed(speedPercent);
  if(dDirection >= 0.0){
    //Serial.println("TURN RIGHT");    
    motorLeft(motorSpeed);
    motorRight(-motorSpeed);
  }else{
    //Serial.println("TURN LEFT");     
    motorLeft(-motorSpeed);
    motorRight(motorSpeed);
  }
}

/****************************************
 *         driveForward
 * Moves forward by driving both side with a positive speed
******************************************/
void driveForward(int speedPercent){
  int motorSpeed;
  
  if(CurrentDirection==Reverse){ // If motor is going reverse, stop motor before going forward
     driveStop();
  }
  CurrentDirection= Forward;

  motorSpeed = calculateSpeed(speedPercent);
  //Serial.print("FORWARD: ");  Serial.println(motorSpeed);
  motorLeft(motorSpeed);
  motorRight(motorSpeed);
}

/****************************************
 *         driveReverse
 * Moves reverse by driving both side with a negative speed
******************************************/
void driveReverse(int speedPercent){  
  int motorSpeed;
  
  if(CurrentDirection==Forward){// If motor is going forward, stop motor before going reverse
     driveStop();
  }
  CurrentDirection= Reverse;
  motorSpeed = calculateSpeed(speedPercent);
  //Serial.print("REVERSE: ");  Serial.println(speedPercent);  
  motorLeft(-motorSpeed);
  motorRight(-motorSpeed);
}

/****************************************
 *         detectObstacle
 * Check ultrasonic sensor for obstacle
******************************************/
float detectObstacle(){
  float distance;
    distance=findDistance(trigPin,echoPin);
    if(distance > 0.0){ // Do not check if distance is negative, it was not calculated due to time interval
      if(distance<=STOP_DISTANCE){
        if(FoundObstacle == 0){ // Dont stop motor if already found
          driveStop();  
        }
        FoundObstacle=1;   
      }else{
        FoundObstacle=0;
      }
    }
  return(distance);
}

void setOptionPin(int value){
  int pinNumber;
  pinNumber=value/10;
  value=value%10;
  //Serial.print("pinNumber= ");Serial.print(pinNumber);Serial.print("Value= ");Serial.println(value);
  switch(pinNumber){
    case 1:
      if(value < 1){
        digitalWrite(option1Pin,LOW);
      }else{
        digitalWrite(option1Pin,HIGH);  
      }
    break;
    case 2:
      if(value < 1){
        digitalWrite(option2Pin,LOW);
      }else{
        digitalWrite(option2Pin,HIGH);  
      }
    break;
  }
}

/****************************************
 *         checkBT
 * Read data from Bluetooth
******************************************/
void checkBT(){
    if(MyPhone.available() > 0){ // check for bluetooth command
      Data=MyPhone.readStringUntil(';');
            Serial.println(Data);
    }
}
/****************************************
 *         checkSerial
 * Read data from Serial
******************************************/
void checkSerial(){
    if(Serial.available() > 0){ // check for bluetooth command
      Data=Serial.readStringUntil(';');
      Serial.println(Data);
    }
}

void setup() {
  Serial.begin(115200); 
  MyPhone.begin(38400);
  
  pinMode(option1Pin, OUTPUT);
  pinMode(option2Pin, OUTPUT);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  pinMode(leftMotorPWM,OUTPUT);
  pinMode(leftMotorA,OUTPUT);
  pinMode(leftMotorB,OUTPUT);
  
  pinMode(rightMotorPWM,OUTPUT);
  pinMode(rightMotorA,OUTPUT);
  pinMode(rightMotorB,OUTPUT);
}

void loop() {
  int value;
  checkBT();
  checkSerial();
  if(Data.length()>0){ // If Data is not an empty string
    if(Data.length()>1){
      value= Data.substring(1).toInt();
    }else{ // If a command was received without a speed, set the MAX_SPEED
      value=MAX_SPEED;
    }
    switch(Data[0]){
      case 's':
        driveStop();
      break;
      case 'f':
        if(FoundObstacle == 0){ // Don't move forward is obstacle is found
          driveForward(value);
        }
      break;
      case 'b':
        driveReverse(value);
      break;
      case 'r':
        driveTurn(90,value);
      break;
      case 'l':
        driveTurn(-90,value);  
      break;    
      case 'o':
        setOptionPin(value);  
      break;  
    }
    RunTimer=millis();
  }
  Data="";
  
  if(millis()-RunTimer>MAX_RUN_TIME){ // If car has not received a command in 15 sec, then stop. Might have lost bluetooth
    driveStop();
    RunTimer=millis();
  }
  detectObstacle();
}




