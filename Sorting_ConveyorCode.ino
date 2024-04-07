// 
// MSE 2202 TCS34725 colour sensor example
// 
//  Language: Arduino (C++)
//  Target:   ESP32-S3
//  Author:   Michael Naish
//  Date:     2024 03 05 
//


#define PRINT_COLOUR                                  // uncomment to turn on output of colour sensor data


#include <ESP32Servo.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"



void setMotor(int dir, int pwm, int in1, int in2);
void encoderISR();


// Function declarations
void doHeartbeat();

#define LEFT_MOTOR_A        35
#define LEFT_MOTOR_B        36
#define RIGHT_MOTOR_A       37
#define RIGHT_MOTOR_B       38
#define ENCODER_LEFT_A      15
#define ENCODER_LEFT_B      16
#define ENCODER_RIGHT_A     11
#define ENCODER_RIGHT_B     12


// Constants
const int cHeartbeatInterval = 75;                    // heartbeat update interval, in milliseconds
const int cSmartLED          = 21;                    // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount     = 1;                     // number of Smart LEDs in use
const int cSDA               = 47;                    // GPIO pin for I2C data
const int cSCL               = 48;                    // GPIO pin for I2C clock
const int cTCSLED            = 14;                    // GPIO pin for LED on TCS34725
const int cLEDSwitch         = 46;                    // DIP switch S1-2 controls LED on TCS32725
const int servoPin           = 42;    

const int cServoPin          = 41;                 // GPIO pin for servo motor
const int cServoChannel      = 5; 

// Constants
const int cPWMRes = 8;
const int cPWMFreq = 20000;
const int cNumMotors = 2;
const int cIN1Pin[] = {LEFT_MOTOR_A, RIGHT_MOTOR_A};
const int cIN1Chan[] = {0, 1};
const int c2IN2Pin[] = {LEFT_MOTOR_B, RIGHT_MOTOR_B};
const int cIN2Chan[] = {2, 3};





// Variables
boolean heartbeatState       = true;                  // state of heartbeat LED
unsigned long lastHeartbeat  = 0;                     // time of last heartbeat state change
unsigned long curMillis      = 0;                     // current time, in milliseconds
unsigned long prevMillis     = 0;                     // start time for delay cycle, in milliseconds
unsigned long previousMicros = 0;                     //hold previous start time of cycle in microseconds
unsigned long currentMicros;                          //hold curreent time in microseconds                 

// Variables
unsigned char driveSpeed;
unsigned char driveIndex;

// Encoder structure
struct Encoder {
   const int chanA;
   const int chanB;
   long pos;
} encoder[] = {{ENCODER_LEFT_A, ENCODER_LEFT_B, 0}, {ENCODER_RIGHT_A, ENCODER_RIGHT_B, 0}};




// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);
Servo servo;


// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 
                                       150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0};


// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found




void setup() {
  for (int k = 0; k < cNumMotors; k++) {
      ledcAttachPin(cIN1Pin[k], cIN1Chan[k]);
      ledcSetup(cIN1Chan[k], cPWMFreq, cPWMRes);
      ledcAttachPin(c2IN2Pin[k], cIN2Chan[k]);
      ledcSetup(cIN2Chan[k], cPWMFreq, cPWMRes);
      pinMode(encoder[k].chanA, INPUT);
      pinMode(encoder[k].chanB, INPUT);
      attachInterrupt(digitalPinToInterrupt(encoder[k].chanA), encoderISR, RISING);
   }

  Serial.begin(115200);                               // Standard baud rate for ESP32 serial monitor
  servo.attach(servoPin);
  pinMode(cServoPin, OUTPUT);                      // configure servo GPIO for output
  ledcSetup(cServoChannel, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  ledcAttachPin(cServoPin, cServoChannel);   


  // Set up SmartLED
  SmartLEDs.begin();                                  // initialize smart LEDs object
  SmartLEDs.clear();                                  // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0,0,0)); // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                         // set brightness [0-255]
  SmartLEDs.show();                                   // update LED




  Wire.setPins(cSDA, cSCL);                           // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);                  // configure GPIO to set state of TCS34725 LED 




  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }
}


void loop() {
        // constantly spin motors
   setMotor(-1, 245, cIN1Chan[0], cIN2Chan[0]);
   setMotor(1, 245, cIN1Chan[1], cIN2Chan[1]);

    currentMicros = micros(); //get current time in microseconds
    
    // Check if 10 seconds have elapsed
    if ((currentMicros - previousMicros) >= 5000000) { //check to see if time is up forone cycle to enter and procees with sorting
        previousMicros = currentMicros;
        Serial.print("time 5 sec\n");


         


            uint16_t r, g, b, c;
            digitalWrite(cTCSLED, !digitalRead(cLEDSwitch)); // turn on onboard LED if switch state is low (on position)
            if (tcsFlag) { // if colour sensor initialized
                tcs.getRawData(&r, &g, &b, &c); // get raw RGBC values   
                Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c); 


                if (2 < r && r < 5 && 2 < g && g < 5 && 1 < b && b < 5 && 8 < c && c < 12) { //check to see if colour detected is green
                    Serial.printf("GREEN\n");
                    //servo.write(180); //turn servo all the way left if green
                    ledcWrite(cServoChannel, degreesToDutyCycle(180));
                }
                else if(r == 3 && g == 3 && b == 3 && c == 8){
                      ledcWrite(cServoChannel, degreesToDutyCycle(90));
                } else {
                    Serial.printf("BAD\n");
                    //servo.write(0); //turn the servo all the way right if colour sensed is not green
                    ledcWrite(cServoChannel, degreesToDutyCycle(0));
                 }


                unsigned long servoStartTime = millis(); //hold time in millisecondds
                const unsigned long servoDuration = 1000; //variable for duration of sorting cycle


                while(millis() - servoStartTime < servoDuration){
                  doHeartbeat();
                } //ensure heartbeat is continuous


                //servo.write(100); //return the servo back to original position
                ledcWrite(cServoChannel, degreesToDutyCycle(90));
            }
        
    }




    doHeartbeat();  // update heartbeat LED
}


// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();                               // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                        // update the heartbeat time for the next update
    LEDBrightnessIndex++;                             // shift to the next brightness level
    if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) { // if all defined levels have been used
      LEDBrightnessIndex = 0;                         // reset to starting brightness
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]); // set brightness of heartbeat LED
    SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 250, 0)); // set pixel colours to green
    SmartLEDs.show();                                 // update LED
  }
}

// send motor control signals
void setMotor(int dir, int pwm, int in1, int in2) {
   if (dir == 1) {
      ledcWrite(in1, pwm);
      ledcWrite(in2, 0);
   }
   else if (dir == -1) {
      ledcWrite(in1, 0);
      ledcWrite(in2, pwm);
   }
   else {
      ledcWrite(in1, 0);
      ledcWrite(in2, 0);
   }
}

// encoder interrupt service routine
void encoderISR() {
   for (int k = 0; k < cNumMotors; k++) {
      int b = digitalRead(encoder[k].chanB);
      if (b > 0) {
         encoder[k].pos++;
      }
      else {
         encoder[k].pos--;
      }
   }
}
long degreesToDutyCycle(int deg) {
  const long cMinDutyCycle = 400;                     // duty cycle for 0 degrees
  const long cMaxDutyCycle = 2100;                    // duty cycle for 180 degrees

  long dutyCycle = map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);  // convert to duty cycle

#ifdef OUTPUT_ON
  float percent = dutyCycle * 0.0061039;              // (dutyCycle / 16383) * 100
  Serial.printf("Degrees %d, Duty Cycle Val: %ld = %f%%\n", servoPos, dutyCycle, percent);
#endif

  return dutyCycle;
}