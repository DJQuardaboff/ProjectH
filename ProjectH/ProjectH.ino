/*
 * Arduino program for controlling a Battle Robot using 2 L298N H Bridge Chip and a Weapon using an ESC.
 * This is updated for the NodeMCU
 * Version 3
 * 8/27/2017 - Adding heartbeat
 * 9/12/2017 - Adding OTA from Austin
 */
 
#include <EEPROM.h>
#include <ArduinoOTA.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <elapsedMillis.h>
#include "WEMOS_Motor.h"
#include "SSD1306.h"

#ifndef _WEMOS_MOTOR_DELAY_REMOVED
#error "If you see this, add the above define and remove the delay calls."
#error "This is here to make sure WEMOS_Motor.cpp has the delays removed from the functions."
#endif

IPAddress ipClient(192, 168, 1, 205);
//IPAddress ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

/* Set these to your desired credentials. */
String ssid;
String password;

const char *updater_ssid = "TimAustinUpdater";
const char *updater_password = "projecthup";

const char *ota_hostname = "ProjectH";

const uint64_t LOADING_CIRCLE_TIME  = 1000;
const uint64_t CHECKMARK_TIME       = 1000;
const uint64_t HEARTBEAT_TIME       = 900;

const float LOADING_CIRCLE_CONST    = (LOADING_CIRCLE_TIME / 8000.0);
const float CHECKMARK_CONST         = (CHECKMARK_TIME / 8000.0);
const float HEARTBEAT_CONST         = (HEARTBEAT_TIME / 8000.0);

/*
  The input to the Anduino is as follows.
  First Number is the function
  Series of parameters
  Ending with one byte #10 - Also know as <lf> or /n or char #10 (These are the same. ).
*/

const uint32_t  START_TOKEN_ADDR            = 0x000;
const String    START_TOKEN                 = "AUSTIN_TIM";
const uint32_t  START_TOKEN_LENGTH          = 0x010;
const uint32_t  SERIES_TOKEN_ADDR           = START_TOKEN_ADDR + START_TOKEN_LENGTH;
const String    SERIES_TOKEN                = "PROJECTH";
const uint32_t  SERIES_TOKEN_LENGTH         = 0x010;
const uint32_t  PROJECT_TOKEN_ADDR          = SERIES_TOKEN_ADDR + SERIES_TOKEN_LENGTH;
const String    PROJECT_TOKEN               = "REMOTECAR";
const uint32_t  PROJECT_TOKEN_LENGTH        = 0x010;
const uint32_t  SSID_TOKEN_ADDR             = PROJECT_TOKEN_ADDR + PROJECT_TOKEN_LENGTH;
const String    SSID_TOKEN_DEFAULT          = "projecth000";
const uint32_t  SSID_TOKEN_LENGTH           = 0x020;
const uint32_t  PW_TOKEN_ADDR               = SSID_TOKEN_ADDR + SSID_TOKEN_LENGTH;
const String    PW_TOKEN_DEFAULT            = "projecth";
const uint32_t  PW_TOKEN_LENGTH             = 0x020;
const uint32_t  UPDATER_SSID_TOKEN_ADDR     = PW_TOKEN_ADDR + PW_TOKEN_LENGTH;
const String    UPDATER_SSID_TOKEN_DEFAULT  = "TimAustinUpdater";
const uint32_t  UPDATER_SSID_LENGTH         = 0x020;
const uint32_t  UPDATER_PW_TOKEN_ADDR       = UPDATER_SSID_TOKEN_ADDR + UPDATER_SSID_LENGTH;
const String    UPDATER_PW_TOKEN            = "projecthup";
const uint32_t  UPDATER_PW_LENGTH           = 0x020;

#define COMMAND_START 999
#define SETUP_FUNCTION 1
#define SETUP_WIFI 5
#define SETUP_WIFI_SSID "SOFT_AP_SSID"
#define SETUP_WIFI_PW "SOFT_AP_PW"
#define SETUP_RESET 6
#define HEARTBEAT_FUNCTION 3
#define MOTOR_FUNCTION 5
#define MOTOR_LEFT 1
#define MOTOR_RIGHT 2
#define SERVO_FUNCTION 6

#define HEARTBEAT_TIMEOUT 3000     // 3 seconds of nothing will stop the motors

/*
  ------------------------------------
  For HEARTBEAT_FUNCTION
  The App will send Heartbeat only
  Example:
  999,3
  ---------------------
  For MOTOR_FUNCTION
  The Android will send MotorControl + 3 additional numbers
  Function, Motor#(1-20), Mode/Direction(0-3), Speed(0-100)
  // Not that a direction of 1 with speed of 0 will allow the motor to coast
  mode is a number 0 -> 3 that determines what the motor
  will do.
  0 = Brake the motor - immediate stop
  1 = turn motor counter clockwise
  2 = turn motor clockwise
  3 = disabble the motor - coast

  speed is a number 0 -> 100 that represents percentage of
  motor speed.
  0   = no power - maybe the same as Mode 0
  50  = 50% of full motor speed
  100 = 100% of full motor speed

  Examples...
  999,5,1,2,100  // Motor 1 counter clockwise 100%
  999,5,2,1,50   // Motor 2 clockwise 50%
  999,5,2,0,0    // Motor 2 allow to coast to a stop
  999,5,1,3,0    // Motor 1 stop immediately. The last 0 is actually ignored but must be sent

  From WEMOS_Motor.h
  _SHORT_BRAKE  0
  _CCW          1
  _CW           2
  _STOP         3
  ----------------------------
  For SERVO_FUNCTION - Normally the weapon
  The Android will send ServoESCControl + 2 additional numbers
  Function, Servo/ESC#(1-20), Speed(0-100)
  speed is a number 0 -> 100 that represents percentage of
  motor speed.
  0 = no power
  50 = 50% of full motor speed
  100 = 100% of full motor speed

  999,6,1,100  // Weapon on the ESC running 100%
  999,6,1,0    // Turn off the weapon
  ------------------------------------
  For QueryPins
 **** This is not implemented yet
  20=Query pin assignments made in this program

  The Android program can get the pin assignments from this program. 20 is used as the query command.
  The Arduino will respond with the pin assigned.
  The Android will send 3 numbers followed by one byte <lf> or '\n' or char #10 (These are the same). Examples...
  20,1,24     Send ENA  // Left Motor PWM
  20,4,24     Send IN1  // Left Motor direction Control
  20,4,24     Send IN2  // Left Motor direction Control
  20,4,24     Send IN3  // Right Motor direction Control
  20,4,24     Send IN4  // Right Motor direction Control
  20,4,24     Send ENB  // Right Motor PWM

  // Pin on the Arduino going to the ESC for the weapon. This should be the yellow control wire.

  This code is in the public domain.
*/

typedef struct {
  Servo servoObject;
  uint8_t pin;
  int position;
  bool changed;
} ServoInfo;

WiFiServer server(23);
Motor leftMotor(0x30, _MOTOR_A, 1000);
Motor rightMotor(0x30, _MOTOR_B, 1000);
SSD1306  display(0x3c, D2, D1);

uint8_t leftMotorDir = _CW;
uint8_t leftMotorSpeed = 0;
bool leftMotorChanged = true;

uint8_t rightMotorDir = _CW;
uint8_t rightMotorSpeed = 0;
bool rightMotorChanged = true;

ServoInfo *servo;
uint8_t servoNum;

// This is using D4
elapsedMillis lastLedChange = 0;
bool ledOn = false;
#define LED_BLINK_TIMEOUT 2000     // 1 off / 1 second on

String sClientIn = "";         // a string to hold incoming data
bool connecting = true;
bool clientConnected = false;
bool disconnected = false;
WiFiClient client;

elapsedMillis lastLoadingCircle;
elapsedMillis lastCheckmark;
elapsedMillis lastHeartbeat;

// Pin on the Arduino going to the ESC for the weapon. This should be the yellow control wire.
const int SERVOESC_PIN = D4; // THIS IS PIN D2.  CHOSEN BECAUSE IT DOESN'T INTERFERE WITH THE BOOT SEQUENCE ON THE D1

void setup() {
  Serial.begin(250000);
  while(!Serial);

  leftMotor.setmotor(_STOP);
  rightMotor.setmotor(_STOP);

  pinMode(LED_BUILTIN, OUTPUT);

  for(uint8_t i = 0; i < servoNum; i++) {
    servo[i].servoObject.attach(SERVOESC_PIN);
  }

  WiFi.begin(updater_ssid, updater_password);
  WiFi.mode(WIFI_STA);
  //WiFi.mode(WIFI_AP_STA);   // New addition for OTA
  WiFi.hostname(ota_hostname);
  //WiFi.config(ipClient, gateway, subnet);  // (DNS not required)
  WiFi.softAP(ssid, password);
  
  // start the server listening
  server.begin();
  // you're connected now, so print out the status:

  ArduinoOTA.onStart([]() {
    /*    String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";
      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    */
    display.clear();
    display.display();
    Serial.println("Start updating ");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    display.clear();
    display.drawProgressBar(31, 40, 64, 10, (progress / (total / 100)));

    display.display();
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  /*
    myservo.write(50);
    motorDrive(1,2,50);
    motorDrive(2,2,50);
    delay(2000);
  */

  display.init();
  display.flipScreenVertically();
  display.clear();

  client = server.available();
}

String readEEPROMToken(uint32_t addr, uint32_t maxLength) {
  String str;
  char temp;
  while(sizeof(str) < maxLength) {
    temp = EEPROM.read(addr++);
    if(!temp) break;
    str += temp;
  }
  return str;
}

bool writeEEPROMToken(uint32_t addr, String str, uint32_t maxSize) {
  uint32_t length = _min(sizeof(str), maxSize - 1);
  for(uint32_t i = 0; i < length; i++) EEPROM.write(addr + i, str[i]);
  EEPROM.write(addr + length, 0x00);
}

String sGetToken(String &str) {
  int index = str.indexOf(',');
  String token;
  if (index >= 0) {
    token = str.substring(0, index);
    str.remove(0, index + 1);
  } else {
    token = str;
    str = "";
  }
  return token;
}

int iGetToken(String &str) {
  return sGetToken(str).toInt();
}

void processCommand(String command) {
  if(iGetToken(command) != COMMAND_START) return; // Be sure the input starts with 999

  int function = iGetToken(command);
  //Serial.print("functionNumber: ");
  //Serial.println(function);

  if(function == SETUP_FUNCTION) {
    int function2 =  iGetToken(command);
    bool projectCheck = sGetToken(command) == PROJECT_TOKEN;

    if(function2 == SETUP_WIFI) {
      String function3 =  sGetToken(command);
      String str =  sGetToken(command);
      if(function3 == SETUP_WIFI_SSID) {
        writeEEPROMToken(SSID_TOKEN_ADDR, str, SSID_TOKEN_LENGTH);
      } else if(function3 == SETUP_WIFI_PW) {
        writeEEPROMToken(PW_TOKEN_ADDR, str, PW_TOKEN_LENGTH);
      }
    } else if (function2 == SETUP_RESET) {
      writeEEPROMToken(START_TOKEN_ADDR, "", START_TOKEN_LENGTH);
      ESP.restart();
    }
  } else if(function == HEARTBEAT_FUNCTION) {
    lastHeartbeat = 0;
  } else if(function == MOTOR_FUNCTION) {
    int motorNum = iGetToken(command);
    int motorDirection = iGetToken(command);
    int motorPower = iGetToken(command);

    if(motorNum == MOTOR_LEFT) {
      //Serial.println("Received command for Motor Number 1");
      leftMotorChanged = true;
      leftMotorDir = motorDirection;
      leftMotorSpeed = motorPower;
    } else if (motorNum == MOTOR_RIGHT) {
      //Serial.println("Received command for Motor Number 2");
      rightMotorChanged = true;
      rightMotorDir = motorDirection;
      rightMotorSpeed = motorPower;
    }
  } else if(function == SERVO_FUNCTION) {
    int servoIndex = iGetToken(command);
    int servoPosition = iGetToken(command);

    if(servoIndex < servoNum) {
      servo[servoIndex].position = servoPosition;
      servo[servoIndex].changed = true;
    }
  }
}

int readClientRetEOFPos() {
  //  loop to read multiple
  while(client.available() > 0) {
    char c = client.read();
    if(c > 0) sClientIn += c; else break;
  }

  int EOFPos = sClientIn.indexOf(char(10));
  return EOFPos;

  return -1; // if nothing
}

void checkHeartbeatTimeout() {
  // Heart Beat - If there are Heart Beat commands sent in the past x milliseconds then turn off all motors
  if((lastHeartbeat > HEARTBEAT_TIMEOUT)) {
    //Serial.print("Heartbeat Lost. Motor shutdown");
    leftMotorDir = 1;
    leftMotorSpeed = 0;
    leftMotorChanged = true;

    rightMotorDir = 1;
    rightMotorSpeed = 0;
    rightMotorChanged = true;

    /* this would move the servos
    for(uint16_t i; i < servoNum; i++) {
      servo[i].position = 0;
      servo[i].changed = true;
    }
    */
  }
}

void checkLedBlinkTimeout() {
  if(lastLedChange >= LED_BLINK_TIMEOUT) {
    lastLedChange -= LED_BLINK_TIMEOUT;
    ledOn = !ledOn;
    digitalWrite(LED_BUILTIN, ledOn);
  }
}

void loop() {
  if(WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
  }
  //Serial.print(WiFi.RSSI());
  yield();
  String command = "";

  checkHeartbeatTimeout();

  //checkLedBlinkTimeout(); led doesn't look good

  //Serial.println("leftMotorChanged-Send to motor");
  motorDrive(MOTOR_LEFT, leftMotorDir, leftMotorSpeed);
  leftMotorChanged = false;

  //Serial.println("rightMotorChanged-Send to motor");
  motorDrive(MOTOR_RIGHT, rightMotorDir, rightMotorSpeed);
  rightMotorChanged = false;

  if(client) {
    if(client.connected()) {
      clientConnected = true;
      connecting = false;
      disconnected = false;
      int EOFPos = readClientRetEOFPos();

      if(EOFPos > 0) {   // Check that we have something to process
        command = sClientIn.substring(0, EOFPos);
        sClientIn.remove(0, EOFPos + 1);
        processCommand(command);
      }
    } else {
      clientConnected = false;
      connecting = true;
      disconnected = true;
      lastLoadingCircle = 0;
      client.stop();
      client = server.available();
      sClientIn = "";   // a string to hold incoming data
      //      Serial.println("client disonnected");
    }
  } else {
    clientConnected = false;
    connecting = true;
    disconnected = false;
    client = server.available();
    if(client) {
      clientConnected = true;
      connecting = false;
      sClientIn = "";   // a string to hold incoming data
    }
  }

  display.clear();
  drawLoadingCircle(32, 23, 14.9, 2, 10);
  drawCheckmark(32, 26, 11, 3);
  drawHeartbeat(46, 46, 15, 2, 2);
  drawMotorVisuals();
  display.drawProgressBar(32, 40, 63, 7, 35);
  display.display();
}

void drawMotorVisuals() {
  uint16_t leftMotorPixels = (leftMotorSpeed / 4.16);
  display.drawRect(getDisplayX(62), getDisplayY((leftMotorDir == 2)?(24 - leftMotorPixels):(24)),2 , leftMotorPixels);
  uint16_t rightMotorPixels = (rightMotorSpeed / 4.16);
  display.drawRect(getDisplayX(0), getDisplayY((rightMotorDir == 1)?(24 - rightMotorPixels):(24)),2 , rightMotorPixels);
}

void WeaponDrive(int servoNum, uint8_t percent) {
  if(servoNum <= 0) return;
  percent = _min(percent, 100);
  int escDuty = map(percent, 0, 100, 50, 179);
  //if(percent == 0) escDuty = 50; else escDuty = 165;

  //myservo.write(Escduty);
  //myservo.attach(SERVOESC_PIN);
  //myservo.write(Escduty);

  /*
    delay(200);
    myservo.write();
    delay(200);
    myservo.detach();
  */
}

//******************   Motor A control   *******************
void motorDrive(int motorNum, int motorDir, int percent) {
  percent = _min(percent, 100);

  //change the percentage range of 0 -> 100 into the PWM range of 0 -> 255 using the map function
  if (motorNum <= 0) {
    //Serial.print("Motor Out of Range: ");
    //Serial.println(motorNum);
    exit;
  }

  switch (motorDir) {
    case 0:  //brake motor
      //Serial.println("Motor brake");
      if (motorNum == 1)
        leftMotor.setmotor(_STOP);
      if (motorNum == 2)
        rightMotor.setmotor(_STOP);
      break;

    case 1:  //turn counter-clockwise
      //Serial.println("turn counter-clockwise");

      if (motorNum == 1) {
        leftMotor.setmotor( _CCW, percent);
      }

      if (motorNum == 2) {
        rightMotor.setmotor( _CCW, percent);
      }
      break;
    case 2:  //turn clockwise
      //Serial.println("turn clockwise");
      if (motorNum == 1)
        leftMotor.setmotor( _CW, percent);
      if (motorNum == 2)
        rightMotor.setmotor( _CW, percent);
      break;

    case 3:  //disable/coast
      //Serial.println("Motor disable/coast");
      if (motorNum == 1)
        leftMotor.setmotor(_STANDBY);
      if (motorNum == 2)
        rightMotor.setmotor(_STANDBY);
      break;
  }
}

uint16_t getDisplayX(uint16_t x) {
  return x + 32;
}

uint16_t getDisplayY(uint16_t y) {
  return y + 16;
}

/*
void drawBatteryBar(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t percent) {
  percent = min(percent, 100):
  uint16_t barHeight = ((height - 1) * (percent / 100.0)) + 1;
  display.drawVerticalLine(getDisplayX(x), getDisplayY(y), height - barHeight);
  display.drawVerticalLine(getDisplayX(x) + width - 1, getDisplayY(y), height - barHeight);
  display.drawHorizontalLine(getDisplayX(x) + 1, getDisplayY(y), width - 2);
  display.fillRect(getDisplayX(x), getDisplayY(y) + height - barHeight, width, barHeight);
}
*/

void drawLoadingCircle(uint16_t x, uint16_t y, float radius, uint16_t thickness, uint16_t length) {
  float temp;
  uint16_t circumference = getRadians(radius);
  float offset = (length / 2) / ((float)circumference);
  bool endAnimation = false;
  if (clientConnected || disconnected) {
    temp = lastLoadingCircle / ((float)LOADING_CIRCLE_TIME);
    if (temp > 1) {
      endAnimation = true;
      temp -= 1;
    }
  } else if (connecting) {
    lastLoadingCircle = (lastLoadingCircle % LOADING_CIRCLE_TIME);
    temp = (lastLoadingCircle % LOADING_CIRCLE_TIME) / ((float)LOADING_CIRCLE_TIME);
  } else return;
  
  temp = (temp < .5) ? (exp(temp / LOADING_CIRCLE_CONST)) : ((exp(.5 / LOADING_CIRCLE_CONST) * 2) - exp((1 - temp) / LOADING_CIRCLE_CONST));
  temp = temp / ((exp(.5 / LOADING_CIRCLE_CONST) * 2) - 1);
  float temp2 = changeResolution(temp, circumference);
  if (endAnimation) {
    float temp3 = changeResolution((temp2 + (offset * 2)) * (1 - (offset * 1.5)), circumference);
    drawArc(x, y, radius, getRadians(changeResolution(- offset - .25, circumference)), getRadians(temp3), circumference * temp3, thickness);
  } else {
    drawArc(x, y, radius, getRadians(changeResolution(temp2 - offset - .25, circumference)), getRadians(offset * 2), length, thickness);
  }
}

void drawCheckmark(uint16_t x, uint16_t y, uint16_t size, uint16_t thickness) {
  if (connecting || disconnected) return;
  float temp = lastCheckmark / ((float)CHECKMARK_TIME);
  temp = (temp < .5) ? (exp(temp / CHECKMARK_CONST)) : ((exp(.5 / CHECKMARK_CONST) * 2) - exp((1 - temp) / CHECKMARK_CONST));
  temp = (temp / ((exp(.5 / CHECKMARK_CONST) * 2) - 1)) * 2 * size;
  if (temp < size) {
    for (uint16_t i = 0; i < thickness; i++) {
      display.drawLine(getDisplayX(x) - (size / 2) - (i / 2), getDisplayY(y) - (size / 2) + ((i + 1) / 2), getDisplayX(x) - ((size - temp) / 2), getDisplayY(y) - ((size - temp) / 2) + i);
    }
  } else {
    for (uint16_t i = 0; i < thickness; i++) {
      display.drawLine(getDisplayX(x) - (size / 2) - (i / 2), getDisplayY(y) - (size / 2) + ((i + 1) / 2), getDisplayX(x), getDisplayY(y) + i);
      display.drawLine(getDisplayX(x) - (size - temp) + 1 + (i / 2), getDisplayY(y) + (size - temp) + ((i + 1) / 2), getDisplayX(x), getDisplayY(y) + i);
    }
  }
}

void drawHeartbeat(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t dotSize) {
  uint16_t phaseNum = ((width - dotSize) * 2) - 1;
  float phase = (((lastHeartbeat / (HEARTBEAT_TIME / ((float)phaseNum)))) + dotSize - 1) / ((float)phaseNum);
  phase = (phase < .5) ? (exp(phase / HEARTBEAT_CONST)) : ((exp(.5 / HEARTBEAT_CONST) * 2) - exp((1 - phase) / HEARTBEAT_CONST));
  phase = ((phase / ((exp(.5 / HEARTBEAT_CONST) * 2) - 1))) * phaseNum;
  dotSize++;
  if (phase > phaseNum) phase = phaseNum;
  if (phase >= width) phase = -phase + (width * 2) - 2;
  for (uint16_t i = 0; i < width; i++) {
    if (i > phase || i < phase - (dotSize - 1)) display.drawVerticalLine(getDisplayX(x) + i, getDisplayY(y), height);
  }
}

float changeResolution(float input, uint16_t denominator) {
  return floor(input * denominator) / denominator;
}

float getRadians(float f) {
  return f * 2 * PI;
}

void drawArc(uint16_t x, uint16_t y, float radius, float startTheta, float endTheta, uint16_t N, uint16_t thickness) {
  float dx = cos(startTheta) * radius;
  float dy = sin(startTheta) * radius;
  float ctheta = cos(endTheta / (N - 1));
  float stheta = sin(endTheta / (N - 1));
  display.fillRect(getDisplayX(x) + dx, getDisplayY(y) + dy, thickness, thickness);
  for (uint16_t i = 1; i < N; i++) {
    float dxtemp = (ctheta * dx) - (stheta * dy);
    float dytemp = (stheta * dx) + (ctheta * dy);
    display.fillRect(getDisplayX(x) + dx, getDisplayY(y) + dy, thickness, thickness);
    dx = dxtemp;
    dy = dytemp;
  }
}
