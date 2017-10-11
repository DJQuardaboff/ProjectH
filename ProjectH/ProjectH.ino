#include <EEPROM.h>
#include "ArduinoOTA.h"

/*

  Arduino program for controlling a Battle Robot using 2 L298N H Bridge Chip and a Weapon using an ESC.
  This is updated for the NodeMCU
  Version 3
  8/27/2017 - Adding heartbeat
  9/12/2017 - Adding OTA from Austin
  /* Create a WiFi access point and provide a web server on it. */


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

#ifndef _OTA_DELAY_REMOVED
#error "If you see this, add the above define and remove the delay calls."
#error "This is here to make sure ArduinoOTA.cpp has the delays removed from the functions."
#endif

IPAddress ipClient(192, 168, 1, 205);
//IPAddress ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);


/* Set these to your desired credentials. */
const char *ssid = "ProjectH002";
const char *password = "projecth";

const char *ota_hostname = "ProjectH";

const uint64_t LOADING_CIRCLE_TIME = 1000;
const uint64_t CHECKMARK_TIME = 1000;
const uint64_t HEARTBEAT_TIME = 900;

const float LOADING_CIRCLE_CONST = (LOADING_CIRCLE_TIME / 8000.0);
const float CHECKMARK_CONST = (CHECKMARK_TIME / 8000.0);
const float HEARTBEAT_CONST = (HEARTBEAT_TIME / 8000.0);

/*
  The input to the Anduino is as follows.
  First Number is the function
  Series of parameters
  Ending with one byte #10 - Also know as <lf> or /n or char #10 (These are the same. ).
*/

#define START_TOKEN "AUSTIN_TIM"
#define SERIES_TOKEN "PROJECTH"
#define PROJECT_TOKEN "REMOTECAR"

#define HEARTBEAT_FUNCTION 3
#define MOTOR_FUNCTION 5
#define SERVO_FUNCTION 6
//#define QueryPins 20

#define HEARTBEAT_TIMEOUT 3000     // 3 seconds of nothing will stop the motors

/*
  ------------------------------------
  For Heartbeat
  The Android will send Heartbeat only
  Example
  999,3
  999,3
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
  0 = no power - maybe the same as Mode 0
  50 = 50% of full motor speed
  100 = 100% of full motor speed

  Examples...
  999,5,1,2,100  // Motor 1 counter clockwise 100%
  999,5,2,1,50   // Motor 2 clockwise 50%
  999,5,2,0,0    // Motor 2 allow to coast to a stop
  999,5,1,3,0    // Motor 1 stop immediately. The last 0 is actually ignored but must be sent

  From WEMOS_Motor.h
  _SHORT_BRAKE  0
  _CCW      1
  _CW         2
  _STOP     3
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
  The Android will send 3 numbers followed by one byte <lf> or /n or char #10 (These are the same). Examples...
  20,1,24     Send ENA  // Left Motor PWM
  20,4,24     Send IN1  // Left Motor direction Control
  20,4,24     Send IN2  // Left Motor direction Control
  20,4,24     Send IN3  // Right Motor direction Control
  20,4,24     Send IN4  // Right Motor direction Control
  20,4,24     Send ENB  // Right Motor PWM

  // Pin on the Arduino going to the ESC for the weapon. This should be the yellow control wire.

  This code is in the public domain.
*/

//SSD1306  display(0x3c, 4, 5);

WiFiServer server(23);
Motor M1(0x30, _MOTOR_A, 1000); //Motor A
Motor M2(0x30, _MOTOR_B, 1000); //Motor B
SSD1306  display(0x3c, D2, D1);
Servo myservo;  // create servo object to control a servo

int displayLines[] = {15, 27, 39, 51}; // 4 Lines on the display

uint8_t M1dir = _CW;
int M1Speed = 0;
bool M1Changed = true;

uint8_t M2dir = _CW;
int M2Speed = 0;
bool M2Changed = true;

int S1Position = 0;
bool S1Changed = true;

int S2Position = 0;
bool S2Changed = true;

int S3Position = 0;
bool S3Changed = true;

// This is using D4
unsigned long NextLEDBlinkTime = 0;
bool LED_On_Off = 0;
#define NextLEDBlinkTimeout 2000     // 1 off / 1 second on

String sClientIn = "";         // a string to hold incoming data
bool connecting;
bool ClientConnected = false;
bool disconnected;
WiFiClient client;

elapsedMillis lastLoadingCircle;
elapsedMillis lastCheckmark;
elapsedMillis lastHeartbeat;
int HeartbeatTimeoutCNT = 0;

// Pin on the Arduino going to the ESC for the weapon. This should be the yellow control wire.
const int SERVOESC_PIN = D4; // THIS IS PIN D2.  CHOSEN BECAUSE IT DOESN'T INTERFERE WITH THE BOOT SEQUENCE ON THE D1

void setup() {
  delay(250);
  Serial.begin(250000);   // if not using the Pro Micro then use this and change every reference to Serial below to Serial
  delay(250);

  

  M1.setmotor(_STOP);
  M2.setmotor(_STOP);
  //  motorDrive(1, 1, 50);
  //  motorDrive(2, 1, 50);

  pinMode(LED_BUILTIN, OUTPUT);

  //Attach to the ESC
  myservo.attach(SERVOESC_PIN);
  //  delay(500);
  //  myservo.write(179);
  //  delay(500);

  //  Serial.print("Configuring access point...");
  /* You can remove the password parameter if you want the AP to be open. */

  /*
    WiFi.mode(WIFI_STA);
    WiFi.hostname(deviceName);      // DHCP Hostname (useful for finding device for static lease)
    WiFi.config(staticIP, gateway, subnet);  // (DNS not required)
    WiFi.begin(ssid, password);
  */

  //WiFi.mode(WIFI_AP);
  WiFi.mode(WIFI_AP_STA);   // New addition for OTA
  WiFi.hostname(ota_hostname);
  //WiFi.config(ipClient, gateway, subnet);  // (DNS not required)
  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();

  Serial.print("AP IP address: ");
  Serial.println(myIP);

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
    //    drawBatteryBar(0, 0, 8, 48, (millis() / 20) % 100);
    //    display.display();
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    display.clear();
    display.drawProgressBar(32, 40, 64, 10, (progress / (total / 100)));

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

  myIP = WiFi.softAPIP();
  Serial.print("2 AP IP address: ");
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

  //printWiFiStatus();

  client = server.available();
}

String getStartToken() {
  String token;
  uint8_t i = 0;
  char temp;
  while(true) {
    temp = EEPROM.read(i++);
    if(!temp) break;
    token += temp;
  }
  return token;
}

void writeStartToken() {
  for(uint8_t i = 0; i < sizeof(START_TOKEN); i++) writeEEPROM(i, START_TOKEN[i]);
}

void writeEEPROM(uint16_t i, uint8_t data) {
  if(EEPROM.read(i) != data) EEPROM.write(i, data);
}

int getNextToken(String &inStr) {
  int index = inStr.indexOf(',');
  String token;
  if (index >= 0) {
    token = inStr.substring(0, index);
    inStr.remove(0, index + 1);
  } else {
    token = inStr;
    inStr = "";
  }
  return token.toInt();
}

void processCommand(String sCommand) {
  int motorNum;
  int motorDirection;
  int motorPower;
  int servoNum;
  int servoPosition;

  int Starting999 = getNextToken(sCommand);

  if (Starting999 != 999) {    // Be sure the input starts with 999
    //    Serial.print("Didn't find 999 Purge line. Found: ");
    //    Serial.println(Starting999);
    return;
  }

  //Serial.print("Found 999. ");
  int functionNumber = getNextToken(sCommand);
  //Serial.print("functionNumber: ");
  //Serial.println(functionNumber);

  if (functionNumber == HEARTBEAT_FUNCTION) {
    lastHeartbeat = 0;
    HeartbeatTimeoutCNT = 0;
  }

  if (functionNumber == MOTOR_FUNCTION) {
    motorNum = getNextToken(sCommand);
    motorDirection = getNextToken(sCommand);
    motorPower = getNextToken(sCommand);

    if (motorNum == 1) {
      //Serial.println("Received command for Motor Number 1");
      M1Changed = true;
      M1dir = motorDirection;
      M1Speed = motorPower;
    }
    else if (motorNum == 2) {
      //Serial.println("Received command for Motor Number 2");
      M2Changed = true;
      M2dir = motorDirection;
      M2Speed = motorPower;
    }
  }

  if (functionNumber == SERVO_FUNCTION) {
    servoNum = getNextToken(sCommand);
    servoPosition = getNextToken(sCommand);

    if (servoNum == 1) {
      if (S1Position = servoPosition) {
        S1Changed = true;
        S1Position = servoPosition;
      }
    } else if (servoNum == 2) {
      if (S2Position = servoPosition) {
        S2Changed = true;
        S2Position = servoPosition;
      }
    } else if (servoNum == 3) {
      if (S3Position = servoPosition) {
        S3Changed = true;
        S3Position = servoPosition;
      };
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
  if ((lastHeartbeat > HEARTBEAT_TIMEOUT)) {// && (HeartbeatTimeoutCNT < 3)) {
    Serial.print("Heartbeat Lost. Motor shutdown");
    HeartbeatTimeoutCNT++;
    M1dir = 1;
    M1Speed = 0;
    M1Changed = true;

    M2dir = 1;
    M2Speed = 0;
    M2Changed = true;

    S1Position = 0;
    S1Changed = true;

    S2Position = 0;
    S2Changed = true;

    S3Position = 0;
    S3Changed = true;
  }
}

void checkLEDBlinkTimeout() {
  if (millis() > NextLEDBlinkTime) {
    NextLEDBlinkTime = millis() + NextLEDBlinkTimeout;
    LED_On_Off = !LED_On_Off;
    digitalWrite(LED_BUILTIN, LED_On_Off);
  }
}

void loop() {
  ArduinoOTA.handle();
  yield();
  String sCommand = "";

  checkHeartbeatTimeout();

  checkLEDBlinkTimeout();

  //Serial.println("M1Changed-Send to motor");
  motorDrive(1, M1dir, M1Speed);
  M1Changed = false;


  //Serial.println("M2Changed-Send to motor");
  motorDrive(2, M2dir, M2Speed);
  M2Changed = false;

  if (S1Changed) {
    myservo.write(S1Position);
    S1Changed = false;
  }

  if (client) {
    if (client.connected()) {
      ClientConnected = true;
      connecting = false;
      disconnected = false;
      int EOFPos = readClientRetEOFPos();

      if (EOFPos > 0) {   // Check that we have something to process
        sCommand = sClientIn.substring(0, EOFPos);
        sClientIn.remove(0, EOFPos + 1);
        processCommand(sCommand);
      }
    } else {
      ClientConnected = false;
      connecting = true;
      disconnected = true;
      lastLoadingCircle = 0;
      client.stop();
      client = server.available();
      sClientIn = "";   // a string to hold incoming data
      //      Serial.println("client disonnected");
    }
  } else {
    ClientConnected = false;
    connecting = true;
    disconnected = false;
    client = server.available();
    if (client) {
      ClientConnected = true;
      connecting = false;
      //      Serial.println("new client");
      sClientIn = "";   // a string to hold incoming data
    }
  }

  display.clear();
//  drawBatteryBar(3, 0, 4, 48, (1.0 - (WiFi.RSSI(0) / -100.0)) * 50);
//  Serial.println(WiFi.RSSI(0));
  drawLoadingCircle(32, 23, 14.9, 2, 10);
  drawCheckmark(32, 26, 11, 3);
  drawHeartbeat(46, 46, 15, 2, 2);
  int M1pixels = (M1Speed / 4.16);
  display.drawRect(getDisplayX(62), getDisplayY((M1dir == 1)?(24 - M1pixels):(24)),2 , M1pixels);
  int M2pixels = (M2Speed / 4.16);
  display.drawRect(getDisplayX(0), getDisplayY((M2dir == 2)?(24 - M2pixels):(24)),2 , M2pixels);
  display.display();
}

void WeaponDrive(int ServoNum, int percent) {
  if (ServoNum <= 0)
    exit;
  int Escduty = map(percent, 0, 100, 50, 179);
  if (percent == 0)
    Escduty = 50;
  else
    Escduty = 165;

  //  myservo.write(Escduty);
  //  myservo.attach(SERVOESC_PIN);
  // myservo.write(Escduty);

  /*
    delay(200);
    myservo.write();
    delay(200);
    myservo.detach();
  */
}

//******************   Motor A control   *******************
void motorDrive(int motorNum, int motorDir, int percent) {

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
        M1.setmotor(_STOP);
      if (motorNum == 2)
        M2.setmotor(_STOP);
      break;

    case 1:  //turn counter-clockwise
      //Serial.println("turn counter-clockwise");

      if (motorNum == 1) {
        M1.setmotor( _CCW, percent);
      }

      if (motorNum == 2) {
        M2.setmotor( _CCW, percent);
      }
      break;
    case 2:  //turn clockwise
      //Serial.println("turn clockwise");
      if (motorNum == 1)
        M1.setmotor( _CW, percent);
      if (motorNum == 2)
        M2.setmotor( _CW, percent);
      break;

    case 3:  //disable/coast
      //Serial.println("Motor disable/coast");
      if (motorNum == 1)
        M1.setmotor(_STANDBY);
      if (motorNum == 2)
        M2.setmotor(_STANDBY);
      break;
  }
}

//void printWiFiStatus() {
  // print your WiFi shield's IP address:
  //IPAddress ip = WiFi.softAPIP();
  //Serial.print("2 softAP IP Address: ");
  //Serial.println(ip);

  //sDisplay(1, ssid);
  //  String sIP = "IP:"+WiFi.softAPIP().toString();
  //sDisplay(2, WiFi.softAPIP().toString());

  //  display.display();

//}

uint16_t getDisplayX(uint16_t x) {
  return x + 32;
}

uint16_t getDisplayY(uint16_t y) {
  return y + 16;
}

/*
void drawBatteryBar(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t percent) {
  uint16_t barHeight = ((height - 1) * (percent / 100.0)) + 1;
  display.drawVerticalLine(getDisplayX(x), getDisplayY(y), height - barHeight);
  display.drawVerticalLine(getDisplayX(x) + width - 1, getDisplayY(y), height - barHeight);
  display.drawHorizontalLine(getDisplayX(x) + 1, getDisplayY(y), width - 2);
  if (percent > 100) percent = 100;
  display.fillRect(getDisplayX(x), getDisplayY(y) + height - barHeight, width, barHeight);
}
*/

void drawLoadingCircle(uint16_t x, uint16_t y, float radius, uint16_t thickness, uint16_t length) {
  float temp;
  uint16_t circumference = getRadians(radius);
  float offset = (length / 2) / ((float)circumference);
  bool endAnimation = false;
  if (ClientConnected || disconnected) {
    temp = lastLoadingCircle / ((float)LOADING_CIRCLE_TIME);
    if (temp > 1) {
      endAnimation = true;
      temp -= 1;
    }
  } else if (connecting) {
    lastLoadingCircle = (lastLoadingCircle % LOADING_CIRCLE_TIME);
    temp = (lastLoadingCircle % LOADING_CIRCLE_TIME) / ((float)LOADING_CIRCLE_TIME);
  } else {
    return;
  }
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
  uint16_t phaseNum = (width * 2) - dotSize;
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


// in loop   RotatingCube();

/*
  void RotatingCube () {
  const int Delta = 9; // Approximation to 1 degree in radians * 2^9
  xOrigin = 32; yOrigin = 24;
  int x = 0, y = 22<<9;
  for (;;) {
    ClearBuffer();
    int x9 = x>>9, y9 = y>>9, x10 = x>>10, y10 = y>>10;
    // Top
    MoveTo(x9, y10 + 12); DrawTo(y9, -x10 + 12);
    DrawTo(-x9, -y10 + 12); DrawTo(-y9, x10 + 12);
    DrawTo(x9, y10 + 12); DrawTo(x9, y10 - 12);
    // Bottom
    DrawTo(y9, -x10 - 12); DrawTo(-x9, -y10 - 12);
    DrawTo(-y9, x10 - 12); DrawTo(x9, y10 - 12);
    // Sides
    MoveTo(y9, -x10 + 12); DrawTo(y9, -x10 - 12);
    MoveTo(-x9, -y10 + 12); DrawTo(-x9, -y10 - 12);
    MoveTo(-y9, x10 + 12); DrawTo(-y9, x10 - 12);
    // Rotate cube
    x = x + (y9 * Delta);
    y = y - ((x>>9) * Delta);
    DisplayBuffer();
  }
  }

*/
