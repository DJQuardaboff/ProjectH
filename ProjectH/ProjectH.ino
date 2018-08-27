/*
   Arduino program for controlling a Battle Robot using 2 L298N H Bridge Chip and a Weapon using an ESC.
   This is updated for the NodeMCU
   Version 3
   8/27/2017 - Adding heartbeat
   9/12/2017 - Adding OTA from Austin
*/

#include <EEPROM.h>
//#include <ArduinoOTA.h>
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

//IPAddress ipClient(192, 168, 1, 205);
//IPAddress ip(192, 168, 4, 1);
//IPAddress gateway(192, 168, 1, 1);
//IPAddress subnet(255, 255, 255, 0);
uint32_t currentClientIP = 0;
uint16_t currentClientPort = 0;

const uint64_t LOADING_CIRCLE_TIME = 1000;
const uint64_t CHECKMARK_TIME      = 1000;
const uint64_t HEARTBEAT_TIME      = 900;

const float LOADING_CIRCLE_CONST = (LOADING_CIRCLE_TIME / 8000.0);
const float CHECKMARK_CONST      = (CHECKMARK_TIME / 8000.0);
const float HEARTBEAT_CONST      = (HEARTBEAT_TIME / 8000.0);

/*
  The input to the Arduino is as follows.
  First Number is the function
  Series of parameters
  Ending with one byte #10 - Also know as <lf> or \n or char #10.
*/

const uint32_t  START_TOKEN_ADDR             = 0x000;
const char      START_TOKEN[]                = "AUSTIN_TIM";
const uint32_t  START_TOKEN_LENGTH           = 0x010;
const uint32_t  SERIES_TOKEN_ADDR            = START_TOKEN_ADDR + START_TOKEN_LENGTH;
const char      SERIES_TOKEN[]               = "PROJECTH";
const uint32_t  SERIES_TOKEN_LENGTH          = 0x010;
const uint32_t  PROJECT_TOKEN_ADDR           = SERIES_TOKEN_ADDR + SERIES_TOKEN_LENGTH;
const char      PROJECT_TOKEN[]              = "REMOTECAR";
const uint32_t  PROJECT_TOKEN_LENGTH         = 0x010;
const uint32_t  SSID_TOKEN_ADDR              = PROJECT_TOKEN_ADDR + PROJECT_TOKEN_LENGTH;
const char      SSID_TOKEN_DEFAULT[]         = "projecth000";
const uint32_t  SSID_TOKEN_LENGTH            = 0x020;
const uint32_t  PW_TOKEN_ADDR                = SSID_TOKEN_ADDR + SSID_TOKEN_LENGTH;
const char      PW_TOKEN_DEFAULT[]           = "projecth";
const uint32_t  PW_TOKEN_LENGTH              = 0x020;
const uint32_t  UPDATER_SSID_TOKEN_ADDR      = PW_TOKEN_ADDR + PW_TOKEN_LENGTH;
const char      UPDATER_SSID_TOKEN_DEFAULT[] = "TimAustinUpdater";
const uint32_t  UPDATER_SSID_TOKEN_LENGTH    = 0x020;
const uint32_t  UPDATER_PW_TOKEN_ADDR        = UPDATER_SSID_TOKEN_ADDR + UPDATER_SSID_TOKEN_LENGTH;
const char      UPDATER_PW_TOKEN_DEFAULT[]   = "projecthup";
const uint32_t  UPDATER_PW_TOKEN_LENGTH      = 0x020;
const uint32_t  DISPLAYTEXT_TOKEN_ADDR       = UPDATER_PW_TOKEN_ADDR + UPDATER_PW_TOKEN_LENGTH;
const char      DISPLAYTEXT_TOKEN_DEFAULT[]  = "";
const uint32_t  DISPLAYTEXT_TOKEN_LENGTH     = 0x020;
const uint32_t  FULL_EEPROM_USE              = DISPLAYTEXT_TOKEN_ADDR + DISPLAYTEXT_TOKEN_LENGTH;

const char DELIMITERS[]                = ",\n";
const char SETUP_EEPROM_SSID[]         = "SOFT_AP_SSID";
const char SETUP_EEPROM_PW[]           = "SOFT_AP_PW";
const char SETUP_EEPROM_UPDATER_SSID[] = "SOFT_AP_SSID";
const char SETUP_EEPROM_UPDATER_PW[]   = "SOFT_AP_PW";
const char SETUP_EEPROM_DISPLAYTEXT[]  = "DISPLAYTEXT";

#define SETUP_FUNCTION            1
#define SETUP_EEPROM              5
#define SETUP_RESET               6
#define SETUP_REBOOT              7
#define HEARTBEAT_FUNCTION        3
#define MOTOR_FUNCTION            5
#define MOTOR_LEFT                1
#define MOTOR_RIGHT               2
#define SERVO_FUNCTION            6
#define HALT_DISCONNECT_FUNCTION  80
#define HALT_FUNCTION             81
#define DISCONNECT_FUNCTION       82

#define ERROR_FUNCTION            1

#define HEARTBEAT_TIMEOUT 3000     // 3 seconds of nothing will stop the motors

#define LED_BLINK_TIMEOUT 2000     // 1 off / 1 second on

/*
  ------------------------------------
  For HEARTBEAT_FUNCTION
  The App will send Heartbeat only
  Example:
  3
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
  5,1,2,100  // Motor 1 counter clockwise 100%
  5,2,1,50   // Motor 2 clockwise 50%
  5,2,0,0    // Motor 2 allow to coast to a stop
  5,1,3,0    // Motor 1 stop immediately. The last 0 is actually ignored but must be sent

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

  6,1,100  // Weapon on the ESC running 100%
  6,1,0    // Turn off the weapon

  This code is in the public domain.
*/

char ssid[SSID_TOKEN_LENGTH];
char password[PW_TOKEN_LENGTH];

char updater_ssid[UPDATER_SSID_TOKEN_LENGTH];
char updater_password[UPDATER_PW_TOKEN_LENGTH];

//const char *ota_hostname = "ProjectH";

char display_text[DISPLAYTEXT_TOKEN_LENGTH];

typedef struct {
  Servo servoObject;
  uint8_t pin;
  int position;
  bool changed;
} ServoInfo;

WiFiUDP server;
Motor leftMotor(0x30, _MOTOR_A, 1000);
Motor rightMotor(0x30, _MOTOR_B, 1000);
SSD1306 display(0x3c, D2, D1);

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

bool setupRequired = false;
const uint16_t SERVER_RX_SIZE = 127;
char serverRx[SERVER_RX_SIZE + 1];         // a string to hold incoming data
bool clientConnected = false;
bool lookingForUpdater = true;

elapsedMillis lastLoadingCircle;
elapsedMillis lastCheckmark;
elapsedMillis lastHeartbeat(HEARTBEAT_TIMEOUT + 1);

// Pin on the Arduino going to the ESC for the weapon. This should be the yellow control wire.
const int SERVOESC_PIN = D4; // THIS IS PIN D2.  CHOSEN BECAUSE IT DOESN'T INTERFERE WITH THE BOOT SEQUENCE ON THE D1

void setup() {
  Serial.begin(74880);
  while (!Serial);
  Serial.println();

  EEPROM.begin(FULL_EEPROM_USE);
  char temp2[START_TOKEN_LENGTH];
  readEEPROMToken(START_TOKEN_ADDR, temp2, START_TOKEN_LENGTH);
  if (strcmp(temp2, START_TOKEN)) {
    writeEEPROMToken(START_TOKEN_ADDR, START_TOKEN, START_TOKEN_LENGTH);
    writeEEPROMToken(SERIES_TOKEN_ADDR, SERIES_TOKEN, SERIES_TOKEN_LENGTH);
    writeEEPROMToken(PROJECT_TOKEN_ADDR, PROJECT_TOKEN, PROJECT_TOKEN_LENGTH);
    writeEEPROMToken(SSID_TOKEN_ADDR, SSID_TOKEN_DEFAULT, SSID_TOKEN_LENGTH);
    writeEEPROMToken(PW_TOKEN_ADDR, PW_TOKEN_DEFAULT, PW_TOKEN_LENGTH);
    writeEEPROMToken(UPDATER_SSID_TOKEN_ADDR, UPDATER_SSID_TOKEN_DEFAULT, UPDATER_SSID_TOKEN_LENGTH);
    writeEEPROMToken(UPDATER_PW_TOKEN_ADDR, UPDATER_PW_TOKEN_DEFAULT, UPDATER_PW_TOKEN_LENGTH);
    writeEEPROMToken(DISPLAYTEXT_TOKEN_ADDR, DISPLAYTEXT_TOKEN_DEFAULT, DISPLAYTEXT_TOKEN_LENGTH);
    setupRequired = true;
  }
  
  readEEPROMToken(SSID_TOKEN_ADDR, ssid, SSID_TOKEN_LENGTH);
  
  readEEPROMToken(PW_TOKEN_ADDR, password, PW_TOKEN_LENGTH);
  char* password_end = strchr(password, 0);
  if (!password_end || (password_end - password) < 8) {
    writeEEPROMToken(PW_TOKEN_ADDR, PW_TOKEN_DEFAULT, PW_TOKEN_LENGTH);
    strcpy(password, PW_TOKEN_DEFAULT);
  }
  
  readEEPROMToken(UPDATER_SSID_TOKEN_ADDR, updater_ssid, UPDATER_SSID_TOKEN_LENGTH);
  
  readEEPROMToken(UPDATER_PW_TOKEN_ADDR, updater_password, UPDATER_PW_TOKEN_LENGTH);
  char* updater_password_end = strchr(updater_password, 0);
  if (!updater_password_end || (updater_password_end - updater_password) < 8) {
    writeEEPROMToken(UPDATER_PW_TOKEN_ADDR, UPDATER_PW_TOKEN_DEFAULT, UPDATER_PW_TOKEN_LENGTH);
    strcpy(updater_password, UPDATER_PW_TOKEN_DEFAULT);
  }
  
  readEEPROMToken(DISPLAYTEXT_TOKEN_ADDR, display_text, DISPLAYTEXT_TOKEN_LENGTH);
  
  EEPROM.commit();

  leftMotor.setmotor(_STOP);
  rightMotor.setmotor(_STOP);

  pinMode(LED_BUILTIN, OUTPUT);

  /*
    for (uint8_t i = 0; i < servoNum; i++) {
      servo[i].servoObject.attach(SERVOESC_PIN);
    }
  */

  //WiFi.hostname(ota_hostname);
  //WiFi.config(ipClient, gateway, subnet);  // (DNS not required)
  //WiFi.begin(updater_ssid, updater_password);
  WiFi.softAP(ssid, password);
  WiFi.mode(WIFI_AP);

  // start the server listening
  server.begin(8192);
  // you're connected now, so print out the status:
  /*
    ArduinoOTA.onStart([]() {
      //String type;
      //if (ArduinoOTA.getCommand() == U_FLASH) type = "sketch"; else type = "filesystem";
      //NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      //Serial.println("Start updating " + type);
      display.clear();
      display.display();
      Serial.println("Start updating ");
    });

    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      String progressStr = String(progress / (total / 100)) + '%';
      display.clear();
      display.setFont(ArialMT_Plain_10);
      display.setTextAlignment(TEXT_ALIGN_CENTER);
      display.drawString(getDisplayX(32), getDisplayY(10), progressStr);
      display.drawProgressBar(getDisplayX(0), getDisplayX(10), 63, 8, (progress / (total / 100)));
      display.display();
      Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
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
  */
  Serial.println("Ready");
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  Serial.println();
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(password);
  Serial.print("Updater SSID: ");
  Serial.println(updater_ssid);
  Serial.print("Updater Password: ");
  Serial.println(updater_password);
  Serial.print("Display text: ");
  Serial.println(display_text);

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.clear();

  digitalWrite(LED_BUILTIN, HIGH); //turn the buit-in led off
}

void readEEPROMToken(uint32_t addr, char* buffer, uint32_t maxLength) {
  for (uint16_t i = 0; i < maxLength; i++) {
    char temp = EEPROM.read(addr++);
    buffer[i] = temp;
    if (!temp) break;
  }
}

void writeEEPROMToken(uint32_t addr, const char* str, uint32_t maxLength) {
  uint32_t length = _min(strlen(str), maxLength - 1);
  for (uint32_t i = 0; i < length; i++) EEPROM.write(addr + i, str[i]);
  EEPROM.write(addr + length, 0x00);
}

void nextToken(const char*& str) {
  str = strchr(str, ',');
  if (str) str++;
}

bool getNextTokenStrEquals(const char*& str, const char* expected, bool advanceOnlyIfTrue = false) {
  if (!str) return false;
  bool result = false;
  if (expected) {
    size_t length = strlen(expected);
    if (strlen(expected) == strcspn(str, DELIMITERS)) {
      result = !memcmp(str, expected, length);
    }
  }
  if (result || !advanceOnlyIfTrue) {
    nextToken(str);
  }
  return result;
}

void getNextTokenStr(char* dest, const char*& str, size_t maxSize) {
  if (!str) return;
  if (dest) {
    size_t length = _min(maxSize - 1, strcspn(str, DELIMITERS));
    memcpy(dest, str, length);
    dest[length] = 0;
  }
  nextToken(str);
}

int getNextTokenInt(const char*& str) {
  if (!str) return 0;
  int result = atoi(str);
  nextToken(str);
  return result;
}

void haltMotors() {
    leftMotorDir = 1;
    leftMotorSpeed = 0;
    leftMotorChanged = true;

    rightMotorDir = 1;
    rightMotorSpeed = 0;
    rightMotorChanged = true;
}

void setIsConnected(bool isConnected, bool haltOnDisconnect = true) {
  if (isConnected && !clientConnected) {
    Serial.println("connected");
    clientConnected = true;
    lastCheckmark = 0;

    currentClientIP = server.remoteIP();
    currentClientPort = server.remotePort();
  } else if (!isConnected && clientConnected) {
    Serial.println("disconnected");
    clientConnected = false;
    lastLoadingCircle = 0;
    
    currentClientIP = 0;
    currentClientPort = 0;

    if (haltOnDisconnect) haltMotors();
  }
}

void processCommand(const char* command) {
  const char* currentToken = command;
  nextToken(currentToken);
  int function = getNextTokenInt(currentToken);
  if (function == SETUP_FUNCTION) {
    int function2 =  getNextTokenInt(currentToken);
    bool projectCheck = getNextTokenStrEquals(currentToken, PROJECT_TOKEN);
    if (function2 == SETUP_EEPROM) {
      if (getNextTokenStrEquals(currentToken, SETUP_EEPROM_SSID, true)) {
        char str[SSID_TOKEN_LENGTH];
        getNextTokenStr(str, currentToken, SSID_TOKEN_LENGTH);
        //Serial.print(String("SSID changed to: " + str));
        writeEEPROMToken(SSID_TOKEN_ADDR, str, SSID_TOKEN_LENGTH);
        EEPROM.commit();
      } else if (getNextTokenStrEquals(currentToken, SETUP_EEPROM_PW, true)) {
        char str[PW_TOKEN_LENGTH];
        getNextTokenStr(str, currentToken, PW_TOKEN_LENGTH);
        if (strlen(str) < 8) {
          //Serial.print(String("Error changing password to: " + str));
          sendErrorShortPassword(str);
        } else {
          //Serial.print(String("Password changed to: " + str));
          writeEEPROMToken(PW_TOKEN_ADDR, str, PW_TOKEN_LENGTH);
          EEPROM.commit();
        }
      } else if (getNextTokenStrEquals(currentToken, SETUP_EEPROM_UPDATER_SSID, true)) {
        char str[UPDATER_SSID_TOKEN_LENGTH];
        getNextTokenStr(str, currentToken, UPDATER_SSID_TOKEN_LENGTH);
        //Serial.print(String("Updater SSID changed to: " + str));
        if (WiFi.status() != WL_CONNECTED) strcpy(updater_ssid, str);
        writeEEPROMToken(UPDATER_SSID_TOKEN_ADDR, str, UPDATER_SSID_TOKEN_LENGTH);
        EEPROM.commit();
      } else if (getNextTokenStrEquals(currentToken, SETUP_EEPROM_UPDATER_PW, true)) {
        char str[UPDATER_PW_TOKEN_LENGTH];
        getNextTokenStr(str, currentToken, UPDATER_PW_TOKEN_LENGTH);
        if (strlen(str) < 8) {
          sendErrorShortPassword(str);
          //Serial.println(ERROR_SHORTPASSWORD(str));
        } else {
          //Serial.print(String("Updater password changed to: " + str));
          if (WiFi.status() != WL_CONNECTED) strcpy(updater_password, str);
          writeEEPROMToken(UPDATER_PW_TOKEN_ADDR, str, UPDATER_PW_TOKEN_LENGTH);
          EEPROM.commit();
        }
      } else if (getNextTokenStrEquals(currentToken, SETUP_EEPROM_DISPLAYTEXT, true)) {
        char str[DISPLAYTEXT_TOKEN_LENGTH];
        getNextTokenStr(str, currentToken, DISPLAYTEXT_TOKEN_LENGTH);
        //Serial.print(String("Display text changed to: " + str));
        strcpy(display_text, str);
        writeEEPROMToken(DISPLAYTEXT_TOKEN_ADDR, str, DISPLAYTEXT_TOKEN_LENGTH);
        EEPROM.commit();
      }
    } else if (function2 == SETUP_RESET) {
      EEPROM.write(START_TOKEN_ADDR, 0x00);
      EEPROM.commit();
    } else if (function2 == SETUP_REBOOT) {
      ESP.restart();
    }
  } else if (function == HEARTBEAT_FUNCTION) {
    lastHeartbeat = 0;
  } else if (function == MOTOR_FUNCTION) {
    int motorNum = getNextTokenInt(currentToken);
    int motorDirection = getNextTokenInt(currentToken);
    int motorPower = getNextTokenInt(currentToken);

    if (motorNum == MOTOR_LEFT) {
      //Serial.println("Received command for Motor Number 1");
      /*
      Serial.print("MOTOR_LEFT(");
      Serial.print(motorDirection);
      Serial.print(", ");
      Serial.print(motorPower);
      Serial.println(")");
      */
      leftMotorChanged = true;
      leftMotorDir = motorDirection;
      leftMotorSpeed = motorPower;
    } else if (motorNum == MOTOR_RIGHT) {
      //Serial.println("Received command for Motor Number 2");
      /*
      Serial.print("MOTOR_RIGHT(");
      Serial.print(motorDirection);
      Serial.print(", ");
      Serial.print(motorPower);
      Serial.println(")");
      */
      rightMotorChanged = true;
      rightMotorDir = motorDirection;
      rightMotorSpeed = motorPower;
    }
  } else if (function == HALT_DISCONNECT_FUNCTION) {
    setIsConnected(false);
  } else if (function == HALT_FUNCTION) {
    haltMotors();
  } else if (function == DISCONNECT_FUNCTION) {
    setIsConnected(false, false);
  }/* else if (function == SERVO_FUNCTION) {
    int servoIndex = iGetToken(command);
    int servoPosition = iGetToken(command);

    if (servoIndex >= 0 && servoIndex < servoNum) {
      servo[servoIndex].position = servoPosition;
      servo[servoIndex].changed = true;
    }
  }*/ else {
    sendErrorParseError(command);
  }
}

void checkHeartbeatTimeout() {
  // heartbeat - if there was no heartbeat command sent in the past HEARTBEAT_TIMEOUT milliseconds then disconnect
  setIsConnected(lastHeartbeat <= HEARTBEAT_TIMEOUT);
}

void checkLedBlinkTimeout() {
  if (lastLedChange >= LED_BLINK_TIMEOUT) {
    lastLedChange -= LED_BLINK_TIMEOUT;
    ledOn = !ledOn;
    //digitalWrite(LED_BUILTIN, ledOn);
  }
}

void sendError(const char* errorMessage) {
  server.beginPacket(server.remoteIP(), server.remotePort());
  server.write(ERROR_FUNCTION);
  server.write(',');
  server.write(errorMessage);
  server.write('\n');
  server.endPacket();
}

void sendErrorShortPassword(const char* password) {
  server.beginPacket(server.remoteIP(), server.remotePort());
  server.write(ERROR_FUNCTION);
  server.write(",Password '");
  server.write(password);
  server.write("' too short\n");
  server.endPacket();
}

void sendErrorParseError(const char* command) {
  server.beginPacket(server.remoteIP(), server.remotePort());
  server.write(ERROR_FUNCTION);
  server.write(",Parse error: '");
  server.write(command);
  server.write("'\n");
  server.endPacket();
}

bool getNextPacket(char* buffer, uint16_t maxSize) {
  int packetSize = server.parsePacket();
  if (packetSize && packetSize <= maxSize) {
    int len = server.read(buffer, maxSize);
    buffer[len] = 0;
    return true;
  } else {
    buffer[0] = 0;
    return false;
  }
}

void doCommand(char* command) {
  char* endChar = strchr(command, '\n');
  if (endChar) {
    server.beginPacket(currentClientIP, currentClientPort);
    server.write(command);
    server.endPacket();

    endChar[0] = 0;
    processCommand(command);
    command[0] = 0;
  }
}

bool isCurrentClient() {
  return (currentClientIP == server.remoteIP() && currentClientPort == server.remotePort()) || (!currentClientIP && !currentClientPort);
}

void loop() {
  /*
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
    yield();
  }
  */

  checkHeartbeatTimeout();

  //checkLedBlinkTimeout(); led doesn't look good

  motorDrive(MOTOR_LEFT, leftMotorDir, leftMotorSpeed);
  leftMotorChanged = false;

  motorDrive(MOTOR_RIGHT, rightMotorDir, rightMotorSpeed);
  rightMotorChanged = false;

  if (getNextPacket(serverRx, SERVER_RX_SIZE) && isCurrentClient()) { // check that we have something to process
    doCommand(serverRx);
  }

  yield();
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(getDisplayX(31), getDisplayY(-3), ssid);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(getDisplayX(3), getDisplayY(34), display_text);
  drawLoadingCircle(32, 23, 11, 2, 10);
  drawCheckmark(32, 25, 8, 3);
  //drawHeartbeat(46, 46, 15, 2, 2);
  if (lastHeartbeat < 25) display.drawRect(getDisplayX(60), getDisplayY(46), 2 , 2);
  drawMotorVisuals();
  display.display();
  yield();
}

void drawMotorVisuals() {
  uint16_t leftMotorPixels = (leftMotorSpeed / 4.16);
  display.drawRect(getDisplayX(0), getDisplayY((leftMotorDir == 2) ? (24 - leftMotorPixels) : (24)), 2 , leftMotorPixels);
  uint16_t rightMotorPixels = (rightMotorSpeed / 4.16);
  display.drawRect(getDisplayX(62), getDisplayY((rightMotorDir == 1) ? (24 - rightMotorPixels) : (24)), 2 , rightMotorPixels);
}

void WeaponDrive(int servoNum, uint8_t percent) {
  if (servoNum <= 0) return;
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

void motorDrive(int motorNum, int motorDir, int percent) {
  percent = _min(percent, 100);

  if (motorNum <= 0 || motorNum > 2) return;

  switch (motorDir) {
    case _SHORT_BRAKE:
      if (motorNum == 1) leftMotor.setmotor(_STOP); else if (motorNum == 2) rightMotor.setmotor(_STOP);
      break;
    case _CCW:
      if (motorNum == 1) leftMotor.setmotor( _CCW, percent); else if (motorNum == 2) rightMotor.setmotor( _CCW, percent);
      break;
    case _CW:
      if (motorNum == 1) leftMotor.setmotor( _CW, percent); else if (motorNum == 2) rightMotor.setmotor( _CW, percent);
      break;
    case _STOP:
      if (motorNum == 1) leftMotor.setmotor(_STANDBY); else if (motorNum == 2) rightMotor.setmotor(_STANDBY);
      break;
  }
}

uint16_t getDisplayX(int x) {
  return x + 32;
}

uint16_t getDisplayY(int y) {
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
  if (clientConnected) {
    temp = lastLoadingCircle / ((float)LOADING_CIRCLE_TIME);
    if (temp > 1) {
      endAnimation = true;
      temp -= 1;
    }
  } else {
    lastLoadingCircle = (lastLoadingCircle % LOADING_CIRCLE_TIME);
    temp = (lastLoadingCircle % LOADING_CIRCLE_TIME) / ((float)LOADING_CIRCLE_TIME);
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
  if (!clientConnected) return;
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
/*
void drawHeartbeat(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t dotSize) {
  uint16_t phaseNum = ((width - (dotSize - 1)) * 2) - 1;
  //if (clientConnected) lastHeartbeat = lastHeartbeat % HEARTBEAT_TIME;
  float phase = (((lastHeartbeat / (HEARTBEAT_TIME / ((float)phaseNum)))) + dotSize - 1) / ((float)phaseNum);
  phase = (phase < .5) ? (exp(phase / HEARTBEAT_CONST)) : ((exp(.5 / HEARTBEAT_CONST) * 2) - exp((1 - phase) / HEARTBEAT_CONST));
  phase = (((phase / ((exp(.5 / HEARTBEAT_CONST) * 2) - 1))) * phaseNum) + 1;
  //phase++;
  if (phase > phaseNum) phase = phaseNum;
  if (phase >= width) phase = -phase + (width * 2) - 2;
  for (uint16_t i = 0; i < width; i++) {
    if (i > phase || i < phase - dotSize) display.drawVerticalLine(getDisplayX(x) + i, getDisplayY(y), height);
  }
}
*/
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
