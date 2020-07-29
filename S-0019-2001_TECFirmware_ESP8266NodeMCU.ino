/* TEC Firmware
    Thermoelectric Controller on a NodeMCU v3
    Using a IBT_2 (dual BTS7960S H-Bridge)and AM2302 temperature/humidity sensor.

  Pin Mapping
  ~ denotes PWM
  Name    Arduino Mapping   Connected To
  D0      GPIO 16
  D1/SCL  GPIO 5            OLED:SCK
  D2/SDA~ GPIO 4            OLED:SDA
  D3      GPIO 0            AM2302:Data
  D4      GPIO 2
  D5~     GPIO 14           IBT_2:RPWM
  D6~     GPIO 12           IBT_2:LPWM
  D7      GPIO 13           IBT_2:R_EN
  D8~     GPIO 15           IBT_2:L_EN
  RX      GPIO 3
  TX      GPIO 1
  A0
  SDT3    GPIO 10           FAN_A
  SDT2    GPIO 9            FAN_B
  SDT1    GPIO 8
  CMD     GPIO 11
  SD0     GPIO 7
  CLK     GPIO 6



*/

#include "DHTesp.h"
#include "RunningAverage.h"
#include "PidController.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>

#ifndef STASSID
#define STASSID "izazzle"
#define STAPSK  "334aWFP52"
#endif

const char* ssid = STASSID;
const char* password = STAPSK;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define MODE_IDLE 0
#define MODE_HEATING 1
#define MODE_COOLING 2

/* Define Pins */
const uint8_t pinHBridgeREn = 13;
const uint8_t pinHBridgeLEn = 15;
const uint8_t pinHBridgeRPwm = 14;
const uint8_t pinHBridgeLPwm = 12;
const uint8_t pinFanA = 10;
const uint8_t pinFanB = 9;

uint8_t fanLevel = 255;
float fanThreshold = 0.25;

DHTesp dht;
float humidity = 50;
float temperature=20;

uint32_t tSerial;
uint32_t tDHTSample;

uint16_t serialUpdatePeriod = 1000;

float kp = 1.5;
float ki = 0.0010;
float kd = 10.0;
float setpoint = 20;
float pidOutput = 0.0;
uint16_t computePidInterval = 10000;

uint16_t addrSetpoint = 0;

PidController *tecPID = NULL;

uint8_t tecDir = 1;

uint32_t tComputePid;
uint32_t tSoftPwm;

uint16_t softPwmPeriod = computePidInterval;
uint16_t softPwmResolution = 100;
uint16_t softPwmInterval = softPwmPeriod / softPwmResolution;
uint16_t softPwmCounter = 0;

RunningAverage<float> raTemperature;
const uint16_t tempMeasNoAvg = 20; //Number of samples in the temperature measurement running average window

bool doPid = true;
bool isOn = true;

uint8_t mode = MODE_IDLE;
float temperatureThreshold = 1.0;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  EEPROM.begin(512);
  //Wire.begin();
  dht.setup(0, DHTesp::DHT22);
  //humidity = dht.getHumidity();
  //temperature = dht.getTemperature();

  tDHTSample = millis();
  tSerial = millis();
  
  tSoftPwm = millis();
  tComputePid = millis();

  pinMode(pinHBridgeREn, OUTPUT);
  pinMode(pinHBridgeLEn, OUTPUT);
  pinMode(pinHBridgeRPwm, OUTPUT);
  pinMode(pinHBridgeLPwm, OUTPUT);
  pinMode(pinFanA, OUTPUT);
  pinMode(pinFanB, OUTPUT);
  digitalWrite(pinFanA, LOW);
  digitalWrite(pinFanB, HIGH);
  raTemperature.init(tempMeasNoAvg);
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  display.clearDisplay();
  

  EEPROM.get(addrSetpoint, setpoint);
  //Serial.println();
  //Serial.println(setpoint);
  delay(100);
  
  tecPID = new PidController(kp, ki, kd, setpoint, computePidInterval);
  //Serial.print(tecPID->GetSetPoint());
  //Serial.print('\t');
  //Serial.print(tecPID->GetIntegralTerm());
  //Serial.println('\t');
  tecPID->SetOutputLimits(-1.0, 1.0);

  delay(5000);
  digitalWrite(pinFanB, LOW);


  
  

}

void loop() {
  // put your main code here, to run repeatedly:

  if (millis() > tDHTSample + dht.getMinimumSamplingPeriod()) {
    tDHTSample += dht.getMinimumSamplingPeriod();
    humidity = dht.getHumidity();
    temperature = dht.getTemperature();
    
    //temperature = raTemperature.addSample(dht.getTemperature());

  }

  switch(mode){
    case MODE_IDLE:
      if (temperature < setpoint - temperatureThreshold){
        mode = MODE_HEATING;
        tecPID->SetIntegralTerm(0.0); //reset integral term 
        doPid = true;
        digitalWrite(pinFanB, HIGH);
      }
      
      if (temperature > setpoint + temperatureThreshold){
        mode = MODE_COOLING;
        tecPID->SetIntegralTerm(0.0); //reset integral term
        doPid = true;
        digitalWrite(pinFanB, HIGH);
      }

      break;
    
    case MODE_HEATING:
      if (temperature > setpoint){
        mode = MODE_IDLE;
        doPid = false;
        digitalWrite(pinFanB, LOW);
      }
      
      break;
    
    case MODE_COOLING:
      if (temperature < setpoint){
        mode = MODE_IDLE;
        doPid = false;
        digitalWrite(pinFanB, LOW);
      }
      
      break;

    default:
      break;
  }

  if (millis() > tComputePid + computePidInterval) {
    tComputePid += computePidInterval;
    if (doPid) {
      if(!isnan(temperature)){
        pidOutput = tecPID->AddSample(temperature);
        if (mode == MODE_IDLE){
          pidOutput = 0;
        }
      }
    }
  }

  if (millis() > tSoftPwm + softPwmInterval) {
    tSoftPwm += softPwmInterval;
    softPwmCounter ++;
    if (softPwmCounter > softPwmResolution) {
      softPwmCounter = 0;
    }
    if (isOn) {
      /*if (abs(pidOutput) < fanThreshold){
        digitalWrite(pinFanB, HIGH);
      }else{
        digitalWrite(pinFanB, LOW);
      }*/
      if (softPwmCounter < abs(pidOutput) * softPwmPeriod) {
        if (pidOutput > 0) { // output positive, heating
          if (tecDir == 0){
            digitalWrite(pinHBridgeREn, HIGH);
            digitalWrite(pinHBridgeLEn, HIGH);
            digitalWrite(pinHBridgeLPwm, LOW);
            digitalWrite(pinHBridgeRPwm, HIGH);
          } else if(tecDir == 1){
            digitalWrite(pinHBridgeREn, HIGH);
            digitalWrite(pinHBridgeLEn, HIGH);
            digitalWrite(pinHBridgeRPwm, LOW);
            digitalWrite(pinHBridgeLPwm, HIGH);
          }

        } else if (pidOutput < 0) { // output negative, cooling
          if (tecDir == 0){
            digitalWrite(pinHBridgeREn, HIGH);
            digitalWrite(pinHBridgeLEn, HIGH);
            digitalWrite(pinHBridgeRPwm, LOW);
            digitalWrite(pinHBridgeLPwm, HIGH);
          }else if(tecDir == 1){
            digitalWrite(pinHBridgeREn, HIGH);
            digitalWrite(pinHBridgeLEn, HIGH);
            digitalWrite(pinHBridgeLPwm, LOW);
            digitalWrite(pinHBridgeRPwm, HIGH);
          }
        }
      } else {
        digitalWrite(pinHBridgeREn, LOW);
        digitalWrite(pinHBridgeLEn, LOW);
        digitalWrite(pinHBridgeLPwm, LOW);
        digitalWrite(pinHBridgeRPwm, LOW);
        
      }
      
    }
    
  }


  if (millis() > tSerial + serialUpdatePeriod) {
    tSerial += serialUpdatePeriod;
    //display.clearDisplay();
    display.setTextColor(WHITE, BLACK);
    
    display.setCursor(0,0);
    display.print("T ");
    display.print(temperature);
    display.println("C");
    display.print("S ");
    display.print(tecPID->GetSetPoint());
    display.println("C");
    display.print("H ");
    display.print(humidity);
    display.println("%");
    display.print("P ");
    display.println(pidOutput);
    
    display.display();
    

    //Serial.print(dht.getStatusString());
    //Serial.print('\t');
    Serial.print(temperature, 1);
    Serial.print('\t');
    Serial.print(humidity, 1);
    Serial.print('\t');
    Serial.print(tecPID->GetSetPoint(), 1);
    Serial.print('\t');
    Serial.print(pidOutput, 2);
    Serial.print('\t');
    Serial.print(tecPID->GetKp(),3);
    Serial.print('\t');
    Serial.print(tecPID->GetKi(),3);
    Serial.print('\t');
    Serial.print(tecPID->GetKd(),3);
    Serial.print('\t');
    Serial.print(fanThreshold,3);
    Serial.print('\t');
    Serial.print(mode);
    Serial.println();
  }
  packetHandler();


}

void packetHandler(){
  if(Serial.available()){
    char indicator = Serial.read();
    switch(indicator)
    {
      /*case 'f': //pump control
        fanLevel = Serial.parseInt();
        analogWrite(pinPumpPwm, fanLevel);
        break;*/

      case 's': //temperature setpoint
        setpoint = Serial.parseFloat();
        tecPID->SetSetPoint(setpoint);
        EEPROM.put(addrSetpoint, setpoint);
        EEPROM.commit();
        break;

      case 'p': //kp
        kp = Serial.parseFloat();
        tecPID->SetPidValues(kp, ki, kd);
        break;

      case 'i': //ki
        ki = Serial.parseFloat();
        tecPID->SetPidValues(kp, ki, kd);
        break;

      case 'd': //kd
        kd = Serial.parseFloat();
        tecPID->SetPidValues(kp, ki, kd);
        break;

      case 'o': //on/off
        isOn = !isOn; //toggle system
        if(isOn){
          
          doPid = true;
          digitalWrite(pinFanB, HIGH);
        }else{

          digitalWrite(pinHBridgeREn, LOW);
          digitalWrite(pinHBridgeLEn, LOW);
          digitalWrite(pinHBridgeLPwm, LOW);
          digitalWrite(pinHBridgeRPwm, LOW);
          digitalWrite(pinFanB, LOW);
          doPid = false;
          pidOutput = 0;
        }

      default:
        break;
        
    }
  }
}
