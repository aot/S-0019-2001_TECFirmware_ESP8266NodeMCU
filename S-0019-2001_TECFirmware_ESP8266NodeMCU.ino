/* TEC Firmware
 *  Thermoelectric Controller on a NodeMCU v3
 *  Using a IBT_2 (dual BTS7960S H-Bridge)and AM2302 temperature/humidity sensor.

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
  SDT3    GPIO 10
  SDT2    GPIO 9
  SDT1    GPIO 8
  CMD     GPIO 11
  SD0     GPIO 7
  CLK     GPIO 6
 
  
 
 */
#define SSD1306_128_32

 #include "DHTesp.h"
 #include "RunningAverage.h"
 #include "PidController.h"
 #include "ACROBOTIC_SSD1306.h"
 #include <Wire.h>

 /* Define Pins */
const uint8_t pinHBridgeREn = 13;
const uint8_t pinHBridgeLEn = 15;
const uint8_t pinHBridgeRPwm = 14;
const uint8_t pinHBridgeLPwm = 12;


 DHTesp dht;
 float humidity;
 float temperature;

 uint32_t tSerial;
 uint32_t tDHTSample;

 uint16_t serialUpdatePeriod = 1000;

float kp = 1.0;
float ki = 0.01;
float kd = 0.0;
float setpoint = 20;
float pidOutput = 0;
uint16_t computePidInterval = 1000;

PidController tecPID(kp, ki, kd, setpoint, computePidInterval);

uint32_t tComputePid;
uint32_t tSoftPwm;

uint16_t softPwmPeriod = computePidInterval;
uint16_t softPwmResolution = 100; 
uint16_t softPwmInterval = softPwmPeriod / softPwmResolution;
uint16_t softPwmCounter = 0;

RunningAverage<uint16_t> raTemperature;
const uint8_t tempMeasNoAvg = 20; //Number of samples in the temperature measurement running average window

bool doPid = true;
bool isOn = true;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  dht.setup(0, DHTesp::DHT22);
  oled.init();
  oled.clearDisplay();
  oled.setTextXY(2,0);
  oled.setFont(font8x8);
  
  tSerial = millis();
  tDHTSample = millis();
  tSoftPwm = millis();
  tComputePid = millis();

  pinMode(pinHBridgeREn, OUTPUT);
  pinMode(pinHBridgeLEn, OUTPUT);
  pinMode(pinHBridgeRPwm, OUTPUT);
  pinMode(pinHBridgeLPwm, OUTPUT);
  raTemperature.init(tempMeasNoAvg);

  
}

void loop() {
  // put your main code here, to run repeatedly:

  if (millis() > tDHTSample + dht.getMinimumSamplingPeriod()){
    tDHTSample+=dht.getMinimumSamplingPeriod();
    humidity = dht.getHumidity();
    temperature = raTemperature.addSample(dht.getTemperature());
    
  }

  if(millis() > tComputePid + computePidInterval){
    tComputePid += computePidInterval;
    if (doPid){
      
      pidOutput = tecPID.AddSample(temperature);    
    }
  }

    if (millis() > tSoftPwm + softPwmInterval){
    tSoftPwm += softPwmInterval;
    softPwmCounter += softPwmInterval;
    if(softPwmCounter > softPwmPeriod){
      softPwmCounter = 0;
    }
    if(isOn){
      if (softPwmCounter < abs(pidOutput) * softPwmPeriod){
        if(pidOutput > 0){ // output positive, heating
          digitalWrite(pinHBridgeREn, HIGH);
          digitalWrite(pinHBridgeLEn, HIGH);
          digitalWrite(pinHBridgeLPwm, LOW);
          digitalWrite(pinHBridgeRPwm, HIGH);
          
        }else if(pidOutput < 0){ // output negative, cooling
          digitalWrite(pinHBridgeREn, HIGH);
          digitalWrite(pinHBridgeLEn, HIGH);
          digitalWrite(pinHBridgeRPwm, LOW);
          digitalWrite(pinHBridgeLPwm, HIGH);
        }
      }else{
        digitalWrite(pinHBridgeREn, LOW);
        digitalWrite(pinHBridgeLEn, LOW);
        digitalWrite(pinHBridgeLPwm, LOW);
        digitalWrite(pinHBridgeRPwm, LOW);
      } 
    }
  }


  if (millis() > tSerial + serialUpdatePeriod){
    tSerial+=serialUpdatePeriod;
    oled.setTextXY(2,0);
    oled.putFloat(temperature,1);
    Serial.print(dht.getStatusString());
    Serial.print('\t');
    Serial.print(temperature,1);
    Serial.print('\t');
    Serial.print(humidity,1);
    Serial.println('\t');
  }
  
  

  

}
