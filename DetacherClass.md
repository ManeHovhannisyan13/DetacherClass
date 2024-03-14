//DetacherClass.ion
#include "RelayClass.h"
#include "LedClass.h"
#include "Detacher.h"

DeviceManager deviceManager;
LED led;
Relay relay;
int ledPin = 2;
int relayPin = 4;
void setup() {

}

void loop() 
{
  deviceManager.detachAll(led, relay, ledPin, relayPin);

}

//LedClass.cpp
#include "esp32-hal.h"
#include "esp32-hal-gpio.h"
#include "LedClass.h"
#include "Arduino.h"

LED::LED(){
  
}

LED::LED(int pin)
{
  attach(pin);
  _pin = pin;
}

void LED::attach(int pin)
{
  pinMode(pin, OUTPUT);
}


void LED::toggleOn(int pin)
{
  digitalWrite(pin, HIGH);
}

void LED::toggleOff(int pin)
{
  digitalWrite(pin, LOW);
}

float LED::MillisecondsToSeconds(int milliseconds) 
{
  float second = float(milliseconds) / 1000;
  return second;
}

bool LED::IsTimePassedFromInterval(float startTimeInSeconds, float intervalInSeconds)
{
  float endTimeSeconds = startTimeInSeconds + intervalInSeconds;
  unsigned long currentMillis = millis();
  float currentSeconds = MillisecondsToSeconds(currentMillis);

  if(currentSeconds < endTimeSeconds)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void LED::Reverse(bool value, int pin)
{
  digitalWrite(pin, !value);
  state = !value;  
}

int LED::getState(bool state)
{
  if(state == 1)
  {
    return 1;
  }
  return 0;
}

void LED::turnOff(int pin)
{
  for(int i = 0; i < 180; i++)
  {
    digitalWrite(pin, HIGH);
  }
}
void LED::turnOn(int pin)
{
  for(int i = 180; i < 0; i--)
  {
    digitalWrite(pin, LOW);
  }
}


void LED::toggleSwitch(int array[], int arraySize, int frequency, int pin) {
  for (int i = 0; i < arraySize; i++) {
    if (array[i] == 1) 
    {
      toggleOff(pin);
    } 
    else if(array[i] == 0)
    {
      toggleOn(pin);
    }
    else {}
    delay(frequency); 
  }
}


//LedClass.h
#include "soc/soc_caps.h"
#ifndef LedClass_h
#define LedClass_h

#include "Arduino.h"

class LED
{
  public:
    bool state;
    LED();
    LED(int pin);
    void attach(int pin);
    void toggleOn(int pin);
    void toggleOff(int pin);
    bool IsTimePassedFromInterval(float startTimeInSeconds, float intervalInSeconds);
    float MillisecondsToSeconds(int milliseconds);
    void Reverse(bool value, int pin);
    int getState(bool state);
    void turnOff(int pin);
    void turnOn(int pin);
    void toggleSwitch(int array[], int arraySize, int frequency, int pin);
  private:
   int _pin; 
};
#endif 

//RelayClass.cpp
#include "esp32-hal.h"
#include "esp32-hal-gpio.h"
#include "RelayClass.h"
#include "Arduino.h"

Relay::Relay(){
  
}

Relay::Relay(int relayPin)
{
  attach(relayPin);
}

void Relay::attach(int relayPin)
{
  pinMode(relayPin, OUTPUT);
}


void Relay::relayToggleOn(int relayPin)
{
  digitalWrite(relayPin, HIGH);
}

void Relay::relayToggleOff(int relayPin)
{
  digitalWrite(relayPin, LOW);
}

float Relay::MillisecondsToSeconds(int milliseconds) 
{
  float second = float(milliseconds) / 1000;
  return second;
}

bool Relay::IsTimePassedFromInterval(float startTimeInSeconds, float intervalInSeconds)
{
  float endTimeSeconds = startTimeInSeconds + intervalInSeconds;
  unsigned long currentMillis = millis();
  float currentSeconds = MillisecondsToSeconds(currentMillis);

  if(currentSeconds < endTimeSeconds)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void Relay::relayReverse(bool relayValue, int relayPin)
{
  digitalWrite(relayPin, !relayValue);
  relayValue = !relayValue;  
}

int Relay::relayGetState(bool relayState)
{
  if(relayState == 1)
  {
    return 1;
  }
  return 0;
}

void Relay::relayToggleSwitch(int relayArray[], int relayArraySize, int relayFrequency, int relayPin) 
{
  for (int i = 0; i < relayArraySize; i++) {
    if (relayArray[i] == 1) 
    {
      relayToggleOff(relayPin);
    } 
    else if(relayArray[i] == 0)
    {
      relayToggleOn(relayPin);
    }
    else {}
    delay(relayFrequency); 
  }
}

void Relay::relayTurnOff(int relayPin)
{
  for(int i = 0; i < 180; i++)
  {
    digitalWrite(relayPin, HIGH);
  }
}
void Relay::relayTurnOn(int relayPin)
{
  for(int i = 180; i < 0; i--)
  {
    digitalWrite(relayPin, LOW);
  }
}

/RelayClass.h
#include "soc/soc_caps.h"
#ifndef RelayClass_h
#define RelayClass_h

#include "Arduino.h"

class Relay
{
  public:
    bool relayState;
    Relay();
    Relay(int relayPin);
    void attach(int relayPin);
    void relayToggleOn(int relayPin);
    void relayToggleOff(int relayPin);
    bool IsTimePassedFromInterval(float startTimeInSeconds, float intervalInSeconds);
    float MillisecondsToSeconds(int milliseconds);
    void relayReverse(bool relayValue, int relayPin);
    int relayGetState(bool relayState);
    void relayTurnOff(int relayPin);
    void relayTurnOn(int relayPin);
    void relayToggleSwitch(int relayArray[], int relayArraySize, int relayFrequency, int relayPin);
};

#endif 

//Detacher.ion

#include "LedClass.h"
#include "RelayClass.h"

#include "soc/soc_caps.h"
#ifndef Detacher_h
#define Detacher_h

#include "esp32-hal.h"
#include "esp32-hal-gpio.h"
#include "Detacher.h"
#include "Arduino.h"

class DeviceManager {
  public:
    DeviceManager() {}

    void detachAll(LED led, Relay relay, int ledPin, int relayPin) {
        detachLEDs(led, ledPin);
        detachRelays(relay, relayPin);
    }

  private:
    void detachLEDs(LED led, int ledPin) {
        led.attach(ledPin); 
        led.toggleOff(ledPin); 
    }

    void detachRelays(Relay relay, int relayPin) {
        relay.attach(relayPin); 
        relay.relayToggleOff(relayPin); 
    }
};
#endif
