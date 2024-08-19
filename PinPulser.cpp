/*
 *  © 2023 Ross Scanlon
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 2.1 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this code.  If not, see <https://www.gnu.org/licenses/>.
*/

// Based on the NMRA Pulsed 8 stationary decoder

#include "PinPulser.h"

#ifdef USE_SHIFT_REGISTER
#include "SPI.h"
#endif

// define empty pin slot value

#define PIN_PULSER_SLOT_EMPTY 255

#ifdef USE_SHIFT_REGISTER
void PinPulser::init(uint16_t servoMin_[], uint16_t servoMax_[], uint8_t servoTime_[], uint8_t servoConfig_[],
                     uint16_t servoPosition_[], Adafruit_PWMServoDriver *pwm_)
#else
void PinPulser::init(uint16_t servoMin_[], uint16_t servoMax_[], uint8_t servoTime_[], uint8_t servoConfig_[],
                     uint16_t servoPosition_[], uint8_t outputs_[], Adafruit_PWMServoDriver *pwm_)
#endif

 {
  this->servoMin = servoMin_;
  this->servoMax = servoMax_;
  this->servoTime = servoTime_;

  this->servoConfig = servoConfig_;
  this->servoPosition = servoPosition_;

#ifdef USE_SHIFT_REGISTER
  this->ledOutput = 0;
#else
  this->outputs = outputs_;
#endif

  this->pwm = pwm_;

  for (uint8_t i = 0; i < NUM_OF_SERVOS; i++)
   {
     this->servoState[i] = 0;                      // 0 = not moving 1 = moving
   }

  state = PP_IDLE;
  targetMs = 0;
  memset(pinQueue, PIN_PULSER_SLOT_EMPTY, PIN_PULSER_MAX_PINS + 1);

  for (uint16_t i = 0; i < NUM_OF_SERVOS; i++)
   {

#if DEBUG == 2
    Serial.print(i);
    Serial.print(" : set ");
    Serial.println(this->servoPosition[i]);
#endif

    this->pwm->setPWM(i, 0, this->servoPosition[i]);
    this->pwm->setPWM(i, 4096, 0);

#if DEBUG == 2  || DEBUG == 4
    Serial.print(i + 1);
#endif

    if (this->servoPosition[i] == this->servoMin[i])
     {
#ifdef USE_SHIFT_REGISTER
    ledOutput &=  ~((uint16_t)1 << i);

#if DEBUG == 4
    Serial.print(" closed : ");
    Serial.println(ledOutput, BIN);
#endif

#else
      digitalWrite(outputs[currentServo], HIGH);
#endif
#if DEBUG == 2
      Serial.print(" : closed : ");
      Serial.println(this->servoMin[i]);
#endif
     }
    else
     {
#ifdef USE_SHIFT_REGISTER
    ledOutput |= ((uint16_t)1 << i);

#if DEBUG == 4
    Serial.print(" thrown : ");
    Serial.println(ledOutput, BIN);
#endif

#else
      digitalWrite(outputs[currentServo], LOW);
#endif
#if DEBUG == 2
      Serial.println(" : thrown");
#endif
     }
   }
#ifdef USE_SHIFT_REGISTER
  this->outputLeds(ledOutput);
#endif

#if DEBUG == 4
  Serial.print("ledOutput : ");
  Serial.println(ledOutput, BIN);
#endif

  updatePosition = 0;
 }

uint8_t PinPulser::addPin(uint8_t Pin)
 {
#ifdef DEBUG_MSG
  Serial.print(" PinPulser::addPin: "); Serial.print(Pin,DEC);
#endif
  for(uint8_t i = 0; i < PIN_PULSER_MAX_PINS; i++)
  {
    if(pinQueue[i] == Pin)
    {
#ifdef DEBUG_MSG
      Serial.print(F(" Already in Index: ")); Serial.println(i,DEC);
#endif
      return i;
    }

    else if(pinQueue[i] == PIN_PULSER_SLOT_EMPTY)
    {
#ifdef DEBUG_MSG
      Serial.print(F(" pinQueue Index: ")); Serial.println(i,DEC);
#endif
      pinQueue[i] = Pin;
      process();
      return i;
    }
  }  

#ifdef DEBUG_MSG
  Serial.println();
#endif
  return PIN_PULSER_SLOT_EMPTY;
 }

PP_State PinPulser::process(void)
 {
  unsigned long now;
  switch(state)
  {
  case PP_IDLE:
    if(pinQueue[0] != PIN_PULSER_SLOT_EMPTY)
    {

      updatePosition = 0;

#ifdef DEBUG_MSG
      Serial.print(F(" PinPulser::process: PP_IDLE: Pin: ")); Serial.println(pinQueue[0],DEC);
#endif

      currentServo = int(pinQueue[0] / 2);
      direction = pinQueue[0] % 2;
      currentConfig = servoConfig[currentServo];

#ifdef DEBUG_MSG
      Serial.print(F("Moving servo: ")); Serial.println(currentServo);
      Serial.print(F("Moving servo to: ")); Serial.println(direction);
      Serial.print(F("currentConfig: ")); Serial.println(currentConfig);
#endif

      targetMs = millis() + servoTime[currentServo] * 100;         // servoTime is in tenth of second so * 100 to get milliseconds

      servoState[currentServo] = 1;                                   // servo is moving

//      move the servo

      numberOfSteps = abs((servoMax[currentServo] - servoMin[currentServo])) / CURRENTSTEP;

#if DEBUG == 3
      Serial.print("numberOfSteps : ");
      Serial.println(numberOfSteps);
#endif

      switch (currentConfig)
      {
      case 0:
      default:
        if ( !direction )
         {
          pwm->setPWM(currentServo, 0, servoMin[currentServo]);

#if DEBUG == 3
          Serial.println("Close default");
#endif

         }
        else
         {
          pwm->setPWM(currentServo, 0, servoMax[currentServo]);

#if DEBUG == 3
          Serial.println("Throw default");
#endif

         }

        state = PP_OUTPUT_ON_DELAY;
        break;

      case 1:
          currentPause = 500 / numberOfSteps;
          state = PP_MOVING;
        break;

      case 2:
          currentPause = 1000 / numberOfSteps;
          state = PP_MOVING;
        break;

      case 3:
          currentPause = 2000 / numberOfSteps;
          state = PP_MOVING;
        break;

      case 4:
          currentPause = 5000 / numberOfSteps;
          state = PP_MOVING;
        break;

      case 5:
          currentPause = 10000 / numberOfSteps;
          state = PP_MOVING;

#ifdef DEBUG_MSG
        Serial.println(F("10 seconds"));
#endif

        break;

      }

      moveCount = 1;
      moveMs = millis() + currentPause;

#if DEBUG == 3
      Serial.print("currentPause : ");
      Serial.println(currentPause);
#endif
#if DEBUG == 2
      Serial.print(F(" millis : "));Serial.print(millis());Serial.print(F(" targetMs : "));Serial.println(targetMs);
#endif

    }
    break;

  case PP_MOVING:
      now = millis();
      if ( now >= moveMs )
       {

#if DEBUG == 3
        Serial.print(moveCount);
        Serial.println(" : MOVING");
#endif

        moveMs = millis() + currentPause;
        if (moveCount <= numberOfSteps)
         {
#if DEBUG == 3
          Serial.println("Moving 2");
#endif
          if ( !direction )
           {
            if (servoPosition[currentServo] != servoMin[currentServo])
             {
              pwm->setPWM(currentServo, 0, servoMax[currentServo] - (CURRENTSTEP * moveCount));

#if DEBUG == 3
              Serial.print(servoMax[currentServo] + (CURRENTSTEP * moveCount));
              Serial.println(" : moving");
#endif

             }
           }
          else
           {
            if (servoPosition[currentServo] != servoMax[currentServo])
             {
              pwm->setPWM(currentServo, 0, servoMin[currentServo] + (CURRENTSTEP * moveCount));

#if DEBUG == 3
              Serial.print(servoMin[currentServo] + (CURRENTSTEP * moveCount));
              Serial.println(" : moving");
#endif

             }
           }
          moveCount++;
         }
        else
         {
          state = PP_OUTPUT_ON_DELAY;
         }
       }
    break;

  case PP_OUTPUT_ON_DELAY:
    now = millis();
    if(now >= targetMs)
    {

      if ( !direction )
       {
        servoPosition[currentServo] = servoMin[currentServo];
#if DEBUG == 3
        Serial.print(currentServo);
        Serial.println(" :  Closed");
#endif
       }
      else
       {
        servoPosition[currentServo] = servoMax[currentServo];
#if DEBUG == 3
        Serial.print(currentServo);
        Serial.println(" :  Thrown");
#endif
       }

#ifdef DEBUG_MSG
      Serial.print(F(" now : "));Serial.print(now);Serial.print(F(" targetMs : "));Serial.println(targetMs);
      Serial.print(F(" PinPulser::process: PP_OUTPUT_ON_DELAY: Done Deactivate Pin: ")); Serial.println(pinQueue[0],DEC);
#endif

#ifdef DEBUG_MSG
          Serial.print(F("currenctServo: ")); Serial.println(currentServo);
          Serial.println(F("Stopped servo"));
#endif
// set servo pwm to 0 confirm this is definitely set to 0
      pwm->setPWM(currentServo, 0, 4096);

#ifdef USE_SHIFT_REGISTER
      if (!direction)
       {
        ledOutput &= ~((uint16_t)1 << currentServo);
       }
      else
       {
        ledOutput |= ((uint16_t)1 << currentServo);
       }
      this->outputLeds(ledOutput);
#else
      digitalWrite(outputs[currentServo], direction);           //    set the turnout LED to the direction 0 = closed 1 = thrown
#endif
      memmove(pinQueue, pinQueue + 1, PIN_PULSER_MAX_PINS);
      state = PP_IDLE;
      updatePosition = 1;

    }
    break;

  }


  return state;
 }

// set servos to start position

void PinPulser::setServoStart()
 {
  for (uint8_t i=0; i < NUM_OF_SERVOS; i++)
   {

    Serial.print(i);
    Serial.println(" : set ");

    pwm->setPWM(i, 0, servoMax[currentServo]);

   }

 
 }

// print arrays to serial

void PinPulser::printArrays()
 {
  for(uint8_t i = 0; i < NUM_TURNOUTS; i++)
   {
    Serial.print(F(" output : "));Serial.print(i+1);Serial.print(F(" servoMin : "));Serial.print(servoMin[i]);
    Serial.print(F(" servoMax : "));Serial.print(servoMax[i]);
    Serial.print(F(" servoTime : "));Serial.print(servoTime[i]);
    Serial.print(F(" servoConfig : "));Serial.print(servoConfig[i]);
    Serial.print(F(" servoPosition : "));Serial.println(servoPosition[i]);
   }
 }

uint16_t PinPulser::getServoMin(uint8_t pin)
 {
    return servoMin[pin];
 }

uint16_t PinPulser::getServoMax(uint8_t pin)
 {
    return servoMax[pin];
 }

uint8_t PinPulser::getServoTime(uint8_t pin)
 {
    return servoTime[pin];
 }
  
uint16_t PinPulser::getServoPosition(uint8_t pin)
 {
    return servoPosition[pin];
 }

bool PinPulser::getUpdatePosition()
 {
    return updatePosition;
 }

void PinPulser::setUpdatePosition()
 {
    updatePosition = 0;
 }

// using 74HC595 shift registers

#ifdef USE_SHIFT_REGISTER
void PinPulser::outputLeds(uint16_t leds)
 {

  SPI.begin();

//  byte loByte, hiByte;
  byte hiByte = highByte(leds);
  byte loByte = lowByte(leds);

#if DEBUG == 4

  Serial.print("ledOutput : ");
  Serial.println(leds, BIN);
  Serial.print("loByte : ");
  Serial.print(loByte, BIN);
  Serial.print(" hiByte : ");
  Serial.println(hiByte, BIN);

#endif

  digitalWrite(LATCH_PIN, LOW);

#if DEBUG == 3
  Serial.print("latch : ");
  Serial.println(LATCH_PIN);
  Serial.print("data : ");
  Serial.println(DATA_PIN);
  Serial.print("clock : ");
  Serial.println(CLOCK_PIN);
  delay(1000);
#endif

  SPI.transfer(loByte);
  SPI.transfer(hiByte);

  digitalWrite(LATCH_PIN, HIGH);

#if DEBUG == 4
  Serial.println("Sent");
#endif

  SPI.end();

 }
#endif

