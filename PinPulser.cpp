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

// define empty pin slot value

#define PIN_PULSER_SLOT_EMPTY 255

void PinPulser::init(uint16_t servoMin_[], uint16_t servoMax_[], uint8_t servoTime_[], uint8_t servoConfig_[],
                     uint16_t servoPosition_[], uint8_t outputs_[], Adafruit_PWMServoDriver *pwm_)


{
  this->servoMin = servoMin_;
  this->servoMax = servoMax_;
  this->servoTime = servoTime_;

  this->servoConfig = servoConfig_;
  this->servoPosition = servoPosition_;

  this->outputs = outputs_;

  this->pwm = pwm_;

  for (uint8_t i = 0; i < NUM_OF_SERVOS; i++)
   {
//     this->servoPosition[i] = this->servoMin[i];
     this->servoState[i] = 0;                      // 0 = not moving 1 = moving
   }

  state = PP_IDLE;
  targetMs = 0;
  memset(pinQueue, PIN_PULSER_SLOT_EMPTY, PIN_PULSER_MAX_PINS + 1);


  for (int i = 0; i < NUM_OF_SERVOS; i++)
   {
    this->pwm->setPWM(i, 0, this->servoPosition[i]);
   }


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

//      currentConfig = uint8_t(servoConfig[currentServo] && B01111111);
      currentConfig = servoConfig[currentServo];

#ifdef DEBUG_MSG
      Serial.print(F("Moving servo: ")); Serial.println(currentServo);
      Serial.print(F("Moving servo to: ")); Serial.println(direction);
      Serial.print(F("currentConfig: ")); Serial.println(currentConfig);
#endif

      targetMs = millis() + servoTime[currentServo] * 100;         // servoTime is in tenth of second so * 100 to get milliseconds

      servoState[currentServo] = 1;                                   // servo is moving

//      move the servo


      switch (currentConfig)
      {
      case 0:
      default:
        if ( !direction )
         {
          pwm->setPWM(currentServo, 0, servoMin[currentServo]);

          Serial.println("Close");

         }
        else
         {
          pwm->setPWM(currentServo, 0, servoMax[currentServo]);
         }
        break;

      case 1:
        currentPause = 500 / NUM_OF_STEPS;                             // 0.5 second / NUM_OF_STEPS in ms
        break;

      case 2:
        currentPause = 1000 / NUM_OF_STEPS;                             // 1 second / NUM_OF_STEPS in ms
        break;

      case 3:
        currentPause = 2000 / NUM_OF_STEPS;                             // 2 second / NUM_OF_STEPS in ms
        break;

      case 5:
        currentPause = 10000 / NUM_OF_STEPS;                             // 10 seconds / NUM_OF_STEPS in ms

#ifdef DEBUG_MSG
        Serial.println(F("10 seconds"));
#endif

        break;

      }

      currentStep = abs((servoMax[currentServo] - servoMin[currentServo])) / NUM_OF_STEPS;
      moveCount = 1;
      moveMs = millis() + currentPause;

      digitalWrite(outputs[currentServo], direction);           //    set the turnout LED to the direction 0 = closed 1 = thrown

#ifdef DEBUG_MSG
      Serial.print(F(" millis : "));Serial.print(millis());Serial.print(F(" targetMs : "));Serial.println(targetMs);
#endif


      state = PP_MOVING;
    }
    break;

  case PP_MOVING:
    if (currentConfig != 0)
     {
      now = millis();
      if ( now >= moveMs )
       {
        moveMs = millis() + currentPause;
        if (moveCount <= NUM_OF_STEPS)
         {
          if (( direction ) && (servoMin[currentServo] < servoMax[currentServo]))
           {
            if (servoPosition[currentServo] != servoMin[currentServo])
             {
              pwm->setPWM(currentServo, 0, servoMin[currentServo] + (currentStep * moveCount));
             }
           }
          else
           {
            if (servoPosition[currentServo] != servoMax[currentServo])
             {
              pwm->setPWM(currentServo, 0, servoMax[currentServo] - (currentStep * moveCount));
             }
           }
          moveCount++;
         }
        else
         {
          state = PP_OUTPUT_ON_DELAY;
         }
       }
     }
    else
     {
      state = PP_OUTPUT_ON_DELAY;
     }
    break;

  case PP_OUTPUT_ON_DELAY:
    now = millis();
    if(now >= targetMs)
    {

      if ( !direction )
       {
        servoPosition[currentServo] = servoMin[currentServo];

        Serial.println("Closed");
       }
      else
       {
        servoPosition[currentServo] = servoMax[currentServo];
       }

#ifdef DEBUG_MSG
      Serial.print(F(" now : "));Serial.print(now);Serial.print(F(" targetMs : "));Serial.println(targetMs);
      Serial.print(F(" PinPulser::process: PP_OUTPUT_ON_DELAY: Done Deactivate Pin: ")); Serial.println(pinQueue[0],DEC);
#endif


// set servo pwm to 0 confirm this is definitely set to 0
#ifdef DEBUG_MSG
          Serial.print(F("currenctServo: ")); Serial.println(currentServo);
          Serial.println(F("Stopped servo"));
#endif
      pwm->setPWM(currentServo, 0, 4096);


      memmove(pinQueue, pinQueue + 1, PIN_PULSER_MAX_PINS);
      state = PP_IDLE;
      updatePosition = 1;
    }
    break;

  }


  return state;
}


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
