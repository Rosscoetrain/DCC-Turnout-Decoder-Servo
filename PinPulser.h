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

#ifndef PINPUSHER_H
#define PINPUSHER_H

#include <Arduino.h>
#include "defines.h"

#include <Adafruit_PWMServoDriver.h>

// this is the maximum number of output pins

#define PIN_PULSER_MAX_PINS    16

enum PP_State
{  
  PP_IDLE = 0,
  PP_OUTPUT_ON_DELAY,
  PP_MOVING,
//  PP_CDU_RECHARGE_DELAY,
};

class PinPulser
{
  private:
//    uint16_t      cduRechargeMs;
    PP_State      state = PP_IDLE;
    unsigned long targetMs = 0;
    unsigned long moveMs = 0;
    uint16_t moveCount = 0;

    uint8_t       pinQueue[PIN_PULSER_MAX_PINS + 1];
    uint8_t       servoState[NUM_OF_SERVOS];

    uint16_t *servoMin;
    uint16_t *servoMax;
    uint8_t  *servoTime;

    uint8_t  *servoConfig;
    uint16_t *servoPosition;

    Adafruit_PWMServoDriver *pwm;

    uint8_t currentServo;
    uint8_t direction;

    uint16_t currentPause;                              // how long to wait between servo steps

    uint16_t numberOfSteps;                             // absolute difference between servoMin[n] and servoMax[n]

    uint8_t currentConfig;                              // current servo configuration

#ifdef USE_SHIFT_REGISTER
    uint16_t ledOutput;
    void outputLeds(uint16_t);
#else
    byte *outputs;
#endif

    bool updatePosition;

  public:
//    void init(uint16_t servoMin_[], uint16_t servoMax_[], uint8_t servoTime_[], uint8_t outputs_[], Adafruit_PWMServoDriver pwm_);
#ifdef USE_SHIFT_REGISTER
    void init(uint16_t servoMin_[], uint16_t servoMax_[], uint8_t servoTime_[], uint8_t servoConfig_[],
              uint16_t servoPosition_[], Adafruit_PWMServoDriver *pwm_);
#else
    void init(uint16_t servoMin_[], uint16_t servoMax_[], uint8_t servoTime_[], uint8_t servoConfig_[],
              uint16_t servoPosition_[], uint8_t outputs_[], Adafruit_PWMServoDriver *pwm_);
#endif

    uint8_t addPin(uint8_t pin);
    PP_State process(void);

    void printArrays();

    void setServoStart();

    uint16_t getServoMin(uint8_t pin);
    uint16_t getServoMax(uint8_t pin);
    uint8_t getServoTime(uint8_t pin);
    uint16_t getServoPosition(uint8_t pin);

    bool getUpdatePosition();
    void setUpdatePosition();

};


#endif

  
