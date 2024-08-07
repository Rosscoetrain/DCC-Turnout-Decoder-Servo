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

// This is a DCC Accessory Decoder to drive 8 Servo Turnouts
// Based on the NMRA Pulsed 8 stationary decoder


#include "defines.h"

#include <NmraDcc.h>

#include <Adafruit_PWMServoDriver.h>

#include "PinPulser.h"


/*
 * Rosscoe Train functions and variables
 */

#include "version.h"

#include "variables.h"

#include "functions.h"


/*
 * RT end
 */



void setup()
{
#ifdef ENABLE_SERIAL
  Serial.begin(115200);
  uint8_t maxWaitLoops = 255;
  while(!Serial && maxWaitLoops--)
    delay(20);
#endif

  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  // Many Arduino Cores now support the digitalPinToInterrupt() function that makes it easier to figure out the
  // Interrupt Number for the Arduino Pin number, which reduces confusion. 

  setVersion();

#ifdef digitalPinToInterrupt
  Dcc.pin(DCC_PIN, 0);
#else
  Dcc.pin(0, DCC_PIN, 1);
#endif
  
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, DCC_DECODER_VERSION_NUM, FLAGS_OUTPUT_ADDRESS_MODE | FLAGS_DCC_ACCESSORY_DECODER, 0 );

#ifdef ENABLE_SERIAL
  Serial.print("Rosscoe Train DCC 8 Turnout Servo Accessory Decoder. ");

  Serial.print(F("Version: "));
  Serial.print(versionBuffer[0]);
  Serial.print(F("."));
  Serial.print(versionBuffer[1]);
  Serial.print(F("."));
  Serial.println(versionBuffer[2]);
 
  Serial.println();
#endif


  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates


// set digital and analog pins defined in outputs to OUTPUT

  for (uint8_t i=0; i < NUM_OF_LEDS; i++)
   {
     pinMode(outputs[i], OUTPUT);
//     digitalWrite(outputs[i], LOW);
   }



#ifdef FORCE_RESET_FACTORY_DEFAULT_CV
  notifyCVResetFactoryDefault(); 
#ifdef ENABLE_SERIAL
  Serial.println("Resetting CVs to Factory Defaults");
#endif
#endif

  if( FactoryDefaultCVIndex == 0)	// Not forcing a reset CV Reset to Factory Defaults so initPinPulser
   {
	  initPinPulser();
   }


}


/*
 *
 * loop start
 *
*/

void loop()
{
#ifdef LEARNING
  static int learningbuttonOldval = 0,learningbuttonVal = 0;
#endif

  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

#ifndef FORCE_RESET_FACTORY_DEFAULT_CV  
  pinPulser.process();
#endif

  if( FactoryDefaultCVIndex && Dcc.isSetCVReady())
   {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array
    uint16_t cv = FactoryDefaultCVs[FactoryDefaultCVIndex].CV;
    uint8_t val = FactoryDefaultCVs[FactoryDefaultCVIndex].Value;
#ifdef ENABLE_SERIAL
#ifdef DEBUG_MSG
    Serial.print("loop: Write Default CV: "); Serial.print(cv,DEC); Serial.print(" Value: "); Serial.println(val,DEC);
#endif     
#endif
    Dcc.setCV( cv, val );
    
    if( FactoryDefaultCVIndex == 0)	// Is this the last Default CV to set? if so re-initPinPulser
	    initPinPulser();
   }


/*
 * added RT
 */

  ////////////////////////////////////////////////////////////////
  // check if the learning button has been enabled
  //
  // This is the jumper on A6 marked Learn on the PCB.
  // jumper must be in place to enter learning mode
  // When in learning mode send a close or throw command from the command station
  // and the address will be programmed into the decoder
  //
  ////////////////////////////////////////////////////////////////

#ifdef LEARNING

  learningbuttonVal = dr(LEARNINGBUTTON);

  if (learningbuttonOldval != learningbuttonVal)
   {
    learningMode = learningbuttonVal;
#ifndef ARDUINO_ARCH_ESP32
    if (learningMode == HIGH) showAcknowledge(3);
#endif

   }
  learningbuttonOldval = learningbuttonVal;
#endif


#ifdef ENABLE_SERIAL
    // see if there are serial commands
  readString="";              //empty for next input

  while (Serial.available())
   {
    char c = Serial.read();     //gets one byte from serial buffer
    readString += c;            //makes the string readString
    delay(10);                   //slow looping to allow buffer to fill with next character
   }

  // act on serial commands

  if (readString.length() >0)
   {
    doSerialCommand(readString);
   } 
#endif

  if (pinPulser.getUpdatePosition())
   {
    for (int i = 0; i < NUM_OF_SERVOS; i++)
     {
      Dcc.setCV(CV_USER_BASE_ADDRESS + 4 + (i * CV_PER_OUTPUT), pinPulser.getServoPosition(i) / 10);
#ifdef ENABLE_SERIAL
#ifdef DEBUG_MSG
      Serial.print("i: ");Serial.print(i);Serial.print(" pos: ");Serial.println(pinPulser.getServoPosition(i));
#endif
#endif
     }
    pinPulser.setUpdatePosition();
   }
}
