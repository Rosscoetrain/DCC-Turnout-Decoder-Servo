/*
 * defines.h
 */

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

#ifndef DEFINES_H
#define DEFINES_H


// this is the serial number for the board look on the back of the decoder board
//
#define SERIAL_NUMBER 999

// Un-Comment the line below to force CVs to be written to the Factory Default values
// defined in the FactoryDefaultCVs below on Start-Up
// THIS NEEDS to be un-commented and uploaded once to setup the eeprom
// after uploading comment out the line and upload again for normal operation
//#define FORCE_RESET_FACTORY_DEFAULT_CV

// To enable serial communications via USB uncomment the following.
#define ENABLE_SERIAL

// You can print every DCC packet by un-commenting the line below
//#define NOTIFY_DCC_MSG

// You can print every notifyDccAccTurnoutOutput call-back by un-commenting the line below
//#define NOTIFY_TURNOUT_MSG

// You can also print other Debug Messages uncommenting the line below this prints most debug messages
//#define DEBUG_MSG

// level of debug messages 0 - 3
//#define DEBUG 0

// Un-Comment the line below to include learning function
#define LEARNING

// Un-Comment the lines below to Enable DCC ACK for Service Mode Programming Read CV Capablilty 

#ifdef ARDUINO_ARCH_ESP32
//#define ENABLE_DCC_ACK  23  // This is IO23 on ESP32 WROOM   
#else
//#define ENABLE_DCC_ACK  15  // This is A1 on the Iowa Scaled Engineering ARD-DCCSHIELD DCC Shield
#endif


// Un-Comment the line below if this firemware is being used on the RT_Pulse_8_HP_SMT with Arduino Nano board.
//#define NANO_SMT_BOARD
// Un-Comment the line below if this firemware is being used on the RT_Pulse_8_HP_SMT with ATMega328p TQFP-32 board.
//#ifdef ATmega328P_TQFP32
//#define ATMEGA328P_SMT_BOARD
//#endif




// Define the Arduino input Pin number for the DCC Signal 
#ifdef ARDUINO_ARCH_ESP32
#define DCC_PIN     22
#else
#define DCC_PIN     2
#endif

#define NUM_TURNOUTS 16                // Set Number of Turnouts (Pairs of Pins)
//#define ACTIVE_OUTPUT_STATE HIGH      // Set the ACTIVE State of the output to Drive the Turnout motor electronics HIGH or LOW 

//#define DEFAULT_PULSE 10              // the default pulse ms/10

//#define DEFAULT_RECHARGE 30           // the default CDU recharge time ms/10

#define DCC_DECODER_VERSION_NUM 10    // Set the Decoder Version - Used by JMRI to Identify the decoder




#define CV_ACCESSORY_DECODER_OUTPUT_PULSE_TIME 2  // CV for the Output Pulse ON ms
#define CV_ACCESSORY_DECODER_SERVO_MOVE_TIME 3  // CV for the delay in ms to allow a CDU to recharge

#define CV_ACCESSORY_DECODER_SERIAL_LSB 255       // lsb for board serial number
#define CV_ACCESSORY_DECODER_SERIAL_MSB 256       // msb for board serial number


#define CV_PER_OUTPUT 5                           // how many CV's are used per decoder output
#define CV_USER_BASE_ADDRESS 33                   // base of user assigned CV's


/*
 * servo defines
*/
  
  
// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVO_MIN  15        // This is the 'minimum' pulse length count (out of 4096) * 10
#define SERVO_MAX  25        // This is the 'maximum' pulse length count (out of 4096) * 10
#define SERVO_FREQ 50        // Analog servos run at ~50 Hz updates
#define SERVO_CONFIG 0       // not in use yet.


// Config is as follows: (from DCC-EX IO_PCA9685.cpp)
//  Bit 7:     0=Set PWM to 0% to power off servo motor when finished
//             1=Keep PWM pulses on (better when using PWM to drive an LED)
//  Bits 6-0:  0           Use specified duration (defaults to 0 deciseconds)
//             1 (Fast)    Move servo in 0.5 seconds
//             2 (Medium)  Move servo in 1.0 seconds
//             3 (Slow)    Move servo in 2.0 seconds
//             4 (Bounce)  Servo 'bounces' at extremes.



#define SERVO_TIME 10        // the time it takes the servo to move in tenths of a second eg 10 = 1 second

#define NUM_OF_SERVOS 16     // total number of servo channels

#define NUM_OF_STEPS 20      // number of steps to move servo in fast medium slow modes

#define NUM_OF_LEDS 16       // number of leds that can be connected

#endif
