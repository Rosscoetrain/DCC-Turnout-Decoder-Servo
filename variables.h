/*
 * variables.h
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



#ifndef VARIABLES_H
#define VARIABLES_H

struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};


/*
 * to use individual CV's for each address output requires five CV's
 * 
 * 1 servo minimum position
 * 2 servo maximum position
 * 3 time for servo to move
 * 4 servo config see below
 * 5 last servo position - this is so that on startup the servo is moved to it's last position
 * 
 * 4 servo config
 * 
 * // Config is as follows: (from DCC-EX IO_PCA9685.cpp)
 * Bit 7:     0=Set PWM to 0% to power off servo motor when finished
 *             1=Keep PWM pulses on (better when using PWM to drive an LED)
 *  Bits 6-0:  0           Use specified duration (defaults to 0 deciseconds)
 *             1 (Fast)    Move servo in 0.5 seconds
 *             2 (Medium)  Move servo in 1.0 seconds
 *             3 (Slow)    Move servo in 2.0 seconds
 *             4 (Bounce)  Servo 'bounces' at extremes.  (not implemented)
 *
 */


/*
 *           address
 *            1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16
 * CV     
 *  close    33   38   43   48   53   58   63   68   73   78   83   88   93   98   103  108
 *  throw    34   39   44   49   54   59   64   69   74   79   84   89   94   99   104  109
 *  time     35   40   45   50   55   60   65   70   75   80   85   90   95   100  105  110
 *  config   36   41   46   51   56   61   66   71   76   81   86   91   96   101  106  111
 *  position 37   42   47   52   57   62   67   72   77   82   87   92   97   102  107  112
 */


/*
 * These are stored in an array of CV pairs.
 * Default array is as below
 * 
 */



CVPair FactoryDefaultCVs [] =
{
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, DEFAULT_ACCESSORY_DECODER_ADDRESS & 0xFF},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, DEFAULT_ACCESSORY_DECODER_ADDRESS >> 8},
  {CV_ACCESSORY_DECODER_SERVO_MOVE_TIME, SERVO_TIME},
  {CV_29_CONFIG, CV29_ACCESSORY_DECODER},


  {33, SERVO_MIN},
  {34, SERVO_MAX},
  {35, SERVO_TIME},
  {36, SERVO_CONFIG},
  {37, SERVO_MIN},

  {38, SERVO_MIN},
  {39, SERVO_MAX},
  {40, SERVO_TIME},
  {41, SERVO_CONFIG},
  {42, SERVO_MIN},
  
  {43, SERVO_MIN},
  {44, SERVO_MAX},
  {45, SERVO_TIME},
  {46, SERVO_CONFIG},
  {47, SERVO_MIN},
  
  {48, SERVO_MIN},
  {49, SERVO_MAX},
  {50, SERVO_TIME},
  {51, SERVO_CONFIG},
  {52, SERVO_MIN},
  
  {53, SERVO_MIN},
  {54, SERVO_MAX},
  {55, SERVO_TIME},
  {56, SERVO_CONFIG},
  {57, SERVO_MIN},
  
  {58, SERVO_MIN},
  {59, SERVO_MAX},
  {60, SERVO_TIME},
  {61, SERVO_CONFIG},
  {62, SERVO_MIN},
  
  {63, SERVO_MIN},
  {64, SERVO_MAX},
  {65, SERVO_TIME},
  {66, SERVO_CONFIG},
  {67, SERVO_MIN},
  
  {68, SERVO_MIN},
  {69, SERVO_MAX},
  {70, SERVO_TIME},
  {71, SERVO_CONFIG},
  {72, SERVO_MIN},
  
  {73, SERVO_MIN},
  {74, SERVO_MAX},
  {75, SERVO_TIME},
  {76, SERVO_CONFIG},
  {77, SERVO_MIN},
  
  {78, SERVO_MIN},
  {79, SERVO_MAX},
  {80, SERVO_TIME},
  {81, SERVO_CONFIG},
  {82, SERVO_MIN},
  
  {83, SERVO_MIN},
  {84, SERVO_MAX},
  {85, SERVO_TIME},
  {86, SERVO_CONFIG},
  {87, SERVO_MIN},
  
  {88, SERVO_MIN},
  {89, SERVO_MAX},
  {90, SERVO_TIME},
  {91, SERVO_CONFIG},
  {92, SERVO_MIN},
  
  {93, SERVO_MIN},
  {94, SERVO_MAX},
  {95, SERVO_TIME},
  {96, SERVO_CONFIG},
  {97, SERVO_MIN},
  
  {98, SERVO_MIN},
  {99, SERVO_MAX},
  {100, SERVO_TIME},
  {101, SERVO_CONFIG},
  {102, SERVO_MIN},
  
  {103, SERVO_MIN},
  {104, SERVO_MAX},
  {105, SERVO_TIME},
  {106, SERVO_CONFIG},
  {107, SERVO_MIN},
  
  {108, SERVO_MIN},
  {109, SERVO_MAX},
  {110, SERVO_TIME},
  {111, SERVO_CONFIG},
  {112, SERVO_MIN},


  {CV_ACCESSORY_DECODER_SERIAL_LSB, uint8_t(SERIAL_NUMBER - (SERIAL_NUMBER * 256))},
  {CV_ACCESSORY_DECODER_SERIAL_MSB, uint8_t(SERIAL_NUMBER / 256)},
  

};



uint8_t FactoryDefaultCVIndex = 0;

// This is the Arduino Pin Mapping to Turnout LEDs with 1 pin per turnout 
// A1 is missing in the sequence as it is used for the DCC ACK


#ifndef USE_SHIFT_REGISTER
byte outputs[] = { 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 1, 0, 13, 14, 16, 17};
//   pins         D12 D11 D10 D9 D8 D7 D6 D5 D4 D3 D1 D0 D13  A0  A2  A3
#endif

NmraDcc  Dcc ;
DCC_MSG  Packet ;
PinPulser pinPulser;

uint16_t BaseTurnoutAddress;


/*
 * Rosscoe Train functions and variables
 */
// for address learning mode
#ifdef LEARNING

#ifdef ARDUINO_ARCH_ESP32
int LEARNINGBUTTON = 2;    // pin 
#else
int LEARNINGBUTTON = A6;    // pin A6
#endif

int learningMode = LOW;
#endif

#ifndef ARDUINO_ARCH_ESP32
#define LEDCONTROL LED_BUILTIN
#endif

// buffer to hold serial commands
String readString;
bool ackOn = false;


static uint16_t servoMin[NUM_TURNOUTS] = {};
static uint16_t servoMax[NUM_TURNOUTS] = {};
static uint8_t  servoTime[NUM_TURNOUTS] = {};
static uint16_t  servoPosition[NUM_TURNOUTS] = {};

static uint8_t servoConfig[NUM_TURNOUTS] = {};

char* version;
uint8_t versionBuffer[3];

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#endif

  
