/*
 * functions.h
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



/*
 * a function to read an analogue pin and return a boolean value depending on reading.
 * works the same as doing a digital read on a digital pin
 * > 512 = TRUE, <= 512 = FALSE
 */
#ifndef FUNCTIONS_H
#define FUNCTIONS_H


bool dr (int pin)
 {
  int val = analogRead(pin);
  return ( val > 512 ) ;
 }


/*
 * setup the version number
 */


void setVersion() {
  const String versionString = VERSION;
  char versionArray[versionString.length() + 1];
  versionString.toCharArray(versionArray, versionString.length() + 1);
  version = strtok(versionArray, "."); // Split version on .
  versionBuffer[0] = atoi(version);  // Major first
  version = strtok(NULL, ".");
  versionBuffer[1] = atoi(version);  // Minor next
  version = strtok(NULL, ".");
  versionBuffer[2] = atoi(version);  // Patch last
}


/**
 * this is just a function to show via the onboard PCB led, the state of the decoder
 */

#ifndef ARDUINO_ARCH_ESP32
void showAcknowledge(int nb) {
  for (int i=0;i<nb;i++) {
    digitalWrite(LEDCONTROL, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);               // wait for a second
    digitalWrite(LEDCONTROL, LOW);    // turn the LED off by making the voltage LOW
    delay(100);               // wait for a second
  }
}
#endif


#include "StringSplitter.h"


void(* resetFunc) (void) = 0;                     // declare reset function at address 0


void doSerialCommand(String readString)
 {
//  byte p = 0;

  readString.trim();

  Serial.println(readString);                    // so you can see the captured string

  if (readString == "<Z>")
   {

    Serial.println(F("Resetting"));

    resetFunc();
   }

  if (readString == "<?>")
   {
    Serial.println(F("Help Text"));

    Serial.println(F("Close a turnout: <C address>"));
    Serial.println(F("Throw a turnout: <T address>"));

    Serial.println(F("Set decoder base address: <A address>"));

    Serial.println(F("Set decoder output closed position: <M output  mS / 10>"));
    Serial.println(F("Set decoder output thrown position: <N output mS / 10>"));
    Serial.println(F("Set decoder output move time: <O output S / 10>"));
    Serial.println(F("Set decoder output configuration: <P output [0:1:2:3]>"));
    
    Serial.println(F("Where output is 0 - 15 as on the decoder pcb"));

 
//    Serial.print(F("Change decoder address LSB: <W ")); Serial.print(CV_ACCESSORY_DECODER_ADDRESS_LSB); Serial.println(F(" address>"));
//    Serial.print(F("Change decoder address MSB: <W ")); Serial.print(CV_ACCESSORY_DECODER_ADDRESS_MSB); Serial.println(F(" address>"));

    Serial.println(F("Show current CVs: <>"));
                     
    Serial.println(F("Soft Reset: <Z>"));

   }
  else
   {
    if (readString.startsWith("<>"))
     {
      Serial.println(F("CVs are:"));

      Serial.print(F("CV"));
      Serial.print(CV_ACCESSORY_DECODER_ADDRESS_LSB);
      Serial.print(F(" = "));
      Serial.println(Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_LSB));
      Serial.print(F("CV"));
      Serial.print(CV_ACCESSORY_DECODER_ADDRESS_MSB);
      Serial.print(F(" = "));
      Serial.println(Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_MSB));

/*
      Serial.print(F("CV"));
      Serial.print(CV_ACCESSORY_DECODER_SERVO_MOVE_TIME);
      Serial.print(F(" = "));
      Serial.println(Dcc.getCV(CV_ACCESSORY_DECODER_SERVO_MOVE_TIME));
*/

      for(uint8_t i = 0; i < NUM_TURNOUTS; i++)
       {
        Serial.print(F("CV"));
        Serial.print(CV_USER_BASE_ADDRESS + (i * CV_PER_OUTPUT));
        Serial.print(F(" = "));
        Serial.print(Dcc.getCV(CV_USER_BASE_ADDRESS + (i * CV_PER_OUTPUT)));
        Serial.print("    ");

        Serial.print(F("CV"));
        Serial.print(CV_USER_BASE_ADDRESS + 1 + (i * CV_PER_OUTPUT));
        Serial.print(F(" = "));
        Serial.print(Dcc.getCV(CV_USER_BASE_ADDRESS + 1 + (i * CV_PER_OUTPUT)));
        Serial.print("    ");

        Serial.print(F("CV"));
        Serial.print(CV_USER_BASE_ADDRESS + 2 + (i * CV_PER_OUTPUT));
        Serial.print(F(" = "));
        Serial.print(Dcc.getCV(CV_USER_BASE_ADDRESS + 2 + (i * CV_PER_OUTPUT)));
        Serial.print("    ");

        Serial.print(F("CV"));
        Serial.print(CV_USER_BASE_ADDRESS + 3 + (i * CV_PER_OUTPUT));
        Serial.print(F(" = "));
        Serial.print(Dcc.getCV(CV_USER_BASE_ADDRESS + 3 + (i * CV_PER_OUTPUT)));
        Serial.print("    ");

        Serial.print(F("CV"));
        Serial.print(CV_USER_BASE_ADDRESS + 4 + (i * CV_PER_OUTPUT));
        Serial.print(F(" = "));
        Serial.print(Dcc.getCV(CV_USER_BASE_ADDRESS + 4 + (i * CV_PER_OUTPUT)));
        Serial.println("    ");

       }


     }
    else
     {
      if (readString.startsWith("<"))
       {
//        int pos = 0;
        // this is where commands are completed

        // command to close turnout <C address>

        if (readString.startsWith("<C"))
         {
          StringSplitter *splitter = new StringSplitter(readString, ' ', 3);  // new StringSplitter(string_to_split, delimiter, limit)
          int itemCount = splitter->getItemCount();


          if ( itemCount == 2)
           {
            int addr = splitter->getItemAtIndex(1).toInt();
            notifyDccAccTurnoutOutput( addr, 0, 1 );
           }
          else
           {
            Serial.println(F("Invalid command: should be <C address>"));
           }
          delete splitter;
          splitter = NULL;
         }


         // command to throw turnout <T address>

        if (readString.startsWith("<T"))
         {
          StringSplitter *splitter = new StringSplitter(readString, ' ', 3);  // new StringSplitter(string_to_split, delimiter, limit)
          int itemCount = splitter->getItemCount();

          if ( itemCount == 2)
           {
            int addr = splitter->getItemAtIndex(1).toInt();
            notifyDccAccTurnoutOutput( addr, 1, 1 );
           }
          else
           {
            Serial.println(F("Invalid command: should be <T address>"));
           }
          delete splitter;
          splitter = NULL;
         }

         // command to set address <A address>
         // address will be adjusted to the correct base turnout address
         // eg if address is 2 this will be corrected to 1 as the address are groups of 8 with an offset of 4
         // ie 1..8, 5..12, ...

        if (readString.startsWith("<A"))
         {
          StringSplitter *splitter = new StringSplitter(readString, ' ', 3);  // new StringSplitter(string_to_split, delimiter, limit)
          int itemCount = splitter->getItemCount();

          if ( itemCount == 2)
           {
            int addr = splitter->getItemAtIndex(1).toInt();

            byte L = (addr + 3) / 4;
            byte H = (addr + 3) / 1024;

#ifdef DEBUG_MSG
            Serial.print(F("Value = ")); Serial.println(addr);
            Serial.print(F(" H = ")); Serial.println(H);
            Serial.print(F(" L = ")); Serial.println(L);
#endif
                  
            Dcc.setCV(CV_ACCESSORY_DECODER_ADDRESS_MSB, H);
            Dcc.setCV(CV_ACCESSORY_DECODER_ADDRESS_LSB, L);
           }
          else
           {
            Serial.println(F("Invalid command: should be <A address>"));
           }
          delete splitter;
          splitter = NULL;
         }

/*
 * command to set output closed value.
 * The value here is the number of milliseconds / 10
 * ie 100ms/10 = 10.
 * 
 */

        if (readString.startsWith("<M"))
         {
          StringSplitter *splitter = new StringSplitter(readString, ' ', 3);  // new StringSplitter(string_to_split, delimiter, limit)
          int itemCount = splitter->getItemCount();


          if ( itemCount == 3)
           {
            int addr = splitter->getItemAtIndex(1).toInt();
            int value = splitter->getItemAtIndex(2).toInt();

#ifdef DEBUG_MSG
            Serial.print(F("Adress = ")); Serial.println(addr);
            Serial.print(F("Value = ")); Serial.println(value);
#endif
            if ( addr >= 0 && addr <= 15 )
             {
              Dcc.setCV(CV_USER_BASE_ADDRESS + (addr) * CV_PER_OUTPUT, value);
             }
            else
             {
              Serial.println(F("Invalid output: should be 0 to 15"));
             }
           }
          else
           {
            Serial.println(F("Invalid command: should be <M output ms/10>"));
           }
          delete splitter;
          splitter = NULL;
         }

/*
 * command to set output thrown value.
 * The value here is the number of milliseconds / 10
 * ie 100ms/10 = 10.
 * 
 */

        if (readString.startsWith("<N"))
         {
          StringSplitter *splitter = new StringSplitter(readString, ' ', 3);  // new StringSplitter(string_to_split, delimiter, limit)
          int itemCount = splitter->getItemCount();


          if ( itemCount == 3)
           {
            int addr = splitter->getItemAtIndex(1).toInt();
            int value = splitter->getItemAtIndex(2).toInt();

#ifdef DEBUG_MSG
            Serial.print(F("Adress = ")); Serial.println(addr);
            Serial.print(F("Value = ")); Serial.println(value);
#endif
            if ( addr >= 0 && addr <= 15 )
             {
              Dcc.setCV(CV_USER_BASE_ADDRESS + 1 + (addr) * CV_PER_OUTPUT, value);
             }
            else
             {
              Serial.println(F("Invalid output: should be 0 to 15"));
             }
           }
          else
           {
            Serial.println(F("Invalid command: should be <N output ms/10>"));
           }
          delete splitter;
          splitter = NULL;
         }

/*
 * command to set output servo move time
 * The value here is the number of seconds / 10
 * ie 100/10 = 10.
 * 
 */

        if (readString.startsWith("<O"))
         {
          StringSplitter *splitter = new StringSplitter(readString, ' ', 3);  // new StringSplitter(string_to_split, delimiter, limit)
          int itemCount = splitter->getItemCount();


          if ( itemCount == 3)
           {
            int addr = splitter->getItemAtIndex(1).toInt();
            int value = splitter->getItemAtIndex(2).toInt();

#ifdef DEBUG_MSG
            Serial.print(F("Adress = ")); Serial.println(addr);
            Serial.print(F("Value = ")); Serial.println(value);
#endif
            if ( addr >= 0 && addr <= 15 )
             {
              Dcc.setCV(CV_USER_BASE_ADDRESS + 2 + (addr) * CV_PER_OUTPUT, value);
             }
            else
             {
              Serial.println(F("Invalid output: should be 0 to 15"));
             }
           }
          else
           {
            Serial.println(F("Invalid command: should be <O output S/10>"));
           }
          delete splitter;
          splitter = NULL;
         }


/*
 * command to set output servo configuration
 * 
 * the value here is one of:
 * 0 = default move servo at maximum speed
 * 1 = fast move 0.5 second
 * 2 = medium move 1.0 second
 * 4 = slow move 2.0 second
 * 
 */

        if (readString.startsWith("<P"))
         {
          StringSplitter *splitter = new StringSplitter(readString, ' ', 3);  // new StringSplitter(string_to_split, delimiter, limit)
          int itemCount = splitter->getItemCount();


          if ( itemCount == 3)
           {
            int addr = splitter->getItemAtIndex(1).toInt();
            int value = splitter->getItemAtIndex(2).toInt();

#ifdef DEBUG_MSG
            Serial.print(F("Adress = ")); Serial.println(addr);
            Serial.print(F("Value = ")); Serial.println(value);
#endif
            if ( addr >= 0 && addr <= 15 )
             {

              Dcc.setCV(CV_USER_BASE_ADDRESS + 3 + (addr) * CV_PER_OUTPUT, value);

             }
            else
             {
              Serial.println(F("Invalid output: should be 0 to 15"));
             }
           }
          else
           {
            Serial.println(F("Invalid command: should be <P output [0:1:2:4]>"));
           }
          delete splitter;
          splitter = NULL;
         }





    if (readString.startsWith("<X"))
     {
      Serial.println(F("Serial number is:"));

      Serial.println(Dcc.getCV(CV_ACCESSORY_DECODER_SERIAL_LSB) + (Dcc.getCV(CV_ACCESSORY_DECODER_SERIAL_MSB) * 256 ));

      Serial.println("");

     }


/*              
        if (readString.startsWith("<W"))
         {
          StringSplitter *splitter = new StringSplitter(readString, ' ', 3);  // new StringSplitter(string_to_split, delimiter, limit)
          int itemCount = splitter->getItemCount();

          if ( itemCount == 3)
           {
            byte addr = splitter->getItemAtIndex(1).toInt();
            int value = splitter->getItemAtIndex(2).toInt();

            switch (addr) {
              case CV_ACCESSORY_DECODER_ADDRESS_LSB:                  // CV1

                    byte L = (value + 3) / 4;
                    byte H = (value + 3) / 1024;

#ifdef DEBUG_MSG
                  Serial.print(F("Value = ")); Serial.println(value);
                  Serial.print(F(" H = ")); Serial.println(H);
                  Serial.print(F(" L = ")); Serial.println(L);
#endif
                  
                  Dcc.setCV(CV_ACCESSORY_DECODER_ADDRESS_MSB, H);
                  Dcc.setCV(CV_ACCESSORY_DECODER_ADDRESS_LSB, L);
              break;
              case CV_ACCESSORY_DECODER_ADDRESS_MSB:                  // CV9
                  Dcc.setCV(CV_ACCESSORY_DECODER_ADDRESS_MSB, value);
              break;

              case 8:
                if (value == 8)
                 {
                 }
              break;
              case CV_ACCESSORY_DECODER_OUTPUT_PULSE_TIME:
                if ((value >= 0) && (value <= 255))
                 {
                  Dcc.setCV(CV_ACCESSORY_DECODER_OUTPUT_PULSE_TIME, value);
                 }
              break;
              case CV_ACCESSORY_DECODER_CDU_RECHARGE_TIME:
                if ((value >= 0) && (value <= 255))
                 {
                  Dcc.setCV(CV_ACCESSORY_DECODER_CDU_RECHARGE_TIME, value);
                 }
              break;
              case CV_ACCESSORY_DECODER_ACTIVE_STATE:
                if ((value == 0) || (value == 1))
                 {
                  Dcc.setCV(CV_ACCESSORY_DECODER_ACTIVE_STATE, value);
                 }
                else
                 {
                  Serial.println(F("Value must be 0 (LOW) or 1 (HIGH)"));
                 }
              break;
              default:
                 Serial.println(F("Invalid cv number: should be <W cv value> "));
              break;
             }
           }
          else
           {
            Serial.println(F("Invalid command: should be <W cv value>"));
           }
          delete splitter;
          splitter = NULL;
         }
*/

       }
      else
       {
        Serial.println(F("ERROR: Unknown command"));
       }
     }
   }
 }

#endif




void initPinPulser(void)
{
  BaseTurnoutAddress = (((Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_MSB) * 256) + Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_LSB) - 1) * 4) + 1  ;


// read the CV's for each address
  for(uint8_t i = 0; i < NUM_TURNOUTS; i++)
  {
    servoMin[i] = Dcc.getCV( CV_USER_BASE_ADDRESS + ( i * CV_PER_OUTPUT ) ) * 10;
    servoMax[i] = Dcc.getCV( CV_USER_BASE_ADDRESS + 1 + ( i * CV_PER_OUTPUT ) ) * 10;
    servoTime[i] = Dcc.getCV( CV_USER_BASE_ADDRESS + 2 + ( i * CV_PER_OUTPUT ) );
    servoConfig[i]  = Dcc.getCV( CV_USER_BASE_ADDRESS + 3 + ( i * CV_PER_OUTPUT ) );
    servoPosition[i]  = Dcc.getCV( CV_USER_BASE_ADDRESS + 4 + ( i * CV_PER_OUTPUT ) ) * 10;

#ifdef DEBUG_MSG
    Serial.print(F(" i : "));Serial.print(i);
    Serial.print(F(" servoMin : "));Serial.print(servoMin[i]);
    Serial.print(F(" servoMax : "));Serial.println(servoMax[i]);
#endif

  }

  Serial.print(F(" DCC Turnout Base Address: ")); Serial.println(BaseTurnoutAddress, DEC);

  // Step through all the Turnout LED pins setting them to OUTPUT and NOT Active State
//  for(uint8_t i = 0; i < NUM_TURNOUTS; i++)
//  {
//    digitalWrite(outputs[i], !activeOutputState[i / 2]); // Set the Output Inactive before the direction so the 
//    digitalWrite(outputs[i], 0);                  // Set the Output Inactive before the direction so the 
//  	pinMode( outputs[i], OUTPUT );                // Pin doesn't momentarily pulse the wrong state
//	}

  // Init the PinPulser with the new settings 
//  pinPulser.init(servoMin, servoMax, servoTime, outputs, pwm);
  pinPulser.init(servoMin, servoMax, servoTime, servoConfig, servoPosition, outputs, &pwm);

  pinPulser.printArrays();

}
 


/*
 *  DCC functions
*/

// This function is called whenever a normal DCC Turnout Packet is received
void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower )
{
#ifdef  NOTIFY_TURNOUT_MSG
  Serial.print("notifyDccAccTurnoutOutput: Turnout: ") ;
  Serial.print(Addr,DEC) ;
  Serial.print(" Direction: ");
  Serial.print(Direction ? "Thrown" : "Closed") ;
  Serial.print(" Output: ");
  Serial.println(OutputPower ? "On" : "Off") ;
#endif

// check to see if in learning mode and update address

#ifdef LEARNING
  if (learningMode == HIGH) {

//    int H = (Addr - 1) / 64;
//    int L = Addr - (H * 64);
    byte L = (Addr + 3) / 4;
    byte H = (Addr + 3) / 1024;

#ifdef DEBUG_MSG
    Serial.println("");
    Serial.print(F("Value = ")); Serial.println(Addr,DEC);
    Serial.print(F(" H = ")); Serial.println(H,DEC);
    Serial.print(F(" L = ")); Serial.println(L,DEC);
#endif
                  
    Dcc.setCV(CV_ACCESSORY_DECODER_ADDRESS_MSB, H);
    Dcc.setCV(CV_ACCESSORY_DECODER_ADDRESS_LSB, L);

   }
  else
#endif

   {
    if(( Addr >= BaseTurnoutAddress ) && ( Addr < (BaseTurnoutAddress + NUM_TURNOUTS )) && OutputPower )
     {
      uint16_t pinIndex = ( (Addr - BaseTurnoutAddress) << 1 ) + Direction ;

/*
 *   To get close or throw for addPin
 *   pinIndex * 2 + Direction
 *   to decode in PinPulser direction = pinIndex % 2, servo = int(pinIndex / 2)
 * 
*/

      pinPulser.addPin(pinIndex);
      
/*  TODO */
// write new position to CV
//      Dcc.setCV();

#ifdef  NOTIFY_TURNOUT_MSG
      Serial.print(" Pin Index: ");
      Serial.print(pinIndex,DEC);
      Serial.print(" Pin: ");
      Serial.println(outputs[pinIndex / 2],DEC);
#endif

     }
   }
#ifdef  NOTIFY_TURNOUT_MSG
  Serial.println();
#endif
}


void notifyCVChange(uint16_t CV, uint8_t Value)
{
#ifdef DEBUG_MSG
  Serial.print("notifyCVChange: CV: ") ;
  Serial.print(CV,DEC) ;
  Serial.print(" Value: ") ;
  Serial.println(Value, DEC) ;
#endif  

  Value = Value;  // Silence Compiler Warnings...

  if( (CV == CV_ACCESSORY_DECODER_ADDRESS_MSB) || (CV == CV_ACCESSORY_DECODER_ADDRESS_LSB) ||
		  (CV == CV_ACCESSORY_DECODER_SERVO_MOVE_TIME) ||
      ( ( CV - 33 ) % 5 == 0 ) ||
      ( ( CV - 34 ) % 5 == 0 ) ||
      ( ( CV - 35 ) % 5 == 0 ) ||
      ( ( CV - 36 ) % 5 == 0 ) )
   {
		initPinPulser();	                                 // Some CV we care about changed so re-init the PinPulser with the new CV settings
   }
}


void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);


  Serial.println("Resetting Factory Default");

};

// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read 
#ifdef  ENABLE_DCC_ACK
void notifyCVAck(void)
{
#ifdef DEBUG_MSG
  Serial.println("notifyCVAck") ;
#endif
  // Configure the DCC CV Programing ACK pin for an output
  pinMode( ENABLE_DCC_ACK, OUTPUT );

  // Generate the DCC ACK 60mA pulse
  digitalWrite( ENABLE_DCC_ACK, HIGH );
  delay( 10 );  // The DCC Spec says 6ms but 10 makes sure... ;)
  digitalWrite( ENABLE_DCC_ACK, LOW );
}
#endif

#ifdef  NOTIFY_DCC_MSG
void notifyDccMsg( DCC_MSG * Msg)
{
  Serial.print("notifyDccMsg: ") ;
  for(uint8_t i = 0; i < Msg->Size; i++)
  {
    Serial.print(Msg->Data[i], HEX);
    Serial.write(' ');
  }
  Serial.println();
}
#endif


  
