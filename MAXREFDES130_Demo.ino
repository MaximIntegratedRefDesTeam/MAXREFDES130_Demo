/**********************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
**********************************************************************/

#include <OneWire.h>
#include <MAX11300.h>
#include <MAX4822.h>
#include <RTClib.h>


using namespace OneWire;
using namespace RomCommands;

MAX4822 rly_drvr;
MAX11300 pixi;
DS2484 owm;
RTC_DS3231 rtc;

const uint8_t D_SCLK = 13;
const uint8_t D_MISO = 12;
const uint8_t D_MOSI = 11;
const uint8_t RLY_DRVR_CS = 10;
const uint8_t PIXI_CNVT = 9;
const uint8_t PIXI_CS = 8;
const uint8_t RLY_DRVR_SET = 7;
const uint8_t RLY_DRVR_RESET = 6;

const char monday[] = "Monday";
const char tuesday[] = "Tuesday";
const char wednesday[] = "Wednesday";
const char thursday[] = "Thursday";
const char friday[] = "Friday";
const char saturday[] = "Saturday";
const char sunday[] = "Sunday";
const char * daysOfTheWeek[] = {
monday, tuesday, wednesday, thursday, friday, saturday, sunday};

const char jan[] = "January";
const char feb[] = "February";
const char mar[] = "March";
const char apr[] = "April";
const char may[] = "May";
const char jun[] = "June";
const char jul[] = "July";
const char aug[] = "August";
const char sep[] = "September";
const char oct[] = "October";
const char nov[] = "November";
const char dec[] = "December";
const char * monthsOfTheYear[] = {
jan, feb, mar, apr, may, jun, jul, aug, sep, oct, nov, dec};


void setup() 
{
  Serial.begin(57600);
  while(!Serial);
  Serial.setTimeout(0x7FFFFFFF);

  Serial.println(F("Starting MAXREFDES130# Demo"));

  rtc.begin();

  rly_drvr.begin(D_MOSI, D_SCLK, RLY_DRVR_CS);
  pinMode(RLY_DRVR_SET, OUTPUT);
  digitalWrite(RLY_DRVR_SET, HIGH);
  pinMode(RLY_DRVR_RESET, OUTPUT);
  digitalWrite(RLY_DRVR_RESET, HIGH);
  
  pixi.begin(D_MOSI, D_MISO, D_SCLK, PIXI_CS, PIXI_CNVT);

  //set all analog outs to 0
  for(uint8_t idx = 0; idx < 9; idx++)
  {
      pixi.single_ended_dac_write(static_cast<MAX11300::MAX11300_Ports>(idx), 0);
  }
  
  //ensure latching relays are reset 
  for(uint8_t idx = 0; idx < 3; idx++)
  {
      pixi.gpio_write(static_cast<MAX11300::MAX11300_Ports>(idx + 9), 1);
      delay(100);
      pixi.gpio_write(static_cast<MAX11300::MAX11300_Ports>(idx + 9), 0); 
  }

  OneWireMaster::CmdResult result = owm.begin();
  if(result != OneWireMaster::Success)
  {
    Serial.println(F("Failed to initialize OneWire Master"));
    result = owm.begin();
    delay(1000);
  }
}

void loop()
{
  uint8_t user_entry = 0;
  uint8_t relay = 0;
  uint16_t analog_out_ch, analog_val;
  float avo_volts, aio_mA, aii_mA;

  MAX11300::CmdResult pixi_result;
  MAX4822::CmdResult rly_drvr_result;
  
  float aio_slope = 0;
  float aio_offset = 0;
  float aii_slope = 0;
  float aii_offset = 0;
  uint16_t calData[4];
  
  

  bool currentLoopCal = false;
  if(check_420_cal(owm, calData))
  {
    aio_slope = (((calData[1]*1.0) - (calData[0]*1.0))/16.0);
    aio_offset = ((-1.0*aio_slope*4.0) + calData[0]);
    
    aii_slope = (16.0/((calData[3]*1.0) - (calData[2]*1.0)));
    aii_offset = ((-1.0*aii_slope*calData[2]) + 4.0);
    
    currentLoopCal = true;
  }
  
  while(user_entry != 15)
  {
    Serial.println(F("1.  Set RTC"));
    Serial.println(F("2.  Get Time/Date"));
    Serial.println(F("3.  Search OneWire bus"));
    Serial.println(F("4.  Set non-latching relay"));
    Serial.println(F("5.  Reset non-latching relay"));
    Serial.println(F("6.  Set all non-latching relays"));
    Serial.println(F("7.  Reset all non-latching relays"));
    Serial.println(F("8.  Set latching relay"));
    Serial.println(F("9.  Reset latching relay"));
    Serial.println(F("10. Set 0-10 analog out"));
    Serial.println(F("11. Set 4-20 out"));
    Serial.println(F("12. Get 4-20 in"));
    Serial.println(F("13. Calibrate 4-20 io"));
    Serial.println(F("14. Clear Screen"));
    Serial.println(F("15. Quit"));
    Serial.println();

    do
    {
      user_entry = get_user_int(NULL, 15);
      if(user_entry == 0)
      {
        Serial.println(F("Please select a valid option"));
      }
    }
    while(user_entry == 0);

    //get non-latching relay
    if((user_entry == 4) || (user_entry == 5))
    {
        do
        {
          relay = get_user_int("Select a relay from 1 to 8: ", 8);
          if(relay == 0)
          {
            Serial.println(F("Please select a valid option"));
          }
        }
        while(relay == 0);
    }
    
    //get latching relay
    if((user_entry == 8) || (user_entry == 9))
    {
        do
        {
          relay = (8 + get_user_int("Select a relay from 1 to 3: ", 3));
          if(relay == 0)
          {
            Serial.println(F("Please select a valid option"));
          }
        }
        while(relay == 0);
    }

    switch(user_entry)
    {
        case 1:
            set_rtc(rtc);
        break;
        
        case 2:
            get_time_date(rtc);
        break;
        
        case 3:
            search_ow_bus();
        break;
        
        case 4:
            rly_drvr_result = rly_drvr.set_relay(static_cast<MAX4822::RelayChannel>(relay));
            if(rly_drvr_result != MAX4822::Success)
            {
                Serial.println(F("Failed to set relay"));
            }
        break;
        
        case 5:
            rly_drvr_result = rly_drvr.reset_relay(static_cast<MAX4822::RelayChannel>(relay));
            if(rly_drvr_result != MAX4822::Success)
            {
                Serial.println(F("Failed to reset relay"));
            }
        break;
        
        case 6:
            rly_drvr.set_all_relays(RLY_DRVR_SET);
            Serial.println();
        break;
        
        case 7:
            rly_drvr.reset_all_relays(RLY_DRVR_RESET);
            Serial.println();
        break;
        
        case 8:
            pixi_result = pixi.gpio_write(static_cast<MAX11300::MAX11300_Ports>(relay), 1);
            if(pixi_result != MAX11300::Success)
            {
                Serial.println(F("Failed to set relay"));
            }
        break;
        
        case 9:
            pixi_result = pixi.gpio_write(static_cast<MAX11300::MAX11300_Ports>(relay), 0);
            if(pixi_result != MAX11300::Success)
            {
                Serial.println(F("Failed to reset relay"));
            }
        break;
        
        case 10:
            do
            {
              analog_out_ch = get_user_int("Please select a 0-10V output; 1-8: ", 8);
              if(analog_out_ch == 0)
              {
                Serial.println(F("Please select a valid option"));                
              }
            }
            while(analog_out_ch == 0);

            analog_out_ch--;
            
            avo_volts = get_user_float("Please enter a voltage from 0.0 to 10.0 volts: ", 10.01);
            analog_val = static_cast<uint16_t>((avo_volts/10.0) * 4095);
            
            Serial.print(F("DAC Code = "));
            Serial.println(analog_val, DEC);
            
            pixi_result = pixi.single_ended_dac_write(static_cast<MAX11300::MAX11300_Ports>(analog_out_ch), analog_val);
            if(pixi_result != MAX11300::Success)
            {
                Serial.println(F("Failed to set 0-10 out"));
            }
        break;
        
        case 11:
            if(currentLoopCal)
            {
              aio_mA = get_user_float("Please enter a current from 4.0 to 20.0 mA: ", 20.01);
              analog_val = static_cast<uint16_t>((aio_slope * aio_mA) + aio_offset);
              
              pixi_result = pixi.single_ended_dac_write(MAX11300::PIXI_PORT8, analog_val);
              if(pixi_result != MAX11300::Success)
              {
                  Serial.println(F("Failed to set 4-20 out"));
              }
            }
            else
            {
              Serial.println(F("Please calibrate 4-20mA loop first, option 13."));
            }
        break;
        
        case 12:
            if(currentLoopCal)
            {
              pixi_result = pixi.single_ended_adc_read(MAX11300::PIXI_PORT12, analog_val);
              aii_mA = ((aii_slope*analog_val) + aii_offset);
              if(pixi_result == MAX11300::Success)
              {
                  Serial.print(F("4-20 in = "));
                  Serial.println(aii_mA, 2);
              }
              else
              {
                  Serial.println(F("Failed to read 4-20 in"));
              }
            }
            else
            {
              Serial.println(F("Please calibrate 4-20mA loop first, option 13."));
            }
        break;
        
        case 13:
            if(calibrate_420_io(pixi, owm, calData, currentLoopCal))
            {
                aio_slope = (((calData[1]*1.0) - (calData[0]*1.0))/16.0);
                aio_offset = ((-1.0*aio_slope*4.0) + calData[0]);
                
                aii_slope = (16.0/((calData[3]*1.0) - (calData[2]*1.0)));
                aii_offset = ((-1.0*aii_slope*calData[2]) + 4.0);
            
                currentLoopCal = true;
                
                Serial.println(F("Calibration data saved to 1-Wire EEPROM"));
            }
        break;

        case 14:
          clear_screen();
        break;
        
        case 15:
            Serial.println(F("Ending Program"));
        break;
        
        default:
            arduino_die();
        break;
    }
    
  }

  while(1);
}

//*********************************************************************
uint16_t get_user_int(char *msg, uint16_t max_val)
{
  uint16_t user_int;
  String inString = "";
  int inChar = 0;

  do
  {
    Serial.print(msg);

    while(inChar != '\n') 
    {
      while(Serial.available() == 0);
      inChar = Serial.read();
      if (inChar != '\n') 
      { 
        inString += (char)inChar;
      }
    }
    user_int = inString.toInt();
    inChar = 0;
    inString = "";
  }
  while(user_int > max_val);
  
  return user_int;
}

//*********************************************************************
char get_user_char(char * msg)
{
  char user_char;
  uint8_t bytes_available;
  uint8_t input_buffer[3];
  
  Serial.println(msg);
  do
  {
    bytes_available = Serial.readBytesUntil(0x0A, input_buffer, 3);
  }
  while(!bytes_available);

  user_char = input_buffer[0];
  Serial.print(F("You entered "));
  Serial.println(user_char);
  
  return user_char;
}

//*********************************************************************
float get_user_float(char * msg, float max_val)
{
  float rtn_val = 0.0;
  String inString = "";
  int inChar = 0;

  do
  {
    Serial.print(msg);

    while(inChar != '\n') 
    {
      while(Serial.available() == 0);
      inChar = Serial.read();
      if (inChar != '\n') 
      { 
        inString += (char)inChar;
      }
    }
    rtn_val = inString.toFloat();
    inChar = 0;
    inString = "";
  }
  while(rtn_val > max_val);
  
  return rtn_val;
}

//*********************************************************************
void set_rtc(RTC_DS3231 & rtc)
{
  uint16_t y;
  uint8_t mon, d, h, m, s;
  
  y = get_user_int("Please enter the year (MCDY): ", 0xFFFF);
  mon = get_user_int("Please enter the month: ", 12);
  d = get_user_int("Please enter the day: ", 31);
  h = get_user_int("Please enter the hour: ", 23);
  m = get_user_int("Please enter the minute: ", 59);
  s = get_user_int("Please enter the second: ", 59);
  
  DateTime dateTime(y, mon, d, h, m, s);

  rtc.adjust(dateTime);
}

//*********************************************************************
void get_time_date(RTC_DS3231 & rtc)
{
  DateTime dateTime = rtc.now();

  Serial.print(F("Today is "));
  Serial.print(daysOfTheWeek[(dateTime.dayOfTheWeek() - 1)]);
  Serial.print(" ");
  Serial.print(monthsOfTheYear[(dateTime.month() - 1)]);
  Serial.print(" ");
  Serial.print(dateTime.day());
  Serial.print(", ");
  Serial.println(dateTime.year());

  Serial.print(F("The time is "));
  Serial.print(dateTime.hour());
  Serial.print(":");
  Serial.print(dateTime.minute());
  Serial.print(":");
  Serial.println(dateTime.second());
}

//*********************************************************************
void search_ow_bus(void)
{
  SearchState searchState;
  OneWireMaster::CmdResult result = owm.OWReset();
  if(result == OneWireMaster::Success)
  {
    Serial.println("1-wire devices detected");

    result = OWFirst(owm, searchState);
    if(result == OneWireMaster::Success)
    {
      do
      {
        print_rom_id(searchState.romId);
        if(searchState.romId.familyCode() == 0x10)
        {
            MultidropRomIterator selector(owm);
            DS1920 tempIbutton(selector);
            tempIbutton.setRomId(searchState.romId);

            DS1920::CmdResult tempResult = DS1920::OpFailure;
            float ds1920Temp;

            tempResult = tempIbutton.convertTemperature(ds1920Temp);
            if(tempResult == DS1920::Success)
            {
                Serial.println("Convert Temperature Success");
                Serial.print("Temperature = ");
                Serial.println(ds1920Temp, 2);
            }
            else
            {
                Serial.print("Convert Temperature Failed with code: ");
                Serial.println(tempResult, DEC);
            }
        }
        result = OWNext(owm, searchState);
      }
      while(result == OneWireMaster::Success);
    }
    else
    {
      Serial.print("OWFirst failed with error code: ");
      Serial.println(result, DEC);
    }
  }
  else
  {
    Serial.println("Failed to detect any 1-wire devices");
  }
}

//*********************************************************************
void print_rom_id(RomId & romId)
{
    //print the rom number
    Serial.println();
    for(uint8_t idx = 0; idx < RomId::byteLen; idx++)
    {
      if(romId[idx] < 16)
      {
        Serial.print("0x0");
        Serial.print(romId[idx], HEX);
        Serial.print(" ");
      }
      else
      {
        Serial.print("0x");
        Serial.print(romId[idx], HEX);
        Serial.print(" ");
      }
    }
    Serial.println();
}

//*********************************************************************
bool calibrate_420_io(MAX11300 & pixi, DS2484 & owm, uint16_t *calData, bool loopAlreadyCal)
{
    char user_entry;
    //initial vals for aio determined imperically on one pcb
    static uint16_t aio_4mA = 518;
    static uint16_t aio_20mA = 2592;

    if(loopAlreadyCal)
    {
        aio_4mA = calData[0];
        aio_20mA = calData[1];
    }
    uint16_t aii_4mA, aii_20mA;
    
    //cal aio
    Serial.println(F("Connect DMM in series with AIO and AII."));
    Serial.println(F("AIO----(DMMM)----AII"));
    Serial.println(F("Use 'i' and 'd' keys to increase/decrease measurement to calibration value."));
    Serial.println(F("Setting AIO calibration val to 4mA, enter 'q' when DMM measurement = 4mA."));
    
    do
    {
        pixi.single_ended_dac_write(MAX11300::PIXI_PORT8, aio_4mA);
        do
        {
          user_entry = get_user_char("increase (i), decrease (d), quit (q) when DMM = 4mA: ");
        }
        while((user_entry != 'i') && (user_entry != 'd') && (user_entry != 'q'));
        
        if(user_entry == 'i')
        {
            aio_4mA++;
        }
        else if(user_entry == 'd')
        {
            aio_4mA--;
        }
        else if(user_entry == 'q')
        {
            Serial.println(F("Setting AIO calibration val to 20mA, enter 'q' when DMM measurement = 20mA."));
        }
        else
        {
            Serial.println(F("Not an option, please read the instructions."));
        }
    }
    while(user_entry != 'q');
    
    do
    {
        pixi.single_ended_dac_write(MAX11300::PIXI_PORT8, aio_20mA);
        do
        {
          user_entry = get_user_char("increase (i), decrease (d), quit (q) when DMM = 20mA: ");
        }
        while((user_entry != 'i') && (user_entry != 'd') && (user_entry != 'q'));
        
        if(user_entry == 'i')
        {
            aio_20mA++;
        }
        else if(user_entry == 'd')
        {
            aio_20mA--;
        }
        else if(user_entry == 'q')
        {
            Serial.println(F("Executing AII Cal"));
        }
        else
        {
            Serial.println(F("Not an option, please read the instructions."));
        }
    }
    while(user_entry != 'q');

    calData[0] = aio_4mA;
    calData[1] = aio_20mA;
    
    uint8_t idx;
    //cal aii
    pixi.single_ended_dac_write(MAX11300::PIXI_PORT8, aio_4mA);
    for(idx = 0; idx < 10; idx++)
    {
        Serial.println(F("AII 4mA input cal..."));
        delay(1000);
    }
    pixi.single_ended_adc_read(MAX11300::PIXI_PORT12, aii_4mA);

    calData[2] = aii_4mA;
    
    pixi.single_ended_dac_write(MAX11300::PIXI_PORT8, aio_20mA);
    for(idx = 0; idx < 10; idx++)
    {
        Serial.println(F("AII 20mA input cal..."));
        delay(1000);
    }
    pixi.single_ended_adc_read(MAX11300::PIXI_PORT12, aii_20mA);

    calData[3] = aii_20mA;

    bool rtn_val = false;

    //Write cal data to eeprom
    OneWireMaster::CmdResult result = owm.OWReset();
    MultidropRomIterator selector(owm);
    SearchState search_state;
    
    DS2431 eeprom(selector);
    
    search_state.findFamily(0x2D);
    result = OWNext(owm, search_state);
    if(result == OneWireMaster::Success)
    {
        eeprom.setRomId(search_state.romId);
        
        OneWireSlave::CmdResult slaveResult = eeprom.writeMemory(0, "CalTrue!", 8);
        if(slaveResult == OneWireSlave::Success)
        {
            uint8_t writeCalData[8];
            //cal data stored MSB first;
            writeCalData[0] = ((aio_4mA >> 8) & 0xFF);
            writeCalData[1] = (aio_4mA & 0xFF);
            writeCalData[2] = ((aio_20mA >> 8) & 0xFF);
            writeCalData[3] = (aio_20mA & 0xFF);
            writeCalData[4] = ((aii_4mA >> 8) & 0xFF);
            writeCalData[5] = (aii_4mA & 0xFF);
            writeCalData[6] = ((aii_20mA >> 8) & 0xFF);
            writeCalData[7] = (aii_20mA & 0xFF);
            
            slaveResult = eeprom.writeMemory(8, writeCalData, 8);
            if(slaveResult == OneWireSlave::Success)
            {
                rtn_val = true;
                Serial.println(F("Cal Done"));
                delay(1000);
            }
            else
            {
                Serial.println(F("Failed to write cal data."));
            }
        }
        else
        {
            Serial.println(F("Failed to write cal message."));
        }
    }
    
    return rtn_val;
}

//*********************************************************************
bool check_420_cal(DS2484 & owm, uint16_t *calData)
{
    OneWireMaster::CmdResult result = owm.OWReset();
    MultidropRomIterator selector(owm);
    SearchState search_state;
    
    DS2431 eeprom(selector);
    
    bool rtn_val = false;

    Serial.println();
    Serial.println(F("Please remove any 1-Wire devices connected to bus on start up and calibrartion of 4-20mA loop."));
    Serial.println();
    delay(2500);
    
    search_state.findFamily(0x2D);
    result = OWNext(owm, search_state);
    if(result == OneWireMaster::Success)
    {
        eeprom.setRomId(search_state.romId);
        uint8_t temp_data[8];
        
        OneWireSlave::CmdResult slaveResult = eeprom.readMemory(0, temp_data, 8);
        
        if((temp_data[0] == 'C') && (slaveResult == OneWireSlave::Success))
        {
            slaveResult = eeprom.readMemory(8, temp_data, 8);
            if(slaveResult == OneWireSlave::Success)
            {
                calData[0] = ((temp_data[0] << 8) | temp_data[1]);
                calData[1] = ((temp_data[2] << 8) | temp_data[3]);
                calData[2] = ((temp_data[4] << 8) | temp_data[5]);
                calData[3] = ((temp_data[6] << 8) | temp_data[7]);
                
                rtn_val = true;
            }
            else
            {
                Serial.println(F("Failed to read row 1 of EEPROM"));
            }
        }
        else
        {
            if(slaveResult == OneWireSlave::Success)
            {
               Serial.println(F("No cal data stored, please calibrate 4-20mA loop"));
            }
            else
            {
                Serial.println(F("Failed to read row 0 of EEPROM"));
            }
        }
    }
    
    return rtn_val;
}

//*********************************************************************
void clear_screen(void)
{
  char ESC = 0x1B;
  Serial.print(ESC);
  Serial.print("[2J"); //clear screen
  Serial.print(ESC);
  Serial.print("[H"); //move cursor to Home
}

//*********************************************************************
void arduino_die(void)
{
  pinMode(13, OUTPUT);

  while(1)
  {
    digitalWrite(13, HIGH);
    delay(250);
    digitalWrite(13, LOW);
    delay(250);
  }
}

