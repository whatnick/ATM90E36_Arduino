/* ATM90E36 Energy Monitor Demo Application

   The MIT License (MIT)

  Copyright (c) 2016 whatnick and Ryzee

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include <SPI.h>
#include <ATM90E36.h>

void setup() {
  /* Initialize the serial port to host */
  Serial.begin(9600);
  /*Initialise the ATM90E36 + SPI port */
  InitEnergyIC();
}



void loop() {
  
  /*Repeatedly fetch some values from the ATM90E36 */
  double voltageA,frequency,voltageB,voltageC,currentA,currentB,currentC,power,pf,new_current,new_power;
  int status0=GetSysStatus0();
  int status1=GetSysStatus1();
  int en0=GetMeterStatus0();
  int en1=GetMeterStatus1();
  if(status0){
    voltageA=0;voltageB=0;voltageC=0;currentA=0;power=0;pf=0;
  }
  else
  {
    Serial.println("e0"+String(en0,HEX)+"E0");
    delay(10);
    Serial.println("e1"+String(en1,HEX)+"E1");
    voltageA=GetLineVoltage();
    delay(10);
    Serial.println("v"+String(voltageA,HEX)+"v");
    currentA = GetLineCurrentA();
    delay(10);
    Serial.println("i"+String(currentA,HEX)+"i");
    frequency=GetFrequency();
    delay(10);
    Serial.println("f"+String(frequency,HEX)+"f");
  }
  delay(1000);
}
