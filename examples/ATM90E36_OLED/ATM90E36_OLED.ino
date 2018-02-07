/* ATM90E36 Energy Monitor Demo Application

   The MIT License (MIT)

  Copyright (c) 2016 whatnick and Ryzee

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include <SPI.h>
#include <ATM90E36.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

ATM90E36 eics[2] = {ATM90E36(10),ATM90E36(9)};

void setup() {
  /* Initialize the serial port to host */
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  Serial.println("Start ATM90E36");
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // init done
  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  /*Initialise the ATM90E36 + SPI port */
  eics[0].begin();
  delay(1000);
  eics[1].begin();
  delay(1000);
}

void scanEic(ATM90E36 eic)
{
  double voltageA,freq,voltageB,voltageC,currentA,currentB,currentC,power,pf,new_current,new_power;
  int sys0=eic.GetSysStatus0();
  int sys1=eic.GetSysStatus1();
  int en0=eic.GetMeterStatus0();
  int en1=eic.GetMeterStatus1();
  Serial.println("S0:0x"+String(sys0,HEX));
  delay(10);
  Serial.println("S1:0x"+String(sys1,HEX));
  delay(10);
  Serial.println("E0:0x"+String(en0,HEX));
  delay(10);
  Serial.println("E1:0x"+String(en1,HEX));
  display.clearDisplay();
  display.setCursor(0,0);
  voltageA=eic.GetLineVoltageA();
  Serial.println("VA:"+String(voltageA)+"V");
  display.println("VA:"+String(voltageA)+"V");
  voltageB=eic.GetLineVoltageB();
  Serial.println("VB:"+String(voltageB)+"V");
  display.println("VB:"+String(voltageB)+"V");
  voltageC=eic.GetLineVoltageC();
  Serial.println("VC:"+String(voltageC)+"V");
  display.println("VC:"+String(voltageC)+"V");
  delay(10);
  currentA = eic.GetLineCurrentA();
  Serial.println("IA:"+String(currentA)+"A");
  currentB = eic.GetLineCurrentB();
  Serial.println("IB:"+String(currentB)+"A");
  currentC = eic.GetLineCurrentC();
  Serial.println("IC:"+String(currentC)+"A");
  delay(10);
  freq=eic.GetFrequency();
  delay(10);
  Serial.println("f"+String(freq)+"Hz");
  display.println("f"+String(freq)+"Hz");
  display.display();
  delay(1000);
}


void loop() {
  /*Repeatedly fetch some values from the ATM90E36 */
  scanEic(eics[0]);
  scanEic(eics[1]);
}
