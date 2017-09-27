/* ATM90E36 Energy Monitor Functions

   The MIT License (MIT)

  Copyright (c) 2016 whatnick,Ryzee and Arun

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include <SPI.h>
#include <ATM90E36.h>

ATM90E36::ATM90E36(int pin)
{
	_cs = pin;
}

unsigned short ATM90E36::CommEnergyIC(unsigned char RW, unsigned short address, unsigned short val) {

  unsigned char* data = (unsigned char*)&val;
  unsigned char* adata = (unsigned char*)&address;
  unsigned short output;
  unsigned short address1;
  //SPI interface rate is 200 to 160k bps. It Will need to be slowed down for EnergyIC
#if !defined(ENERGIA)
  SPISettings settings(160000, MSBFIRST, SPI_MODE3);
#endif
  //switch MSB and LSB of value
  output = (val >> 8) | (val << 8);
  val = output;

  //Set read write flag
  address |= RW << 15;
  //Swap address bytes
  address1 = (address >> 8) | (address << 8);
  address = address1;


  //Transmit and receive data
#if !defined(ENERGIA)
  SPI.beginTransaction(settings);
#endif
  //enable chip and wait for SPI bus to activate
  digitalWrite (_cs, LOW);
  delayMicroseconds(10);
  //Write address byte by byte
  for (byte i=0; i<2; i++)
  {
    SPI.transfer (*adata);
    adata++;
  }
  //SPI.transfer16(address);
  /* Must wait 4 us for data to become valid */
  delayMicroseconds(4);

  //Read data
  //Do for each byte in transfer
  if (RW)
  {
	for (byte i=0; i<2; i++)
    {
      *data = SPI.transfer (0x00);
      data++;
    }
    //val = SPI.transfer16(0x00);
  }
  else
  {
	for (byte i=0; i<2; i++)
    {
      SPI.transfer(*data);             // write all the bytes
      data++;
    }
    //SPI.transfer16(val);
  }

  digitalWrite(_cs, HIGH);
  delayMicroseconds(10);
#if !defined(ENERGIA)
  SPI.endTransaction();
#endif

  output = (val >> 8) | (val << 8); //reverse MSB and LSB
  return output;
  //Use with transfer16
  //return val;
}

double  ATM90E36::GetLineVoltageA() {
  unsigned short voltage = CommEnergyIC(1, UrmsA, 0xFFFF);
  return (double)voltage / 238.5;
}

double  ATM90E36::GetLineVoltageB() {
  unsigned short voltage = CommEnergyIC(1, UrmsB, 0xFFFF);
  return (double)voltage / 238.5;
}

double  ATM90E36::GetLineVoltageC() {
  unsigned short voltage = CommEnergyIC(1, UrmsC, 0xFFFF);
  return (double)voltage / 238.5;
}

unsigned short  ATM90E36::GetMeterStatus0() {
  return CommEnergyIC(1, EnStatus0, 0xFFFF);
}

unsigned short  ATM90E36::GetMeterStatus1() {
  return CommEnergyIC(1, EnStatus1, 0xFFFF);
}
double ATM90E36::GetLineCurrentA() {
  unsigned short current = CommEnergyIC(1, IrmsA, 0xFFFF);
  return (double)current * 7.13 / 1000;
}

double ATM90E36::GetLineCurrentB() {
  unsigned short current = CommEnergyIC(1, IrmsB, 0xFFFF);
  return (double)current * 7.13 / 1000;
}

double ATM90E36::GetLineCurrentC() {
  unsigned short current = CommEnergyIC(1, IrmsC, 0xFFFF);
  return (double)current * 7.13 / 1000;
}

double ATM90E36::GetActivePowerA() {
  short int apower = (short int)CommEnergyIC(1, PmeanA, 0xFFFF); //Complement, MSB is signed bit
  return (double)apower * 2.94;
}

double ATM90E36::GetActivePowerB() {
  short int apower = (short int)CommEnergyIC(1, PmeanB, 0xFFFF); //Complement, MSB is signed bit
  return (double)apower * 2.94;
}

double ATM90E36::GetActivePowerC() {
  short int apower = (short int)CommEnergyIC(1, PmeanC, 0xFFFF); //Complement, MSB is signed bit
  return (double)apower * 2.94;
}

double ATM90E36::GetFrequency() {
  unsigned short freq = CommEnergyIC(1, Freq, 0xFFFF);
  return (double)freq / 100;
}

double ATM90E36::GetPowerFactor() {
  short int pf = (short int)CommEnergyIC(1, PFmeanT, 0xFFFF); //MSB is signed bit
  //if negative
  if (pf & 0x8000) {
    pf = (pf & 0x7FFF) * -1;
  }
  return (double)pf / 1000;
}

double ATM90E36::GetImportEnergy() {
  //Register is cleared after reading
  unsigned short ienergy = CommEnergyIC(1, APenergyA, 0xFFFF);
  return (double)ienergy / 10 / 1000; //returns kWh if PL constant set to 1000imp/kWh
}

double ATM90E36::GetExportEnergy() {
  //Register is cleared after reading
  unsigned short eenergy = CommEnergyIC(1, ANenergyT, 0xFFFF);
  return (double)eenergy / 10 / 1000; //returns kWh if PL constant set to 1000imp/kWh
}

unsigned short ATM90E36::GetSysStatus0() {
  return CommEnergyIC(1, SysStatus0, 0xFFFF);
}

unsigned short ATM90E36::GetSysStatus1() {
  return CommEnergyIC(1, SysStatus1, 0xFFFF);
}


void ATM90E36::InitEnergyIC() {
  //Serial.println("Initialising:");
  unsigned short systemstatus0;

  //pinMode(energy_IRQ,INPUT );
  pinMode(_cs, OUTPUT );
  //pinMode(energy_WO,INPUT );

  /* Enable SPI */
  SPI.begin();
#if defined(ENERGIA)
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
#endif

  //Serial.println("Initialising1:");

  CommEnergyIC(0, SoftReset, 0x789A); //Perform soft reset
  CommEnergyIC(0, FuncEn0, 0x0030); //Voltage sag irq=1, report on warnout pin=1, energy dir change irq=0
  CommEnergyIC(0, FuncEn1, 0x0030); //Voltage sag irq=1, report on warnout pin=1, energy dir change irq=0
  CommEnergyIC(0, SagTh, 0x1F2F); //Voltage sag threshhold
  
  //Set metering config values
  CommEnergyIC(0, ConfigStart, 0x5678); //Metering calibration startup command. Register 31 to 3B need to be set
  CommEnergyIC(0, PLconstH, 0x00B9); //PL Constant MSB
  CommEnergyIC(0, PLconstL, 0xC1F3); //PL Constant LSB
  CommEnergyIC(0, MMode0, 0x0087); //Metering Mode Configuration. All defaults. See pg 58 of datasheet.
  CommEnergyIC(0, MMode1, 0x5555); //PGA Gain Configuration. x2 for DPGA and PGA. See pg 59 of datasheet
  CommEnergyIC(0, PStartTh, 0x08BD); //Active Startup Power Threshold
  CommEnergyIC(0, QStartTh, 0x0AEC); //Reactive Startup Power Threshold
  CommEnergyIC(0, CSZero, 0x5F59); //Write CSOne, as self calculated
  
  Serial.print("Checksum 0:");
  Serial.println(CommEnergyIC(1, CSZero, 0x0000), HEX); //Checksum 0. Needs to be calculated based off the above values.
  
  //Set metering calibration values
  CommEnergyIC(0, CalStart, 0x5678); //Metering calibration startup command. Register 41 to 4D need to be set
  CommEnergyIC(0, GainA, 0x1D39); //Line calibration gain
  CommEnergyIC(0, PhiA, 0x0000); //Line calibration angle
  CommEnergyIC(0, GainB, 0x1D39); //Line calibration gain
  CommEnergyIC(0, PhiB, 0x0000); //Line calibration angle
  CommEnergyIC(0, GainC, 0x1D39); //Line calibration gain
  CommEnergyIC(0, PhiC, 0x0000); //Line calibration angle
  CommEnergyIC(0, PoffsetA, 0x0000); //A line active power offset
  CommEnergyIC(0, QoffsetA, 0x0000); //A line reactive power offset
  CommEnergyIC(0, PoffsetB, 0x0000); //B line active power offset
  CommEnergyIC(0, QoffsetB, 0x0000); //B line reactive power offset
  CommEnergyIC(0, PoffsetC, 0x0000); //C line active power offset
  CommEnergyIC(0, QoffsetC, 0x0000); //C line reactive power offset
  CommEnergyIC(0, CSOne, 0x2402); //Write CSOne, as self calculated
  
  Serial.print("Checksum 1:");
  Serial.println(CommEnergyIC(1, CSOne, 0x0000), HEX); //Checksum 1. Needs to be calculated based off the above values.


  //Set measurement calibration values
  CommEnergyIC(0, AdjStart, 0x5678); //Measurement calibration startup command, registers 61-6F
  CommEnergyIC(0, UgainA, 0xD8E9);  //A SVoltage rms gain
  CommEnergyIC(0, IgainA, 0x1BC9); //A line current gain
  CommEnergyIC(0, UoffsetA, 0x0000); //A Voltage offset
  CommEnergyIC(0, IoffsetA, 0x0000); //A line current offset
  CommEnergyIC(0, UgainB, 0xD8E9);  //B Voltage rms gain
  CommEnergyIC(0, IgainB, 0x1BC9); //B line current gain
  CommEnergyIC(0, UoffsetB, 0x0000); //B Voltage offset
  CommEnergyIC(0, IoffsetB, 0x0000); //B line current offset
  CommEnergyIC(0, UgainC, 0xD8E9);  //C Voltage rms gain
  CommEnergyIC(0, IgainC, 0x1BC9); //C line current gain
  CommEnergyIC(0, UoffsetC, 0x0000); //C Voltage offset
  CommEnergyIC(0, IoffsetC, 0x0000); //C line current offset
  CommEnergyIC(0, CSThree, 0xA694); //Write CSThree, as self calculated

  Serial.print("Checksum 3:");
  Serial.println(CommEnergyIC(1, CSThree, 0x0000), HEX); //Checksum 3. Needs to be calculated based off the above values.

  CommEnergyIC(0, ConfigStart, 0x8765); //Checks correctness of 31-3B registers and starts normal metering if ok
  CommEnergyIC(0, CalStart, 0x8765); //Checks correctness of 41-4D registers and starts normal metering if ok
  CommEnergyIC(0, AdjStart, 0x8765); //Checks correct ness of 61-6F registers and starts normal measurement  if ok

  systemstatus0 = GetSysStatus0();

  if (systemstatus0 & 0x4000) {
    //checksum 1 error
    Serial.println("checksum error 0");
  }
  if (systemstatus0 & 0x1000) {
    //checksum 2 error
     Serial.println("checksum error 1");
  }
  if (systemstatus0 & 0x0400) {
    //checksum 2 error
     Serial.println("checksum error 2");
  }
  if (systemstatus0 & 0x0100) {
    //checksum 2 error
     Serial.println("checksum error 3");
  }
}