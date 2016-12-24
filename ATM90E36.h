/* ATM90E36 Energy Monitor Header

   The MIT License (MIT)

  Copyright (c) 2016 whatnick,Ryzee and Arun

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#ifndef __ATM90E36_H__
#define __ATM90E36_H__

#define SoftReset 0x00 //Software Reset
#define SysStatus0 0x01 //System Status0
#define SysStatus1 0x02 //System Status1
#define FuncEn0 0x03 //Function Enable0
#define FuncEn1 0x04 //Function Enable1
#define SagTh 0x08 //Voltage Sag Threshold
//#define SmallPMod 0x04 //Small-Power Mode
#define LastSPIData 0x0F //Last Read/Write SPI Value
//#define LSB 0x08 //RMS/Power 16-bit LSB

#define ConfigStart 0x30 //Configuration start command
#define PLconstH 0x31 //High Word of PL_Constant
#define PLconstL 0x32 //Low Word of PL_Constant
#define MMode0 0x33 //Metering Mode Configuration
#define MMode1 0x34 //Metering Mode Configuration
#define PStartTh 0x35 //Active Startup Power Threshold
#define QStartTh 0x36 //Reactive Startup Power Threshold
#define CSZero 0x3B	//Checksum 0

#define CalStart 0x40 //Calibration Start Command
#define PoffsetA 0x41 //L Line Active Power Offset
#define QoffsetA 0x42 //L Line Reactive Power Offset
#define PoffsetB 0x43 //L Line Active Power Offset
#define QoffsetB 0x44 //L Line Reactive Power Offset
#define PoffsetC 0x45 //L Line Active Power Offset
#define QoffsetC 0x46 //L Line Reactive Power Offset
#define GainA 0x47 //A Line Calibration Gain
#define PhiA 0x48  //A Line Calibration Angle
#define GainB 0x49 //B Line Calibration Gain
#define PhiB 0x4A  //B Line Calibration Angle
#define GainC 0x4B //C Line Calibration Gain
#define PhiC 0x4C  //C Line Calibration Angle
#define CSOne 0x4D //Checksum 1

#define AdjStart 0x60 //Measurement Calibration Start Command
#define UgainA 0x61 //A Voltage rms Gain
#define IgainA 0x62 //A Line Current rms Gain
#define UgainB 0x65 //B Voltage rms Gain
#define UgainC 0x69 //C Voltage rms Gain
#define IgainB 0x66 //B Line Current rms Gain
#define IgainC 0x6A //C Line Current rms Gain
#define UoffsetA 0x63 //Voltage Offset
#define UoffsetB 0x67 //Voltage Offset
#define UoffsetC 0x6B //Voltage Offset
#define IoffsetA 0x64 //L Line Current Offset
#define IoffsetB 0x68 //N Line Current Offse
#define IoffsetC 0x6C //N Line Current Offse
#define CSThree 0x6F //Checksum 3


#define APenergyA 0x81 //Forward Active Energy
#define APenergyB 0x82 //Forward Active Energy
#define APenergyC 0x83 //Forward Active Energy
#define ANenergyA 0x85 //Reverse Active Energy
#define ANenergyB 0x86 //Reverse Active Energy
#define ANenergyC 0x87 //Reverse Active Energy
#define ANenergyT 0x84 //Absolute Active Energy
#define RPenergyA 0x89 //Forward (Inductive) Reactive Energy
#define RPenergyB 0x8A //Forward (Inductive) Reactive Energy
#define RPenergyC 0x8B //Forward (Inductive) Reactive Energy
#define RnenergyA 0x8D //Reverse (Capacitive) Reactive Energy
#define RnenergyB 0x8E //Reverse (Capacitive) Reactive Energy
#define RnenergyC 0x8F //Reverse (Capacitive) Reactive Energy
#define RPenergyT 0x88 //Absolute Reactive Energy
#define EnStatus0 0x95 //Metering Status
#define EnStatus1 0x96 //Metering Status
#define IrmsA 0xDD //L Line Current rms A
#define IrmsB 0xDE //L Line Current rms B
#define IrmsC 0xDF //L Line Current rms C
#define UrmsA 0xD9 //Voltage rms A
#define UrmsB 0xDA //Voltage rms B
#define UrmsC 0xDB //Voltage rms C

#define PmeanT 0xB0 //Total Mean Active Power
#define PmeanA 0xB1 //A Line Mean Active Power
#define PmeanB 0xB2 //B Line Mean Active Power
#define PmeanC 0xB3 //C Line Mean Active Power
#define QmeanA 0xB5 //A Line Mean Reactive Power
#define QmeanB 0xB6 //B Line Mean Reactive Power
#define QmeanC 0xB7 //C Line Mean Reactive Power
#define SmeanA 0xB9 //A Line Mean Apparent Power
#define SmeanB 0xBA //B Line Mean Apparent Power
#define SmeanC 0xBB //C Line Mean Apparent Power
#define PFmeanT 0xBC //Mean Power Factor

#define Freq 0xF8//Voltage Frequency
#define PangleA 0xF9 //Phase Angle between Voltage and A Line Current
#define PangleB 0xFA //Phase Angle between Voltage and B Line Current
#define PangleC 0xFB //Phase Angle between Voltage and C Line Current


// pins used for the connection with the sensor
// the other you need are controlled by the SPI library):
const int energy_CS =10;

unsigned short CommEnergyIC(unsigned char RW, unsigned short address, unsigned short val);
double  GetLineVoltage();
double GetLineCurrentA();
double GetActivePower();
double GetFrequency();
double GetPowerFactor();
double GetImportEnergy();
double GetExportEnergy();
void InitEnergyIC();
unsigned short GetSysStatus0();
unsigned short GetSysStatus1();
unsigned short  GetMeterStatus0();
unsigned short  GetMeterStatus1();

#endif