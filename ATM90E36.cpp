#include "ATM90E36.h"
typedef struct
{
  int Ugaina =0x0002;
  int Igaina =0x0001;
  int Uoffseta =0x0000;
  int Ioffseta=0x0000;
  int Ugainb =0x0002;
  int Igainb=0x0001;
  int Uoffsetb=0x0000;
  int Ioffsetb=0x0000;
  int Ugainc =0x0002;
  int Igainc=0x0001;
  int Uoffsetc=0x0000;
  int Ioffsetc=0x0000;
  int Igainn=0x0001;
}Calib_val;
Calib_val calib_val;
ATM90E36::ATM90E36(int pin) 	// Object
{
  //energy_IRQ = 2; 	// (In development...)
  _energy_CS = pin; 	// SS PIN
  //energy_WO = 8; 		// (In development...)
}

/* CommEnergyIC - Communication Establishment */
/*
- Defines Register Mask
- Treats the Register and SPI Comms
- Outputs the required value in the register
*/
unsigned short ATM90E36::CommEnergyIC(unsigned char RW, unsigned short address, unsigned short val) 
{
  unsigned char* data = (unsigned char*)&val;
  unsigned char* adata = (unsigned char*)&address;
  unsigned short output;
  unsigned short address1;

  // Slows the SPI interface to communicate
#if !defined(ENERGIA) && !defined(ESP8266) && !defined(ARDUINO_ARCH_SAMD)
  SPISettings settings(400000, MSBFIRST, SPI_MODE0);
#endif

#if defined(ESP8266)
  SPISettings settings(200000, MSBFIRST, SPI_MODE2);
#endif

#if defined(ARDUINO_ARCH_SAMD)
  SPISettings settings(200000, MSBFIRST, SPI_MODE3);
#endif

  // Switch MSB and LSB of value
  output = (val >> 8) | (val << 8);
  val = output;

  // Set R/W flag
  address |= RW << 15;

  // Swap byte address
  address1 = (address >> 8) | (address << 8);
  address = address1;

  // Transmit & Receive Data
#if !defined(ENERGIA)
  SPI.beginTransaction(settings);
#endif

  // Chip enable and wait for SPI activation
  digitalWrite (_energy_CS, LOW);
  delayMicroseconds(10);

  // Write address byte by byte
  for (byte i=0; i<2; i++)
  {
    SPI.transfer (*adata);
    adata++;
  }

  // SPI.transfer16(address);
  /* Must wait 4 us for data to become valid */
  delayMicroseconds(4);

  // READ Data
  // Do for each byte in transfer
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
      SPI.transfer(*data);
      data++;
    }
    // SPI.transfer16(val);
  }

  // Chip enable and wait for transaction to end
  digitalWrite(_energy_CS, HIGH);
  delayMicroseconds(10);
#if !defined(ENERGIA)
  SPI.endTransaction();
#endif

  output = (val >> 8) | (val << 8); // reverse MSB and LSB
  return output;

  // Use with transfer16
  // return val;
}

/* Parameters Functions*/
/*
- Gets main electrical parameters,
such as: Voltage, Current, Power, Energy,
and Frequency
- Also gets the temperature
*/
// VOLTAGE
double  ATM90E36::GetLineVoltageA() {
  unsigned short voltage = CommEnergyIC(READ, UrmsA, 0x0000);
  return (double)voltage / 100;
}

double  ATM90E36::GetLineVoltageB() {
  unsigned short voltage = CommEnergyIC(READ, UrmsB, 0x0000);
  return (double)voltage / 100;
}

double  ATM90E36::GetLineVoltageC() {
  unsigned short voltage = CommEnergyIC(READ, UrmsC, 0x0000);
  return (double)voltage / 100;
}

// CURRENT
double ATM90E36::GetLineCurrentA() {
  unsigned short current = CommEnergyIC(READ, IrmsA, 0x0000);
  return (double)current / 1000;
}
double ATM90E36::GetLineCurrentB() {
  unsigned short current = CommEnergyIC(READ, IrmsB, 0x0000);
  return (double)current / 1000;
}
double ATM90E36::GetLineCurrentC() {
  unsigned short current = CommEnergyIC(READ, IrmsC, 0x0000);
  return (double)current / 1000;
}
double ATM90E36::GetLineCurrentN() {
  unsigned short current = CommEnergyIC(READ, IrmsN0, 0x0000);
  return (double)current / 1000;
}

// ACTIVE POWER
double ATM90E36::GetActivePowerA() {
  signed short apower = (signed short) CommEnergyIC(READ, PmeanA, 0x0000); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double ATM90E36::GetActivePowerB() {
  signed short apower = (signed short) CommEnergyIC(READ, PmeanB, 0x0000); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double ATM90E36::GetActivePowerC() {
  signed short apower = (signed short) CommEnergyIC(READ, PmeanC, 0x0000); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double ATM90E36::GetTotalActivePower() {
  signed short apower = (signed short) CommEnergyIC(READ, PmeanT, 0x0000); 
  /*if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }*/
  return (double)apower / 250;
}

// REACTIVE POWER
double ATM90E36::GetReactivePowerA() {
  signed short apower = (signed short) CommEnergyIC(READ, QmeanA, 0x0000); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double ATM90E36::GetReactivePowerB() {
  signed short apower = (signed short) CommEnergyIC(READ, QmeanB, 0x0000); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double ATM90E36::GetReactivePowerC() {
  signed short apower = (signed short) CommEnergyIC(READ, QmeanC, 0xFFFF);
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double ATM90E36::GetTotalReactivePower() {
  signed short apower = (signed short) CommEnergyIC(READ, QmeanT, 0x0000); 
  /*if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }*/
  return (double)apower / 250;
}

// APPARENT POWER
double ATM90E36::GetApparentPowerA() {
  signed short apower = (signed short) CommEnergyIC(READ, SmeanA, 0x0000); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double ATM90E36::GetApparentPowerB() {
 signed short apower = (signed short) CommEnergyIC(READ, SmeanB, 0x0000);
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double ATM90E36::GetApparentPowerC() {
  signed short apower = (signed short) CommEnergyIC(READ, SmeanC, 0x0000); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 1000;
}
double ATM90E36::GetTotalApparentPower() {
  signed short apower = (signed short) CommEnergyIC(READ, SmeanT, 0x0000); 
  if (apower & 0x8000) {
    apower= (apower & 0x7FFF) * -1;
  }
  return (double)apower / 250;
}

// FREQUENCY
double ATM90E36::GetFrequency() {
  unsigned short freq = CommEnergyIC(READ, Freq, 0xFFFF);
  return (double)freq / 100;
}

// // POWER FACTOR
// double ATM90E36::GetPowerFactorA() {
//   short pf = (short) CommEnergyIC(READ, PFmeanA, 0xFFFF); 
//   Serial.print(pf);
//   Serial.print(", ");
//   Serial.print("From Lib A : ");
//   Serial.println(String(pf, HEX));
//   return (double)pf;
// }
// double ATM90E36::GetPowerFactorB() {
//   short pf = (short) CommEnergyIC(READ, PFmeanB, 0xFFFF); 
//   Serial.print(pf);
//   Serial.print(", ");
//   Serial.print("From Lib B : ");
//   Serial.println(String(pf, HEX));
//   return (double)pf;
// }
// double ATM90E36::GetPowerFactorC() {
//   short pf = (short) CommEnergyIC(READ, PFmeanC, 0xFFFF); 
//   Serial.print("From Lib C : ");
//   Serial.print(pf);
//   Serial.print(", ");
//   Serial.println(String(pf, HEX));
//   //if negative
  
//   return (double)pf;
// }
// double ATM90E36::GetTotalPowerFactor() {
//   short pf = (short) CommEnergyIC(READ, PFmeanT, 0xFFFF); 
//   return (double)pf ;
// }


// VOLTAGE Harmonics
double  ATM90E36::GetVHarmA() {
  unsigned short value = CommEnergyIC(READ, THDNUA, 0xFFFF);
  return (double)value;
}
double  ATM90E36::GetVHarmB() {
  unsigned short value = CommEnergyIC(READ, THDNUB, 0xFFFF);
  return (double)value;
}
double  ATM90E36::GetVHarmC() {
  unsigned short value = CommEnergyIC(READ, THDNUC, 0xFFFF);
  return (double)value;
}
// Current Harmonics
double  ATM90E36::GetCHarmA() {
  unsigned short value = CommEnergyIC(READ, THDNIA, 0xFFFF);
  return (double)value;
}
double  ATM90E36::GetCHarmB() {
  unsigned short value = CommEnergyIC(READ, THDNIB, 0xFFFF);
  return (double)value;
}
double  ATM90E36::GetCHarmC() {
  unsigned short value = CommEnergyIC(READ, THDNIC, 0xFFFF);
  return (double)value;
}

double ATM90E36::GetPowerFactorA() {
  short pf = (short) CommEnergyIC(READ, PFmeanA, 0xFFFF); 
  //if negative
  //  if (pf & 0x8000) {
  //   pf = (pf & 0x7FFF) * -1;
  // }
  return (double)pf / 1000;
}
double ATM90E36::GetPowerFactorB() {
  short pf = (short) CommEnergyIC(READ, PFmeanB, 0xFFFF); 
  // if (pf & 0x8000) {
  //   pf = (pf & 0x7FFF) * -1;
  // }
  return (double)pf / 1000;
}
double ATM90E36::GetPowerFactorC() {
  short pf = (short) CommEnergyIC(READ, PFmeanC, 0xFFFF); 
  //if negative
  // if (pf & 0x8000) {
  //   pf = (pf & 0x7FFF) * -1;
  // }
  return (double)pf / 1000;
}
double ATM90E36::GetTotalPowerFactor() {
  short pf = (short) CommEnergyIC(READ, PFmeanT, 0xFFFF); 
  // //if negative
  // if (pf & 0x8000) {
  //   pf = (pf & 0x7FFF) * -1;
  // }
  return (double)pf / 1000;
}
// PHASE ANGLE
double ATM90E36::GetPhaseA() {
  signed short apower = (signed short) CommEnergyIC(READ, UangleA, 0xFFFF);
  return (double)apower / 10;
}
double ATM90E36::GetPhaseB() {
  signed short apower = (signed short) CommEnergyIC(READ, UangleB, 0xFFFF);
  return (double)apower / 10;
}
double ATM90E36::GetPhaseC() {
  signed short apower = (signed short) CommEnergyIC(READ, UangleC, 0xFFFF);
  return (double)apower / 10;
}

// TEMPERATURE
double ATM90E36::GetTemperature() {
  short int apower = (short int) CommEnergyIC(READ, Temp, 0xFFFF); 
  return (double)apower;
}

/* Gets the Register Value if Desired */
// REGISTER
unsigned short ATM90E36::GetValueRegister(unsigned short registerRead) {
  return (CommEnergyIC(READ, registerRead, 0xFFFF)); //returns value register
}

// ENERGY MEASUREMENT
double ATM90E36::GetImportEnergy() {
  unsigned short ienergyT = CommEnergyIC(READ, APenergyT, 0xFFFF);
  // Serial.print("Value of energy from Lib");
  // Serial.println(ienergyT);
  for(int a=0;a<1000;a++){
  }
  unsigned short ienergyA = CommEnergyIC(READ, APenergyA, 0xFFFF);
  for(int a=0;a<1000;a++){
  }
  unsigned short ienergyB = CommEnergyIC(READ, APenergyB, 0xFFFF);
  // unsigned short ienergyC = CommEnergyIC(READ, APenergyC, 0xFFFF);

  // unsigned short renergyT = CommEnergyIC(READ, RPenergyT, 0xFFFF);
  // unsigned short renergyA = CommEnergyIC(READ, RPenergyA, 0xFFFF);
  // unsigned short renergyB = CommEnergyIC(READ, RPenergyB, 0xFFFF);
  // unsigned short renergyC = CommEnergyIC(READ, RPenergyC, 0xFFFF);

  // unsigned short senergyT = CommEnergyIC(READ, SAenergyT, 0xFFFF);
  // unsigned short senergyA = CommEnergyIC(READ, SenergyA, 0xFFFF);
  // unsigned short senergyB = CommEnergyIC(READ, SenergyB, 0xFFFF);
  // unsigned short senergyC = CommEnergyIC(READ, SenergyC, 0xFFFF);
  // return (double)(((ienergyB-ienergyA) /3.2)*3600);
  return (double)(((ienergyT) /3.2)*3600); //returns kWh
}
double ATM90E36::GetImportEnergydiff() {
  // unsigned short ienergyT = CommEnergyIC(READ, APenergyT, 0xFFFF);
  // Serial.print("Value of energy from Lib");
  // Serial.println(ienergyT);
  for(int a=0;a<1000;a++){
  }
  unsigned short ienergyA = CommEnergyIC(READ, APenergyA, 0xFFFF);
  for(int a=0;a<1000;a++){
  }
  unsigned short ienergyB = CommEnergyIC(READ, APenergyB, 0xFFFF);
  // unsigned short ienergyC = CommEnergyIC(READ, APenergyC, 0xFFFF);

  // unsigned short renergyT = CommEnergyIC(READ, RPenergyT, 0xFFFF);
  // unsigned short renergyA = CommEnergyIC(READ, RPenergyA, 0xFFFF);
  // unsigned short renergyB = CommEnergyIC(READ, RPenergyB, 0xFFFF);
  // unsigned short renergyC = CommEnergyIC(READ, RPenergyC, 0xFFFF);

  // unsigned short senergyT = CommEnergyIC(READ, SAenergyT, 0xFFFF);
  // unsigned short senergyA = CommEnergyIC(READ, SenergyA, 0xFFFF);
  // unsigned short senergyB = CommEnergyIC(READ, SenergyB, 0xFFFF);
  // unsigned short senergyC = CommEnergyIC(READ, SenergyC, 0xFFFF);
  return (double)(((ienergyB-ienergyA) /3.2)*3600);
  // return (double)(((ienergyT) /3.2)*3600); //returns kWh
}
double ATM90E36::GetExportEnergy() {

  unsigned short eenergyT = CommEnergyIC(READ, ANenergyT, 0xFFFF);
  // Serial.print("Value of Export energy from Lib");
  // Serial.println(eenergyT);
  // unsigned short eenergyA = CommEnergyIC(READ, ANenergyA, 0xFFFF);
  // unsigned short eenergyB = CommEnergyIC(READ, ANenergyB, 0xFFFF);
  // unsigned short eenergyC = CommEnergyIC(READ, ANenergyC, 0xFFFF);

  // unsigned short reenergyT = CommEnergyIC(READ, RNenergyT, 0xFFFF);
  // unsigned short reenergyA = CommEnergyIC(READ, RNenergyA, 0xFFFF);
  // unsigned short reenergyB = CommEnergyIC(READ, RNenergyB, 0xFFFF);
  // unsigned short reenergyC = CommEnergyIC(READ, RNenergyC, 0xFFFF);

  return (double)eenergyT / 100 / 3200; //returns kWh 
}

/* System Status Registers */
unsigned short ATM90E36::GetSysStatus0() {    
  return CommEnergyIC(READ, SysStatus0, 0xFFFF);
}
unsigned short ATM90E36::GetSysStatus1() {
  return CommEnergyIC(READ, SysStatus1, 0xFFFF);
}
unsigned short  ATM90E36::GetMeterStatus0() {
  return CommEnergyIC(READ, EnStatus0, 0xFFFF);
}
unsigned short  ATM90E36::GetMeterStatus1() {
  return CommEnergyIC(READ, EnStatus1, 0xFFFF);
}
bool ATM90E36::errorCheckSPI()
{ 
  int  PLConstH =0x0861;
  int  PLConstL =0xC468;
  int  MMode00  =0x0087;
  int  MMode01  =0x5555;
 int i= (int)CommEnergyIC(READ, UoffsetA,0xFFFF);
 int j= (int)CommEnergyIC(READ, PLconstH, 0xFFFF);    
 int k= (int)CommEnergyIC(READ, PLconstL, 0xFFFF);    
 int l= (int)CommEnergyIC(READ, MMode0, 0xFFFF);      
 int m= (int)CommEnergyIC(READ, MMode1, 0xFFFF);
 if((i!=0)||(j!=PLConstH)||(k!=PLConstL)||(l!=MMode00)||(m!=MMode01)){
   return 1;
 }
 else{
   return 0;
 }
}

/* Checksum Error Function */
bool ATM90E36::calibrationError()
{
  bool CS0, CS1, CS2, CS3;
  unsigned short systemstatus0 = GetSysStatus0();

  if (systemstatus0 & 0x4000)
  {
    CS0 = true;
  }
  else
  {
    CS0 = false;
  } 

  if (systemstatus0 & 0x0100)
  {
    CS1 = true; 
  } 
  else 
  {
    CS1 = false;
  }
  if (systemstatus0 & 0x0400)
  {
    CS2 = true;
  }
  else
  {
    CS2 = false;
  } 
  if (systemstatus0 & 0x0100)
  {
    CS3 = true;
  }
  else 
  {
    CS3 = false;
  }

#ifdef DEBUG_SERIAL
    // Serial.print("Checksum 0: ");
    // Serial.println(CS0);
    // Serial.print("Checksum 1: ");
    // Serial.println(CS1);
    // Serial.print("Checksum 2: ");
    // Serial.println(CS2);
    // Serial.print("Checksum 3: ");
    // Serial.println(CS3);
#endif

  if (CS0 || CS1 || CS2 || CS3) return (true); 
  else return (false);

}

/* BEGIN FUNCTION */
/* 
- Define the pin to be used as Chip Select
- Set serialFlag to true for serial debugging
- Use SPI MODE 0 for the ATM90E36
*/
void ATM90E36::begin()
{  
// pinMode(energy_IRQ, INPUT); // (In development...)
 // pinMode(energy_IRQ, INPUT); // (In development...)
 pinMode(_energy_CS, OUTPUT);
 // pinMode(energy_WO, INPUT);  // (In development...)

 /* Enable SPI */
 SPI.begin();
#if defined(ENERGIA)
 SPI.setBitOrder(MSBFIRST);
 SPI.setDataMode(SPI_MODE0);
 SPI.setClockDivider(SPI_CLOCK_DIV16);
#endif

 CommEnergyIC(WRITE, SoftReset, 0x789A);   // Perform soft reset
 delay(100);
 CommEnergyIC(WRITE, FuncEn0, 0x0000);     // Voltage sag
 CommEnergyIC(WRITE, FuncEn1, 0x0000);     // Voltage sag
 CommEnergyIC(WRITE, SagTh, 0x0001);       // Voltage sag threshold

 /* SagTh = Vth * 100 * sqrt(2) / (2 * Ugain / 32768) */
 
 //Set metering config values (CONFIG)
 CommEnergyIC(WRITE, ConfigStart, 0x5678); // Metering calibration startup
 CommEnergyIC(WRITE, PLconstH, 0x0861);    // PL Constant MSB (default)
 CommEnergyIC(WRITE, PLconstL, 0xC468);    // PL Constant LSB (default)
 CommEnergyIC(WRITE, MMode0, 0x0087);      // Mode Config (50 Hz, 3P3W)
 CommEnergyIC(WRITE, MMode1, 0x5555);      // 0x5555 (x2) // 0x0000 (1x)
 CommEnergyIC(WRITE, PStartTh, 0x0000);    // Active Startup Power Threshold
 CommEnergyIC(WRITE, QStartTh, 0x0000);    // Reactive Startup Power Threshold
 CommEnergyIC(WRITE, SStartTh, 0x0000);    // Apparent Startup Power Threshold
 CommEnergyIC(WRITE, PPhaseTh, 0x0000);    // Active Phase Threshold
 CommEnergyIC(WRITE, QPhaseTh, 0x0000);    // Reactive Phase Threshold
 CommEnergyIC(WRITE, SPhaseTh, 0x0000);    // Apparent  Phase Threshold
//  CommEnergyIC(WRITE, CSZero, 0x4741);      // Checksum 0
 
/*
 //Set metering calibration values (CALIBRATION)
 CommEnergyIC(WRITE, CalStart, 0x5678);    // Metering calibration startup
 CommEnergyIC(WRITE, GainA, 0x5768);       // Line calibration gain
 CommEnergyIC(WRITE, PhiA, 0X0000);        // Line calibration angle
 CommEnergyIC(WRITE, GainB, 0x5768);       // Line calibration gain
 CommEnergyIC(WRITE, PhiB, 0X0000);        // Line calibration angle
 CommEnergyIC(WRITE, GainC, 0x5768);       // Line calibration gain
 CommEnergyIC(WRITE, PhiC, 0X0000);        // Line calibration angle
 CommEnergyIC(WRITE, PoffsetA, 0x5678);    // A line active power offset
 CommEnergyIC(WRITE, QoffsetA, 0x5678);    // A line reactive power offset
 CommEnergyIC(WRITE, PoffsetB, 0x5678);    // B line active power offset
 CommEnergyIC(WRITE, QoffsetB, 0x5678);    // B line reactive power offset
 CommEnergyIC(WRITE, PoffsetC, 0x5678);    // C line active power offset
 CommEnergyIC(WRITE, QoffsetC, 0x5678);    // C line reactive power offset
//  CommEnergyIC(WRITE, CSOne, 0x0000);       // Cheksum 1
 
 //Set metering calibration values (HARMONIC)
 CommEnergyIC(WRITE, HarmStart, 0x5678);   // Metering calibration startup
 CommEnergyIC(WRITE, POffsetAF, 0x0000);   // A Fund. active power offset
 CommEnergyIC(WRITE, POffsetBF, 0x0000);   // B Fund. active power offset
 CommEnergyIC(WRITE, POffsetCF, 0x0000);   // C Fund. active power offset
 CommEnergyIC(WRITE, PGainAF, 0x0000);     // A Fund. active power gain
 CommEnergyIC(WRITE, PGainBF, 0x0000);     // B Fund. active power gain
 CommEnergyIC(WRITE, PGainCF, 0x0000);     // C Fund. active power gain
//  CommEnergyIC(WRITE, CSTwo, 0x0000);       // Checksum 2
*/
//Set metering calibration values (CALIBRATION)
  CommEnergyIC(WRITE, CalStart, 0x5678);    // Metering calibration startup 
  CommEnergyIC(WRITE, GainA, 0x0000);       // Line calibration gain
  CommEnergyIC(WRITE, PhiA, 0x0000);        // Line calibration angle
  CommEnergyIC(WRITE, GainB, 0x0000);       // Line calibration gain
  CommEnergyIC(WRITE, PhiB, 0x0000);        // Line calibration angle
  CommEnergyIC(WRITE, GainC, 0x0000);       // Line calibration gain
  CommEnergyIC(WRITE, PhiC, 0x0000);        // Line calibration angle
  CommEnergyIC(WRITE, PoffsetA, 0x0000);    // A line active power offset
  CommEnergyIC(WRITE, QoffsetA, 0x0000);    // A line reactive power offset
  CommEnergyIC(WRITE, PoffsetB, 0x0000);    // B line active power offset
  CommEnergyIC(WRITE, QoffsetB, 0x0000);    // B line reactive power offset
  CommEnergyIC(WRITE, PoffsetC, 0x0000);    // C line active power offset
  CommEnergyIC(WRITE, QoffsetC, 0x0000);    // C line reactive power offset
  // CommEnergyIC(WRITE, CSOne, 0x0000);       // Checksum 1
  
  //Set metering calibration values (HARMONIC)
  CommEnergyIC(WRITE, HarmStart, 0x5678);   // Metering calibration startup 
  CommEnergyIC(WRITE, POffsetAF, 0x0000);   // A Fund. active power offset
  CommEnergyIC(WRITE, POffsetBF, 0x0000);   // B Fund. active power offset
  CommEnergyIC(WRITE, POffsetCF, 0x0000);   // C Fund. active power offset
  CommEnergyIC(WRITE, PGainAF, 0x0001);     // A Fund. active power gain
  CommEnergyIC(WRITE, PGainBF, 0x0001);     // B Fund. active power gain
  CommEnergyIC(WRITE, PGainCF, 0x0001);     // C Fund. active power gain
  // CommEnergyIC(WRITE, CSTwo, 0x0000);       // Checksum 2 

/* //Set measurement calibration values (ADJUST)
 // CommEnergyIC(WRITE, CSThree, 0x02F6);     // Checksum 3*/
  // REPLACE 81FB FOR OTHER CTS
  CommEnergyIC(WRITE, AdjStart, 0x5678);    // Measurement calibration
  CommEnergyIC(WRITE, UgainA, 0x678C);   //678C   74BC// A SVoltage rms gain //for 100 A CT 
  CommEnergyIC(WRITE, IgainA,0x3FE6);      //0x2859 A line current gain//for 100 A CT 27E5
  CommEnergyIC(WRITE, UoffsetA, 0x0000);    // A Voltage offset
  CommEnergyIC(WRITE, IoffsetA, 0x0000);    // A line current offset
  CommEnergyIC(WRITE, UgainB, 0x66C8);     //066D8  537E B Voltage rms gain
  CommEnergyIC(WRITE, IgainB,0x3FE6);      // B line current gain
  CommEnergyIC(WRITE, UoffsetB, 0x0005);    // B Voltage offset
  CommEnergyIC(WRITE, IoffsetB, 0x0000);    // B line current offset
  CommEnergyIC(WRITE, UgainC, 0x66D2);      //66D2  6D64C Voltage rms gain
  CommEnergyIC(WRITE, IgainC,0x3FE6);      // C line current gain
  CommEnergyIC(WRITE, UoffsetC, 0x0000);    // C Voltage offset
  CommEnergyIC(WRITE, IoffsetC, 0x0000);    // C line current offset
  CommEnergyIC(WRITE, IgainN,0x3FE6);    // C line current gain
  // CommEnergyIC(WRITE, CSThree, 0x02F6);     // Checksum 3    // Checksum 3

 // Done with the configuration
//  CommEnergyIC(WRITE, ConfigStart, 0x8765);
//  CommEnergyIC(WRITE, CalStart, 0x8765);    // 0x6886 //0x5678 //8765);
//  CommEnergyIC(WRITE, HarmStart, 0x8765);   // 0x6886 //0x5678 //8765);    
//  CommEnergyIC(WRITE, AdjStart, 0x8765);    // 0x6886 //0x5678 //8765);  

//  CommEnergyIC(WRITE, SoftReset, 0x789A);   // Perform soft reset  
 delay(1000);
 int i=(int)CommEnergyIC(READ, UoffsetA,0xFFFF);
//  Serial.println(i,HEX);

}
void ATM90E36::calibrate(unsigned int Ugaina,unsigned int Ugainb,unsigned int Ugainc, unsigned int Igaina, unsigned int Igainb, unsigned int Igainc,unsigned int mMode0 )
{  
// pinMode(energy_IRQ, INPUT); // (In development...)
 // pinMode(energy_IRQ, INPUT); // (In development...)
 pinMode(_energy_CS, OUTPUT);
 // pinMode(energy_WO, INPUT);  // (In development...)

 /* Enable SPI */
 SPI.begin();
#if defined(ENERGIA)
 SPI.setBitOrder(MSBFIRST);
 SPI.setDataMode(SPI_MODE0);
 SPI.setClockDivider(SPI_CLOCK_DIV16);
#endif

 CommEnergyIC(WRITE, SoftReset, 0x789A);   // Perform soft reset
 delay(100);
 CommEnergyIC(WRITE, FuncEn0, 0x0000);     // Voltage sag
 CommEnergyIC(WRITE, FuncEn1, 0x0000);     // Voltage sag
 CommEnergyIC(WRITE, SagTh, 0x0001);       // Voltage sag threshold

 /* SagTh = Vth * 100 * sqrt(2) / (2 * Ugain / 32768) */
 
 //Set metering config values (CONFIG)
 CommEnergyIC(WRITE, ConfigStart, 0x5678); // Metering calibration startup
 CommEnergyIC(WRITE, PLconstH, 0x0861);    // PL Constant MSB (default)
 CommEnergyIC(WRITE, PLconstL, 0xC468);    // PL Constant LSB (default)
 CommEnergyIC(WRITE, MMode0, mMode0);      // Mode Config (50 Hz, 3P3W)
 CommEnergyIC(WRITE, MMode1, 0x5555);      // 0x5555 (x2) // 0x0000 (1x)
 CommEnergyIC(WRITE, PStartTh, 0x0000);    // Active Startup Power Threshold
 CommEnergyIC(WRITE, QStartTh, 0x0000);    // Reactive Startup Power Threshold
 CommEnergyIC(WRITE, SStartTh, 0x0000);    // Apparent Startup Power Threshold
 CommEnergyIC(WRITE, PPhaseTh, 0x0000);    // Active Phase Threshold
 CommEnergyIC(WRITE, QPhaseTh, 0x0000);    // Reactive Phase Threshold
 CommEnergyIC(WRITE, SPhaseTh, 0x0000);    // Apparent  Phase Threshold
//  CommEnergyIC(WRITE, CSZero, 0x4741);      // Checksum 0
 
/*
 //Set metering calibration values (CALIBRATION)
 CommEnergyIC(WRITE, CalStart, 0x5678);    // Metering calibration startup
 CommEnergyIC(WRITE, GainA, 0x5768);       // Line calibration gain
 CommEnergyIC(WRITE, PhiA, 0X0000);        // Line calibration angle
 CommEnergyIC(WRITE, GainB, 0x5768);       // Line calibration gain
 CommEnergyIC(WRITE, PhiB, 0X0000);        // Line calibration angle
 CommEnergyIC(WRITE, GainC, 0x5768);       // Line calibration gain
 CommEnergyIC(WRITE, PhiC, 0X0000);        // Line calibration angle
 CommEnergyIC(WRITE, PoffsetA, 0x5678);    // A line active power offset
 CommEnergyIC(WRITE, QoffsetA, 0x5678);    // A line reactive power offset
 CommEnergyIC(WRITE, PoffsetB, 0x5678);    // B line active power offset
 CommEnergyIC(WRITE, QoffsetB, 0x5678);    // B line reactive power offset
 CommEnergyIC(WRITE, PoffsetC, 0x5678);    // C line active power offset
 CommEnergyIC(WRITE, QoffsetC, 0x5678);    // C line reactive power offset
//  CommEnergyIC(WRITE, CSOne, 0x0000);       // Cheksum 1
 
 //Set metering calibration values (HARMONIC)
 CommEnergyIC(WRITE, HarmStart, 0x5678);   // Metering calibration startup
 CommEnergyIC(WRITE, POffsetAF, 0x0000);   // A Fund. active power offset
 CommEnergyIC(WRITE, POffsetBF, 0x0000);   // B Fund. active power offset
 CommEnergyIC(WRITE, POffsetCF, 0x0000);   // C Fund. active power offset
 CommEnergyIC(WRITE, PGainAF, 0x0000);     // A Fund. active power gain
 CommEnergyIC(WRITE, PGainBF, 0x0000);     // B Fund. active power gain
 CommEnergyIC(WRITE, PGainCF, 0x0000);     // C Fund. active power gain
//  CommEnergyIC(WRITE, CSTwo, 0x0000);       // Checksum 2
*/
//Set metering calibration values (CALIBRATION)
  CommEnergyIC(WRITE, CalStart, 0x5678);    // Metering calibration startup 
  CommEnergyIC(WRITE, GainA, 0x0000);       // Line calibration gain
  CommEnergyIC(WRITE, PhiA, 0x0000);        // Line calibration angle
  CommEnergyIC(WRITE, GainB, 0x0000);       // Line calibration gain
  CommEnergyIC(WRITE, PhiB, 0x0000);        // Line calibration angle
  CommEnergyIC(WRITE, GainC, 0x0000);       // Line calibration gain
  CommEnergyIC(WRITE, PhiC, 0x0000);        // Line calibration angle
  CommEnergyIC(WRITE, PoffsetA, 0x0000);    // A line active power offset
  CommEnergyIC(WRITE, QoffsetA, 0x0000);    // A line reactive power offset
  CommEnergyIC(WRITE, PoffsetB, 0x0000);    // B line active power offset
  CommEnergyIC(WRITE, QoffsetB, 0x0000);    // B line reactive power offset
  CommEnergyIC(WRITE, PoffsetC, 0x0000);    // C line active power offset
  CommEnergyIC(WRITE, QoffsetC, 0x0000);    // C line reactive power offset
  // CommEnergyIC(WRITE, CSOne, 0x0000);       // Checksum 1
  
  //Set metering calibration values (HARMONIC)
  CommEnergyIC(WRITE, HarmStart, 0x5678);   // Metering calibration startup 
  CommEnergyIC(WRITE, POffsetAF, 0x0000);   // A Fund. active power offset
  CommEnergyIC(WRITE, POffsetBF, 0x0000);   // B Fund. active power offset
  CommEnergyIC(WRITE, POffsetCF, 0x0000);   // C Fund. active power offset
  CommEnergyIC(WRITE, PGainAF, 0x0001);     // A Fund. active power gain
  CommEnergyIC(WRITE, PGainBF, 0x0001);     // B Fund. active power gain
  CommEnergyIC(WRITE, PGainCF, 0x0001);     // C Fund. active power gain
  // CommEnergyIC(WRITE, CSTwo, 0x0000);       // Checksum 2 

/* //Set measurement calibration values (ADJUST)
 // CommEnergyIC(WRITE, CSThree, 0x02F6);     // Checksum 3*/
  // REPLACE 81FB FOR OTHER CTS
  CommEnergyIC(WRITE, AdjStart, 0x5678);    // Measurement calibration
  CommEnergyIC(WRITE, UgainA, Ugaina);   //678C   74BC// A SVoltage rms gain //for 100 A CT 
  CommEnergyIC(WRITE, IgainA,Igaina);      //0x2859 A line current gain//for 100 A CT 27E5
  CommEnergyIC(WRITE, UoffsetA, 0x0000);    // A Voltage offset
  CommEnergyIC(WRITE, IoffsetA, 0x0000);    // A line current offset
  CommEnergyIC(WRITE, UgainB, Ugainb);     //066D8  537E B Voltage rms gain
  CommEnergyIC(WRITE, IgainB,Igainb);      // B line current gain
  CommEnergyIC(WRITE, UoffsetB, 0x0005);    // B Voltage offset
  CommEnergyIC(WRITE, IoffsetB, 0x0000);    // B line current offset
  CommEnergyIC(WRITE, UgainC, Ugainc);      //66D2  6D64C Voltage rms gain
  CommEnergyIC(WRITE, IgainC,Igainc);      // C line current gain
  CommEnergyIC(WRITE, UoffsetC, 0x0000);    // C Voltage offset
  CommEnergyIC(WRITE, IoffsetC, 0x0000);    // C line current offset
  CommEnergyIC(WRITE, IgainN,Igainc);    // C line current gain
  // CommEnergyIC(WRITE, CSThree, 0x02F6);     // Checksum 3    // Checksum 3

 // Done with the configuration
//  CommEnergyIC(WRITE, ConfigStart, 0x8765);
//  CommEnergyIC(WRITE, CalStart, 0x8765);    // 0x6886 //0x5678 //8765);
//  CommEnergyIC(WRITE, HarmStart, 0x8765);   // 0x6886 //0x5678 //8765);    
//  CommEnergyIC(WRITE, AdjStart, 0x8765);    // 0x6886 //0x5678 //8765);  

//  CommEnergyIC(WRITE, SoftReset, 0x789A);   // Perform soft reset  
 delay(1000);
 int i=(int)CommEnergyIC(READ, UoffsetA,0xFFFF);
//  Serial.println(i,HEX);

}
