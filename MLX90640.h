#ifndef MLX90640_h
#define MLX90640_h

//MLX90640API
#include <math.h>

//I2cDriver
#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

//=================================================================================
//Try to redo that like in the original lib
#ifndef I2C_BUFFER_LENGTH
#define I2C_BUFFER_LENGTH 32 //The catch-all default is 32
#endif
//=================================================================================

//parameter struct
typedef struct {
  int16_t kVdd;
  int16_t vdd25;
  float KvPTAT;
  float KtPTAT;
  uint16_t vPTAT25;
  float alphaPTAT;
  int16_t gainEE;
  float tgc;
  float cpKv;
  float cpKta;
  uint8_t resolutionEE;
  uint8_t calibrationModeEE;
  float KsTa;
  float ksTo[4];
  int16_t ct[4];
  float alpha[768];
  int16_t offset[768];
  float kta[768];
  float kv[768];
  float cpAlpha[2];
  int16_t cpOffset[2];
  float ilChessC[3];
  uint16_t brokenPixels[5];
  uint16_t outlierPixels[5];
} parameters;

//Object class
class MLX90640 {
 public:
    
    int           SDA;                  // sda Pin
    int           SCL;                  // scl pin
    uint8_t       address = 0x33;       // Sensor address default 0x33
    float         Data[768];      // Sensor data
    int           TA_SHIFT = 8;
    parameters    Params;               // Parameters

//constructor 
    MLX90640(int sda, int scl) {
      SDA = sda;
      SCL = scl;
    };



/***************************************************************************************************************************
                                             API functions
  Todo:
   Rewrite Functions to use the object Variables instead of parameters (maybe as @deprecated) and
   implement get/set function
   implement easy function combinations like get Frame
   implement documentation
 ***************************************************************************************************************************/

//experimental not implemented yet
signed char setNewaddressess(uint8_t newAddress) {
  //1.Power on the device
  //2.Write 0X000 (i.e. erase the cell) to 0X240F
  //3.Wait 10ms (so the write process is complete)
  //4.read back 0X240F to make sure the erase process has been successful
  //5.Write the new value for instance 0XBE01 (make sure the BE remain intact and 01 is the new slave address)
  //6.Wait 10ms (so the write process is complete)
  //7.read back 0X240F to make sure the write process has been successful
  //8.Power off the device

  //Implement update the object Address when used and Failsafe?
  return 0;
}


/***************************************************************************************************************************
                                   Implementation of the Functions
****************************************************************************************************************************/
int DumpEE( uint16_t *eeData) {
/*
This function reads all the necessary EEPROM data from a MLX90640 device with a given slave address into a
MCU memory location defined by the user. The allocated memory should be at least 832 words for proper
operation. If the result is -1, NACK occurred during the communication and this is not a valid EEPROM data.

 uint16_t *eeData – pointer to the MCU memory location where the user wants the EEPROM data to be stored
*/
  return I2CRead( 0x2400, 832, eeData);
}
//------------------------------------------------------------------------------
int GetFrameData( uint16_t *frameData) {
/*  
This function reads all the necessary frame data from the sensor into a MCU memory location defined by the user. The allocated memory should be at least 834 words for proper
operation. If the result is -1, NACK occurred during the communication and this is not a valid EEPROM data,
else if the result is -8, the data could not be acquired for a certain time – the most probable reason is that
the I2C frequency is too low, else the result is the sub-page of the acquired data.

 uint16_t *frameData – pointer to the MCU memory location where the user wants the frame data to be stored 
example: static uint16_t mlx90640Frame[834];*/
  
  uint16_t dataReady = 1;
  uint16_t controlRegister1;
  uint16_t statusRegister;
  int error = 1;
  uint8_t cnt = 0;

  dataReady = 0;
  while (dataReady == 0)
  {
    error = I2CRead(0x8000, 1, &statusRegister);
    if (error != 0)
    {
      return error;
    }
    dataReady = statusRegister & 0x0008;
  }

  while (dataReady != 0 && cnt < 5)
  {
    error = I2CWrite( 0x8000, 0x0030);
    if (error == -1)
    {
      return error;
    }

    error = I2CRead( 0x0400, 832, frameData);
    if (error != 0)
    {
      return error;
    }

    error = I2CRead( 0x8000, 1, &statusRegister);
    if (error != 0)
    {
      return error;
    }
    dataReady = statusRegister & 0x0008;
    cnt = cnt + 1;
  }

  if (cnt > 4)
  {
    return -8;
  }

  error = I2CRead( 0x800D, 1, &controlRegister1);
  frameData[832] = controlRegister1;
  frameData[833] = statusRegister & 0x0001;

  if (error != 0)
  {
    return error;
  }

  return frameData[833];
}
//------------------------------------------------------------------------------
int ExtractParameters(uint16_t *eeData) {
  int error = CheckEEPROMValid(eeData);

  if (error == 0)
  {
    ExtractVDDParameters(eeData);
    ExtractPTATParameters(eeData);
    ExtractGainParameters(eeData);
    ExtractTgcParameters(eeData);
    ExtractResolutionParameters(eeData);
    ExtractKsTaParameters(eeData);
    ExtractKsToParameters(eeData);
    ExtractAlphaParameters(eeData);
    ExtractOffsetParameters(eeData);
    ExtractKtaPixelParameters(eeData);
    ExtractKvPixelParameters(eeData);
    ExtractCPParameters(eeData);
    ExtractCILCParameters(eeData);
    error = ExtractDeviatingPixels(eeData);
  }

  return error;

}
//------------------------------------------------------------------------------
int SetResolution( uint8_t resolution) {
  uint16_t controlRegister1;
  int value;
  int error;

  value = (resolution & 0x03) << 10;

  error = I2CRead( 0x800D, 1, &controlRegister1);

  if (error == 0)
  {
    value = (controlRegister1 & 0xF3FF) | value;
    error = I2CWrite(0x800D, value);
  }

  return error;
}
//------------------------------------------------------------------------------
int GetCurResolution() {
  uint16_t controlRegister1;
  int resolutionRAM;
  int error;

  error = I2CRead( 0x800D, 1, &controlRegister1);
  if (error != 0)
  {
    return error;
  }
  resolutionRAM = (controlRegister1 & 0x0C00) >> 10;

  return resolutionRAM;
}
//------------------------------------------------------------------------------
int SetRefreshRate( uint8_t refreshRate) {
  uint16_t controlRegister1;
  int value;
  int error;

  value = (refreshRate & 0x07) << 7;

  error = I2CRead( 0x800D, 1, &controlRegister1);
  if (error == 0)
  {
    value = (controlRegister1 & 0xFC7F) | value;
    error = I2CWrite( 0x800D, value);
  }

  return error;
}
//------------------------------------------------------------------------------
int GetRefreshRate() {
  uint16_t controlRegister1;
  int refreshRate;
  int error;

  error = I2CRead( 0x800D, 1, &controlRegister1);
  if (error != 0)
  {
    return error;
  }
  refreshRate = (controlRegister1 & 0x0380) >> 7;

  return refreshRate;
}
//------------------------------------------------------------------------------
int SetInterleavedMode() {
  uint16_t controlRegister1;
  int value;
  int error;

  error = I2CRead( 0x800D, 1, &controlRegister1);

  if (error == 0)
  {
    value = (controlRegister1 & 0xEFFF);
    error = I2CWrite( 0x800D, value);
  }

  return error;
}
//------------------------------------------------------------------------------
int SetChessMode() {
  uint16_t controlRegister1;
  int value;
  int error;

  error = I2CRead(0x800D, 1, &controlRegister1);

  if (error == 0)
  {
    value = (controlRegister1 | 0x1000);
    error = I2CWrite( 0x800D, value);
  }

  return error;
}
//------------------------------------------------------------------------------
int GetCurMode() {
  uint16_t controlRegister1;
  int modeRAM;
  int error;

  error = I2CRead( 0x800D, 1, &controlRegister1);
  if (error != 0)
  {
    return error;
  }
  modeRAM = (controlRegister1 & 0x1000) >> 12;

  return modeRAM;
}
//------------------------------------------------------------------------------
void CalculateTo(uint16_t *frameData, float emissivity, float tr) {
  float vdd;
  float ta;
  float ta4;
  float tr4;
  float taTr;
  float gain;
  float irDataCP[2];
  float irData;
  float alphaCompensated;
  uint8_t mode;
  int8_t ilPattern;
  int8_t chessPattern;
  int8_t pattern;
  int8_t conversionPattern;
  float Sx;
  float To;
  float alphaCorrR[4];
  int8_t range;
  uint16_t subPage;

  subPage = frameData[833];
  vdd = GetVdd(frameData);
  ta = GetTa(frameData);
  ta4 = pow((ta + 273.15), (double)4);
  tr4 = pow((tr + 273.15), (double)4);
  taTr = tr4 - (tr4 - ta4) / emissivity;

  alphaCorrR[0] = 1 / (1 + Params.ksTo[0] * 40);
  alphaCorrR[1] = 1 ;
  alphaCorrR[2] = (1 + Params.ksTo[2] * Params.ct[2]);
  alphaCorrR[3] = alphaCorrR[2] * (1 + Params.ksTo[3] * (Params.ct[3] - Params.ct[2]));

  //------------------------- Gain calculation -----------------------------------
  gain = frameData[778];
  if (gain > 32767)
  {
    gain = gain - 65536;
  }

  gain = Params.gainEE / gain;

  //------------------------- To calculation -------------------------------------
  mode = (frameData[832] & 0x1000) >> 5;

  irDataCP[0] = frameData[776];
  irDataCP[1] = frameData[808];
  for ( int i = 0; i < 2; i++)
  {
    if (irDataCP[i] > 32767)
    {
      irDataCP[i] = irDataCP[i] - 65536;
    }
    irDataCP[i] = irDataCP[i] * gain;
  }
  irDataCP[0] = irDataCP[0] - Params.cpOffset[0] * (1 + Params.cpKta * (ta - 25)) * (1 + Params.cpKv * (vdd - 3.3));
  if ( mode ==  Params.calibrationModeEE)
  {
    irDataCP[1] = irDataCP[1] - Params.cpOffset[1] * (1 + Params.cpKta * (ta - 25)) * (1 + Params.cpKv * (vdd - 3.3));
  }
  else
  {
    irDataCP[1] = irDataCP[1] - (Params.cpOffset[1] + Params.ilChessC[0]) * (1 + Params.cpKta * (ta - 25)) * (1 + Params.cpKv * (vdd - 3.3));
  }

  for ( int pixelNumber = 0; pixelNumber < 768; pixelNumber++)
  {
    ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2;
    chessPattern = ilPattern ^ (pixelNumber - (pixelNumber / 2) * 2);
    conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) * (1 - 2 * ilPattern);

    if (mode == 0)
    {
      pattern = ilPattern;
    }
    else
    {
      pattern = chessPattern;
    }

    if (pattern == frameData[833])
    {
      irData = frameData[pixelNumber];
      if (irData > 32767)
      {
        irData = irData - 65536;
      }
      irData = irData * gain;

      irData = irData - Params.offset[pixelNumber] * (1 + Params.kta[pixelNumber] * (ta - 25)) * (1 + Params.kv[pixelNumber] * (vdd - 3.3));
      if (mode !=  Params.calibrationModeEE)
      {
        irData = irData + Params.ilChessC[2] * (2 * ilPattern - 1) - Params.ilChessC[1] * conversionPattern;
      }

      irData = irData / emissivity;

      irData = irData - Params.tgc * irDataCP[subPage];

      alphaCompensated = (Params.alpha[pixelNumber] - Params.tgc * Params.cpAlpha[subPage]) * (1 + Params.KsTa * (ta - 25));

      Sx = pow((double)alphaCompensated, (double)3) * (irData + alphaCompensated * taTr);
      Sx = sqrt(sqrt(Sx)) * Params.ksTo[1];

      To = sqrt(sqrt(irData / (alphaCompensated * (1 - Params.ksTo[1] * 273.15) + Sx) + taTr)) - 273.15;

      if (To < Params.ct[1])
      {
        range = 0;
      }
      else if (To < Params.ct[2])
      {
        range = 1;
      }
      else if (To < Params.ct[3])
      {
        range = 2;
      }
      else
      {
        range = 3;
      }

      To = sqrt(sqrt(irData / (alphaCompensated * alphaCorrR[range] * (1 + Params.ksTo[range] * (To - Params.ct[range]))) + taTr)) - 273.15;

      Data[pixelNumber] = To;
    }
  }
}
//------------------------------------------------------------------------------
void GetImage(uint16_t *frameData) {
  float vdd;
  float ta;
  float gain;
  float irDataCP[2];
  float irData;
  float alphaCompensated;
  uint8_t mode;
  int8_t ilPattern;
  int8_t chessPattern;
  int8_t pattern;
  int8_t conversionPattern;
  float image;
  uint16_t subPage;

  subPage = frameData[833];
  vdd = GetVdd(frameData);
  ta = GetTa(frameData);

  //------------------------- Gain calculation -----------------------------------
  gain = frameData[778];
  if (gain > 32767)
  {
    gain = gain - 65536;
  }

  gain = Params.gainEE / gain;

  //------------------------- Image calculation -------------------------------------
  mode = (frameData[832] & 0x1000) >> 5;

  irDataCP[0] = frameData[776];
  irDataCP[1] = frameData[808];
  for ( int i = 0; i < 2; i++)
  {
    if (irDataCP[i] > 32767)
    {
      irDataCP[i] = irDataCP[i] - 65536;
    }
    irDataCP[i] = irDataCP[i] * gain;
  }
  irDataCP[0] = irDataCP[0] - Params.cpOffset[0] * (1 + Params.cpKta * (ta - 25)) * (1 + Params.cpKv * (vdd - 3.3));
  if ( mode ==  Params.calibrationModeEE)
  {
    irDataCP[1] = irDataCP[1] - Params.cpOffset[1] * (1 + Params.cpKta * (ta - 25)) * (1 + Params.cpKv * (vdd - 3.3));
  }
  else
  {
    irDataCP[1] = irDataCP[1] - (Params.cpOffset[1] + Params.ilChessC[0]) * (1 + Params.cpKta * (ta - 25)) * (1 + Params.cpKv * (vdd - 3.3));
  }

  for ( int pixelNumber = 0; pixelNumber < 768; pixelNumber++)
  {
    ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2;
    chessPattern = ilPattern ^ (pixelNumber - (pixelNumber / 2) * 2);
    conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) * (1 - 2 * ilPattern);

    if (mode == 0)
    {
      pattern = ilPattern;
    }
    else
    {
      pattern = chessPattern;
    }

    if (pattern == frameData[833])
    {
      irData = frameData[pixelNumber];
      if (irData > 32767)
      {
        irData = irData - 65536;
      }
      irData = irData * gain;

      irData = irData - Params.offset[pixelNumber] * (1 + Params.kta[pixelNumber] * (ta - 25)) * (1 + Params.kv[pixelNumber] * (vdd - 3.3));
      if (mode !=  Params.calibrationModeEE)
      {
        irData = irData + Params.ilChessC[2] * (2 * ilPattern - 1) - Params.ilChessC[1] * conversionPattern;
      }

      irData = irData - Params.tgc * irDataCP[subPage];

      alphaCompensated = (Params.alpha[pixelNumber] - Params.tgc * Params.cpAlpha[subPage]) * (1 + Params.KsTa * (ta - 25));

      image = irData / alphaCompensated;

      Data[pixelNumber] = image;
    }
  }
}
//------------------------------------------------------------------------------
float GetVdd(uint16_t *frameData) {
  float vdd;
  float resolutionCorrection;

  int resolutionRAM;

  vdd = frameData[810];
  if (vdd > 32767)
  {
    vdd = vdd - 65536;
  }
  resolutionRAM = (frameData[832] & 0x0C00) >> 10;
  resolutionCorrection = pow(2, (double)Params.resolutionEE) / pow(2, (double)resolutionRAM);
  vdd = (resolutionCorrection * vdd - Params.vdd25) / Params.kVdd + 3.3;

  return vdd;
}
//------------------------------------------------------------------------------
float GetTa(uint16_t *frameData) {
  float ptat;
  float ptatArt;
  float vdd;
  float ta;

  vdd = GetVdd(frameData);

  ptat = frameData[800];
  if (ptat > 32767)
  {
    ptat = ptat - 65536;
  }

  ptatArt = frameData[768];
  if (ptatArt > 32767)
  {
    ptatArt = ptatArt - 65536;
  }
  ptatArt = (ptat / (ptat * Params.alphaPTAT + ptatArt)) * pow(2, (double)18);

  ta = (ptatArt / (1 + Params.KvPTAT * (vdd - 3.3)) - Params.vPTAT25);
  ta = ta / Params.KtPTAT + 25;

  return ta;
}
//------------------------------------------------------------------------------
int GetSubPageNumber(uint16_t *frameData) {
  return frameData[833];

}


/***************************************************************************************************************************
                                             API functions
  Todo:
   Rewrite Functions to use the object Variables instead of parameters (maybe as @deprecated) and
   implement get/set function
   implement documentation
 ***************************************************************************************************************************/
void ExtractVDDParameters(uint16_t *eeData) {
  int16_t kVdd;
  int16_t vdd25;

  kVdd = eeData[51];

  kVdd = (eeData[51] & 0xFF00) >> 8;
  if (kVdd > 127)
  {
    kVdd = kVdd - 256;
  }
  kVdd = 32 * kVdd;
  vdd25 = eeData[51] & 0x00FF;
  vdd25 = ((vdd25 - 256) << 5) - 8192;

  Params.kVdd = kVdd;
  Params.vdd25 = vdd25;
}
//------------------------------------------------------------------------------
void ExtractPTATParameters(uint16_t *eeData) {
  float KvPTAT;
  float KtPTAT;
  int16_t vPTAT25;
  float alphaPTAT;

  KvPTAT = (eeData[50] & 0xFC00) >> 10;
  if (KvPTAT > 31)
  {
    KvPTAT = KvPTAT - 64;
  }
  KvPTAT = KvPTAT / 4096;

  KtPTAT = eeData[50] & 0x03FF;
  if (KtPTAT > 511)
  {
    KtPTAT = KtPTAT - 1024;
  }
  KtPTAT = KtPTAT / 8;

  vPTAT25 = eeData[49];

  alphaPTAT = (eeData[16] & 0xF000) / pow(2, (double)14) + 8.0f;

  Params.KvPTAT = KvPTAT;
  Params.KtPTAT = KtPTAT;
  Params.vPTAT25 = vPTAT25;
  Params.alphaPTAT = alphaPTAT;
}
//------------------------------------------------------------------------------
void ExtractGainParameters(uint16_t *eeData) {
  int16_t gainEE;

  gainEE = eeData[48];
  if (gainEE > 32767)
  {
    gainEE = gainEE - 65536;
  }

  Params.gainEE = gainEE;
}
//------------------------------------------------------------------------------
void ExtractTgcParameters(uint16_t *eeData) {
  float tgc;
  tgc = eeData[60] & 0x00FF;
  if (tgc > 127)
  {
    tgc = tgc - 256;
  }
  tgc = tgc / 32.0f;

  Params.tgc = tgc;
}
//------------------------------------------------------------------------------
void ExtractResolutionParameters(uint16_t *eeData) {
  uint8_t resolutionEE;
  resolutionEE = (eeData[56] & 0x3000) >> 12;

  Params.resolutionEE = resolutionEE;
}
//------------------------------------------------------------------------------
void ExtractKsTaParameters(uint16_t *eeData) {
  float KsTa;
  KsTa = (eeData[60] & 0xFF00) >> 8;
  if (KsTa > 127)
  {
    KsTa = KsTa - 256;
  }
  KsTa = KsTa / 8192.0f;

  Params.KsTa = KsTa;
}
//------------------------------------------------------------------------------
void ExtractKsToParameters(uint16_t *eeData) {
  int KsToScale;
  int8_t step;

  step = ((eeData[63] & 0x3000) >> 12) * 10;

  Params.ct[0] = -40;
  Params.ct[1] = 0;
  Params.ct[2] = (eeData[63] & 0x00F0) >> 4;
  Params.ct[3] = (eeData[63] & 0x0F00) >> 8;

  Params.ct[2] = Params.ct[2] * step;
  Params.ct[3] = Params.ct[2] + Params.ct[3] * step;

  KsToScale = (eeData[63] & 0x000F) + 8;
  KsToScale = 1 << KsToScale;

  Params.ksTo[0] = eeData[61] & 0x00FF;
  Params.ksTo[1] = (eeData[61] & 0xFF00) >> 8;
  Params.ksTo[2] = eeData[62] & 0x00FF;
  Params.ksTo[3] = (eeData[62] & 0xFF00) >> 8;


  for (int i = 0; i < 4; i++)
  {
    if (Params.ksTo[i] > 127)
    {
      Params.ksTo[i] = Params.ksTo[i] - 256;
    }
    Params.ksTo[i] = Params.ksTo[i] / KsToScale;
  }
}
//------------------------------------------------------------------------------
void ExtractAlphaParameters(uint16_t *eeData) {
  int accRow[24];
  int accColumn[32];
  int p = 0;
  int alphaRef;
  uint8_t alphaScale;
  uint8_t accRowScale;
  uint8_t accColumnScale;
  uint8_t accRemScale;


  accRemScale = eeData[32] & 0x000F;
  accColumnScale = (eeData[32] & 0x00F0) >> 4;
  accRowScale = (eeData[32] & 0x0F00) >> 8;
  alphaScale = ((eeData[32] & 0xF000) >> 12) + 30;
  alphaRef = eeData[33];

  for (int i = 0; i < 6; i++)
  {
    p = i * 4;
    accRow[p + 0] = (eeData[34 + i] & 0x000F);
    accRow[p + 1] = (eeData[34 + i] & 0x00F0) >> 4;
    accRow[p + 2] = (eeData[34 + i] & 0x0F00) >> 8;
    accRow[p + 3] = (eeData[34 + i] & 0xF000) >> 12;
  }

  for (int i = 0; i < 24; i++)
  {
    if (accRow[i] > 7)
    {
      accRow[i] = accRow[i] - 16;
    }
  }

  for (int i = 0; i < 8; i++)
  {
    p = i * 4;
    accColumn[p + 0] = (eeData[40 + i] & 0x000F);
    accColumn[p + 1] = (eeData[40 + i] & 0x00F0) >> 4;
    accColumn[p + 2] = (eeData[40 + i] & 0x0F00) >> 8;
    accColumn[p + 3] = (eeData[40 + i] & 0xF000) >> 12;
  }

  for (int i = 0; i < 32; i ++)
  {
    if (accColumn[i] > 7)
    {
      accColumn[i] = accColumn[i] - 16;
    }
  }

  for (int i = 0; i < 24; i++)
  {
    for (int j = 0; j < 32; j ++)
    {
      p = 32 * i + j;
      Params.alpha[p] = (eeData[64 + p] & 0x03F0) >> 4;
      if (Params.alpha[p] > 31)
      {
        Params.alpha[p] = Params.alpha[p] - 64;
      }
      Params.alpha[p] = Params.alpha[p] * (1 << accRemScale);
      Params.alpha[p] = (alphaRef + (accRow[i] << accRowScale) + (accColumn[j] << accColumnScale) + Params.alpha[p]);
      Params.alpha[p] = Params.alpha[p] / pow(2, (double)alphaScale);
    }
  }
}
//------------------------------------------------------------------------------
void ExtractOffsetParameters(uint16_t *eeData) {
  int occRow[24];
  int occColumn[32];
  int p = 0;
  int16_t offsetRef;
  uint8_t occRowScale;
  uint8_t occColumnScale;
  uint8_t occRemScale;


  occRemScale = (eeData[16] & 0x000F);
  occColumnScale = (eeData[16] & 0x00F0) >> 4;
  occRowScale = (eeData[16] & 0x0F00) >> 8;
  offsetRef = eeData[17];
  if (offsetRef > 32767)
  {
    offsetRef = offsetRef - 65536;
  }

  for (int i = 0; i < 6; i++)
  {
    p = i * 4;
    occRow[p + 0] = (eeData[18 + i] & 0x000F);
    occRow[p + 1] = (eeData[18 + i] & 0x00F0) >> 4;
    occRow[p + 2] = (eeData[18 + i] & 0x0F00) >> 8;
    occRow[p + 3] = (eeData[18 + i] & 0xF000) >> 12;
  }

  for (int i = 0; i < 24; i++)
  {
    if (occRow[i] > 7)
    {
      occRow[i] = occRow[i] - 16;
    }
  }

  for (int i = 0; i < 8; i++)
  {
    p = i * 4;
    occColumn[p + 0] = (eeData[24 + i] & 0x000F);
    occColumn[p + 1] = (eeData[24 + i] & 0x00F0) >> 4;
    occColumn[p + 2] = (eeData[24 + i] & 0x0F00) >> 8;
    occColumn[p + 3] = (eeData[24 + i] & 0xF000) >> 12;
  }

  for (int i = 0; i < 32; i ++)
  {
    if (occColumn[i] > 7)
    {
      occColumn[i] = occColumn[i] - 16;
    }
  }

  for (int i = 0; i < 24; i++)
  {
    for (int j = 0; j < 32; j ++)
    {
      p = 32 * i + j;
      Params.offset[p] = (eeData[64 + p] & 0xFC00) >> 10;
      if (Params.offset[p] > 31)
      {
        Params.offset[p] = Params.offset[p] - 64;
      }
      Params.offset[p] = Params.offset[p] * (1 << occRemScale);
      Params.offset[p] = (offsetRef + (occRow[i] << occRowScale) + (occColumn[j] << occColumnScale) + Params.offset[p]);
    }
  }
}
//------------------------------------------------------------------------------
void ExtractKtaPixelParameters(uint16_t *eeData) {
  int p = 0;
  int8_t KtaRC[4];
  int8_t KtaRoCo;
  int8_t KtaRoCe;
  int8_t KtaReCo;
  int8_t KtaReCe;
  uint8_t ktaScale1;
  uint8_t ktaScale2;
  uint8_t split;

  KtaRoCo = (eeData[54] & 0xFF00) >> 8;
  if (KtaRoCo > 127)
  {
    KtaRoCo = KtaRoCo - 256;
  }
  KtaRC[0] = KtaRoCo;

  KtaReCo = (eeData[54] & 0x00FF);
  if (KtaReCo > 127)
  {
    KtaReCo = KtaReCo - 256;
  }
  KtaRC[2] = KtaReCo;

  KtaRoCe = (eeData[55] & 0xFF00) >> 8;
  if (KtaRoCe > 127)
  {
    KtaRoCe = KtaRoCe - 256;
  }
  KtaRC[1] = KtaRoCe;

  KtaReCe = (eeData[55] & 0x00FF);
  if (KtaReCe > 127)
  {
    KtaReCe = KtaReCe - 256;
  }
  KtaRC[3] = KtaReCe;

  ktaScale1 = ((eeData[56] & 0x00F0) >> 4) + 8;
  ktaScale2 = (eeData[56] & 0x000F);

  for (int i = 0; i < 24; i++)
  {
    for (int j = 0; j < 32; j ++)
    {
      p = 32 * i + j;
      split = 2 * (p / 32 - (p / 64) * 2) + p % 2;
      Params.kta[p] = (eeData[64 + p] & 0x000E) >> 1;
      if (Params.kta[p] > 3)
      {
        Params.kta[p] = Params.kta[p] - 8;
      }
      Params.kta[p] = Params.kta[p] * (1 << ktaScale2);
      Params.kta[p] = KtaRC[split] + Params.kta[p];
      Params.kta[p] = Params.kta[p] / pow(2, (double)ktaScale1);
    }
  }
}
//------------------------------------------------------------------------------
void ExtractKvPixelParameters(uint16_t *eeData) {
  int p = 0;
  int8_t KvT[4];
  int8_t KvRoCo;
  int8_t KvRoCe;
  int8_t KvReCo;
  int8_t KvReCe;
  uint8_t kvScale;
  uint8_t split;

  KvRoCo = (eeData[52] & 0xF000) >> 12;
  if (KvRoCo > 7)
  {
    KvRoCo = KvRoCo - 16;
  }
  KvT[0] = KvRoCo;

  KvReCo = (eeData[52] & 0x0F00) >> 8;
  if (KvReCo > 7)
  {
    KvReCo = KvReCo - 16;
  }
  KvT[2] = KvReCo;

  KvRoCe = (eeData[52] & 0x00F0) >> 4;
  if (KvRoCe > 7)
  {
    KvRoCe = KvRoCe - 16;
  }
  KvT[1] = KvRoCe;

  KvReCe = (eeData[52] & 0x000F);
  if (KvReCe > 7)
  {
    KvReCe = KvReCe - 16;
  }
  KvT[3] = KvReCe;

  kvScale = (eeData[56] & 0x0F00) >> 8;


  for (int i = 0; i < 24; i++)
  {
    for (int j = 0; j < 32; j ++)
    {
      p = 32 * i + j;
      split = 2 * (p / 32 - (p / 64) * 2) + p % 2;
      Params.kv[p] = KvT[split];
      Params.kv[p] = Params.kv[p] / pow(2, (double)kvScale);
    }
  }
}
//------------------------------------------------------------------------------
void ExtractCPParameters(uint16_t *eeData) {
  float alphaSP[2];
  int16_t offsetSP[2];
  float cpKv;
  float cpKta;
  uint8_t alphaScale;
  uint8_t ktaScale1;
  uint8_t kvScale;

  alphaScale = ((eeData[32] & 0xF000) >> 12) + 27;

  offsetSP[0] = (eeData[58] & 0x03FF);
  if (offsetSP[0] > 511)
  {
    offsetSP[0] = offsetSP[0] - 1024;
  }

  offsetSP[1] = (eeData[58] & 0xFC00) >> 10;
  if (offsetSP[1] > 31)
  {
    offsetSP[1] = offsetSP[1] - 64;
  }
  offsetSP[1] = offsetSP[1] + offsetSP[0];

  alphaSP[0] = (eeData[57] & 0x03FF);
  if (alphaSP[0] > 511)
  {
    alphaSP[0] = alphaSP[0] - 1024;
  }
  alphaSP[0] = alphaSP[0] /  pow(2, (double)alphaScale);

  alphaSP[1] = (eeData[57] & 0xFC00) >> 10;
  if (alphaSP[1] > 31)
  {
    alphaSP[1] = alphaSP[1] - 64;
  }
  alphaSP[1] = (1 + alphaSP[1] / 128) * alphaSP[0];

  cpKta = (eeData[59] & 0x00FF);
  if (cpKta > 127)
  {
    cpKta = cpKta - 256;
  }
  ktaScale1 = ((eeData[56] & 0x00F0) >> 4) + 8;
  Params.cpKta = cpKta / pow(2, (double)ktaScale1);

  cpKv = (eeData[59] & 0xFF00) >> 8;
  if (cpKv > 127)
  {
    cpKv = cpKv - 256;
  }
  kvScale = (eeData[56] & 0x0F00) >> 8;
  Params.cpKv = cpKv / pow(2, (double)kvScale);

  Params.cpAlpha[0] = alphaSP[0];
  Params.cpAlpha[1] = alphaSP[1];
  Params.cpOffset[0] = offsetSP[0];
  Params.cpOffset[1] = offsetSP[1];
}
//------------------------------------------------------------------------------
void ExtractCILCParameters(uint16_t *eeData) {
  float ilChessC[3];
  uint8_t calibrationModeEE;

  calibrationModeEE = (eeData[10] & 0x0800) >> 4;
  calibrationModeEE = calibrationModeEE ^ 0x80;

  ilChessC[0] = (eeData[53] & 0x003F);
  if (ilChessC[0] > 31)
  {
    ilChessC[0] = ilChessC[0] - 64;
  }
  ilChessC[0] = ilChessC[0] / 16.0f;

  ilChessC[1] = (eeData[53] & 0x07C0) >> 6;
  if (ilChessC[1] > 15)
  {
    ilChessC[1] = ilChessC[1] - 32;
  }
  ilChessC[1] = ilChessC[1] / 2.0f;

  ilChessC[2] = (eeData[53] & 0xF800) >> 11;
  if (ilChessC[2] > 15)
  {
    ilChessC[2] = ilChessC[2] - 32;
  }
  ilChessC[2] = ilChessC[2] / 8.0f;

  Params.calibrationModeEE = calibrationModeEE;
  Params.ilChessC[0] = ilChessC[0];
  Params.ilChessC[1] = ilChessC[1];
  Params.ilChessC[2] = ilChessC[2];
}
//------------------------------------------------------------------------------
int ExtractDeviatingPixels(uint16_t *eeData) {
  uint16_t pixCnt = 0;
  uint16_t brokenPixCnt = 0;
  uint16_t outlierPixCnt = 0;
  int warn = 0;
  int i;

  for (pixCnt = 0; pixCnt < 5; pixCnt++)
  {
    Params.brokenPixels[pixCnt] = 0xFFFF;
    Params.outlierPixels[pixCnt] = 0xFFFF;
  }

  pixCnt = 0;
  while (pixCnt < 768 && brokenPixCnt < 5 && outlierPixCnt < 5)
  {
    if (eeData[pixCnt + 64] == 0)
    {
      Params.brokenPixels[brokenPixCnt] = pixCnt;
      brokenPixCnt = brokenPixCnt + 1;
    }
    else if ((eeData[pixCnt + 64] & 0x0001) != 0)
    {
      Params.outlierPixels[outlierPixCnt] = pixCnt;
      outlierPixCnt = outlierPixCnt + 1;
    }

    pixCnt = pixCnt + 1;

  }

  if (brokenPixCnt > 4)
  {
    warn = -3;
  }
  else if (outlierPixCnt > 4)
  {
    warn = -4;
  }
  else if ((brokenPixCnt + outlierPixCnt) > 4)
  {
    warn = -5;
  }
  else
  {
    for (pixCnt = 0; pixCnt < brokenPixCnt; pixCnt++)
    {
      for (i = pixCnt + 1; i < brokenPixCnt; i++)
      {
        warn = CheckAdjacentPixels(Params.brokenPixels[pixCnt], Params.brokenPixels[i]);
        if (warn != 0)
        {
          return warn;
        }
      }
    }

    for (pixCnt = 0; pixCnt < outlierPixCnt; pixCnt++)
    {
      for (i = pixCnt + 1; i < outlierPixCnt; i++)
      {
        warn = CheckAdjacentPixels(Params.outlierPixels[pixCnt], Params.outlierPixels[i]);
        if (warn != 0)
        {
          return warn;
        }
      }
    }

    for (pixCnt = 0; pixCnt < brokenPixCnt; pixCnt++)
    {
      for (i = 0; i < outlierPixCnt; i++)
      {
        warn = CheckAdjacentPixels(Params.brokenPixels[pixCnt], Params.outlierPixels[i]);
        if (warn != 0)
        {
          return warn;
        }
      }
    }

  }


  return warn;

}
//------------------------------------------------------------------------------
int CheckAdjacentPixels(uint16_t pix1, uint16_t pix2) {
  int pixPosDif;

  pixPosDif = pix1 - pix2;
  if (pixPosDif > -34 && pixPosDif < -30)
  {
    return -6;
  }
  if (pixPosDif > -2 && pixPosDif < 2)
  {
    return -6;
  }
  if (pixPosDif > 30 && pixPosDif < 34)
  {
    return -6;
  }

  return 0;
}
//------------------------------------------------------------------------------
int CheckEEPROMValid(uint16_t *eeData) {
  int deviceSelect;
  deviceSelect = eeData[10] & 0x0040;
  if (deviceSelect == 0)
  {
    return 0;
  }

  return -7;
}

boolean isConnected(void) {
  Wire.beginTransmission(address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  else
    return (true);// true if Sensor is detected at Address
}

void begin(int freq) {

      if (freq < 0)I2CInit();
      else I2CInit(freq);

      // Connect to sensor
      if (isConnected() == false)
      {
        Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
        while (1);
      }
      Serial.println("MLX90640 online!");

      //Get device parameters - We only have to do this once
      int status = 1 ;
      uint16_t eeMLX90640[832];
      while (status != 0) {                     //Added
        status = DumpEE( eeMLX90640);
        if (status != 0)
          Serial.println("Failed to load system parameters");

        status = ExtractParameters(eeMLX90640);
        if (status != 0)
          Serial.println("Parameter extraction failed");

        //Once params are extracted, we can release eeMLX90640 array
      }
    }

/******************************************************************************************************
                                  API I2C Moved Here
    Todo:
    Implement I2CInit
    Rewrite Functions to use the object Variables instead of parameters (maybe as @deprecated)
 ******************************************************************************************************/
//Read a number of words from startAddress. Store into Data array.
//Returns 0 if successful, -1 if error
int I2CRead( unsigned int startAddress, unsigned int nWordsRead, uint16_t *data) {

  //Caller passes number of 'unsigned ints to read', increase this to 'bytes to read'
  uint16_t bytesRemaining = nWordsRead * 2;

  //It doesn't look like sequential read works. Do we need to re-issue the address command each time?

  uint16_t dataSpot = 0; //Start at beginning of array

  //Setup a series of chunked I2C_BUFFER_LENGTH byte reads
  while (bytesRemaining > 0)
  {
    Wire.beginTransmission(address);
    Wire.write(startAddress >> 8); //MSB
    Wire.write(startAddress & 0xFF); //LSB
    if (Wire.endTransmission(false) != 0) //Do not release bus
    {
      Serial.println("No ack read");
      return (0); //Sensor did not ACK
    }

    uint16_t numberOfBytesToRead = bytesRemaining;
    if (numberOfBytesToRead > I2C_BUFFER_LENGTH) numberOfBytesToRead = I2C_BUFFER_LENGTH;

    Wire.requestFrom((uint8_t)address, numberOfBytesToRead);
    if (Wire.available())
    {
      for (uint16_t x = 0 ; x < numberOfBytesToRead / 2; x++)
      {
        //Store data into array
        data[dataSpot] = Wire.read() << 8; //MSB
        data[dataSpot] |= Wire.read(); //LSB

        dataSpot++;
      }
    }

    bytesRemaining -= numberOfBytesToRead;

    startAddress += numberOfBytesToRead / 2;
  }

  return (0); //Success
}

//Set I2C Freq, in kHz
//I2CFreqSet(1000) sets frequency to 1MHz
void I2CFreqSet(int freq) {
  Wire.setClock((long)1000 * freq);
}

//Write two bytes to a two byte address
int I2CWrite(unsigned int writeAddress, uint16_t data) {
  Wire.beginTransmission((uint8_t)address);
  Wire.write(writeAddress >> 8); //MSB
  Wire.write(writeAddress & 0xFF); //LSB
  Wire.write(data >> 8); //MSB
  Wire.write(data & 0xFF); //LSB
  if (Wire.endTransmission() != 0)
  {
    //Sensor did not ACK
    Serial.println("Error: Sensor did not ack");
    return (-1);
  }

  uint16_t dataCheck;
  I2CRead( writeAddress, 1, &dataCheck);
  if (dataCheck != data)
  {
    //Serial.println("The write request didn't stick");
    return -2;
  }

  return (0); //Success
}

void I2CInit() {
  Wire.begin(SDA, SCL);
}
void I2CInit(int freq) {
  I2CFreqSet(freq);
  Wire.begin(SDA, SCL);

}

void getTemp() {                   // stores Frame in Data
  for (byte x = 0 ; x < 2 ; x++) //Read both subpages
  {
    uint16_t mlx90640Frame[834];
    int status = GetFrameData( mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = GetVdd(mlx90640Frame);
    float Ta = GetTa(mlx90640Frame);

    float tr = Ta - TA_SHIFT; //Reflected Dataerature based on the sensor ambient Dataerature
    float emissivity = 0.95;
    CalculateTo(mlx90640Frame, emissivity, tr);
  }
}
void setTASHIFT(int TA_shift){
  TA_SHIFT = TA_shift;
}

};


#endif
