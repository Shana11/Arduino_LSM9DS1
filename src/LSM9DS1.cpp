/*
  This file is part of the Arduino_LSM9DS1 library.
  Copyright (c) 2019 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "LSM9DS1.h"

#define LSM9DS1_ADDRESS            0x6b

#define LSM9DS1_WHO_AM_I           0x0f
#define LSM9DS1_CTRL_REG1_G        0x10
#define LSM9DS1_STATUS_REG         0x17
#define LSM9DS1_OUT_X_G            0x18
#define LSM9DS1_CTRL_REG6_XL       0x20
#define LSM9DS1_CTRL_REG8          0x22
#define LSM9DS1_OUT_X_XL           0x28

// magnetometer
#define LSM9DS1_ADDRESS_M          0x1e

#define LSM9DS1_CTRL_REG1_M        0x20
#define LSM9DS1_CTRL_REG2_M        0x21
#define LSM9DS1_CTRL_REG3_M        0x22
#define LSM9DS1_STATUS_REG_M       0x27
#define LSM9DS1_OUT_X_L_M          0x28

LSM9DS1Class::LSM9DS1Class(TwoWire& wire) :
  continuousMode(false), _wire(&wire)
{
}

LSM9DS1Class::~LSM9DS1Class()
{
}

// void LSM9DS1Class::setAccelScale(uint8_t aScl)
// {
//   // We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
//   uint8_t tempRegValue = readRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL);
//   // Mask out accel scale bits:
//   tempRegValue &= 0xE7;
  
//   switch (aScl)
//   {
//     case 4:
//       tempRegValue |= (0x2 << 3);
//       //settings.accel.scale = 4;
//       break;
//     case 8:
//       tempRegValue |= (0x3 << 3);
//      // settings.accel.scale = 8;
//       break;
//     case 16:
//       tempRegValue |= (0x1 << 3);
//      // settings.accel.scale = 16;
//       break;
//     default: // Otherwise it'll be set to 2g (0x0 << 3)
//      // settings.accel.scale = 2;
//       break;
//   }
//   writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL, tempRegValue);
  
//   // Then calculate a new aRes, which relies on aScale being set correctly:
//  // calcaRes();
// }

int LSM9DS1Class::getOperationalMode() //0=off , 1= Accel only , 2= Gyro +Accel
{
  if ((readRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL) & 0b11100000) ==0 ) return 0;
  if ((readRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG1_G)  & 0b11100000) ==0 ) return 1;
  else return 2;
}

float LSM9DS1Class::measureAccelGyroODR()
{  if (getOperationalMode()==0) return 0;
   float x, y, z;                               //dummies
   unsigned long lastEventTime, 
                 start=micros(); 
   long count = -3;   
   int fifoEna=continuousMode;                  //store FIFO status
   setOneShotMode();                            //switch off FIFO
   while ((micros()- start) < ODRCalibrationTime)         // measure
   { if (accelAvailable())
         {  lastEventTime = micros();
            readAccel(x, y, z);  
            count++;
            if (count<=0) start=lastEventTime;
         }
   }
//    Serial.println("measureAccelGyroODR Count "+String( count ) );
//    Serial.println("dTa= "+String(lastEventTime-start)   );
//    Serial.println("ODR= "+String(1000000.0*float(count)/float(lastEventTime-start))   ); 
  if (fifoEna) setContinuousMode(); 
  return (1000000.0*float(count)/float(lastEventTime-start) );
}

int LSM9DS1Class::setGyroODR(uint8_t range) // 0:off, 1:10Hz, 2:50Hz, 3:119Hz, 4:238Hz, 5:476Hz, 6:952Hz
{ if (range >= 7) return 0;
  uint8_t setting = ((readRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG1_G) & 0b00011111) | (range << 5 ) );
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG1_G,setting);  
    if (range > 0 )
  { setting = ((readRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL) & 0b00011111) | (range << 5));
    writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL,setting); 
  }
  switch (getOperationalMode()) {
  case 0: { accelODR=0;             //off
        gyroODR=0; 
        break;
      }
  case 1: { accelODR=  measureAccelGyroODR(); //accelerometer only
        gyroODR = 0;
        break;
      } 
  case 2: { accelODR=  measureAccelGyroODR(); //shared ODR
        gyroODR = accelODR;
      }
  }
  return 1;
}

float LSM9DS1Class::getGyroODR()
{  return gyroODR;
// float Ranges[] ={0.0, 10.0, 50.0, 119.0, 238.0, 476.0, 952.0, 0.0 };  //Hz
//   uint8_t setting = readRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG1_G)  >> 5;
//   return Ranges [setting];   //  used to be  return 119.0F;
}

int LSM9DS1Class::accelAvailable()
{
  if (continuousMode) {
    // Read FIFO_SRC. If any of the rightmost 8 bits have a value, there is data.
    if (readRegister(LSM9DS1_ADDRESS, 0x2F) & 63) {
      return 1;
    }
  } else {
    if (readRegister(LSM9DS1_ADDRESS, LSM9DS1_STATUS_REG) & 0x01) {
      return 1;
    }
  }
  return 0;
}

int LSM9DS1Class::readAccel(float& x, float& y, float& z)  // return calibrated data in a unit of choise
{   if (!readRawAccel(x,y,z)) return 0; 
  // See releasenotes     read =  Unit * Slope * (FS / 32786 * Data - Offset )
  x = accelUnit * accelSlope[0] * (x - accelOffset[0]);
  y = accelUnit * accelSlope[1] * (y - accelOffset[1]);
  z = accelUnit * accelSlope[2] * (z - accelOffset[2]);
  return 1;
}

int LSM9DS1Class::readRawAccel(float& x, float& y, float& z)   // return raw uncalibrated data 
{ int16_t data[3];
  if (!readRegisters(LSM9DS1_ADDRESS, LSM9DS1_OUT_X_XL, (uint8_t*)data, sizeof(data))) 
  {  x = NAN;     y = NAN;     z = NAN;   return 0;
  }
  // See releasenotes     read =  Unit * Slope * (PFS / 32786 * Data - Offset )
  float scale =  getAccelFS()/32768.0 ;   
  x = scale * data[0];
  y = scale * data[1];
  z = scale * data[2];
  return 1;
}

void LSM9DS1Class::setAccelOffset(float x, float y, float z) 
{  accelOffset[0] = x /(accelUnit * accelSlope[0]);  
   accelOffset[1] = y /(accelUnit * accelSlope[1]);
   accelOffset[2] = z /(accelUnit * accelSlope[2]);
}
//Slope is already dimensionless, so it can be stored as is.
void LSM9DS1Class::setAccelSlope(float x, float y, float z) 
{  if (x==0) x=1;  //zero slope not allowed
   if (y==0) y=1;
   if (z==0) z=1;
   accelSlope[0] = x ;   
   accelSlope[1] = y ;
   accelSlope[2] = z ;
}

int LSM9DS1Class::setAccelFS(uint8_t range) // 0: ±2g ; 1: ±16g ; 2: ±4g ; 3: ±8g  
{ if (range >=4) return 0;
    range = (range & 0b00000011) << 3;
  uint8_t setting = ((readRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL) & 0xE7) | range);
  return writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL,setting) ;
}

float LSM9DS1Class::getAccelFS() // Full scale dimensionless, but its value corresponds to g
{   float ranges[] ={2.0, 24.0, 4.0, 8.0}; //g
    uint8_t setting = (readRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL) & 0x18) >> 3;
    return ranges[setting] ;
}

int LSM9DS1Class::begin()
{
  _wire->begin();

  // reset
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG8, 0x05);
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x0c);

  delay(10);

  if (readRegister(LSM9DS1_ADDRESS, LSM9DS1_WHO_AM_I) != 0x68) {
    end();

    return 0;
  }

  if (readRegister(LSM9DS1_ADDRESS_M, LSM9DS1_WHO_AM_I) != 0x3d) {
    end();

    return 0;
  }

  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG1_G, 0x78); // 119 Hz, 2000 dps, 16 Hz BW
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL, 0xD0); //(0xD0 952hz) (0x70 119 Hz, 4G)

  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG1_M, 0xb4); // Temperature compensation enable, medium performance, 20 Hz
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x00); // 4 Gauss
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG3_M, 0x00); // Continuous conversion mode

  return 1;
}

void LSM9DS1Class::setContinuousMode() {
  // Enable FIFO (see docs https://www.st.com/resource/en/datasheet/DM00103319.pdf)
  writeRegister(LSM9DS1_ADDRESS, 0x23, 0x02);
  // Set continuous mode
  writeRegister(LSM9DS1_ADDRESS, 0x2E, 0xC0);

  continuousMode = true;
}

void LSM9DS1Class::setOneShotMode() {
  // Disable FIFO (see docs https://www.st.com/resource/en/datasheet/DM00103319.pdf)
  writeRegister(LSM9DS1_ADDRESS, 0x23, 0x00);
  // Disable continuous mode
  writeRegister(LSM9DS1_ADDRESS, 0x2E, 0x00);

  continuousMode = false;
}

void LSM9DS1Class::end()
{
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG3_M, 0x03);
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG1_G, 0x00);
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL, 0x00);

  _wire->end();
}

int LSM9DS1Class::readAcceleration(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM9DS1_ADDRESS, LSM9DS1_OUT_X_XL, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 4.0 / 32768.0;
  y = data[1] * 4.0 / 32768.0;
  z = data[2] * 4.0 / 32768.0;

  return 1;
}

int LSM9DS1Class::accelerationAvailable()
{
  if (continuousMode) {
    // Read FIFO_SRC. If any of the rightmost 8 bits have a value, there is data.
    if (readRegister(LSM9DS1_ADDRESS, 0x2F) & 63) {
      return 1;
    }
  } else {
    if (readRegister(LSM9DS1_ADDRESS, LSM9DS1_STATUS_REG) & 0x01) {
      return 1;
    }
  }

  return 0;
}

float LSM9DS1Class::accelerationSampleRate()
{
  return 119.0F;
}

int LSM9DS1Class::readGyroscope(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM9DS1_ADDRESS, LSM9DS1_OUT_X_G, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 2000.0 / 32768.0;
  y = data[1] * 2000.0 / 32768.0;
  z = data[2] * 2000.0 / 32768.0;

  return 1;
}

int LSM9DS1Class::gyroscopeAvailable()
{
  if (readRegister(LSM9DS1_ADDRESS, LSM9DS1_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;
}

float LSM9DS1Class::gyroscopeSampleRate()
{
  return 119.0F;
}

int LSM9DS1Class::readMagneticField(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM9DS1_ADDRESS_M, LSM9DS1_OUT_X_L_M, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 4.0 * 100.0 / 32768.0;
  y = data[1] * 4.0 * 100.0 / 32768.0;
  z = data[2] * 4.0 * 100.0 / 32768.0;

  return 1;
}

int LSM9DS1Class::magneticFieldAvailable()
{
  if (readRegister(LSM9DS1_ADDRESS_M, LSM9DS1_STATUS_REG_M) & 0x08) {
    return 1;
  }

  return 0;
}

float LSM9DS1Class::magneticFieldSampleRate()
{
  return 20.0;
}

int LSM9DS1Class::readRegister(uint8_t slaveAddress, uint8_t address)
{
  _wire->beginTransmission(slaveAddress);
  _wire->write(address);
  if (_wire->endTransmission() != 0) {
    return -1;
  }

  if (_wire->requestFrom(slaveAddress, 1) != 1) {
    return -1;
  }

  return _wire->read();
}

int LSM9DS1Class::readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t* data, size_t length)
{
  _wire->beginTransmission(slaveAddress);
  _wire->write(0x80 | address);
  if (_wire->endTransmission(false) != 0) {
    return -1;
  }

  if (_wire->requestFrom(slaveAddress, length) != length) {
    return 0;
  }

  for (size_t i = 0; i < length; i++) {
    *data++ = _wire->read();
  }

  return 1;
}

int LSM9DS1Class::writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value)
{
  _wire->beginTransmission(slaveAddress);
  _wire->write(address);
  _wire->write(value);
  if (_wire->endTransmission() != 0) {
    return 0;
  }

  return 1;
}

#ifdef ARDUINO_ARDUINO_NANO33BLE
LSM9DS1Class IMU(Wire1);
#else
LSM9DS1Class IMU(Wire);
#endif
