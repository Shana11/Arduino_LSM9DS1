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

#include <Arduino.h>
#include <Wire.h>

#define GRAVITY           1.0 

class LSM9DS1Class {
  public:
    LSM9DS1Class(TwoWire& wire);
    virtual ~LSM9DS1Class();

    int begin();
    void end();

    // Controls whether a FIFO is continuously filled, or a single reading is stored.
    // Defaults to one-shot.
    void setContinuousMode();
    void setOneShotMode();

    int getOperationalMode();

    float accelOffset[3] = {0,0,0}; // zero point offset correction factor for calibration
    float accelSlope[3] = {1,1,1};
    float accelUnit = GRAVITY; 
    //void setAccelScale(uint8_t aScl);

    virtual int   setGyroODR(uint8_t range); //Sample Rate Hz 0:off,1:10,2:50 3:119,4:238,5:476,6:does not work 952Hz 
    virtual float getGyroODR(); // Measured Sample rate of the sensor.
    virtual int   accelAvailable();
    virtual int   readAccel(float& x, float& y, float& z); 
    virtual int   readRawAccel(float& x, float& y, float& z);
    virtual void  setAccelOffset(float x, float y, float z);  //Store zero-point measurements as offset
    virtual void  setAccelSlope(float x, float y, float z);  
    virtual float getAccelFS();
    virtual int   setAccelFS(uint8_t range); 


    // Accelerometer
    virtual int readAcceleration(float& x, float& y, float& z); // Results are in G (earth gravity).
    virtual int accelerationAvailable(); // Number of samples in the FIFO.
    virtual float accelerationSampleRate(); // Sampling rate of the sensor.

    // Gyroscope
    virtual int readGyroscope(float& x, float& y, float& z); // Results are in degrees/second.
    virtual int gyroscopeAvailable(); // Number of samples in the FIFO.
    virtual float gyroscopeSampleRate(); // Sampling rate of the sensor.

    // Magnetometer
    virtual int readMagneticField(float& x, float& y, float& z); // Results are in uT (micro Tesla).
    virtual int magneticFieldAvailable(); // Number of samples in the FIFO.
    virtual float magneticFieldSampleRate(); // Sampling rate of the sensor.

  private:

    unsigned long ODRCalibrationTime=250000;
    float gyroODR; 
    float accelODR; 
    float measureAccelGyroODR();

    bool continuousMode;
    int readRegister(uint8_t slaveAddress, uint8_t address);
    int readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t* data, size_t length);
    int writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value);



  private:
    TwoWire* _wire;
};

extern LSM9DS1Class IMU;
