/*!
 * @file Galaxy_HMC5883.cpp
 * @brief Simple connect HMC5883
 * @copyright   Copyright (c) 2024 Roman Murnik
 * @license     The MIT License (MIT)
 * @author      Roman Murnik
 * @version  V0.0.2
 * @date  2024-02-02
 * @url https://github.com/error911/Galaxy_HMC5883
 */
#include "Galaxy_HMC5883.h"

Galaxy_HMC5883::Galaxy_HMC5883(uint8_t I2C_addr)
{
  isHMC_ = false;
  minX  = 0;
  maxX  = 0;
  minY  = 0;
  maxY  = 0;
  minZ  = 0;
  maxZ  = 0;
  firstRun = true;
  this->_pWire = &Wire;
  this->_I2C_addr = I2C_addr;
}

bool Galaxy_HMC5883::begin(void)
{
  bool ret = false;
  if(ICType == IC_NONE)
  {
    for(uint8_t i = 0; i < 5; i++)
    {
      _pWire->begin();
      _pWire->beginTransmission(HMC5883L_ADDRESS);
      if(_pWire->endTransmission() == 0)
      {
        ICType = IC_HMC5883L;
        break;
      }
    }
  }

  switch(ICType)
  {
    case IC_NONE:
      ret = false;
      break;
    case IC_HMC5883L:
      if ((fastRegister8(HMC5883L_REG_IDENT_A) != 0x48)|| (fastRegister8(HMC5883L_REG_IDENT_B) != 0x34) || (fastRegister8(HMC5883L_REG_IDENT_C) != 0x33))
      {
        return false;
      }
      setRange(HMC5883L_RANGE_1_3GA);
      setMeasurementMode(HMC5883L_CONTINOUS);
      setDataRate(HMC5883L_DATARATE_15HZ);
      setSamples(HMC5883L_SAMPLES_1);
      mgPerDigit = 0.92f;
      ret = true;
      break;
    default:
      ret = false;
      break;
  }
  return ret;
}

Vector_t Galaxy_HMC5883::readRaw(void)
{
  if(ICType == IC_HMC5883L)
  {
    rawData.x = readRegister16(HMC5883L_REG_OUT_X_M);
    rawData.y = readRegister16(HMC5883L_REG_OUT_Y_M);
    rawData.z = readRegister16(HMC5883L_REG_OUT_Z_M);
  }
  return rawData;
}

void Galaxy_HMC5883::setRange(eRange_t range)
{
  if(ICType == IC_HMC5883L)
  {
    switch(range)
    {
      case HMC5883L_RANGE_0_88GA:
        Gauss_LSB_XY = 1370.0;
        break;
      case HMC5883L_RANGE_1_3GA:
        Gauss_LSB_XY = 1090.0;
        break;
      case HMC5883L_RANGE_1_9GA:
        Gauss_LSB_XY = 820.0;
        break;
      case HMC5883L_RANGE_2_5GA:
        Gauss_LSB_XY = 660.0;
        break;
      case HMC5883L_RANGE_4GA:
        Gauss_LSB_XY = 440.0;
        break;
      case HMC5883L_RANGE_4_7GA:
        Gauss_LSB_XY = 390.0;
        break;
      case HMC5883L_RANGE_5_6GA:
        Gauss_LSB_XY = 330.0;
        break;
      case HMC5883L_RANGE_8_1GA:
        Gauss_LSB_XY = 230.0;
        break;
      default:
        break;
    }
    writeRegister8(HMC5883L_REG_CONFIG_B, range << 5);
  }
}

eRange_t Galaxy_HMC5883::getRange(void)
{
  eRange_t ret;
  switch(ICType){
    case IC_HMC5883L:
      ret = (eRange_t)((readRegister8(HMC5883L_REG_CONFIG_B) >> 5));
      break;
    default:
      ret = HMC5883L_RANGE_1_3GA;
      break;
  }
  return ret;
}

void Galaxy_HMC5883::setMeasurementMode(eMode_t mode)
{
  uint8_t value;
  switch(ICType)
  {
    case IC_HMC5883L:
      value = readRegister8(HMC5883L_REG_MODE);
      value &= 0b11111100;
      value |= mode;
      writeRegister8(HMC5883L_REG_MODE, value);
      break;
    default:
      break;
  }
}

eMode_t Galaxy_HMC5883::getMeasurementMode(void)
{
  uint8_t value=0;
  switch(ICType)
  {
    case IC_HMC5883L:
      value = readRegister8(HMC5883L_REG_MODE);
      value &= 0b00000011;  
      break;
    default:
      break;
  }
  return (eMode_t)value;
}

void Galaxy_HMC5883::setDataRate(eDataRate_t dataRate)
{
  uint8_t value;
  switch(ICType)
  {
    case IC_HMC5883L:
      value = readRegister8(HMC5883L_REG_CONFIG_A);
      value &= 0b11100011;
      value |= (dataRate << 2);
      writeRegister8(HMC5883L_REG_CONFIG_A, value);
      break;
    default:
      break;
  }
}

eDataRate_t Galaxy_HMC5883::getDataRate(void)
{
  uint8_t value=0;
  switch(ICType)
  {
    case IC_HMC5883L:
      value = readRegister8(HMC5883L_REG_CONFIG_A);
      value &= 0b00011100;
      value >>= 2;
      break;
    default:
      break;
  }
  return (eDataRate_t)value;
}

void Galaxy_HMC5883::setSamples(eSamples_t samples)
{
  uint8_t value;
  switch(ICType)
  {
    case IC_HMC5883L:
      value = readRegister8(HMC5883L_REG_CONFIG_A);
      value &= 0b10011111;
      value |= (samples << 5);
      writeRegister8(HMC5883L_REG_CONFIG_A, value);
      break;
    default:
      break;
  }
}

eSamples_t Galaxy_HMC5883::getSamples(void)
{
  uint8_t value=0;
  switch(ICType)
  {
    case IC_HMC5883L:
      value = readRegister8(HMC5883L_REG_CONFIG_A);
      value &= 0b01100000;
      value >>= 5;
      break;
    default:
      break;
  }
  return (eSamples_t)value;
}

void Galaxy_HMC5883::setDeclinationAngle(float declinationAngle)
{
  this->ICdeclinationAngle = declinationAngle;
}

float Galaxy_HMC5883::getHeadingDegrees(void)
{
  float heading = atan2(rawData.y ,rawData.x);
  heading += this->ICdeclinationAngle;
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;

  //AngleXY = (atan2((double)rawData.y,(double)rawData.x) * (180 / PI) + 180);
  //AngleXZ = (atan2((double)rawData.z,(double)rawData.x) * (180 / PI) + 180);
  //AngleYZ = (atan2((double)rawData.z,(double)rawData.y) * (180 / PI) + 180);

  float headingDegress = heading * 180/PI;
  return headingDegress;
}

int Galaxy_HMC5883::getICType(void)
{
  return ICType;
}

void Galaxy_HMC5883::writeRegister8(uint8_t reg, uint8_t value)
{
  _pWire->beginTransmission(this->_I2C_addr);
  #if ARDUINO >= 100
    _pWire->write(reg);
    _pWire->write(value);
  #else
    _pWire->send(reg);
    _pWire->send(value);
  #endif
    _pWire->endTransmission();
}

uint8_t Galaxy_HMC5883::fastRegister8(uint8_t reg)
{
  uint8_t value=0;
  _pWire->beginTransmission(this->_I2C_addr);
  #if ARDUINO >= 100
    _pWire->write(reg);
  #else
    _pWire->send(reg);
  #endif
  _pWire->endTransmission();
  _pWire->requestFrom((uint8_t)this->_I2C_addr, (uint8_t)1);
  #if ARDUINO >= 100
    value = _pWire->read();
  #else
    value = _pWire->receive();
  #endif
  _pWire->endTransmission();
  return value;
}


uint8_t Galaxy_HMC5883::readRegister8(uint8_t reg)
{
  uint8_t value=0;
  _pWire->beginTransmission(this->_I2C_addr);
  #if ARDUINO >= 100
    _pWire->write(reg);
  #else
    _pWire->send(reg);
  #endif
  _pWire->endTransmission();
  _pWire->requestFrom((uint8_t)this->_I2C_addr, (uint8_t)1);
  while(!_pWire->available()) {};
  #if ARDUINO >= 100
    value = _pWire->read();
  #else
    value = _pWire->receive();
  #endif
  return value;
}

int16_t Galaxy_HMC5883::readRegister16(uint8_t reg)
{
  int16_t value=0;
  uint8_t vha,vla;
  _pWire->beginTransmission(this->_I2C_addr);
  #if ARDUINO >= 100
    _pWire->write(reg);
  #else
    _pWire->send(reg);
  #endif
  _pWire->endTransmission();
  _pWire->requestFrom((uint8_t)this->_I2C_addr, (uint8_t)2);
  while(!_pWire->available()) {};
  if(ICType == IC_HMC5883L){
    #if ARDUINO >= 100
      vha = _pWire->read();
      vla = _pWire->read();
    #else
      vha = _pWire->receive();
      vla = _pWire->receive();
    #endif
  }
  value = vha << 8 | vla;
  return value;
}
