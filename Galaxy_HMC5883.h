/*!
 * @file Galaxy_HMC5883.h
 * @brief Simple connect HMC5883
 * @copyright   Copyright (c) 2024 Murnik Roman
 * @license     The MIT License (MIT)
 * @author      Murnik Roman (murnikr@gmail.com)
 * @version  V1.0.0
 * @date  2024-02-02
 * @url https://github.com/error911/Galaxy_HMC5883
 */

#ifndef GALAXY_HMC5883_H
#define GALAXY_HMC5883_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Wire.h>

#define HMC5883L_ADDRESS             (0x1E)
//#define QMC5883_ADDRESS              (0x0D)
//#define VCM5883L_ADDRESS             (0x0C)

#define IC_NONE     0
#define IC_HMC5883L 1
#define IC_QMC5883  2
#define IC_VCM5883L 3
#define IC_ERROR    4

#define HMC5883L_REG_CONFIG_A         (0x00)
#define HMC5883L_REG_CONFIG_B         (0x01)
#define HMC5883L_REG_MODE             (0x02)
#define HMC5883L_REG_OUT_X_M          (0x03)
#define HMC5883L_REG_OUT_X_L          (0x04)
#define HMC5883L_REG_OUT_Z_M          (0x05)
#define HMC5883L_REG_OUT_Z_L          (0x06)
#define HMC5883L_REG_OUT_Y_M          (0x07)
#define HMC5883L_REG_OUT_Y_L          (0x08)
#define HMC5883L_REG_STATUS           (0x09)
#define HMC5883L_REG_IDENT_A          (0x0A)
#define HMC5883L_REG_IDENT_B          (0x0B)
#define HMC5883L_REG_IDENT_C          (0x0C)

typedef enum
{
  HMC5883L_SAMPLES_8    = 0b11,
  HMC5883L_SAMPLES_4    = 0b10,
  HMC5883L_SAMPLES_2    = 0b01,
  HMC5883L_SAMPLES_1    = 0b00
} eSamples_t;

typedef enum
{
  HMC5883L_DATARATE_75HZ       = 0b110,
  HMC5883L_DATARATE_30HZ       = 0b101,
  HMC5883L_DATARATE_15HZ       = 0b100,
  HMC5883L_DATARATE_7_5HZ      = 0b011,
  HMC5883L_DATARATE_3HZ        = 0b010,
  HMC5883L_DATARATE_1_5HZ      = 0b001,
  HMC5883L_DATARATE_0_75_HZ    = 0b000
} eDataRate_t;

typedef enum
{
  HMC5883L_RANGE_8_1GA     = 0b111,
  HMC5883L_RANGE_5_6GA     = 0b110,
  HMC5883L_RANGE_4_7GA     = 0b101,
  HMC5883L_RANGE_4GA       = 0b100,
  HMC5883L_RANGE_2_5GA     = 0b011,
  HMC5883L_RANGE_1_9GA     = 0b010,
  HMC5883L_RANGE_1_3GA    = 0b001,
  HMC5883L_RANGE_0_88GA    = 0b000
} eRange_t;

typedef enum
{
  HMC5883L_IDLE         = 0b10,
  HMC5883_SINGLE        = 0b01,
  HMC5883L_CONTINOUS    = 0b00
} eMode_t;

#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
typedef struct 
{
  int16_t XAxis;
  int16_t YAxis;
  int16_t ZAxis;
  float   AngleXY;
  float   AngleXZ;
  float   AngleYZ;
  float   HeadingDegress;
} sVector_t;
#endif

class Galaxy_HMC5883
{
  public:
    Galaxy_HMC5883(TwoWire*pWire = &Wire, uint8_t I2C_addr = 0x0C);
    /**
     * @fn begin
     * @brief Sensor init
     * @return bool init status
     * @retval true init succeeded
     * @retval false init failed
     */
    bool begin(void);

    /**
     * @fn readRaw
     * @brief Get the data collected by the sensor
     * @return sVector_t The data collected by the sensor
     */
    sVector_t readRaw(void);

    /**
     * @fn setRange
     * @brief Set sensor signal gain range
     * @param range 
     * @n    HMC5883L_RANGE_8_1GA    
     * @n    HMC5883L_RANGE_5_6GA    
     * @n    HMC5883L_RANGE_4_7GA    
     * @n    HMC5883L_RANGE_4GA      
     * @n    HMC5883L_RANGE_2_5GA    
     * @n    HMC5883L_RANGE_1_9GA    
     * @n    HMC5883L_RANGE_1_3GA //default    
     * @n    HMC5883L_RANGE_0_88GA   
     */
    void  setRange(eRange_t range);

    /**
     * @fn getRange
     * @brief Get sensor signal gain range
     * @return eRange_t
     */
    eRange_t getRange(void);

    /**
     * @fn setMeasurementMode
     * @brief Set measurement mode
     * @param mode
     * @n     HMC5883L_IDLE
     * @n     HMC5883_SINGLE
     * @n     HMC5883L_CONTINOUS
     */
    void setMeasurementMode(eMode_t mode);

    /**
     * @fn  getMeasurementMode
     * @brief Get measurement mode
     * @return eMode_t
     */
    eMode_t getMeasurementMode(void);

    /**
     * @fn setDataRate
     * @brief Set the data collection rate of the sensor
     * @param dataRate
     * @n     HMC5883L_DATARATE_75HZ
     * @n     HMC5883L_DATARATE_30HZ
     * @n     HMC5883L_DATARATE_15HZ
     * @n     HMC5883L_DATARATE_7_5HZ
     * @n     HMC5883L_DATARATE_3HZ
     * @n     HMC5883L_DATARATE_1_5HZ
     * @n     HMC5883L_DATARATE_0_75_HZ
     */
    void  setDataRate(eDataRate_t dataRate);

    /**
     * @fn getDataRate
     * @brief Get the data collection rate of the sensor
     * @return eDataRate_t
     */
    eDataRate_t getDataRate(void);

    /**
     * @fn setSamples
     * @brief Set sensor status
     * @param samples
     * @n     HMC5883L_SAMPLES_8
     * @n     HMC5883L_SAMPLES_4
     * @n     HMC5883L_SAMPLES_2
     * @n     HMC5883L_SAMPLES_1
      */
    void  setSamples(eSamples_t samples);

    /**
     * @fn getSamples
     * @brief Get sensor status
     * @return eSamples_t
     */
    eSamples_t getSamples(void);

    /**
     * @fn  setDeclinationAngle
     * @brief Set sensor declination angle
     * @param declinationAngle
     */
    void setDeclinationAngle(float declinationAngle);

    /**
     * @fn getHeadingDegrees
     * @brief Set the sensor range
     */
    void getHeadingDegrees(void);

    /**
     * @fn getICType
     * @brief Get sensor type
     * @return int
     */
    int getICType(void);

    /**
     * @fn  isHMC
     * @brief Determine if the sensor type is HMC5883
     * @return bool 
     * @retval ture it is
     * @retval false it isn't
     */
    bool isHMC(void){if(ICType == IC_HMC5883L ){return true;}return false;}

  private:
    void writeRegister8(uint8_t reg, uint8_t value);
    uint8_t readRegister8(uint8_t reg);
    uint8_t fastRegister8(uint8_t reg);
    int16_t readRegister16(uint8_t reg);
    TwoWire *_pWire;
    uint8_t _I2C_addr;
    float ICdeclinationAngle;
    int ICType = IC_NONE;
    bool isHMC_;
    float mgPerDigit;
    float Gauss_LSB_XY = 1090.0;
    sVector_t v;
    float minX, maxX;
    float minY, maxY;
    float minZ, maxZ;
    bool firstRun;
};

#endif
