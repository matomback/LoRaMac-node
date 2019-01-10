/*!
 * \file      sht21.h
 *
 * \brief     SHT21 Temperature and relative humidity sensor driver implementation
 *            - From Sensirion SHT21 Sample Code (SHT2x.h) and SHT21 MBED examples
 *
 * \author    Matthew Tomback (Colca)
 *
 */

#ifndef __SHT21_H__
#define __SHT21_H__

#include <stdint.h>

// SHT21 I2C address
#define SHT_I2C_ADDRESS     0x40

// *Addresses with read and write bits are not needed, as STMCube HAL functions handle read / write bit inclusion
// #define I2C_ADR_W           (SHT_I2C_ADDRESS << 1) + 0      // Sensor I2C address + write bit (128) 
// #define I2C_ADR_R           (SHT_I2C_ADDRESS << 1) + 1      // Sensor I2C address + read bit (129)

// SHT21 Commands
#define SHT_TRIG_TEMP_HOLD  0xE3    //Trigger Temp measure with hold master
#define SHT_TRIG_RH_HOLD    0xE5    //Trigger RH measure with hold master
#define SHT_TRIG_TEMP       0xF3    //Trigger Temp measure with no hold master
#define SHT_TRIG_RH         0xF5    //Trigger RH measure with no hold master
#define SHT_WRITE_REG       0xE6    //Write to user register
#define SHT_READ_REG        0xE7    //Read from user register
#define SHT_SOFT_RESET      0xFE    //Soft reset the sensor

// User Register Info - data precision settings
#define SHT_PREC_1214       0x00    //RH 12 T 14 - default
#define SHT_PREC_0812       0x01    //RH 8  T 10 
#define SHT_PREC_1013       0x80    //RH 10 T 13
#define SHT_PREC_1111       0x81    //RH 11 T 11
#define SHT2x_RES_MASK      0x81    //Mask for resolution bits (7,0) in user reg.

#define SHT_BATTERY_STAT    0x40    //Battery status
#define SHT_EOB_MASK        0x40    //Mask for battery stat bit(6) in user reg.

#define SHT_HEATER          0x04    //Enable on chip heater
#define SHT_HEATER_OFF      0x00    //Disable Heater
#define SHT_DISABLE_OTP     0x02    //Disable OTP reload
#define SHT_HEATER_MASK     0x04    //Mask for Heater bit(2) in user reg.

// Fail conditions on the I2C bus
#define SHT_FAIL            1
#define SHT_SUCCESS         0

// Author fail conditions
// 1, 2, 3 can be used because these are status bits in the received measurement value
#define SHT_GOOD            0xFFFC
#define SHT_TRIG_FAIL       1
#define SHT_READ_FAIL       2

//  CRC
const uint16_t SHT_POLY;    //P(x)=x^8+x^5+x^4+1 = 100110001

// Measurement signal selection
typedef enum{
  HUMIDITY,
  TEMP
}etSHT2xMeasureType;

// Error codes
typedef enum{
  ACK_ERROR                = 0x01,
  TIME_OUT_ERROR           = 0x02,
  CHECKSUM_ERROR           = 0x04,
  UNIT_ERROR               = 0x08
}etError;

// I2C Level
typedef enum{
  LOW                      = 0,
  HIGH                     = 1,
}etI2cLevel;

// I2C Acknowledge
typedef enum{
  ACK                      = 0,
  NO_ACK                   = 1,
}etI2cAck;



// I2c SHT Function Declarations
//
//==============================================================================
uint8_t I2c_Write_SHT (uint8_t *txByte);
//==============================================================================
//
//
//==============================================================================
uint8_t I2c_Read_SHT (etI2cAck ack, uint8_t* data, uint8_t size);
//==============================================================================


// Calculates checksum for n bytes of data and compares it with expected checksum
// input:  data[]       checksum is built based on this data
//         nbrOfBytes   checksum is built for n bytes of data
//         checksum     expected checksum
// return: error:       CHECKSUM_ERROR = checksum does not match
//                      0              = checksum matches
//==============================================================================
uint8_t SHT2x_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum);
//==============================================================================


// Measures humidity or temp. Function polls every 10ms until measurement ready. Timing for timeout may be changed.
// input: eSHT2xMeasureType
// output: *result - humidity/temp as raw value
// return: (error)
//==============================================================================
uint8_t SHT2x_Measure(etSHT2xMeasureType eSHT2xMeasureType, uint16_t *result);  
//==============================================================================


// calculates the relative humidity
// input:  humidity raw value (16bit scaled)
// return: pHumidity relative humidity [%RH]
//==============================================================================
float SHT2x_CalcRH(uint16_t RH);
//==============================================================================


// calculates temperature
// input:  temperature raw value (16bit scaled)
// return: temperature [°C]
//==============================================================================
float SHT2x_CalcTemperatureC(uint16_t TC);
//==============================================================================


#endif // __SHT21_H__