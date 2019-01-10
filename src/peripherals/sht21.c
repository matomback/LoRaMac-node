/*!
 * \file      sht21.c
 *
 * \brief     SHT21 Temperature and relative humidity sensor driver implementation
 *            - From Sensirion SHT21 Sample Code (SHT2x.c) and SHT21 MBED examples
 *
 * \author    Matthew Tomback (Colca)
 *
 */

#include "sht21.h"
#include "utilities.h"
#include "i2c.h"
#include "delay.h"

const uint16_t SHT_POLY = 0x131;    // P(x)=x^8+x^5+x^4+1 = 100110001

// extern I2c_t I2c1;                  /////***** I2c_t initialized in board.c
extern I2c_t I2c2;                  /////***** I2c_t initialized in board.c


// /*!
//  * \brief Writes a byte to SHT sensor over I2C (not at specified address within the device)
//  *
//  * \param [IN]:    data
//  * \retval status [SUCCESS, FAIL]
//  */
uint8_t I2c_Write_SHT (uint8_t* txByte)
{
    uint8_t status = FAIL;

    // // Declaration: HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,uint8_t *pData, uint16_t Size, uint32_t Timeout);
    // status = ( HAL_I2C_Master_Transmit( &I2cHandle, SHT_I2C_ADDRESS, txByte, sizeof(txByte), 2000 ) == HAL_OK ) ? SUCCESS : FAIL;

    status = SHTI2cWrite (&I2c2, SHT_I2C_ADDRESS << 1, txByte);
    return status;
}


// /*!
//  * \brief Reads a byte(s) from SHT sensor over I2C (not from specified address in the device)
//  *
//  * \param [IN]: size
//  * \param [OUT]: data
//  * \retval status [SUCCESS, FAIL]
//  */
uint8_t I2c_Read_SHT (etI2cAck ack, uint8_t* data, uint8_t size)
{
    uint8_t status = FAIL;

    // // Declaration: HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,uint8_t *pData, uint16_t Size, uint32_t Timeout);
    // status = ( HAL_I2C_Master_Receive( &I2cHandle, SHT_I2C_Address, txByte, sizeof(txByte), 2000 ) == HAL_OK ) ? SUCCESS : FAIL;

    status = SHTI2cRead (&I2c2, SHT_I2C_ADDRESS << 1, data, size);
    return status;
    
    // uint8_t* data;  /////*****/////*****
    // __STATIC_INLINE uint8_t LL_I2C_ReceiveData8(I2C_TypeDef *I2Cx)
    // data = LL_I2C_ReceiveData8(I2c);
    // return data;
}


// /*!
//  * \brief Crc check from SHT sensor reading
//  *
//  * \param [IN]: data, nbrOfBytes, checksum
//  * \param [OUT]: 
//  * \retval [0, CHECKSUM_ERROR]
//  */
uint8_t SHT2x_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
{
  uint8_t crc = 0;	
  uint8_t byteCtr;
  //calculates 8-Bit checksum with given polynomial
  for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
  { crc ^= (data[byteCtr]);
    for (uint8_t bit = 8; bit > 0; --bit)
    { if (crc & 0x80) crc = (crc << 1) ^ SHT_POLY;
      else crc = (crc << 1);
    }
  }
  if (crc != checksum) return CHECKSUM_ERROR;
  else return 0;
}


// /*!
//  * \brief Measure (Hold type) from SHT sensor
//  *
//  * \param [IN]: eSHT2xMeasureType
//  * \param [OUT]: result
//  * \retval error []
//  */
uint8_t SHT2x_Measure(etSHT2xMeasureType eSHT2xMeasureType, uint16_t *result)
{
    uint8_t checksum;   // checksum
    uint8_t data[2];    // data array for checksum verific
    uint8_t error = 0;  // error variable

    uint8_t cmd_rh = SHT_TRIG_RH_HOLD;      // command to write to sensor to take humidity (hold) measurement
    uint8_t cmd_tc = SHT_TRIG_TEMP_HOLD;    // command to write to sensor to take temperature (hold) measurement

    uint8_t * cmd;

    // // Write I2C Sensor Address and Command
    // LL_I2C_GenerateStartCondition(I2c);
    // error |= I2c_Write_SHT(I2C_ADR_W);

    switch(eSHT2xMeasureType)
    {
        case HUMIDITY: 
            cmd = &cmd_rh;
            error |= I2c_Write_SHT(cmd);
            break;
        case TEMP    : 
            cmd = &cmd_tc;
            error |= I2c_Write_SHT(cmd); 
            break;
        // default: assert(0);   /////*****
    }

    // uint16_t i = 0;     // counter variable
    // //-- poll every 10ms for measurement ready. Timeout after 20 retries (200ms)--
    // do
    // { //LL_I2C_GenerateStartCondition(I2c); 
    //     DelayMs(10);  //delay 10ms
    //     if(i++ >= 20) break;
    // } while(I2c_Write_SHT(I2C_ADR_R) == FAIL);    // Changed from ACK_ERROR to FAIL
    // if (i>=20) error |= TIME_OUT_ERROR;

    error = I2c_Read_SHT(ACK, data, 2);             /////***** Pass data[] array to function (array name serves as pointer)
    error = I2c_Read_SHT(NO_ACK, &checksum, 1);

    *result = data[0] << 8;
    *result += data[1];

    //-- verify checksum --
    error |= SHT2x_CheckCrc (data,2,checksum);
    //LL_I2C_GenerateStopCondition(I2c);

    return error;
}



float SHT2x_CalcRH(uint16_t RH)
{
  float humidityRH;             // variable for result

  RH &= ~0x0003;                // clear bits [1..0] (status bits)
  
  //-- calculate relative humidity [%RH]  (RH = -6 + 125 * SRH/2^16)
  humidityRH = -6.0 + 125.0/65536 * (float)RH;
  return humidityRH;
}



float SHT2x_CalcTemperatureC(uint16_t TC)
{
  float temperatureC;           // variable for result

  TC &= ~0x0003;                // clear bits [1..0] (status bits)

  //float raw = TC;
  //printf("Temp RAW***: %f \r\n", raw);

  //-- calculate temperature [�C]  (T = -46.85 + 175.72 * ST/2^16)
  temperatureC = -46.85 + 175.72/65536 * (float)TC;
  return temperatureC;
} 



// // From MPL3115WriteBuffer
// uint8_t SHTWrite( uint8_t addr, uint8_t *data)
// {
//     return I2cWriteBuffer( &I2c, I2cDeviceAddr << 1, addr, data );
// }

// // From MPL3115ReadBuffer
// uint8_t SHTRead( uint8_t addr, uint8_t *data)
// {
//     return I2cReadBuffer( &I2c, I2cDeviceAddr << 1, addr, data );
// }


// // * LL_I2C Functions (Reference)

// /** @brief  Generate a START or RESTART condition
//   * @note   The START bit can be set even if bus is BUSY or I2C is in slave mode.
//   *         This action has no effect when RELOAD is set. */
// __STATIC_INLINE void LL_I2C_GenerateStartCondition(I2C_TypeDef *I2Cx)
// {
//   SET_BIT(I2Cx->CR2, I2C_CR2_START);
// }

// /** @brief  Generate a STOP condition after the current byte transfer (master mode). */
// __STATIC_INLINE void LL_I2C_GenerateStopCondition(I2C_TypeDef *I2Cx)
// {
//   SET_BIT(I2Cx->CR2, I2C_CR2_STOP);
// }




// uint8_t measureTemp (float *tempC) {
//     uint8_t command;
//     command = SHT_TRIG_TEMP;
//     //I2cWrite(&I2C, )
//     //I2cWriteBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size );
//     I2cWriteBuffer( &I2c, SHT_I2C_ADDRESS << 1, &command, command, 1 );
    //uint8_t I2cWrite( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t data )
    // uint8_t I2cWrite( &I2c, SHT_I2C_ADDRESS << 1, &command, command)
    
// }



// void I2c_StartCondition()
// {
//     SDA = 1;
//     SCL = 1;
//     SSI2C_DELAY;
//     SSI2C_DELAY;
//     SDA = 0;
//     SSI2C_DELAY;
//     SSI2C_DELAY;
//     SCL = 0;
//     SSI2C_DELAY;
//     SSI2C_DELAY;
// }

// void I2c_StopCondition()
// {
//     SDA = 0;
//     SCL = 0;
//     SSI2C_DELAY;
//     SSI2C_DELAY;
//     SCL = 1;
//     SSI2C_DELAY;
//     SSI2C_DELAY;
//     SDA = 1;
//     SSI2C_DELAY;
//     SSI2C_DELAY;
// }

// uint8_t I2c_WriteByte (uint8_t txByte)
// {
//     uint8_t i;
//     uint8_t t;
//     uint8_t r;

//     for (i = 0; i < 8; i++)
//     {
//         if (txByte & 0x80)
//         {
//             SDA = 0;
//         }
//         else
//         {
//             SDA = 1;
//         }
//         txByte <<= 1;

//         SSI2C_DELAY;
//         SCL = 1;
//         SSI2C_DELAY;


//     }
// }




//////////////////////////////////////
/* FROM SHT2X SENSIRION SAMPLE CODE */
//////////////////////////////////////

// //==============================================================================
// uint8_t SHT2x_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
// //==============================================================================
// {
//   uint8_t crc = 0;	
//   uint8_t byteCtr;
//   //calculates 8-Bit checksum with given polynomial
//   for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
//   { crc ^= (data[byteCtr]);
//     for (uint8_t bit = 8; bit > 0; --bit)
//     { if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
//       else crc = (crc << 1);
//     }
//   }
//   if (crc != checksum) return CHECKSUM_ERROR;
//   else return 0;
// }

// //===========================================================================
// uint8_t SHT2x_ReadUserRegister(uint8_t *pRegisterValue)
// //===========================================================================
// {
//   uint8_t checksum;   //variable for checksum byte
//   uint8_t error=0;    //variable for error code

//   I2c_StartCondition();
  

  

//   error |= I2c_WriteByte (I2C_ADR_W);
//   error |= I2c_WriteByte (USER_REG_R);
//   I2c_StartCondition();
//   error |= I2c_WriteByte (I2C_ADR_R);
//   *pRegisterValue = I2c_ReadByte(ACK);
//   checksum=I2c_ReadByte(NO_ACK);
//   error |= SHT2x_CheckCrc (pRegisterValue,1,checksum);
//   I2c_StopCondition();
//   return error;
// }

// //===========================================================================
// uint8_t SHT2x_WriteUserRegister(uint8_t *pRegisterValue)
// //===========================================================================
// {
//   uint8_t error=0;   //variable for error code

//   I2c_StartCondition();
//   error |= I2c_WriteByte (I2C_ADR_W);
//   error |= I2c_WriteByte (USER_REG_W);
//   error |= I2c_WriteByte (*pRegisterValue);
//   I2c_StopCondition();
//   return error;
// }

// //===========================================================================
// uint8_t SHT2x_MeasureHM(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand)
// //===========================================================================
// {
//   uint8_t  checksum;   //checksum
//   uint8_t  data[2];    //data array for checksum verification
//   uint8_t  error=0;    //error variable
//   u16t i;          //counting variable

//   //-- write I2C sensor address and command --
//   I2c_StartCondition();
//   error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
//   switch(eSHT2xMeasureType)
//   { case HUMIDITY: error |= I2c_WriteByte (TRIG_RH_MEASUREMENT_HM); break;
//     case TEMP    : error |= I2c_WriteByte (TRIG_T_MEASUREMENT_HM);  break;
//     default: assert(0);
//   }
//   //-- wait until hold master is released --
//   I2c_StartCondition();
//   error |= I2c_WriteByte (I2C_ADR_R);
//   SCL=HIGH;                     // set SCL I/O port as input
//   for(i=0; i<1000; i++)         // wait until master hold is released or
//   { DelayMicroSeconds(1000);    // a timeout (~1s) is reached
//     if (SCL_CONF==1) break;
//   }
//   //-- check for timeout --
//   if(SCL_CONF==0) error |= TIME_OUT_ERROR;

//   //-- read two data bytes and one checksum byte --
//   pMeasurand->s16.u8H = data[0] = I2c_ReadByte(ACK);
//   pMeasurand->s16.u8L = data[1] = I2c_ReadByte(ACK);
//   checksum=I2c_ReadByte(NO_ACK);

//   //-- verify checksum --
//   error |= SHT2x_CheckCrc (data,2,checksum);
//   I2c_StopCondition();
//   return error;
// }

// //===========================================================================
// uint8_t SHT2x_Measure(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand)
// //===========================================================================
// {
//   uint8_t  checksum;   //checksum
//   uint8_t  data[2];    //data array for checksum verification
//   uint8_t  error=0;    //error variable
//   u16t i=0;        //counting variable

//   //-- write I2C sensor address and command --
//   I2c_StartCondition();
//   error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
//   switch(eSHT2xMeasureType)
//   { case HUMIDITY: error |= I2c_WriteByte (TRIG_RH_MEASUREMENT_POLL); break;
//     case TEMP    : error |= I2c_WriteByte (TRIG_T_MEASUREMENT_POLL);  break;
//     default: assert(0);
//   }
//   //-- poll every 10ms for measurement ready. Timeout after 20 retries (200ms)--
//   do
//   { I2c_StartCondition();
//     DelayMicroSeconds(10000);  //delay 10ms
//     if(i++ >= 20) break;
//   } while(I2c_WriteByte (I2C_ADR_R) == ACK_ERROR);
//   if (i>=20) error |= TIME_OUT_ERROR;

//   //-- read two data bytes and one checksum byte --
//   pMeasurand->s16.u8H = data[0] = I2c_ReadByte(ACK);
//   pMeasurand->s16.u8L = data[1] = I2c_ReadByte(ACK);
//   checksum=I2c_ReadByte(NO_ACK);

//   //-- verify checksum --
//   error |= SHT2x_CheckCrc (data,2,checksum);
//   I2c_StopCondition();

//   return error;
// }

// //===========================================================================
// uint8_t SHT2x_SoftReset()
// //===========================================================================
// {
//   uint8_t  error=0;           //error variable

//   I2c_StartCondition();
//   error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
//   error |= I2c_WriteByte (SOFT_RESET);                            // Command
//   I2c_StopCondition();

//   DelayMicroSeconds(15000); // wait till sensor has restarted

//   return error;
// }

// //==============================================================================
// float SHT2x_CalcRH(u16t u16sRH)
// //==============================================================================
// {
//   ft humidityRH;              // variable for result

//   u16sRH &= ~0x0003;          // clear bits [1..0] (status bits)
//   //-- calculate relative humidity [%RH] --

//   humidityRH = -6.0 + 125.0/65536 * (ft)u16sRH; // RH= -6 + 125 * SRH/2^16
//   return humidityRH;
// }

// //==============================================================================
// float SHT2x_CalcTemperatureC(u16t u16sT)
// //==============================================================================
// {
//   ft temperatureC;            // variable for result

//   u16sT &= ~0x0003;           // clear bits [1..0] (status bits)

//   //-- calculate temperature [�C] --
//   temperatureC= -46.85 + 175.72/65536 *(ft)u16sT; //T= -46.85 + 175.72 * ST/2^16
//   return temperatureC;
// }

// //==============================================================================
// uint8_t SHT2x_GetSerialNumber(uint8_t u8SerialNumber[])
// //==============================================================================
// {
//   uint8_t  error=0;                          //error variable

//   //Read from memory location 1
//   I2c_StartCondition();
//   error |= I2c_WriteByte (I2C_ADR_W);    //I2C address
//   error |= I2c_WriteByte (0xFA);         //Command for readout on-chip memory
//   error |= I2c_WriteByte (0x0F);         //on-chip memory address
//   I2c_StartCondition();
//   error |= I2c_WriteByte (I2C_ADR_R);    //I2C address
//   u8SerialNumber[5] = I2c_ReadByte(ACK); //Read SNB_3
//   I2c_ReadByte(ACK);                     //Read CRC SNB_3 (CRC is not analyzed)
//   u8SerialNumber[4] = I2c_ReadByte(ACK); //Read SNB_2
//   I2c_ReadByte(ACK);                     //Read CRC SNB_2 (CRC is not analyzed)
//   u8SerialNumber[3] = I2c_ReadByte(ACK); //Read SNB_1
//   I2c_ReadByte(ACK);                     //Read CRC SNB_1 (CRC is not analyzed)
//   u8SerialNumber[2] = I2c_ReadByte(ACK); //Read SNB_0
//   I2c_ReadByte(NO_ACK);                  //Read CRC SNB_0 (CRC is not analyzed)
//   I2c_StopCondition();

//   //Read from memory location 2
//   I2c_StartCondition();
//   error |= I2c_WriteByte (I2C_ADR_W);    //I2C address
//   error |= I2c_WriteByte (0xFC);         //Command for readout on-chip memory
//   error |= I2c_WriteByte (0xC9);         //on-chip memory address
//   I2c_StartCondition();
//   error |= I2c_WriteByte (I2C_ADR_R);    //I2C address
//   u8SerialNumber[1] = I2c_ReadByte(ACK); //Read SNC_1
//   u8SerialNumber[0] = I2c_ReadByte(ACK); //Read SNC_0
//   I2c_ReadByte(ACK);                     //Read CRC SNC0/1 (CRC is not analyzed)
//   u8SerialNumber[7] = I2c_ReadByte(ACK); //Read SNA_1
//   u8SerialNumber[6] = I2c_ReadByte(ACK); //Read SNA_0
//   I2c_ReadByte(NO_ACK);                  //Read CRC SNA0/1 (CRC is not analyzed)
//   I2c_StopCondition();

//   return error;
// }

