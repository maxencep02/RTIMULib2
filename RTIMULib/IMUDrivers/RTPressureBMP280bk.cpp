////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "RTPressureBMP280.h"
#include "bmp280.h"

#define BMP280_API
/*Enable the macro BMP280_API to use this support file */
/*----------------------------------------------------------------------------*
*  The following functions are used for reading and writing of
*	sensor data using I2C or SPI communication
*----------------------------------------------------------------------------*/
#ifdef BMP280_API
/*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is going to be read
 *	\param reg_data : This is the data read from the sensor, which is held in an array
 *	\param cnt : The no of bytes of data to be read
 */
s8 BMP280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
 /*	\Brief: The function is used as I2C bus reas
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is to be read
 *	\param reg_data : It is a value held in the array,
 *		which is read in the register
 *	\param cnt : The no of bytes of data to be read
 */
s8 BMP280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is to be written
 *	\param reg_data : It is a value held in the array,
 *		which is written in the register
 *	\param cnt : The no of bytes of data to be written
 */
s8 I2C_routine(void);
#endif
/********************End of I2C/SPI function declarations***********************/
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMP280_delay_msek(u32 msek);
/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */
s32 bmp280_data_readout_template(void);
/*----------------------------------------------------------------------------*
 *  struct bmp280_t parameters can be accessed by using bmp280
 *	bmp280_t having the following parameters
 *	Bus write function pointer: BMP280_WR_FUNC_PTR
 *	Bus read function pointer: BMP280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/
struct bmp280_t bmp280;
/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */
s32 bmp280_data_readout_template(void)
{
	/* The variable used to assign the standby time*/
	u8 v_standby_time_u8 = BMP280_INIT_VALUE;

	/* The variables used in individual data read APIs*/
	/* The variable used to read uncompensated temperature*/
	s32 v_data_uncomp_tem_s32 = BMP280_INIT_VALUE;
	/* The variable used to read uncompensated pressure*/
	s32 v_data_uncomp_pres_s32 = BMP280_INIT_VALUE;
	/* The variable used to read real temperature*/
	s32 v_actual_temp_s32 = BMP280_INIT_VALUE;
	/* The variable used to read real pressure*/
	u32 v_actual_press_u32 = BMP280_INIT_VALUE;

	/* The variables used in combined data read APIs*/
	/* The variable used to read uncompensated temperature*/
	s32 v_data_uncomp_tem_combined_s32 = BMP280_INIT_VALUE;
	/* The variable used to read uncompensated pressure*/
	s32 v_data_uncomp_pres_combined_s32 = BMP280_INIT_VALUE;
	/* The variable used to read real temperature*/
	s32 v_actual_temp_combined_s32 = BMP280_INIT_VALUE;
	/* The variable used to read real pressure*/
	u32 v_actual_press_combined_u32 = BMP280_INIT_VALUE;

	/* result of communication results*/
	s32 com_rslt = ERROR;
    
    
    
/*********************** START INITIALIZATION ************************/
  /*	Based on the user need configure I2C or SPI interface.
   *	It is example code to explain how to use the bmp280 API*/
   #ifdef BMP280_API
	I2C_routine();
	/*SPI_routine(); */
	#endif
/*--------------------------------------------------------------------------*
 *  This function used to assign the value/reference of
 *	the following parameters
 *	I2C address
 *	Bus Write
 *	Bus read
 *	Chip id
*-------------------------------------------------------------------------*/
	com_rslt = bmp280_init(&bmp280);

	/*	For initialization it is required to set the mode of
	 *	the sensor as "NORMAL"
	 *	data acquisition/read/write is possible in this mode
	 *	by using the below API able to set the power mode as NORMAL*/
	/* Set the power mode as NORMAL*/
	com_rslt += bmp280_set_power_mode(BMP280_NORMAL_MODE);
	/*	For reading the pressure and temperature data it is required to
	 *	set the work mode
	 *	The measurement period in the Normal mode is depends on the setting of
	 *	over sampling setting of pressure, temperature and standby time
	 *
	 *	OSS				pressure OSS	temperature OSS
	 *	ultra low power			x1			x1
	 *	low power			x2			x1
	 *	standard resolution		x4			x1
	 *	high resolution			x8			x2
	 *	ultra high resolution		x16			x2
	 */
	/* The oversampling settings are set by using the following API*/
	com_rslt += bmp280_set_work_mode(BMP280_STANDARD_RESOLUTION);
/*------------------------------------------------------------------------*
************************* START GET and SET FUNCTIONS DATA ****************
*---------------------------------------------------------------------------*/
	/* This API used to Write the standby time of the sensor input
	 *	value have to be given*/
	 /*	Normal mode comprises an automated perpetual cycling between an (active)
	 *	Measurement period and an (inactive) standby period.
	 *	The standby time is determined by the contents of the register t_sb.
	 *	Standby time can be set using BMP280_STANDBYTIME_125_MS.
	 *	Usage Hint : BMP280_set_standbydur(BMP280_STANDBYTIME_125_MS)*/

	com_rslt += bmp280_set_standby_durn(BMP280_STANDBY_TIME_125_MS);

	/* This API used to read back the written value of standby time*/
	com_rslt += bmp280_get_standby_durn(&v_standby_time_u8);
/*-----------------------------------------------------------------*
************************* END GET and SET FUNCTIONS ****************
*------------------------------------------------------------------*/

/************************* END INITIALIZATION *************************/




// /*------------------------------------------------------------------*
// ****** INDIVIDUAL APIs TO READ UNCOMPENSATED PRESSURE AND TEMPERATURE*******
// *---------------------------------------------------------------------*/
	// /* API is used to read the uncompensated temperature*/
	// com_rslt += bmp280_read_uncomp_temperature(&v_data_uncomp_tem_s32);

	// /* API is used to read the uncompensated pressure*/
	// com_rslt += bmp280_read_uncomp_pressure(&v_data_uncomp_pres_s32);

	// /* API is used to read the true temperature*/
	// /* Input value as uncompensated temperature*/
	// v_actual_temp_s32 = bmp280_compensate_temperature_int32(v_data_uncomp_tem_s32);

	// /* API is used to read the true pressure*/
	// /* Input value as uncompensated pressure*/
	// v_actual_press_u32 = bmp280_compensate_pressure_int32(v_data_uncomp_pres_s32);

/*------------------------------------------------------------------*
******* STAND-ALONE APIs TO READ COMBINED TRUE PRESSURE AND TEMPERATURE********
*---------------------------------------------------------------------*/


	/* API is used to read the uncompensated temperature and pressure*/
//	com_rslt += bmp280_read_uncomp_pressure_temperature(&v_data_uncomp_pres_combined_s32,
//	&v_data_uncomp_tem_combined_s32);

	/* API is used to read the true temperature and pressure*/
	com_rslt += bmp280_read_pressure_temperature(&v_actual_press_combined_u32,
	&v_actual_temp_combined_s32);



/************************* START DE-INITIALIZATION ***********************/

	/*	For de-initialization it is required to set the mode of
	 *	the sensor as "SLEEP"
	 *	the device reaches the lowest power consumption only
	 *	In SLEEP mode no measurements are performed
	 *	All registers are accessible
	 *	by using the below API able to set the power mode as SLEEP*/
	 /* Set the power mode as SLEEP*/
//	com_rslt += bmp280_set_power_mode(BMP280_SLEEP_MODE);

//   return com_rslt;
/************************* END DE-INITIALIZATION **********************/
}


#ifdef BMP280_API
/*--------------------------------------------------------------------------*
*	The following function is used to map the I2C bus read, write, delay and
*	device address with global structure bmp280_t
*-------------------------------------------------------------------------*/
s8 I2C_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bmp280 the following structure parameter can be accessed
 *	Bus write function pointer: BMP280_WR_FUNC_PTR
 *	Bus read function pointer: BMP280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
	bmp280.bus_write = BMP280_I2C_bus_write;
	bmp280.bus_read = BMP280_I2C_bus_read;
	bmp280.dev_addr = BMP280_I2C_ADDRESS2;
	bmp280.delay_msec = BMP280_delay_msek;

	return BMP280_INIT_VALUE;
}


/************** I2C/SPI buffer length ******/

#define	I2C_BUFFER_LEN 8
//#define SPI_BUFFER_LEN 5
#define BUFFER_LENGTH	0xFF
//#define	SPI_READ	0x80
//#define SPI_WRITE	0x7F
#define BMP280_DATA_INDEX	1
#define BMP280_ADDRESS_INDEX	2

/*-------------------------------------------------------------------*
*	This is a sample code for read and write the data by using I2C/SPI
*	Use either I2C or SPI based on your need
*	The device address defined in the bmp180.c
*
*-----------------------------------------------------------------------*/
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is to be written
 *	\param reg_data : It is a value held in the array,
 *		which is written in the register
 *	\param cnt : The no of bytes of data to be written
 */
s8  BMP280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP280_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN];
	u8 stringpos = BMP280_INIT_VALUE;
	array[BMP280_INIT_VALUE] = reg_addr;
	for (stringpos = BMP280_INIT_VALUE; stringpos < cnt; stringpos++) {
		array[stringpos + BMP280_DATA_INDEX] = *(reg_data + stringpos);
	}
	/*
	* Please take the below function as your reference for
	* write the data using I2C communication
	* "IERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	* add your I2C write function here
	* iError is an return value of I2C read function
	* Please select your valid return value
	* In the driver SUCCESS defined as BMP280_INIT_VALUE
	* and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated.Thus cnt+1 operation done in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/
	return (s8)iError;
}

 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is going to be read
 *	\param reg_data : This is the data read from the sensor, which is held in an array
 *	\param cnt : The no of data to be read
 */
s8  BMP280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP280_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN] = {BMP280_INIT_VALUE};
	u8 stringpos = BMP280_INIT_VALUE;
	array[BMP280_INIT_VALUE] = reg_addr;
	/* Please take the below function as your reference
	 * to read the data using I2C communication
	 * add your I2C rad function here.
	 * "IERROR = I2C_WRITE_READ_STRING(DEV_ADDR, ARRAY, ARRAY, 1, CNT)"
	 * iError is an return value of SPI write function
	 * Please select your valid return value
	 * In the driver SUCCESS defined as BMP280_INIT_VALUE
	 * and FAILURE defined as -1
	 */
	for (stringpos = BMP280_INIT_VALUE; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = array[stringpos];
	}
	return (s8)iError;
}

/*	Brief : The delay routine
 *	\param : delay in ms
*/
void  BMP280_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
}
#endif





















RTPressureBMP280::RTPressureBMP280(RTIMUSettings *settings) : RTPressure(settings)
{
    m_validReadings = false;
}

RTPressureBMP280::~RTPressureBMP280()
{
}

bool RTPressureBMP280::pressureInit()
{
    unsigned char result;
    unsigned char data[22];

    m_pressureAddr = m_settings->m_I2CPressureAddress;

    // check ID of chip

    if (!m_settings->HALRead(m_pressureAddr, BMP280_REG_ID, 1, &result, "Failed to read BMP280 id"))
        return false;

    if (result != BMP280_ID) {
        HAL_ERROR1("Incorrect BMP280 id %d\n", result);
        return false;
    }
}

bool RTPressureBMP280::pressureRead(RTIMU_DATA& data)
{
    data.pressureValid = false;
    data.temperatureValid = false;
    data.temperature = 0;
    data.pressure = 0;

    if (m_state == BMP280_STATE_IDLE) {
        // start a temperature conversion
        if (!m_settings->HALWrite(m_pressureAddr, BMP280_REG_SCO, BMP280_SCO_TEMPCONV, "Failed to start temperature conversion")) {
            return false;
        } else {
            m_state = BMP280_STATE_TEMPERATURE;
        }
    }

    pressureBackground();

    if (m_validReadings) {
        data.pressureValid = true;
        data.temperatureValid = true;
        data.temperature = m_temperature;
        data.pressure = m_pressure;
        // printf("P: %f, T: %f\n", m_pressure, m_temperature);
    }
    return true;
}


void RTPressureBMP280::pressureBackground()
{
    uint8_t data[2];

    switch (m_state) {
        case BMP280_STATE_IDLE:
        break;

        case BMP280_STATE_TEMPERATURE:
        if (!m_settings->HALRead(m_pressureAddr, BMP280_REG_SCO, 1, data, "Failed to read BMP280 temp conv status")) {
            break;
        }
        if ((data[0] & 0x20) == 0x20)
            break;                                      // conversion not finished
        if (!m_settings->HALRead(m_pressureAddr, BMP280_REG_RESULT, 2, data, "Failed to read BMP280 temp conv result")) {
            m_state = BMP280_STATE_IDLE;
            break;
        }
        m_rawTemperature = (((uint16_t)data[0]) << 8) + (uint16_t)data[1];

        data[0] = 0x34 + (m_oss << 6);
        if (!m_settings->HALWrite(m_pressureAddr, BMP280_REG_SCO, 1, data, "Failed to start pressure conversion")) {
            m_state = BMP280_STATE_IDLE;
            break;
        }
        m_state = BMP280_STATE_PRESSURE;
        break;

        case BMP280_STATE_PRESSURE:
        if (!m_settings->HALRead(m_pressureAddr, BMP280_REG_SCO, 1, data, "Failed to read BMP280 pressure conv status")) {
            break;
        }
        if ((data[0] & 0x20) == 0x20)
            break;                                      // conversion not finished
        if (!m_settings->HALRead(m_pressureAddr, BMP280_REG_RESULT, 2, data, "Failed to read BMP280 temp conv result")) {
            m_state = BMP280_STATE_IDLE;
            break;
        }
        m_rawPressure = (((uint16_t)data[0]) << 8) + (uint16_t)data[1];

        if (!m_settings->HALRead(m_pressureAddr, BMP280_REG_XLSB, 1, data, "Failed to read BMP280 XLSB")) {
            m_state = BMP280_STATE_IDLE;
            break;
        }


        
        
        
        
        

        int32_t pressure = ((((uint32_t)(m_rawPressure)) << 8) + (uint32_t)(data[0])) >> (8 - m_oss);

        m_state = BMP280_STATE_IDLE;


        
        
        
        
        
        
        m_temperature = (RTFLOAT)((B5 + 8) / 16) / (RTFLOAT)10;

        m_pressure = (RTFLOAT)(p + (X1 + X2 + 3791) / 16) / (RTFLOAT)100;      // the extra 100 factor is to get 1hPa units

        m_validReadings = true;

        break;
    }
}

