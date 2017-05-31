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
    unsigned char data[24];

    m_pressureAddr = m_settings->m_I2CPressureAddress;

    // check ID of chip

    if (!m_settings->HALRead(m_pressureAddr, BMP280_REG_ID, 1, &result, "Failed to read BMP280 id"))
        return false;

    if (result != BMP280_ID) {
        HAL_ERROR1("Incorrect BMP280 id %d\n", result);
        return false;
    }

    // get calibration data

    if (!m_settings->HALRead(m_pressureAddr, BMP280_REG_dig_T1, 24, data, "Failed to read BMP280 calibration data"))
        return false;

    m_dig_T1 = (int16_t)(((uint16_t)data[0]) << 8) + (uint16_t)data[1];
    m_dig_T2 = (int16_t)(((uint16_t)data[2]) << 8) + (uint16_t)data[3];
    m_dig_T3 = (int16_t)(((uint16_t)data[4]) << 8) + (uint16_t)data[5];
    m_dig_P1 = (((uint16_t)data[6]) << 8) + (uint16_t)data[7];
    m_dig_P2 = (((uint16_t)data[8]) << 8) + (uint16_t)data[9];
    m_dig_P3 = (((uint16_t)data[10]) << 8) + (uint16_t)data[11];
    m_dig_P4 = (((uint16_t)data[12]) << 8) + (uint16_t)data[13];
    m_dig_P5 = (((uint16_t)data[14]) << 8) + (uint16_t)data[15];
    m_dig_P6 = (((uint16_t)data[16]) << 8) + (uint16_t)data[17];
    m_dig_P7 = (((uint16_t)data[18]) << 8) + (uint16_t)data[19];
    m_dig_P8 = (((uint16_t)data[20]) << 8) + (uint16_t)data[21];
    m_dig_P9 = (((uint16_t)data[22]) << 8) + (uint16_t)data[23];
    // m_B1 = (int16_t)(((uint16_t)data[12]) << 8) + (uint16_t)data[13];
    // m_B2 = (int16_t)(((uint16_t)data[14]) << 8) + (uint16_t)data[15];
    // m_MB = (int16_t)(((uint16_t)data[16]) << 8) + (uint16_t)data[17];
    // m_MC = (int16_t)(((uint16_t)data[18]) << 8) + (uint16_t)data[19];
    // m_MD = (int16_t)(((uint16_t)data[20]) << 8) + (uint16_t)data[21];

    m_state = BMP280_STATE_IDLE;
    m_oss = BMP280_SCO_PRESSURECONV_ULP;
    return true;
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

        // call this function for testing only
        // should give T = 150 (15.0C) and pressure 6996 (699.6hPa)

        // setTestData();

        int32_t pressure = ((((uint32_t)(m_rawPressure)) << 8) + (uint32_t)(data[0])) >> (8 - m_oss);

        m_state = BMP280_STATE_IDLE;

        
        // calculate compensated temperature

        int32_t X1 = (((int32_t)m_rawTemperature - m_dig_T1) * m_dig_T2);

        // if ((X1 + m_MD) == 0) {
            // break;
        // }
        int32_t X2 = ((((int32_t)m_rawTemperature - m_dig_T1) * ((int32_t)m_rawTemperature)- m_dig_T1) * m_dig_T3);
        int32_t B5 = X1 + X2;
        m_temperature = (RTFLOAT)((B5 * 5) + 128);

        // calculate compensated pressure
        
        X1 = (B5 / 2.0 - 64000.0;
        X2 = (X1 * X1 * m_dig_P6) / 32786.0;
        int32_t X3 = X2 + X1 * m_dig_P5 * 2.0;
        int32_t B3 = ((X3 / 4.0 + m_dig_P4 * 65536.0) << m_oss);
        X1 = (m_dig_P3 * X1 * X1 / 524288.0 + m_dig_P2 * X1) / 524288.0;
        X1 = (1.0 + X1 / 32768.0) * m_dig_P1;
        
        
        if (X1 == 0)
        p = 0;
            else
        p = 1048576.0 - m_rawPressure;
        p = ((p - B3 / 4096.0) * 6250.0 / X1;
        X1 = m_dig_P9 * p * p / 2147483648.0;
        B3 = p * m_dig_P8 / 32768.0;
        m_pressure = (RTFLOAT)(p + (X1 + B3 + m_dig_P7) / 16.0) / (RTFLOAT)100;      // the extra 100 factor is to get 1hPa units

        m_validReadings = true;

        // printf("UP = %d, P = %f, UT = %d, T = %f\n", m_rawPressure, m_pressure, m_rawTemperature, m_temperature);
        break;
    }
}

void RTPressureBMP280::setTestData()
{
    m_dig_T1 = 408;
    m_dig_T2 = -72;
    m_dig_T3 = -14383;
    m_dig_P4 = 32741;
    m_dig_P5 = 32757;
    m_dig_P6 = 23153;
    m_dig_P7 = 32741;
    m_dig_P8 = 32757;
    m_dig_P9 = 23153;
    // m_B1 = 6190;
    // m_B2 = 4;
    // m_MB = -32767;
    // m_MC = -8711;
    // m_MD = 2868;

    m_rawTemperature = 27898;
    m_rawPressure = 23843;
}
