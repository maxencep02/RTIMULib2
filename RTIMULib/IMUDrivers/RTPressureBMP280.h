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

#ifndef _RTPRESSUREBMP280_H_
#define _RTPRESSUREBMP280_H_


#include "RTPressure.h"

//  State definitions

#define BMP280_STATE_IDLE               0
#define BMP280_STATE_TEMPERATURE        1
#define BMP280_STATE_PRESSURE           2

//  Conversion reg defs

#define BMP280_status_TEMPCONV             0xf4                // temperature conversion
#define BMP280_status_PRESSURECONV_ULP     0                   // ultra low power pressure conversion
#define BMP280_status_PRESSURECONV_LP      1                   // low power pressure conversion
#define BMP280_status_PRESSURECONV_STD     2                   // standard pressure conversion
#define BMP280_status_PRESSURECONV_HR      3                   // high res pressure conversion
#define BMP280_status_PRESSURECONV_UHR     4                   // ultra high res pressure conversion

class RTIMUSettings;

class RTPressureBMP280 : public RTPressure
{
public:
    RTPressureBMP280(RTIMUSettings *settings);
    ~RTPressureBMP280();

    virtual const char *pressureName() { return "BMP280"; }
    virtual int pressureType() { return RTPRESSURE_TYPE_BMP280; }
    virtual bool pressureInit();
    virtual bool pressureRead(RTIMU_DATA& data);

private:
    void pressureBackground();
    void setTestData();

    unsigned char m_pressureAddr;                           // I2C address
    RTFLOAT m_pressure;                                     // the current pressure
    RTFLOAT m_temperature;                                  // the current temperature

    // This is the calibration data read from the sensor

    int32_t m_dig_T1;
    int32_t m_dig_T2;
    int32_t m_dig_T3;
    uint32_t m_dig_P1;
    uint32_t m_dig_P2;
    uint32_t m_dig_P3;
    uint32_t m_dig_P4;
    uint32_t m_dig_P5;
    uint32_t m_dig_P6;
    uint32_t m_dig_P7;
    uint32_t m_dig_P8;
    uint32_t m_dig_P9;
    // int32_t m_B1;
    // int32_t m_B2;
    // int32_t m_MB;
    // int32_t m_MC;
    // int32_t m_MD;

    int m_state;
    int m_oss;

    uint16_t m_rawPressure;
    uint16_t m_rawTemperature;

    bool m_validReadings;
};

#endif // _RTPRESSUREBMP280_H_

