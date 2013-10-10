/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, ISR University of Coimbra.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the ISR University of Coimbra nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Pedro Sousa pvsousa@isr.uc.pt
 * Converted to an Arduino library by Gonçalo Cabrita goncabrita@isr.uc.pt
 * ADS1115.h - Library for the ADS1115 I2C ADC.
 * 
 *********************************************************************/

#ifndef ADS1115_h
#define ADS1115_h

#include "Arduino.h"
#include <Wire.h>

#define DEFAULT_ADDRESS     0x49

#define CONVERSION_REG      0x00    // Conversion Register
#define CONFIGURATION_REG   0x01    // Configuration Register
#define LOW_THRESHOLD_REG   0x02    // Low threshold value Register
#define HIGH_THRESHOLD_REG  0x03    // High threshold value Register

/** \brief  Config Register Organization
 *	OS | MUX2 | MUX1 | MUX0 | PGA2 | PGA1 | PGA0 | MODE
 *	DR2 | DR1 | DR0 | COMP_MODE | COMP_POL | COMP_LAT | COMP_QUE1 | COMP_QUE0
 *
 *	register for single value, AN0 to GND FS = 4.096V, DR to 128 SPS
 *               C  3  8  3
 */
#define CH0_CONFIG_REG_H 	0b11000011
#define CH0_CONFIG_REG_L 	0b10000011

/** \brief Register for single value, AN1 to GND FS = 4.096V, DR to 128 SPS
 *               D  3  8  3
 */
#define CH1_CONFIG_REG_H 	0b11010011
#define CH1_CONFIG_REG_L 	0b10000011

/** \brief Register for single value, AN2 to GND FS = 4.096V, DR to 128 SPS
 *               E  3  8  3
 */
#define CH2_CONFIG_REG_H 	0b11100011
#define CH2_CONFIG_REG_L 	0b10000011

/** \brief Register for single value, AN3 to GND FS = 4.096V, DR to 128 SPS
 *               F  3  8  3
 */
#define CH3_CONFIG_REG_H 	0b11110011
#define CH3_CONFIG_REG_L 	0b10000011

class ADS1115
{
    public:
    /*! \fn ADS1115(char addr)
     \brief Constructor
     
     \param addr The I2C device address.
     */
    ADS1115(char addr);
    
    /*! \fn void startConversion(char channel)
     \brief Starts a new convertion for the desired channel and address.
     
     \param channel The required sensor channel.
     */
    void startConversion(char channel);
    
    /*! \fn getData()
     \brief Starts a new convertion for address addr.
     
     \return The previously acquired/converted data.
     */
    int getData();
    
    /*! \fn readConfigurationRegister()
     \brief Reads the current data in the configuration word (2 bytes register).
     
     \return The current configuration data.
     */
    int readConfigurationRegister();
    
    private:
    
    char _address;
    
};

#endif