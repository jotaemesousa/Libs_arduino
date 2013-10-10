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
 * ADS1115.cpp - Library for the ADS1115 I2C ADC.
 * 
 *********************************************************************/

#include "Arduino.h"
#include "ADS1115.h"

ADS1115::ADS1115(char addr)
{
    _address = addr;
}

void ADS1115::startConversion(char channel)
{
    char high_byte;
    char low_byte;
    
    // Start communticating with ADS1115
    Wire.beginTransmission(_address);
    // Send configuration byte
    Wire.write(byte(CONFIGURATION_REG));              
    
    // Select channel to read from
    switch(channel)
    {
        case 0:
            high_byte = CH0_CONFIG_REG_H;
            low_byte = CH0_CONFIG_REG_L;
            break;
        case 1:
            high_byte = CH1_CONFIG_REG_H;
            low_byte = CH1_CONFIG_REG_L;
            break;
        case 2:
            high_byte = CH2_CONFIG_REG_H;
            low_byte = CH2_CONFIG_REG_L;
            break;
        case 3:
            high_byte = CH3_CONFIG_REG_H;
            low_byte = CH3_CONFIG_REG_L;
            break;
        default:
            high_byte = CH0_CONFIG_REG_H;
            low_byte = CH0_CONFIG_REG_L;
            break;
    }
    
    // Send configuration data high byte
    Wire.write(high_byte);
    // Send configuration data low byte
    Wire.write(low_byte);
    
    Wire.endTransmission();
}

int ADS1115::getData()
{
    byte high_byte;
    byte low_byte;
    
    int data = 0; 
    
    // Start communicating with ADS1115
    Wire.beginTransmission(_address);
    Wire.write(byte(CONVERSION_REG));
    Wire.endTransmission();
    
    // Request 2 bytes from the ADS1115
    Wire.requestFrom(_address, 2);
    
    delayMicroseconds(10);
    
    // If two bytes were received
    if(2 <= Wire.available())
    {
        // Get high byte
        high_byte = Wire.read();
        // Get low byte
        low_byte = Wire.read();
    
        // Put them together
        data = (high_byte << 8) + low_byte;
    }
    
    return data;                   
}

int ADS1115::readConfigurationRegister()
{
    char high_byte;
    char low_byte;
    
    int data = 0;
    
    // Start communicating with ADS1115
    Wire.beginTransmission(_address);
    Wire.write(byte(CONFIGURATION_REG));
    Wire.endTransmission();
    
    Wire.requestFrom(_address, 2);
    
    delayMicroseconds(10);
    
    // If two bytes were received
    if(2 <= Wire.available())
    {
        // Get high byte
        high_byte = Wire.read();
        // Get low byte
        low_byte = Wire.read();
    
        // Put them together
        data = (high_byte << 8) + low_byte;
    }
    
    return data;
}

// EOF
