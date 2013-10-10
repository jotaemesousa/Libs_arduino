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
    gain_ = 1;
}

void ADS1115::startConversion(char channel, char gain, char mode)
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

    if(gain >= 0 && gain < 7)
    {
    	high_byte &= ~0b00001110;
    	high_byte |= gain << 1;
    	gain_ = gain;
    }


    if(mode)
    {
    	high_byte |= 0x01;
    }
    else
    {
    	high_byte &= ~0x01;
    }

    
    // Send configuration data high byte
    Wire.write(high_byte);
    // Send configuration data low byte
    Wire.write(low_byte);
    
    Wire.endTransmission();
}

void ADS1115::startConversion_dif(char channel_pos, char channel_neg, char gain, char mode)
{
	byte high_byte;
	byte low_byte;
	char bits14_12 = 0;

	// Start communicating with ADS1115
	Wire.beginTransmission(_address);
	// Send configuration byte
	Wire.write(byte(CONFIGURATION_REG));

	switch(channel_pos)
	{
		case 0:

			if(channel_neg == 1)
			{
				bits14_12 = 0b000;
			}
			else if(channel_neg == 3)
			{
				bits14_12 = 0b001;
			}
			else
			{
				bits14_12 = 0b100;
			}
			high_byte = CH0_CONFIG_REG_H;
			low_byte = CH0_CONFIG_REG_L;
			break;

		case 1:
			if(channel_neg == 3)
			{
				bits14_12 = 0b010;
			}
			else
			{
				bits14_12 = 0b101;
			}
			high_byte = CH1_CONFIG_REG_H;
			low_byte = CH1_CONFIG_REG_L;
			break;

		case 2:
			if(channel_neg == 3)
			{
				bits14_12 = 0b011;
			}
			else
			{
				bits14_12 = 0b110;
			}
			high_byte = CH2_CONFIG_REG_H;
			low_byte = CH2_CONFIG_REG_L;
			break;

		default:
			bits14_12 = 0b111;
			high_byte = CH3_CONFIG_REG_H;
			low_byte = CH3_CONFIG_REG_L;
			break;

	}
	high_byte &= 0b10001111;
	high_byte |= bits14_12 << 4;

	if(gain >= 0 && gain < 7)
	{
		high_byte &= ~0b00001110;
		high_byte |= gain << 1;
	}

	if(mode)
	{
		high_byte |= 0x01;
	}
	else
	{
		high_byte &= ~0x01;
	}

	// Send configuration data high byte
	Wire.write(high_byte);
	// Send configuration data low byte
	Wire.write(low_byte);

	Wire.endTransmission();
}

void ADS1115::set_gain(char gain)
{
	int reg_data = readConfigurationRegister();
	byte high_byte = reg_data >> 8;
	byte low_byte = reg_data & 0x00FF;

	if(gain >= 0 && gain < 7)
	{
		high_byte &= ~0b00001110;
		high_byte |= gain << 1;
		gain_ = gain;
	}

	// Start communticating with ADS1115
	Wire.beginTransmission(_address);
	// Send configuration byte
	Wire.write(byte(CONFIGURATION_REG));
	// Send configuration data high byte
	Wire.write(high_byte);
	// Send configuration data low byte
	Wire.write(low_byte);

	Wire.endTransmission();
}

void ADS1115::set_continuous_conversion(void)
{
	int reg_data = readConfigurationRegister();
	byte high_byte = reg_data >> 8;
	byte low_byte = reg_data & 0x00FF;
	high_byte &= ~0x01;

	// Start communticating with ADS1115
	Wire.beginTransmission(_address);
	// Send configuration byte
	Wire.write(byte(CONFIGURATION_REG));
	// Send configuration data high byte
	Wire.write(high_byte);
	// Send configuration data low byte
	Wire.write(low_byte);

	Wire.endTransmission();
}

void ADS1115::set_single_shot_conversion(void)
{
	int reg_data = readConfigurationRegister();
	byte high_byte = reg_data >> 8;
	byte low_byte = reg_data & 0x00FF;
	high_byte |= 0x01;

	// Start communicating with ADS1115
	Wire.beginTransmission(_address);
	// Send configuration byte
	Wire.write(byte(CONFIGURATION_REG));
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
    
    uint16_t data = 0;
    
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
        data = (high_byte << 8) | low_byte;
//        Serial.print("new ");
//        Serial.print(high_byte, BIN);
//        Serial.println("");
//        Serial.print(low_byte, BIN);
//        Serial.println("");
    }

    

    return voltageConverter(data);
}

long int ADS1115::getRawValue()
{
    byte high_byte;
    byte low_byte;

     long int data = 0;

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
        data = (high_byte << 8) | low_byte;
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

int ADS1115::voltageConverter(int data)
{
    int converted_data = 0;

    if(gain_ == 0)
    {
    	unsigned long int data_temp = data;
    	converted_data = (data_temp * 100) / 533;
    }
    else if(gain_ == 1)	// estes valores deviam estar em micro volts....
    {
    	converted_data = data / 8;
    }
    else if(gain_ == 2)
    {
    	converted_data = data / 16;
    }
    else if(gain_ == 3)
    {
    	converted_data = data / 32;
    }
    else if(gain_ == 4)
    {
    	converted_data = data / 64;
    }
    else
    {
    	converted_data = data / 128;
    }

    return converted_data;
} 

// EOF
