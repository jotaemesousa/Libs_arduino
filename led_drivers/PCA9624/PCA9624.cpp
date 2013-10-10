/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, ISR University of Coimbra.
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
 * Author: Gonçalo Cabrita on 02/04/2013
 *********************************************************************/

//#if ARDUINO >= 100
//#include <Arduino.h>
//#else
//#include <WProgram.h>
//#endif
#include "Arduino.h"
#include "PCA9624.h"

PCA9624::PCA9624(char addr)
{
    address_[0] = addr;  
}

void PCA9624::writeMode1(char autoinc, char sleep, char sub1, char sub2, char sub3, char allcall, char addr)
{
    autoinc_ = autoinc;
    sleep_ = sleep;
    address_state_[0] = sub1;
    address_state_[1] = sub2;
    address_state_[3] = sub3;
    address_state_[4] = allcall;
    
    char mode1 = 0x00;
    
    mode1 = allcall;
    mode1 += sub1 << 1;
    mode1 += sub2 << 2;
    mode1 += sub3 << 3;
    mode1 += sleep << 4;
    mode1 += autoinc << 5;
    
    writeRegister(PCA9624_REG_MODE1, mode1, addr);
}

char PCA9624::readMode1()
{
    char mode1 = readRegister(PCA9624_REG_MODE1);
    
    address_state_[3] = mode1 & 0x01; // All Call
    address_state_[2] = (mode1 >> 1) & 0x01; // Sub3
    address_state_[1] = (mode1 >> 2) & 0x01; // Sub2
    address_state_[0] = (mode1 >> 3) & 0x01; // Sub3
    sleep_ = (mode1 >> 4) & 0x01;
    autoinc_ = (mode1 >> 5) & 0x07;
    
    return mode1;
}

void PCA9624::writeAddress(char addr, char value)
{
    if(addr >= PCA9624_ADDR_SUBADR1 && addr <= PCA9624_ADDR_ALLCALLADR)
    {
        address_[addr] = value;
        writeRegister(addr+0x0D, value);
    }
}

char PCA9624::readAddress(char addr)
{
    if(addr >= PCA9624_ADDR_SUBADR1 && addr <= PCA9624_ADDR_ALLCALLADR)
    {
        char value = readRegister(addr+0x0D);
        address_[addr] = value;
        return value;
    }
    
    return 0x00;
}

char PCA9624::getAddressFlag(char addr)
{
    if(addr >= PCA9624_ADDR_SUBADR1 && addr <= PCA9624_ADDR_ALLCALLADR)
    {
        return address_state_[addr-1];
    }
    return 0;
}

void PCA9624::writeMode2(char dmblk, char invrt, char och, char addr)
{
    dmblk_ = dmblk;
    invrt_ = invrt;
    och_ = och;
    
    char mode2 = 0x00;
    
    mode2 += 0x5;
    mode2 += och << 3;
    mode2 += invrt << 4;
    mode2 += dmblk << 5;
    
    writeRegister(PCA9624_REG_MODE2, mode2, addr);
}

char PCA9624::readMode2()
{
    char mode2 = readRegister(PCA9624_REG_MODE2);
    
    och_ = (mode2 >> 3) & 0x01;
    invrt_ = (mode2 >> 4) & 0x01;
    dmblk_ = (mode2 >> 5) & 0x01;
    
    return mode2;
}

void PCA9624::writeLedOut(char led0, char led1, char led2, char led3, char led4, char led5, char led6, char led7, char addr)
{
    ledout_[0] = led0;
    ledout_[1] = led1;
    ledout_[2] = led2;
    ledout_[3] = led3;
    ledout_[4] = led4;
    ledout_[5] = led5;
    ledout_[6] = led6;
    ledout_[7] = led7;
    
    char ledout;
    
    ledout = led0;
    ledout += led1 << 2;
    ledout += led2 << 4;
    ledout += led3 << 6;
    
    writeRegister(PCA9624_REG_LEDOUT0, ledout, addr);
    
    ledout = led4;
    ledout += led5 << 2;
    ledout += led6 << 4;
    ledout += led7 << 6;
    
    writeRegister(PCA9624_REG_LEDOUT1, ledout, addr);
}

void PCA9624::readLedOut()
{
    char ledout;
    
    ledout = readRegister(PCA9624_REG_LEDOUT0);
    
    ledout_[0] = ledout & 0x03;
    ledout_[1] = (ledout >> 2) & 0x03;
    ledout_[2] = (ledout >> 4) & 0x03;
    ledout_[3] = (ledout >> 6) & 0x03;
    
    ledout = readRegister(PCA9624_REG_LEDOUT1);
    
    ledout_[4] = ledout & 0x03;
    ledout_[5] = (ledout >> 2) & 0x03;
    ledout_[6] = (ledout >> 4) & 0x03;
    ledout_[7] = (ledout >> 6) & 0x03;
}

char PCA9624::getLedOut(char led)
{
    if(led >= PCA9624_LED_0 && led <= PCA9624_LED_7)
    {
        return ledout_[led];
    }
}

void PCA9624::writePWM(char led, char duty, char addr)
{
    if(led >= PCA9624_LED_0 && led <= PCA9624_LED_ALL)
    {
        writeRegister(led+0x02, duty, addr);
    }
}

char PCA9624::readPWM(char led)
{
    if(led >= PCA9624_LED_0 && led <= PCA9624_LED_7)
    {
        return readRegister(led+0x02);
    }
    return 0x00;
}

void PCA9624::writeGroupFrequency(char freq, char addr)
{
    writeRegister(PCA9624_REG_GRPFREQ, freq, addr);
}

char PCA9624::readGroupFrequency()
{
    return readRegister(PCA9624_REG_GRPFREQ);
}

void PCA9624::writeRegister(char reg, char value, char addr)
{        
    Wire.beginTransmission(address_[addr]);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
	delay(10);
}

char PCA9624::readRegister(char reg, char addr)
{
    char reply = 0;
    
    Wire.beginTransmission(address_[addr]);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(address_[addr], 1);
    delayMicroseconds(10);
    if(1 <= Wire.available())
    {
        reply = int(Wire.read());
    }
    return reply;
}

// EOF
