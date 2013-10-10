/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, ISR University of Coimbra.
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
 * Author: Gonçalo Cabrita on 22/10/2011
 *********************************************************************/

#include "MCP444x.h"

MCP444x::MCP444x(char addr)
{

	wiper_0_status = LOCKED;
	wiper_1_status = LOCKED;
	if (addr > 0)
	{
		addr_ = addr;
	}
	else
	{
		addr_ = POT_ADDR;
	}
}

int MCP444x::wiperLocked(char wiper)
{
    if(wiper == WIPER_0) return !wiper_0_status;
    else if(wiper == WIPER_1) return !wiper_1_status;
    else if(wiper == WIPER_2) return !wiper_2_status;
    else if(wiper == WIPER_3) return !wiper_3_status;
    else return -1;
}

int MCP444x::unlockWiper(char wiper)
{
    char memory_address;
    if(wiper == WIPER_0) memory_address = NON_VOLATILE_WIPER_0 << 4;
    else if(wiper == WIPER_1)  memory_address = NON_VOLATILE_WIPER_1 << 4;
    else if(wiper == WIPER_2)  memory_address = NON_VOLATILE_WIPER_2 << 4;
    else if(wiper == WIPER_3)  memory_address = NON_VOLATILE_WIPER_3 << 4;
    else return -1;

    char command = memory_address + (DECREMENT << 2);

    Wire.beginTransmission(addr_);
    Wire.write(command);
    Wire.endTransmission();

    if(wiper == WIPER_0) wiper_0_status = UNLOCKED;
    else if(wiper == WIPER_1) wiper_1_status = UNLOCKED;
    else if(wiper == WIPER_2) wiper_2_status = UNLOCKED;
    else if(wiper == WIPER_3) wiper_3_status = UNLOCKED;

    return 0;
}

int MCP444x::lockWiper(char wiper)
{
    char memory_address;
    if(wiper == WIPER_0) memory_address = NON_VOLATILE_WIPER_0 << 4;
    else if(wiper == WIPER_1)  memory_address = NON_VOLATILE_WIPER_1 << 4;
    else if(wiper == WIPER_2)  memory_address = NON_VOLATILE_WIPER_2 << 4;
    else if(wiper == WIPER_3)  memory_address = NON_VOLATILE_WIPER_3 << 4;
    else return -1;

    char command = memory_address + (INCREMENT << 2);

    Wire.beginTransmission(addr_);
    Wire.write(command);
    Wire.endTransmission();

    if(wiper == WIPER_0) wiper_0_status = LOCKED;
    else if(wiper == WIPER_1) wiper_1_status = LOCKED;
    else if(wiper == WIPER_2) wiper_2_status = LOCKED;
    else if(wiper == WIPER_3) wiper_3_status = LOCKED;

    return 0;
}

int MCP444x::setWiper(int value, char wiper, char type)
{
    if(value < 0) value = 0;
    if(value > MAX_POT_VALUE) value = MAX_POT_VALUE;

    char memory_address;
    if(wiper == WIPER_0 && type == VOLATILE) memory_address = VOLATILE_WIPER_0 << 4;
    else if(wiper == WIPER_1 && type == VOLATILE) memory_address = VOLATILE_WIPER_1 << 4;
    else if(wiper == WIPER_2 && type == VOLATILE) memory_address = VOLATILE_WIPER_2 << 4;
    else if(wiper == WIPER_3 && type == VOLATILE) memory_address = VOLATILE_WIPER_3 << 4;
    else if(wiper == WIPER_0 && type == NON_VOLATILE)
    {
		if(wiper_0_status == LOCKED) return -1;
		memory_address = NON_VOLATILE_WIPER_0 << 4;
    }
    else if(wiper == WIPER_1 && type == NON_VOLATILE)
    {
		if(wiper_1_status == LOCKED) return -1;
		memory_address = NON_VOLATILE_WIPER_1 << 4;
    }
    else if(wiper == WIPER_2 && type == NON_VOLATILE)
    {
    	if(wiper_2_status == LOCKED) return -1;
    	memory_address = NON_VOLATILE_WIPER_2 << 4;
    }
    else if(wiper == WIPER_3 && type == NON_VOLATILE)
    {
    	if(wiper_3_status == LOCKED) return -1;
    	memory_address = NON_VOLATILE_WIPER_3 << 4;
    }
    else return -1;

    char command = memory_address + (value >> 8);
    char data = value;

    Wire.beginTransmission(addr_);
    Wire.write(command);
    Wire.write(data);
    Wire.endTransmission();

    return 0;
}

int MCP444x::getWiper(char wiper, char type)
{
    int value;

    char memory_address;
    if(wiper == WIPER_0 && type == VOLATILE) memory_address = VOLATILE_WIPER_0 << 4;
    else if(wiper == WIPER_1 && type == VOLATILE) memory_address = VOLATILE_WIPER_1 << 4;
    else if(wiper == WIPER_2 && type == VOLATILE) memory_address = VOLATILE_WIPER_2 << 4;
    else if(wiper == WIPER_3 && type == VOLATILE) memory_address = VOLATILE_WIPER_3 << 4;
    else if(wiper == WIPER_0 && type == NON_VOLATILE)
    {
		if(wiper_0_status == LOCKED) return -1;
		memory_address = NON_VOLATILE_WIPER_0 << 4;
    }
    else if(wiper == WIPER_1 && type == NON_VOLATILE)
    {
		if(wiper_1_status == LOCKED) return -1;
		memory_address = NON_VOLATILE_WIPER_1 << 4;
    }
    else if(wiper == WIPER_2 && type == NON_VOLATILE)
    {
    	if(wiper_2_status == LOCKED) return -1;
    	memory_address = NON_VOLATILE_WIPER_2 << 4;
    }
    else if(wiper == WIPER_3 && type == NON_VOLATILE)
    {
    	if(wiper_3_status == LOCKED) return -1;
    	memory_address = NON_VOLATILE_WIPER_3 << 4;
    }
    else return -1;

    char command = memory_address + (READ_DATA << 2);

    Wire.beginTransmission(addr_);
    Wire.write(command);
    Wire.endTransmission();

    Wire.requestFrom(addr_, 2);

    if(2 <= Wire.available())
    {
        value = Wire.read();
        value = value << 8;
        value |= Wire.read();

        return value;
    }
    return -1;
}

int MCP444x::incrementWiper(char wiper)
{
    char memory_address;
    if(wiper == WIPER_0) memory_address = VOLATILE_WIPER_0 << 4;
    else if(wiper == WIPER_1)  memory_address = VOLATILE_WIPER_1 << 4;
    else if(wiper == WIPER_2)  memory_address = VOLATILE_WIPER_2 << 4;
    else if(wiper == WIPER_3)  memory_address = VOLATILE_WIPER_3 << 4;
    else return -1;

    char command = memory_address + (INCREMENT << 2);

    Wire.beginTransmission(addr_);
    Wire.write(command);
    Wire.endTransmission();

    return 0;
}

int MCP444x::decrementWiper(char wiper)
{
    char memory_address;
    if(wiper == WIPER_0) memory_address = VOLATILE_WIPER_0 << 4;
    else if(wiper == WIPER_1)  memory_address = VOLATILE_WIPER_1 << 4;
    else if(wiper == WIPER_2)  memory_address = VOLATILE_WIPER_2 << 4;
    else if(wiper == WIPER_3)  memory_address = VOLATILE_WIPER_3 << 4;
    else return -1;

    char command = memory_address + (DECREMENT << 2);

    Wire.beginTransmission(addr_);
    Wire.write(command);
    Wire.endTransmission();

    return 0;
}

// EOF
