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
 * Author: Gonçalo Cabrita on 30/10/2011
 *********************************************************************/

#include <Wire.h>

#ifndef MCP444x_h
#define MCP444x_h

#define POT_ADDR  					0x2C

#define MAX_POT_VALUE  				256

// Addresses
#define VOLATILE_WIPER_0        	0x00
#define VOLATILE_WIPER_1        	0x01
#define NON_VOLATILE_WIPER_0    	0x02
#define NON_VOLATILE_WIPER_1    	0x03
#define VOLATILE_TCON_REGISTER_0 	0x04
#define STATUS_REGISTER         	0x05
#define VOLATILE_WIPER_2        	0x06
#define VOLATILE_WIPER_3        	0x07
#define NON_VOLATILE_WIPER_2    	0x08
#define NON_VOLATILE_WIPER_3    	0x09
#define VOLATILE_TCON_REGISTER_1  	0x0A
#define DATA_EEPROM_0           	0x0B
#define DATA_EEPROM_1           	0x0C
#define DATA_EEPROM_2           	0x0D
#define DATA_EEPROM_3           	0x0E
#define DATA_EEPROM_4           	0x0F

// Command Operations
#define WRITE_DATA					0
#define INCREMENT					1
#define DECREMENT					2
#define READ_DATA					3

// Wipers
#define WIPER_0						0
#define WIPER_1						1
#define WIPER_2						2
#define WIPER_3						3

// Memory Type
#define NON_VOLATILE    			0
#define VOLATILE        			1

// State
#define LOCKED						0
#define UNLOCKED					1

class MCP444x
{
	public:
    MCP444x(char addr = 0);

    int wiperLocked(char wiper);
    int unlockWiper(char wiper);
	int lockWiper(char wiper);
	int setAddr(unsigned int addr) { addr_ = addr;};
	int getAddr(void) {return addr_;};
	int setReset(char reset) { reset_ = reset;};
	int getReset(void) {return reset_;};
	int setWriteProtect(char wp) { wp_ = wp;};
	int getWriteProtect(void) {return wp_;};



	int setWiper(int value, char wiper, char type);
	int getWiper(char wiper, char type);
	int incrementWiper(char wiper);
	int decrementWiper(char wiper);

	private:
	char addr_;
    char wiper_0_status;
	char wiper_1_status;
	char wiper_2_status;
	char wiper_3_status;
	char reset_;
	char wp_;
};

#endif

// EOF
