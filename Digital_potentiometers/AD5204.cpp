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
 * Author: João Sousa on 03/11/2012
 *********************************************************************/

#include "AD5204.h"
#include "Arduino.h"


AD5204::AD5204(int cs)
{
	cs_ = cs;					// sets pin 22 as default
	pinMode(cs_, OUTPUT); 		// we use this for SS pin
	SPI.begin(); 				// wake up the SPI bus.
	SPI.setBitOrder(MSBFIRST);

	for(int i = 0; i < N_POT; i++)
	{
		value_[i] = MAX_POT_VALUE/2;
		setWiper(value_[i],i);
	}
}

AD5204::~AD5204() {
	// TODO Auto-generated destructor stub
}

int AD5204::setWiper(int value, int wiper)
{
	if(value < 0) value = 0;
    if(value > MAX_POT_VALUE) value = MAX_POT_VALUE;

    if(wiper >= 0 && wiper < N_POT)
    {
    	this->value_[wiper] = value;
    	digitalWrite(cs_, 0);
    	SPI.transfer(wiper);
		SPI.transfer(value);
		digitalWrite(cs_, 1);
		return 0;
    }
    else
    {
    	return -1;
    }
}

int AD5204::incrementWiper(int wiper)
{
	if(wiper >= 0 && wiper < N_POT)
	{
		if(value_[wiper] < MAX_POT_VALUE)
		{
			digitalWrite(cs_, 0);
			SPI.transfer(wiper);
			SPI.transfer(++value_[wiper]);
			digitalWrite(cs_, 1);

			return 0;
		}
		else
		{
			return -1;
		}
	}
	else
	{
		return -1;
	}
}

int AD5204::decrementWiper(int wiper)
{
	if(wiper >= 0 && wiper < N_POT)
	{
		if(value_[wiper] > 0)
		{
			digitalWrite(cs_, 0);
			SPI.transfer(wiper);
			SPI.transfer(--value_[wiper]);
			digitalWrite(cs_, 1);

			return 0;
		}
		else
		{
			return -1;
		}
	}
	else
	{
		return -1;
	}
}
