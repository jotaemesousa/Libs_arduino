/*
 * DS1803.cpp
 *
 *  Created on: 5 de Nov de 2012
 *      Author: João
 */

#include "DS1803.h"

DS1803::DS1803(int address) {
	// TODO Auto-generated constructor stub
	addr = address;
	Wire.begin();

}

/*DS1803::~DS1803() {
	// TODO Auto-generated destructor stub
}*/


/* Set the values of the wiper (potentiometers)
    wiper 0 is pot0, wiper 1 is pot1 and wiper2 is pot0 and pot1
    Value is between 0 and 255
*/
int DS1803::setWiper(int value, int wiper)
{
	Wire.beginTransmission(this->addr);

	if(value < 0)
	{
		value = 0;
	}
	else if(value > MAX_POT_VALUE)
	{
		value = MAX_POT_VALUE ;
	}

	switch(wiper)
	{
	case 0:
		Wire.write(WIPER_0);
		value_[0] = value;
		break;
	case 1:
		Wire.write(WIPER_1);
		value_[1] = value;
		break;
	case 2:
		Wire.write(WIPER_01);
		value_[0] = value;
		value_[1] = value;
		break;
	default:
		return -1;
	}


	Wire.write(value);
	Wire.endTransmission();

	return 0;
}

// Return the value of wiper read from the IC
unsigned int DS1803::getValue(int wiper)
{
	Wire.requestFrom(this->addr,2);
	int *values=(int*)malloc(sizeof(int)*2);

	int k=0;
	while (Wire.available()) {
		values[k]=Wire.read();
		value_[k]=values[k];
		k++;
	}
	int ret;
	if(wiper >= 0 && wiper < 2)
	{
		ret = values[wiper];
		value_[wiper] = ret;
		free(values);
		return ret;
	}
	else
	{
		free(values);
		return -1;
	}
}

int DS1803::incrementWiper(int wiper, int value)
{
	if(wiper >= 0 && wiper < N_POT)
	{
		if (value_[wiper] + value <= MAX_POT_VALUE)
		{
			setWiper(++value_[wiper], wiper);
		}
		return 0;
	}
	else if(wiper == 2)
	{
		for (int i = 0; i < N_POT; ++i)
		{
			if (value_[wiper] + value <= MAX_POT_VALUE)
			{
				setWiper(++value_[wiper], wiper);
			}
		}
		return 0;
	}
	else
	{
		return -1;
	}
}

int DS1803::decrementWiper(int wiper, int value)
{
	if(wiper >= 0 && wiper < N_POT)
	{
		if (value_[wiper] - value >= 0)
		{
			setWiper(--value_[wiper], wiper);
		}
		return 0;
	}
	else if(wiper == 2)
	{
		for (int i = 0; i < N_POT; ++i)
		{
			if (value_[wiper] - value >= 0)
			{
				setWiper(--value_[wiper], wiper);
			}
		}
		return 0;
	}
	else
	{
		return -1;
	}
}
