/*
 * Sabertooth.cpp
 *
 *  Created on: Jun 11, 2013
 *      Author: joao
 */

#include "Sabertooth.h"
#include "Arduino.h"

Sabertooth::Sabertooth()
{
	pwm1 = 0;
	pwm2 = 0;
	duty1 = 0;
	duty2 = 0;
	duty_min1 = 10;
	duty_min2 = 10;
	max_pwm_fwd1 = 100;
	max_pwm_fwd2 = 100;
	max_pwm_rev1 = 100;
	max_pwm_rev2 = 100;
	last_time_stream = 0;
	driver_update = 10;
	last_driver_update = 0;
	communication_mode = 3;
	serial_port = 1;
	brate = 19200;
}

void Sabertooth::update_driver(void)
{
	if(driver_update > 0)
	{
		if(millis() - last_driver_update > driver_update)
		{
			last_driver_update = millis();
			normalize_pwm();

			uint8_t inv = action_motor(0);

			// motor 1
			if(duty1 < pwm1)
			{
				if(inv >= 1)
				{
					if(duty1 < duty_min1 && duty1 > 0)
					{
						duty1 = duty1 + ((duty_min1 - duty1) / 4) + 1;
					}
					else
					{
						duty1++;
					}
				}
				else if(inv == 0)
				{
					duty1++;
				}
			}
			else if(duty1 > pwm1)
			{
				if(inv >= 1)
				{
					if(duty1 > -duty_min1 && duty1 < 0)
					{
						duty1 = duty1-((duty_min1 + duty1) / 4) - 1;
					}
					else
					{
						duty1--;
					}
				}
				else if(inv == 0)
				{
					duty1--;
				}
			}

			inv = action_motor(1);
			// motor 2
			if(duty2 < pwm2)
			{
				if(inv >= 1)
				{
					if(duty2 < duty_min2 && duty2 > 0)
					{
						duty2 = duty2 + ((duty_min2 - duty2) / 4) + 1;
					}
					else
					{
						duty2++;
					}
				}
				else if(inv == 0)
				{
					duty2++;
				}
			}
			else if(duty2 > pwm2)
			{
				if(inv >= 1)
				{
					if(duty2 > -duty_min2 && duty2 < 0)
					{
						duty2 = duty2-((duty_min2 + duty2) / 4) - 1;
					}
					else
					{
						duty2--;
					}
				}
				else if(inv == 0)
				{
					duty2--;
				}
			}

			normalize_duty();
			send_driver_action();

		}
	}
	else
	{
		normalize_pwm();
		duty1 = pwm1;
		duty2 = pwm2;
		normalize_duty();
		send_driver_action();
	}
}

void Sabertooth::normalize_pwm(void)
{
	// Check whether if pwm is between max and min.
	if(pwm1 < -max_pwm_rev1)
	{
		pwm1 = -max_pwm_rev1;
	}
	else if(pwm1 > max_pwm_fwd1)
	{
		pwm1 = max_pwm_fwd1;
	}

	if(pwm1 > -duty_min1 && pwm1 < duty_min1)
	{
		pwm1 = 0;
	}

	if(pwm2 < -max_pwm_rev2)
	{
		pwm2 = -max_pwm_rev2;
	}
	else if(pwm2 > max_pwm_fwd2)
	{
		pwm2 = max_pwm_fwd2;
	}

	if(pwm2 > -duty_min2 && pwm2 < duty_min2)
	{
		pwm2 = 0;
	}
}
void Sabertooth::normalize_duty(void)
{
	// Check whether if duty is between max and min.
	if(duty1 < -max_pwm_rev1)
	{
		duty1 = -max_pwm_rev1;
	}
	else if(duty1 > max_pwm_fwd1)
	{
		duty1 = max_pwm_fwd1;
	}
	if(duty2 < -max_pwm_rev2)
	{
		duty2 = -max_pwm_rev2;
	}
	else if(duty2 > max_pwm_fwd2)
	{
		duty2 = max_pwm_fwd2;
	}
}

int Sabertooth::action_motor(bool channel)
{
	int a;

	if(!channel)
	{
		a = duty1;

		if(a * pwm1 > 0)
			return 2;
		else if(a * pwm1 < 0)
			return 0;
		else
			return 1;
	}
	else
	{
		a = duty2;

		if(a * pwm2 > 0)
			return 2;
		else if(a * pwm2 < 0)
			return 0;
		else
			return 1;
	}
}

void Sabertooth::select_comunnication_mode(int mode)
{
	if(mode >= 0 && mode <= 5)
	{
		communication_mode = mode;

		if(communication_mode == 5)
		{
			soft_serial = new SoftwareSerial(rx,tx);
		}
	}
}
void Sabertooth::select_baud_rate(unsigned int br)
{
	brate = br;
}
void Sabertooth::select_serial_port(char serial)
{
	if(serial >= 0 && serial <= 3)
	{
		serial_port = serial;
	}
}
void Sabertooth::init_communication(void)
{

	switch(communication_mode)
	{
#if __AVR_ATmega1280__
	case 3:
		switch(serial_port)
		{
		case 0:
			Serial.begin(brate);
			break;
		case 1:
			Serial1.begin(brate);
			break;
		case 2:
			Serial2.begin(brate);
			break;
		case 3:
			Serial3.begin(brate);
			break;
		}
		break;
#endif

		case 5:
			soft_serial->begin(brate);
			break;


	}
}

void Sabertooth::send_driver_action(void)
{
	uint8_t ser1,ser2;

	switch(communication_mode)
	{
	case 0:
		break;
	case 1:
		break;
	case 2:
		break;
	case 3:

		ser1 = 64 + duty1 * 63.0 / 100.0;
		ser2 = 191 + duty2 * 63.0 / 100.0;

		switch(serial_port)
		{
		case 0:
			Serial.write(ser1);
			Serial.write(ser2);
			break;
#if __AVR_ATmega1280__
		case 1:
			Serial1.write(ser1);
			Serial1.write(ser2);
			break;
		case 2:
			Serial2.write(ser1);
			Serial2.write(ser2);
			break;
		case 3:
			Serial3.write(ser1);
			Serial3.write(ser2);
			break;
#endif
		}

		break;
	case 4:
		break;
	case 5:

		ser1 = 64 + duty1 * 63.0 / 100.0;
		ser2 = 191 + duty2 * 63.0 / 100.0;
		soft_serial->write(ser1);
		soft_serial->write(ser2);

		break;
	}
}

void Sabertooth::set_pwm(int hb1, int hb2)
{
	pwm1 = hb1;
	pwm2 = hb2;
}
void Sabertooth::set_max_pwm_fwd(int pwm1, int pwm2)
{
	max_pwm_fwd1 = pwm1;
	max_pwm_fwd2 = pwm2;
}
void Sabertooth::set_max_pwm_rev(int pwm1, int pwm2)
{
	max_pwm_rev1 = pwm1;
	max_pwm_rev2 = pwm2;
}
void Sabertooth::set_driver_time_update(unsigned int time)
{
	driver_update = time;
}
