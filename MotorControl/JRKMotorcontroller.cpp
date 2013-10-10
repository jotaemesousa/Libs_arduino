/*
 * JRKMotorcontroller.cpp
 *
 *  Created on: Jun 12, 2013
 *      Author: joao
 */

#include "JRKMotorcontroller.h"
#include "Arduino.h"

JRK_Motor_controller::JRK_Motor_controller()
{
	driver_update = 100;
	set_point = LINEAR_ACTUATOR_CENTER;
	max_length = LINEAR_ACTUATOR_MAX;
	min_length = LINEAR_ACTUATOR_ZERO;
	last_driver_update = 0;
	communication_mode = 3;
	serial_port = 2;
	brate = 19200;
	deviceID = 0x11;
}

void JRK_Motor_controller::send_byte(uint8_t byte)
{
	switch(serial_port)
	{
	case 0:
		Serial.write(byte);
		break;
#if __AVR_ATmega1280__
	case 1:
		Serial1.write(byte);
		break;
	case 2:
		Serial2.write(byte);
		break;
	case 3:
		Serial3.write(byte);
		break;
#endif
	}
}

uint8_t JRK_Motor_controller::get_nbytes(uint8_t nbytes, uint8_t *byte)
{
	unsigned long int last_millis = millis();
	uint8_t ret = nbytes;

	while(millis() - last_millis < RECEIVE_TIMEOUT && ret)
	{
		switch(serial_port)
		{

		case 0:
			if(Serial.available())
			{
				byte[nbytes - ret] = Serial.read();
				ret--;
			}
			break;
#if __AVR_ATmega1280__
		case 1:
			if(Serial1.available())
			{
				byte[nbytes - ret] = Serial1.read();
				ret--;
			}
			break;
		case 2:
			if(Serial2.available())
			{
				byte[nbytes - ret] = Serial2.read();
				ret--;
			}
			break;
		case 3:
			if(Serial3.available())
			{
				byte[nbytes - ret] = Serial3.read();
				ret--;
			}
			break;
#endif
		}
	}
	return ret;
}

void JRK_Motor_controller::send_driver_action(void)
{
	uint8_t ser1,ser2;

	switch(communication_mode)
	{
	case 0:
		break;
	case 1:
		break;

	case 3:

		ser1 = 0xAA;
		ser2 = deviceID;

		send_byte(ser1);
		send_byte(ser2);

	case 2:

		ser1= 0xC0 + (set_point & 0x1F);
		ser2 = (set_point >> 5) & 0x7F;

		send_byte(ser1);
		send_byte(ser2);

		break;
	}
}

void JRK_Motor_controller::send_driver_action(uint8_t cmd, uint8_t nr_received_bytes, uint8_t *ret)
{
	switch(communication_mode)
	{
	case 3:
		send_byte(0xAA);
		send_byte(deviceID);
		send_byte(cmd);

		get_nbytes(nr_received_bytes,ret);
		break;

	case 2:
		send_byte(cmd);
		get_nbytes(nr_received_bytes, ret);

		break;
	}
}

void JRK_Motor_controller::select_comunnication_mode(Comm_mode mode)
{
	if(mode >= 0 && mode <= 3)
	{
		communication_mode = mode;
	}
}
void JRK_Motor_controller::select_baud_rate(unsigned int br)
{
	brate = br;
}
void JRK_Motor_controller::select_serial_port(Serial_port serial)
{
	if(serial >= 0 && serial <= 3)
	{
		serial_port = serial;
	}
}
void JRK_Motor_controller::init_communication(void)
{
	switch(communication_mode)
	{
	case 3:
		switch(serial_port)
		{
		case 0:
			Serial.begin(brate);
			break;
#if __AVR_ATmega1280__
		case 1:
			Serial1.begin(brate);
			break;
		case 2:
			Serial2.begin(brate);
			break;
		case 3:
			Serial3.begin(brate);
			break;
#endif
		}
		break;
	}
}

void JRK_Motor_controller::set_position(unsigned int pos)
{
	if(pos <= max_length && pos >= min_length)
	{
		set_point = pos;
	}
	else if(pos > max_length)
	{
		set_point = max_length;
	}
	else if(pos < min_length)
	{
		set_point = min_length;
	}
}

void JRK_Motor_controller::update_driver(void)
{
	if(millis() - last_driver_update > driver_update)
	{
		last_driver_update = millis();

		send_driver_action();
	}

}

void JRK_Motor_controller::set_driver_time_update(unsigned int time)
{
	if(time >= 0 && time <= 100)
	{
		driver_update = time;
	}
}

uint16_t JRK_Motor_controller::get_feedback(void)
{
	uint8_t ret[2];
	send_driver_action(0xA5, 2, ret);

	return ret[0] | (ret[1]<<8);
}

uint16_t JRK_Motor_controller::get_input(void)
{
	uint8_t ret[2];
	send_driver_action(0xA1, 2, ret);

	return ret[0] | (ret[1]<<8);
}

uint16_t JRK_Motor_controller::get_target(void)
{
	uint8_t ret[2];
	send_driver_action(0xA3, 2, ret);

	return ret[0] | (ret[1]<<8);
}

uint16_t JRK_Motor_controller::get_scaled_feedback(void)
{
	uint8_t ret[2];
	send_driver_action(0xA7, 2, ret);

	return ret[0] | (ret[1]<<8);
}

int16_t JRK_Motor_controller::get_error_sum(void)
{
	//TODO:
}

int16_t JRK_Motor_controller::get_duty_cycle_target(void)
{
	//TODO:
}

int16_t JRK_Motor_controller::get_duty_cycle(void)
{
	//TODO:
}

uint8_t JRK_Motor_controller::get_current(void)
{
	uint8_t ret[1];
	send_driver_action(0x8F, 1, ret);

	return ret[0];
}

uint16_t JRK_Motor_controller::get_PID_period_count(void)
{
	uint8_t ret[2];
	send_driver_action(0xB1, 2, ret);

	return ret[0] | (ret[1]<<8);
}
