/*
 * Sabertooth.h
 *
 *  Created on: Jun 11, 2013
 *      Author: joao
 */

#ifndef SABERTOOTH_H_
#define SABERTOOTH_H_

#include "SoftwareSerial.h"

#define COMUNICATION_SIMPLE_SERIAL			3
#define COMUNICATION_SIMPLE_SOFT_SERIAL		5

class Sabertooth
{
	int pwm1, duty1;	//pwm is the desired pwm. duty is the real value of duty cycle.
						// these two variables are needed to avoid high dPWM/dT
	int pwm2, duty2;

	long unsigned int last_time_stream, last_driver_update;
	unsigned int driver_update;

	int duty_min1, duty_min2;
	int max_pwm_fwd1, max_pwm_fwd2, max_pwm_rev1, max_pwm_rev2;

	unsigned int brate;
	char communication_mode, serial_port;
	char tx, rx;

	SoftwareSerial *soft_serial;

	void set_duty(void);
	void normalize_pwm(void);
	void normalize_duty(void);
	int action_motor(bool channel);
	void send_driver_action(void);



public:
	Sabertooth();

	void select_comunnication_mode(int mode);
	void select_baud_rate(unsigned int br);
	void select_serial_port(char serial);
	void set_tx_pin(char p){tx = p;}
	void set_rx_pin(char p){rx = p;}
	void init_communication(void);

	void set_pwm(int hb1, int hb2);
	void set_max_pwm_fwd(int pwm1, int pwm2);
	void set_max_pwm_rev(int pwm1, int pwm2);
	void set_driver_time_update(unsigned int time);
	void update_driver(void);

	int get_pwm1(void) { return pwm1;}
	int get_pwm2(void) { return pwm2;}
	int get_now_pwm1(void) { return duty1;}
	int get_now_pwm2(void) { return duty2;}

	unsigned int get_last_time_stream(void){return last_time_stream;}
	void set_last_time_stream(unsigned int time){last_time_stream = time;}
};

#endif /* SABERTOOTH_H_ */
