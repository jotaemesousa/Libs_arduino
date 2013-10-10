/*
 * JRKMotorcontroller.h
 *
 *  Created on: Jun 12, 2013
 *      Author: joao
 */

#ifndef JRKMOTORCONTROLLER_H_
#define JRKMOTORCONTROLLER_H_

#include "stdint.h"

#define LINEAR_ACTUATOR_CENTER 		2048
#define LINEAR_ACTUATOR_ZERO 		0
#define LINEAR_ACTUATOR_MAX	 		4095

#define RECEIVE_TIMEOUT				100

class JRK_Motor_controller
{
	unsigned int driver_update;
	unsigned int set_point;
	unsigned int max_length, min_length;
	unsigned long last_driver_update;

	unsigned int brate;
	uint8_t	deviceID;
	char communication_mode, serial_port;

	void send_driver_action(void);
	void send_driver_action(uint8_t cmd, uint8_t nr_received_bytes, uint8_t *ret);
	void send_byte(uint8_t byte);
	uint8_t get_nbytes(uint8_t nbytes, uint8_t *byte);

public:

	typedef enum {Serial0 = 0, Serial01, Serial02, Serial03} Serial_port;
	typedef enum {Analog = 0, PulseRC, SerialTTL, SerialDaisyTTL} Comm_mode;

	JRK_Motor_controller();

	void select_comunnication_mode(Comm_mode mode);
	void select_baud_rate(unsigned int br);
	void select_serial_port(Serial_port serial);
	void select_device_id(uint8_t id){deviceID = id;}
	uint8_t get_device_id(void){return deviceID;}
	void init_communication(void);

	void set_position(unsigned int pos);
	unsigned int get_last_position(void){return set_point;}
	void update_driver(void);

	void set_max_length(unsigned int m){ max_length = m;}
	void set_min_length(unsigned int m){ min_length = m;}
	void set_driver_time_update(unsigned int time);

	uint16_t get_scaled_feedback(void);
	uint16_t get_feedback(void);
	uint16_t get_input(void);
	uint16_t get_target(void);
	int16_t get_error_sum(void);
	int16_t get_duty_cycle_target(void);
	int16_t get_duty_cycle(void);
	uint8_t get_current(void);
	uint16_t get_PID_period_count(void);
};

#endif /* JRKMOTORCONTROLLER_H_ */
