// Do not remove the include below
#include "asf.h"




// Function prototypes
void serial_receive(void);
uint8_t serial_parse(char *buffer);
void(* resetFunc) (void) = 0;

//The setup function is called once at startup of the sketch
void setup()
{
	Serial.begin(57600);
	Serial.println("ola");
}

// The loop function is called in an endless loop
void loop()
{
	serial_receive();
}

void serial_receive(void)
{
	char inChar;			// temporary input char
	static char inData_[100];
	static int index_ = 0;
	static uint8_t receiving_cmd = 0;

	while(Serial.available() > 0)	//if bytes available at Serial port
	{
		inChar = Serial.read();		// read from port

		if(index_ < 98)		// read up to 98 bytes
		{
			if(inChar == ':')
			{
				if(receiving_cmd == 0)
				{
					receiving_cmd = 1;

					inData_[index_] = inChar;	// store char
					++index_;			// increment index
					inData_[index_] = 0;		// just to finish string
				}
			}
			else if(receiving_cmd == 1)
			{
				inData_[index_] = inChar;	// store char
				++index_;			// increment index
				inData_[index_] = 0;		// just to finish string

			}
		}
		else			// put end char ";"
		{
			index_ = 0;

		}

		if(receiving_cmd)
		{
			if(inChar == ';')
			{			// if the last char is ";"

				if(!serial_parse(inData_))	//parse data
				{
					receiving_cmd = 0;
					index_ = 0;

					inData_[index_] = 0;
				}
			}
			else
			{
				// no parse action
				receiving_cmd = 0;
				index_ = 0;

				inData_[index_] = 0;
			}
		}
	}

}

uint8_t serial_parse(char *buffer)
{
	int d1;
	//char f1;last_received_byte_millis

	if(!strncmp(buffer,":nanoTec_cmd;",13))		// motor cmd
	{
		Serial.println("weeeee");
		//do stuff
		return 0;
	}
	else if(!strncmp(buffer,":AHR;",4))			// motor cmd
	{
		resetFunc();
		return 0;
	}
	return 1;
}
