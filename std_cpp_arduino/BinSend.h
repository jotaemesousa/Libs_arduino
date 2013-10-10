/*
 * BinSsend.h
 *
 *  Created on: 10 de Nov de 2012
 *      Author: Jo�o
 */

#ifndef BINSSEND_H_
#define BINSSEND_H_
#include "Arduino.h"
#include "list_arduino.h"

template <class T>
class BinSend
{
	unsigned short bits_;
	char send_char[1];
	char first_bit_;
	List_arduino<T> buffer_;

	short bpb_;

public:
	BinSend()
	{
		bits_ = 1;
		memset(send_char,0,2);
		first_bit_ = 0;
		bpb_ = 8;
	};

	BinSend(unsigned short n_bits)
	{
		bits_ = n_bits;
		memset(send_char,0,2);
		first_bit_ = 0;
		bpb_ = 8;
	};

	BinSend(unsigned short n_bits, char first_char)
	{
		bits_ = n_bits;
		memset(send_char,0,2);
		first_bit_ = first_char;
		bpb_ = 7;
	};


	void add_to_buffer(T var)
	{
		buffer_.push_front(var);
	};

	void reset_buffer(void)
	{
		buffer_.clear();
	};

	unsigned int send_value(T val)
	{
		unsigned int c = 0;
		send_char[0] = first_bit_ & 0x80;

		for (int f = bits_ - 1; f >= 0 ; --f)
		{
			send_char[0] |= (((1 << f) & val) >> f) << ((bpb_-1) - (c % bpb_));
			c++;
			if (!(c % bpb_))
			{
				if(f == 0)
				{
					if(first_bit_ & 0x80 == 0x80)
					{
						send_char[0] &= ~0x80;
					}
					else
					{
						send_char[0] |= 0x80;
					}
				}

				Serial.write(send_char[0]);
				send_char[0] = first_bit_ & 0x80;
			}
		}
		if (c % bpb_)
		{
			if(first_bit_ & 0x80 == 0x80)
			{
				send_char[0] &= ~0x80;
			}
			else
			{
				send_char[0] |= 0x80;
			}
			Serial.write(send_char[0]);
		}
		return c;
	};

	unsigned int send_buffer(int multip = 1)
	{
		unsigned int c = 0;
		send_char[0] = first_bit_ & 0x80;

		if(multip < 1)
		{
			multip = 1;
		}

		int buffer_sz = floor(buffer_.size() / multip) * multip;

		for (int var = 0; var < buffer_sz; ++var)
		{
			for (int f = bits_ - 1; f >= 0 ; --f)
			{
				send_char[0] |= (((1 << f) & buffer_[var]) >> f) << ((bpb_-1) - (c % bpb_));
				c++;
				if (!(c % bpb_))
				{
					/*if(f == 0)
					{
						if(first_bit_ & 0x80 == 0x80)
						{
							send_char[0] &= ~0x80;
						}
						else
						{
							send_char[0] |= 0x80;
						}
					}*/

					Serial.write(send_char[0]);
					send_char[0] = first_bit_ & 0x80;
				}
			}
		}
		if (c % bpb_)
		{
			if(first_bit_ & 0x80 == 0x80)
			{
				send_char[0] &= ~0x80;
			}
			else
			{
				send_char[0] |= 0x80;
			}
			Serial.write(send_char[0]);
		}
		buffer_.pop_back(buffer_sz);
		//Serial.println(buffer_sz);
		return c;
	};
};

#endif /* BINSSEND_H_ */
