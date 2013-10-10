#ifndef LIST_ARDUINO_H
#define LIST_ARDUINO_H

#include "stdio.h"

#include "stdlib.h"
#include "string.h"
// Minimal class to replace std::vector
template<class Data>
class List_arduino
{
	int d_size; 				// Stores no. of actually stored objects
	bool circ_;					// Normal buffer (0) or circular buffer
	int d_capacity;				// Stores allocated capacity
	Data *d_data; 				// Stores data
	int last_;					// Points to the last element
	bool flag_last_;			// In case of a circular buffer, this flag marks if the buffer is already filled

	public:

		// Default constructor
		List_arduino() : d_size(0), d_capacity(0), d_data(0)
		{
			circ_ = 0;
			last_ = 0;
			flag_last_ = 0;
		};

		// Copy constuctor
		List_arduino(List_arduino const &other) : d_size(other.d_size), d_capacity(other.d_capacity), d_data(0)
		{
			d_data = (Data *)malloc(d_capacity*sizeof(Data));
			memcpy(d_data, other.d_data, d_size*sizeof(Data));
			circ_ = other.circ_;
			last_ = other.last_;
			flag_last_ = other.flag_last_;

		};
		
		// Destructor
		~List_arduino()
		{
			free(d_data);
		};

		// Needed for memory management
		List_arduino &operator=(List_arduino const &other)
		{
			free(d_data);
			d_size = other.d_size;
			d_capacity = other.d_capacity;
			d_data = (Data *)malloc(d_capacity*sizeof(Data));
			memcpy(d_data, other.d_data, d_size*sizeof(Data));
			circ_ = other.circ_;
			return *this;
		};
		
		// Adds new value. If needed, allocates more space
		Data push_front(Data const &x)
		{
			Data temp;
			if(circ_ == 1)
			{
				if (d_capacity == d_size)
				{
					d_size = 0;
					flag_last_ = 1;
				}
				if (flag_last_)
				{
					last_ = d_size + 1;
					if (last_ >= d_capacity)
					{
						last_ = 0;
					}
				}
				else
				{
					last_ = d_size;
				}
				temp = d_data[d_size];
				d_data[d_size++] = x;
			}
			else
			{
				if (d_capacity == d_size)
				{
					resize();
				}
				d_data[d_size++] = x;
			}

			return temp;
		};

		int pop_back(int n = 1)
		{
			if(circ_ == 0)
			{
				if(last_ + n <= d_size)
				{
					last_ += n;


					if(last_ > 10)
					{
						memmove(d_data, d_data + last_ , (d_size - last_) * sizeof(Data));
						d_size -= last_;
						last_ = 0;
					}
					return 0;
				}
				else
				{
					return -2;
				}
			}
			else
			{
				return -1;
			}
		}

		// Makes the list circular, and a max_size must be defined
		void make_circular_list(int size = -1)
		{
			circ_ = 1;
			if (size != -1)
			{
				resize(size);
			}
		};

		// Makes the list endless
		void make_endless_list(void)
		{
			circ_ = 0;
			flag_last_ = 0;
			last_ = 0;
		};

		// front getter
		Data const front(void)
		{
			int temp = d_size - 1;

			if (temp < 0)
			{
				temp = d_capacity - 1;
			}

			return d_data[temp];
		}

		// back getter
		Data const back(void)
		{
			return d_data[last_];
		}

		// Size getter
		size_t size() const
		{
			if(circ_)
			{
				return d_capacity;
			}
			else
			{
				return d_size - last_;
			}
		};

		// Const getter
		Data const &operator[](size_t idx) const
		{
			if(circ_ == 0)
			{
				return d_data[idx + last_];
			}
			else
			{
				return d_data[idx];
			}
		};

		// Changeable getter
		Data &operator[](size_t idx)
		{
			if(circ_ == 0)
			{
				return d_data[idx + last_];
			}
			else
			{
				return d_data[idx];
			}
		};
		
		// Allocates double the old space
		void resize(size_t tam)
		{
			d_data = (Data *)realloc(d_data, tam*sizeof(Data));
			d_capacity = tam;
		};

		// Clear the list
		void clear (void)
		{
			d_size = 0;
		}
		

	private:
		// Allocates double the old space
		void resize()
		{
			d_capacity = d_capacity ? d_capacity+5 : 1;
			d_data = (Data *)realloc(d_data, d_capacity*sizeof(Data));
		};
};

#endif
