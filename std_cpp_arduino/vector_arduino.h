#ifndef VECTOR_ARDUINO_H_
#define VECTOR_ARDUINO_H_

//#include "Arduino.h"

// Minimal class to replace std::vector
template<class Data>
class Vector
{
	size_t d_size; // Stores no. of actually stored objects
	size_t d_capacity; // Stores allocated capacity
	Data *d_data; // Stores data

	public:
		Vector() : d_size(0), d_capacity(0), d_data(0) {}; // Default constructor

		Vector(Vector const &other) : d_size(other.d_size), d_capacity(other.d_capacity), d_data(0)
		{
			d_data = (Data *)malloc(d_capacity*sizeof(Data));
			memcpy(d_data, other.d_data, d_size*sizeof(Data));
		}; // Copy constuctor
		
		~Vector()
		{
			free(d_data);
		}; // Destructor

		Vector &operator=(Vector const &other)
		{
			free(d_data);
			d_size = other.d_size;
			d_capacity = other.d_capacity;
			d_data = (Data *)malloc(d_capacity*sizeof(Data));
			memcpy(d_data, other.d_data, d_size*sizeof(Data));
			return *this;
		}; // Needed for memory management
		
		int push_back(Data const &x, int index = -1)
		{
			//Serial.print("pushinggg\n");
			if(index == -1)
			{
				if (d_capacity == d_size)
				{
					resize();
				}
				d_data[d_size++] = x;

				return 0;
			}
			else
			{
				if(index <= d_capacity && index >= 0)
				{
					if(d_capacity <= d_size)
					{
						d_data =(Data *)realloc(d_data,++d_capacity);
					}
					memmove(d_data + 1 + index, d_data + index, (d_size - index)*sizeof(Data));
					d_data[index] = x;
					d_size++;

					return 0;
				}
				else
				{
					return -1;
				}

			}
		}; // Adds new value. If needed, allocates more space

		int pop_back(int n = 1)
		{
			if(n >= 1)
			{
				if(d_size - n >= 0)
				{
					d_size -= n;
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

		size_t size() const
		{
			return d_size;
		}; // Size getter

		Data const &operator[](size_t idx) const {
			return d_data[idx];
		}; // Const getter

		Data &operator[](size_t idx) {
			return d_data[idx];
		}; // Changeable getter
		
		void resize(size_t tam)
		{
			d_data = (Data *)realloc(d_data, tam*sizeof(Data));
			d_capacity = tam;

		};
		void clear (void)
		{
			d_size = 0;
		}
		

		private:
		void resize()
		{
			d_capacity = d_capacity ? d_capacity+10 : 1;
			d_data = (Data *)realloc(d_data, d_capacity*sizeof(Data));

		};// Allocates double the old space
};

#endif
