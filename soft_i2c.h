/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 2014 Anton B. Gusev aka AHTOXA
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 *
 *
 *  file         : soft_i2c.h
 *  description  : software i2c implementation
 *
 */

#ifndef STM32TPL_SOFT_I2C_H_INCLUDED
#define STM32TPL_SOFT_I2C_H_INCLUDED

#include <cstdint>
#include "pin.h"

/**
 * Sample structure for SoftI2c properties
 */
struct SampleSoftI2cProps
{
	typedef Pin<'B', 10> SCL;
	typedef Pin<'B', 11> SDA;
	enum { 	NOP_COUNT = 100 };
};

/**
 * SoftI2c - software i2c
 */
template<typename props = SampleSoftI2cProps>
class SoftI2c
{
public:
	SoftI2c()
	{
		SCL::Mode(OUTPUT_OD_2MHZ);
		SDA::Mode(OUTPUT_OD_2MHZ);
		SCL::On();
		SDA::On();
	}
	bool write(uint8_t b);
	uint8_t read(bool last);
	void write(uint8_t device, uint8_t addr, void *data, size_t length);
	void write(uint8_t addr, void *data, size_t length);
	void read(uint8_t device, uint8_t addr, void *data, size_t length);
	void read(uint8_t addr, void *data, size_t length);
	void start()
	{
	    SDA::Off();
	    qdel();
	    SCL::Off();
	}
	void stop()
	{
		SDA::Off();
		hdel();
		SCL::On();
		qdel();
		SDA::On();
		hdel();
	}
private:
	enum { 	NOP_COUNT = props::NOP_COUNT };
	typedef typename props::SCL SCL;
	typedef typename props::SDA SDA;

	void qdel()
	{
		for (int i = 0; i < NOP_COUNT; i++)
			__asm__ __volatile__ ("nop");
	}
	void hdel()
	{
		qdel();
		qdel();
	}
	void toggleScl()
	{
		hdel();
		SCL::On();
		hdel();
		SCL::Off();
	}
};

template<typename props>
bool SoftI2c<props>::write(uint8_t b)
{
	for(int i=0; i<8; i++)
	{
		SDA::On(b & 0x80);
		b <<= 1;
		toggleScl();
	}

	SDA::On();
	SDA::Mode(INPUT);
	hdel();
	SCL::On();
	bool ret = SDA::Signalled();
	hdel();
	SCL::Off();
	SDA::Mode(OUTPUT_OD_2MHZ);
	hdel();
	return ret;
}

template<typename props>
uint8_t SoftI2c<props>::read(bool last)
{
	uint8_t b = 0;

	SDA::On();
	SDA::Mode(INPUT);

	for(int i=0; i<8; i++)
	{
		hdel();
		SCL::On();
		b <<= 1;
		if (SDA::Signalled())
			b |= 1;
		hdel();
	    SCL::Off();
	}

	SDA::Mode(OUTPUT_OD_2MHZ);
	SDA::On(last);
	toggleScl();
	SDA::On();
	return b;
}

template<typename props>
void SoftI2c<props>::write(uint8_t device, uint8_t addr, void *data, size_t length)
{
	uint8_t *p = reinterpret_cast<uint8_t *>(data);
	start();
	write(device);
	write(addr);

	while (length--)
		write(*p++);

	stop();
}

template<typename props>
void SoftI2c<props>::write(uint8_t addr, void *data, size_t length)
{
	uint8_t *p = reinterpret_cast<uint8_t *>(data);
	start();
	write(addr);

	while (length--)
		write(*p++);

	stop();
}

template<typename props>
void SoftI2c<props>::read(uint8_t device, uint8_t addr, void *data, size_t length)
{
	uint8_t *p = reinterpret_cast<uint8_t *>(data);
	start();
	write(device);
	write(addr);
	hdel();
	SCL::On();
	start();

	write(device | 1);

	// receive data bytes
	while (length--)
		*p++ = read(length == 0);

	stop();
}

template<typename props>
void SoftI2c<props>::read(uint8_t addr, void *data, size_t length)
{
	uint8_t *p = reinterpret_cast<uint8_t *>(data);
	start();
	write(addr | 1);

	// receive data bytes
	while (length--)
		*p++ = read(length == 0);

	stop();
}

#endif // STM32TPL_SOFT_I2C_H_INCLUDED
