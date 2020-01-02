/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *  Visit https://github.com/antongus/stm32tpl for new versions
 *
 *  Copyright (c) 2011-2020 Anton B. Gusev aka AHTOXA
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
	bool Write(uint8_t b);
	uint8_t Read(bool last);
	void Write(uint8_t device, uint8_t addr, void *data, size_t length);
	void Write(uint8_t addr, void *data, size_t length);
	void Read(uint8_t device, uint8_t addr, void *data, size_t length);
	void Read(uint8_t addr, void *data, size_t length);
	void Start()
	{
	    SDA::Off();
	    Qdel();
	    SCL::Off();
	}
	void Stop()
	{
		SDA::Off();
		Hdel();
		SCL::On();
		Qdel();
		SDA::On();
		Hdel();
	}
private:
	enum { 	NOP_COUNT = props::NOP_COUNT };
	typedef typename props::SCL SCL;
	typedef typename props::SDA SDA;

	void Qdel()
	{
		for (int i = 0; i < NOP_COUNT; i++)
			__asm__ __volatile__ ("nop");
	}
	void Hdel()
	{
		Qdel();
		Qdel();
	}
	void ToggleScl()
	{
		Hdel();
		SCL::On();
		Hdel();
		SCL::Off();
	}
};

template<typename props>
bool SoftI2c<props>::Write(uint8_t b)
{
	for(int i=0; i<8; i++)
	{
		SDA::On(b & 0x80);
		b <<= 1;
		ToggleScl();
	}

	SDA::On();
	SDA::Mode(INPUT);
	Hdel();
	SCL::On();
	bool ret = !SDA::Signalled();
	Hdel();
	SCL::Off();
	SDA::Mode(OUTPUT_OD_2MHZ);
	Hdel();
	return ret;
}

template<typename props>
uint8_t SoftI2c<props>::Read(bool last)
{
	uint8_t b = 0;

	SDA::On();
	SDA::Mode(INPUT);

	for(int i=0; i<8; i++)
	{
		Hdel();
		SCL::On();
		b <<= 1;
		if (SDA::Signalled())
			b |= 1;
		Hdel();
	    SCL::Off();
	}

	SDA::Mode(OUTPUT_OD_2MHZ);
	SDA::On(last);
	ToggleScl();
	SDA::On();
	return b;
}

template<typename props>
void SoftI2c<props>::Write(uint8_t device, uint8_t addr, void *data, size_t length)
{
	uint8_t *p = reinterpret_cast<uint8_t *>(data);
	Start();
	Write(device);
	Write(addr);

	while (length--)
		Write(*p++);

	Stop();
}

template<typename props>
void SoftI2c<props>::Write(uint8_t addr, void *data, size_t length)
{
	uint8_t *p = reinterpret_cast<uint8_t *>(data);
	Start();
	Write(addr);

	while (length--)
		Write(*p++);

	Stop();
}

template<typename props>
void SoftI2c<props>::Read(uint8_t device, uint8_t addr, void *data, size_t length)
{
	uint8_t *p = reinterpret_cast<uint8_t *>(data);
	Start();
	Write(device);
	Write(addr);
	Hdel();
	SCL::On();
	Start();

	Write(device | 1);

	// receive data bytes
	while (length--)
		*p++ = Read(length == 0);

	Stop();
}

template<typename props>
void SoftI2c<props>::Read(uint8_t addr, void *data, size_t length)
{
	uint8_t *p = reinterpret_cast<uint8_t *>(data);
	Start();
	Write(addr | 1);

	// receive data bytes
	while (length--)
		*p++ = Read(length == 0);

	Stop();
}

#endif // STM32TPL_SOFT_I2C_H_INCLUDED
