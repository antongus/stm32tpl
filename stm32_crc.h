/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 2015 Anton B. Gusev aka AHTOXA
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
 *  file         : stm32_crc.h
 *  description  : STM32 Hardware CRC32 calculator
 *
 */

#ifndef STM32TPL_STM32_CRC_H_INCLUDED
#define STM32TPL_STM32_CRC_H_INCLUDED

#include "stm32.h"
#include <cstddef>

namespace STM32
{

class CrcCalculator
{
public:
	CrcCalculator()
	{
		EnableClocks();
		Reset();
	}
	~CrcCalculator() { DisableClocks(); }

	static void EnableClocks()  { RCC->AHBENR |= RCC_AHBENR_CRCEN; __DSB(); }
	static void DisableClocks() { RCC->AHBENR &= ~RCC_AHBENR_CRCEN; }
	static void Reset()
	{
		CRC->CR = CRC_CR_RESET;
		__asm__ volatile ("nop");
		__asm__ volatile ("nop");
		__asm__ volatile ("nop");
	}
	static void Add(uint32_t w) { CRC->DR = w; }
#if defined STM32TPL_STM32F0XX || defined STM32TPL_STM32L0XX
	static void AddByte(uint8_t byte)
	{
		// access to DR by byte
		*(reinterpret_cast<volatile uint8_t*>(&CRC->DR)) = byte;
	}
#endif

#if defined STM32TPL_STM32F0XX || defined STM32TPL_STM32L0XX
	// STM32F0xx CRC module supports byte-wide operations
	// and hardware bit order reverse on input and output.
	// Therefore it is possible generate standard Ethernet
	// CRC without tricks.
	static uint32_t CalcEthernet(void const* buf, size_t len)
	{
		EnableClocks();
		Reset();
		CRC->CR |= CRC_CR_REV_IN_0 | CRC_CR_REV_OUT;

		uint8_t const * p = reinterpret_cast<uint8_t const *>(buf);
		for (size_t i = 0; i < len; ++i)
			AddByte(p[i]);
		uint32_t result = ~Result();

		DisableClocks();
		return result;
	}
#else
	static uint32_t CalcEthernet(void const* buf, size_t len)
	{
		CrcCalculator calc;
		uint32_t const * p = reinterpret_cast<uint32_t const *>(buf);

		if (len < 4)
			calc.Add(0xEBABAB);
		else while (len >= 4)
		{
			calc.Add(__RBIT(*p++));
			len -= 4;
		}
		uint32_t crc = __RBIT(calc.Result());

		switch (len)
		{
		case 1:
			calc.Add(calc.Result()); // zero-out CRC result register
			calc.Add(__RBIT((*p & 0xFF) ^ crc) >> 24);
			crc = (crc >> 8) ^ __RBIT(calc.Result());
			break;
		case 2:
			calc.Add(calc.Result()); // zero-out CRC result register
			calc.Add(__RBIT((*p & 0xFFFF) ^ crc) >> 16);
			crc = (crc >> 16) ^ __RBIT(calc.Result());
			break;
		case 3:
			calc.Add(calc.Result()); // zero-out CRC result register
			calc.Add(__RBIT((*p & 0xFFFFFF) ^ crc) >> 8);
			crc = (crc >> 24) ^ __RBIT(calc.Result());
			break;
		}
		return ~crc;
	}
#endif
	static uint32_t Calc(void const* buf, size_t len)
	{
		CrcCalculator calc;
		uint32_t const * p = reinterpret_cast<uint32_t const *>(buf);
		for (size_t i = 0; i < len; ++i)
			calc.Add(p[i]);
		return calc.Result();
	}
	static uint32_t Result() { return CRC->DR; }
};

} // namespace STM32

#endif // STM32TPL_STM32_CRC_H_INCLUDED
