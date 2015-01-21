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

namespace STM32
{

class CrcCalculator
{
public:
	CrcCalculator()
	{
		RCC->AHBENR |= RCC_AHBENR_CRCEN;
		__DSB();
		CRC->CR = 1;
		__asm__ volatile ("nop");
		__asm__ volatile ("nop");
		__asm__ volatile ("nop");
	}
	~CrcCalculator() { RCC->AHBENR &= ~RCC_AHBENR_CRCEN; }
	void Add(uint32_t w) { CRC->DR = w; }
	uint32_t Result() { return CRC->DR; }
	static uint32_t Calc(void const* buf, size_t len)
	{
		CrcCalculator crc;
		uint32_t const * p = reinterpret_cast<uint32_t const *>(buf);
		for (size_t i = 0; i < len; ++i)
			crc.Add(p[i]);
		return crc.Result();
	}
};

} // namespace STM32

#endif // STM32TPL_STM32_CRC_H_INCLUDED
