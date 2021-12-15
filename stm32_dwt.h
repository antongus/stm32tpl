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
 *  file         : stm32_dwt.h
 *  description  : Data Watchpoint and Trace (DWT) Unit class
 *  created on   : 11.02.2012
 *
 */

#ifndef STM32TPL_STM32_DWT_H_INCLUDED
#define STM32TPL_STM32_DWT_H_INCLUDED

#include "stm32.h"

struct DWT_t
{
	DWT_t()
	{
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT_->CYCCNT = 0;     // reset the counter
		DWT_->CTRL |= 1;   // enable the counter
	}
	static IOStruct<0xE0001000, DWT_Type> DWT_;
	static struct
	{
		uint32_t operator=(uint32_t value) { DWT_->CYCCNT = value; return value; }
		void operator|=(uint32_t value) { DWT_->CYCCNT |= value; }
		void operator&=(uint32_t value) { DWT_->CYCCNT &= value; }
		void operator^=(uint32_t value) { DWT_->CYCCNT ^= value; }
		operator uint32_t() { return DWT_->CYCCNT; }
	}cycles;
};

extern DWT_t dwt;

#endif // STM32TPL_STM32_DWT_H_INCLUDED
