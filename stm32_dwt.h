/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 2012-2014 Anton B. Gusev aka AHTOXA
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

#ifndef STM32_DWT_H_INCLUDED
#define STM32_DWT_H_INCLUDED

#include "stm32.h"

namespace
{

struct DWT_Type
{
	uint32_t CONTROL;    // Control Register
	uint32_t CYCCNT;     // Cycle Count Register
	uint32_t CPICNT;     // CPI Count Register
	uint32_t EXCCNT;     // Exception Overhead Count Register
	uint32_t SLEEPCNT;   // Sleep Count Register
	uint32_t LSUCNT;     // LSU Count Register
	uint32_t FOLDCNT;    // Folded-instruction Count Register
	uint32_t PCSR;       // Program Counter Sample Register (R/O)
	uint32_t COMP0;      // Comparator Register0
	uint32_t MASK0;      // Mask Register0
	uint32_t FUNCTION0;  // Function Register0
	uint32_t COMP1;      // Comparator Register1
	uint32_t MASK1;      // Mask Register1
	uint32_t FUNCTION1;  // Function Register1
	uint32_t COMP2;      // Comparator Register2
	uint32_t MASK2;      // Mask Register2
	uint32_t FUNCTION2;  // Function Register2
	uint32_t COMP3;      // Comparator Register3
	uint32_t MASK3;      // Mask Register3
	uint32_t FUNCTION3;  // Function Register3
	uint32_t PID4;       // Peripheral identification registers
	uint32_t PID5;       //
	uint32_t PID6;       //
	uint32_t PID7;       //
	uint32_t PID0;       //
	uint32_t PID1;       //
	uint32_t PID2;       //
	uint32_t PID3;       //
	uint32_t CID0;       // Component identification registers
	uint32_t CID1;       //
	uint32_t CID2;       //
	uint32_t CID3;       //
};


}

struct DWT_t
{
	DWT_t()
	{
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT_->CYCCNT = 0;     // reset the counter
		DWT_->CONTROL |= 1;   // enable the counter
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

extern DWT_t DWT;

#endif // STM32_DWT_H_INCLUDED
