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
 *  @file         : cortex-m3-regs.h
 *  @description  : Wrapper classes for some Cortex-M3 core registers.
 *  created on    : 05.05.2011
 *
 */

#ifndef STM32TPL_CORTEX_M3_REGS_H_INCLUDED
#define STM32TPL_CORTEX_M3_REGS_H_INCLUDED

/**
 * main stack pointer (MSP)
 */
struct RegisterMSP
{
	uint32_t operator=(uint32_t value) {
		__asm__ __volatile__ (
				"MSR MSP, %[val]  \n\t"
				:                    /* output */
				: [val]"r" (value)   /* input */
				:"memory"            /* clobbers */
		);
		return value;
	}
	operator uint32_t() {
		uint32_t res;
		__asm__ __volatile__ (
				"MRS %[result], MSP  \n\t"
				: [result] "=r"(res)	/* output */
		);
		return res;
	}
	void operator|=(uint32_t value) {
		uint32_t temp;
		__asm__ __volatile__ (
				"MRS %[tmp], MSP \n\t"
				"ORR %[tmp], %[tmp], %[val]  \n\t"
				"MSR MSP, %[tmp]  \n\t"
				:[tmp]"+r" (temp)     /* output */
				:[val]"r"  (value)    /* input */
				:"memory"             /* clobbers */
		);
	}
	void operator&=(uint32_t value) {
		uint32_t temp;
		__asm__ __volatile__ (
				"MRS %[tmp], MSP \n\t"
				"AND %[tmp], %[tmp], %[val]  \n\t"
				"MSR MSP, %[tmp]  \n\t"
				:[tmp]"+r" (temp)     /* output */
				:[val]"r" (value)     /* input */
				:"memory"             /* clobbers */
		);
	}
};

/**
 * Process stack pointer (PSP)
 */
struct RegisterPSP
{
	uint32_t operator=(uint32_t value) {
		__asm__ __volatile__ (
				"MSR PSP, %[val]  \n\t"
				:                    /* output */
				: [val]"r" (value)   /* input */
				:"memory"            /* clobbers */
		);
		return value;
	}
	operator uint32_t() {
		uint32_t res;
		__asm__ __volatile__ (
				"MRS %[result], PSP  \n\t"
				: [result] "=r"(res)	/* output */
		);
		return res;
	}
	void operator|=(uint32_t value) {
		uint32_t temp;
		__asm__ __volatile__ (
				"MRS %[tmp], PSP \n\t"
				"ORR %[tmp], %[tmp], %[val]  \n\t"
				"MSR PSP, %[tmp]  \n\t"
				:[tmp]"+r" (temp)     /* output */
				:[val]"r"  (value)    /* input */
				:"memory"             /* clobbers */
		);
	}
	void operator&=(uint32_t value) {
		uint32_t temp;
		__asm__ __volatile__ (
				"MRS %[tmp], PSP \n\t"
				"AND %[tmp], %[tmp], %[val]  \n\t"
				"MSR PSP, %[tmp]  \n\t"
				:[tmp]"+r" (temp)     /* output */
				:[val]"r" (value)     /* input */
				:"memory"             /* clobbers */
		);
	}
};

/**
 * CONTROL register (CONTROL)
 */
struct RegisterCONTROL
{
	uint32_t operator=(uint32_t value) {
		__asm__ __volatile__ (
				"MSR CONTROL, %[val]  \n\t"
				:                    /* output */
				: [val]"r" (value)   /* input */
				:"memory"            /* clobbers */
		);
		return value;
	}
	operator uint32_t() {
		uint32_t res;
		__asm__ __volatile__ (
				"MRS %[result], CONTROL  \n\t"
				: [result] "=r"(res)	/* output */
		);
		return res;
	}
	void operator|=(uint32_t value) {
		uint32_t temp;
		__asm__ __volatile__ (
				"MRS %[tmp], CONTROL \n\t"
				"ORR %[tmp], %[tmp], %[val]  \n\t"
				"MSR CONTROL, %[tmp]  \n\t"
				:[tmp]"+r" (temp)     /* output */
				:[val]"r"  (value)    /* input */
				:"memory"             /* clobbers */
		);
	}
	void operator&=(uint32_t value) {
		uint32_t temp;
		__asm__ __volatile__ (
				"MRS %[tmp], CONTROL \n\t"
				"AND %[tmp], %[tmp], %[val]  \n\t"
				"MSR CONTROL, %[tmp]  \n\t"
				:[tmp]"+r" (temp)     /* output */
				:[val]"r" (value)     /* input */
				:"memory"             /* clobbers */
		);
	}
};


#endif // STM32TPL_CORTEX_M3_REGS_H_INCLUDED
