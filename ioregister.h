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
 *  file         : ioregister.h
 *  description  : I/O register templates, bit-band template.
 *  created on   : 14.01.2011
 *
 */

#ifndef STM32TPL_IOREGISTER_H_INCLUDED
#define STM32TPL_IOREGISTER_H_INCLUDED

#include <cstdint>

/**
 * IO (read/write accessible) register
 * @param addr  address of register
 * @param type  register type
 */
template<uint32_t addr, typename type = uint32_t>
struct IORegister
{
	type operator=(type value) { *(volatile type*)addr = value; return value; }
	const IORegister& operator|=(type value) { *(volatile type*)addr |= value; return *this; }
	const IORegister& operator&=(type value) { *(volatile type*)addr &= value; return *this; }
	const IORegister& operator^=(type value) { *(volatile type*)addr ^= value; return *this; }
	const IORegister& operator+=(type value) { *(volatile type*)addr += value; return *this; }
	const IORegister& operator-=(type value) { *(volatile type*)addr -= value; return *this; }
	operator type() { return *(volatile type*)addr; }
};

/**
 * Read-only register
 * @param addr  address of register
 * @param type  register type
 */
template<uint32_t addr, typename type = uint32_t>
struct IRegister
{
	operator type() { return *(volatile type*)addr; }
};

/**
 * IO structure.
 * @param addr  address of register
 * @param T     structure type
 */
template<uint32_t addr, class T>
struct IOStruct
{
	volatile T* operator->() { return (volatile T*)addr; }
};

/**
 * Peripheral bit - bit-band accessed bit.
 * @param addr  memory/peripheral address
 * @param bit   bit number
 */
template <uint32_t addr, uint32_t bit> struct PeriphBit
{
	enum
	{
		pPERIPH_BASE      = 0x40000000UL,     // Peripheral base address
		pPERIPH_BB_BASE   = 0x42000000UL,     // Peripheral base address in the bit-band region
		pSRAM1_BB_BASE    = 0x22000000UL,     // SRAM1(112 KB) base address in the bit-band region
		pSRAM2_BB_BASE    = 0x2201C000UL      // SRAM2(16 KB) base address in the bit-band region
	};
	enum { BB_ADDR  = pPERIPH_BB_BASE + (addr - pPERIPH_BASE) * 32 + bit * 4 };
	uint32_t operator=(uint32_t value) { *(volatile uint32_t*)BB_ADDR = (bool)value; return value; }
	operator uint32_t() { return *(volatile uint32_t*)addr; }
};


#endif // STM32TPL_IOREGISTER_H_INCLUDED
