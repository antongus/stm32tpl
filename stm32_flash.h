/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 2011-2014 Anton B. Gusev aka AHTOXA
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
 *  file         : stm32_flash.h
 *  description  : stm32 flash class
 *  created on   : 28.04.2011
 *
 */

#ifndef STM32_FLASH_H_INCLUDED
#define STM32_FLASH_H_INCLUDED

#include "stm32.h"

/**
 * Template to define flash properties for selected chip.
 */
template <ChipType chip_type> struct Stm32FlashProps;

/**
 * Specializations for different chip types
 */
template<> struct Stm32FlashProps<stm32F10X_LD>
{
	enum { PAGE_COUNT = 32 };
	enum { PAGE_SIZE = 1024 };
};

template<> struct Stm32FlashProps<stm32F10X_MD>
{
	enum { PAGE_COUNT = 128 };
	enum { PAGE_SIZE = 1024 };
};

template<> struct Stm32FlashProps<stm32F10X_HD>
{
	enum { PAGE_COUNT = 256 };
	enum { PAGE_SIZE = 2048 };
};

template<> struct Stm32FlashProps<stm32F10X_CL>
{
	enum { PAGE_COUNT = 128 };
	enum { PAGE_SIZE = 2048 };
};

template<> struct Stm32FlashProps<stm32F10X_LD_VL>
{
	enum { PAGE_COUNT = 32 };
	enum { PAGE_SIZE = 1024 };
};

template<> struct Stm32FlashProps<stm32F10X_MD_VL>
{
	enum { PAGE_COUNT = 128 };
	enum { PAGE_SIZE = 1024 };
};

template<> struct Stm32FlashProps<stm32F10X_HD_VL>
{
	enum { PAGE_COUNT = 256 };
	enum { PAGE_SIZE = 2048 };
};

/**
 * Flash properties for current chip.
 */
typedef Stm32FlashProps<chip::type> stm32_flash_props;



/**
 * Default properties for STM32 Flash class template.
 */
struct FlashDefaultProps
{
	enum { WORD_TIMEOUT = 20 };
	enum { PAGE_ERASE_TIMEOUT = 0xFFF };
	enum { MASS_ERASE_TIMEOUT = 0xFFFF };
};

/**
 * STM32 Flash class template.
 */
template<typename props = FlashDefaultProps> class Stm32Flash;


template <typename props>
class Stm32Flash
{
public:
	enum { PAGE_COUNT = stm32_flash_props::PAGE_COUNT };
	enum { PAGE_SIZE = stm32_flash_props::PAGE_SIZE };
	enum { START_ADDRESS = 0x08000000UL };

	Stm32Flash() {}

	bool isLocked()	{ return FLASH->CR & FLASH_CR_LOCK; }
	void lock()		{ FLASH->CR |= FLASH_CR_LOCK; }
	void unlock()		{ FLASH->KEYR = KEY1; FLASH->KEYR = KEY2; }

	bool erasePage(uint32_t addr);
	bool massErase();

	bool writeWord(uint32_t addr, uint32_t data);
	bool writeHalfword(uint32_t addr, uint32_t data);
	bool write(uint32_t addr, void* buf, uint32_t count);
	uint32_t read(uint32_t addr) { return *(uint32_t*)addr; }
    uint32_t operator[](uint32_t addr) { return *(uint32_t*)addr; }
private:
	enum
	{
		RDPRT_KEY = 0x00A5,
		KEY1 = 0x45670123,
		KEY2 = 0xCDEF89AB
	};
	enum { WORD_TIMEOUT = props::WORD_TIMEOUT };
	enum { PAGE_ERASE_TIMEOUT = props::PAGE_ERASE_TIMEOUT };
	enum { MASS_ERASE_TIMEOUT = props::MASS_ERASE_TIMEOUT };


	bool isPgerr()			{ return FLASH->SR & FLASH_SR_PGERR; }
	bool isWrprterr()		{ return FLASH->SR & FLASH_SR_WRPRTERR; }

	void delay();
	void wait();
	bool wait(uint32_t timeout);

	bool isPageErased(uint32_t addr)
	{
		const volatile uint32_t* addr_32 = static_cast<volatile uint32_t*>(addr);
		for (int i = 0; i < 128; i++)
			if (*addr_32 != 0xFFFFFFFF)
				return false;
		return true;
	}
};

typedef Stm32Flash<> Flash;
//extern Flash flash;

template<typename props>
void Stm32Flash<props>::wait()
{
	while (FLASH->SR & FLASH_SR_BSY) ;
}

template<typename props>
bool Stm32Flash<props>::wait(uint32_t timeout)
{
	while (FLASH->SR & FLASH_SR_BSY)
	{
		if (!--timeout)
			return false;
		delay();
	}
	return true;
}

template<typename props>
void Stm32Flash<props>::delay()
{
	for (volatile int i = 0; i < 0xFF; i++) ;
}

template<typename props>
bool Stm32Flash<props>::writeWord(uint32_t addr, uint32_t data)
{
	TCritSect cs;
	volatile uint16_t* addr_16 = reinterpret_cast<volatile uint16_t*>(addr);

	uint32_t cr = FLASH->CR;
	FLASH->CR = FLASH_CR_PG;
	wait();
	addr_16[0] = data;
	wait();
	addr_16[1] = data >> 16;
	wait();
	FLASH->CR = cr & ~FLASH_CR_PG;
	return (read(addr) == data);
}

template<typename props>
bool Stm32Flash<props>::writeHalfword(uint32_t addr, uint32_t data)
{
	TCritSect cs;
	volatile uint16_t* addr_16 = reinterpret_cast<volatile uint16_t*>(addr);

	uint32_t cr = FLASH->CR;
	FLASH->CR = FLASH_CR_PG;
	wait();
	addr_16[0] = data;
	wait();
	FLASH->CR = cr & ~FLASH_CR_PG;
//	return (*(uint16_t*)addr == data);
	return true;
}

template<typename props>
bool Stm32Flash<props>::write(uint32_t addr, void* buf, uint32_t count)
{
	uint8_t* src = static_cast<uint8_t*>(buf);
	uint32_t end = addr + count;

	while (addr < end)
	{
		uint32_t w = 0xFFFFFFFF;
		if ((uint32_t)addr & 1)	// odd flash address - step back
			--addr;
		else
			w = 0xFFFFFF00 | (*src++);
		if (addr < end)
			w = (w & 0xFFFF00FF) | (*src++ << 8);

		if (!writeHalfword(addr, w))
			return false;
		addr += 2;
	}
	return true;
}

template<typename props>
bool Stm32Flash<props>::erasePage(uint32_t addr)
{
	TCritSect cs;
	wait();
	FLASH->CR |= FLASH_CR_PER;
	FLASH->AR = addr;
	FLASH->CR |= FLASH_CR_STRT;
	return wait(PAGE_ERASE_TIMEOUT);
}

template<typename props>
bool Stm32Flash<props>::massErase()
{
	TCritSect cs;
	wait();
	FLASH->CR |= FLASH_CR_MER;
	FLASH->CR |= FLASH_CR_STRT;
	return wait(MASS_ERASE_TIMEOUT);
}

#endif // STM32_FLASH_H_INCLUDED
