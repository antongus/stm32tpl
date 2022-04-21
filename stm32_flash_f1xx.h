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
 *  file         : stm32_flash.h
 *  description  : stm32 flash class
 *  created on   : 28.04.2011
 *
 */

#ifndef STM32TPL_STM32_FLASH_F1XX_H_INCLUDED
#define STM32TPL_STM32_FLASH_F1XX_H_INCLUDED

#include "stm32.h"
#include "scmRTOS.h"
#include <cstring>

namespace STM32
{

enum WriteResult
{
	wrOk = 0,
	wrLocked,
	wrNotErased,
	wrWriteProtected,
	wrVerifyError
};

namespace
{

/**
 * Template to define flash properties for selected chip.
 */
template <ChipType chipType> struct Stm32FlashProps;

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

template<> struct Stm32FlashProps<stm32F0XX>
{
	enum { PAGE_COUNT = 256 };
	enum { PAGE_SIZE = 2048 };
};

template<> struct Stm32FlashProps<stm32F3XX>
{
	enum { PAGE_COUNT = 128 };
	enum { PAGE_SIZE = 2048 };
};


} // anon namespace

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
template<class props = FlashDefaultProps> class Stm32Flash;


template <class props>
class Stm32Flash
{
public:
	enum { PAGE_COUNT = Stm32FlashProps<chip::type>::PAGE_COUNT };
	enum { PAGE_SIZE = Stm32FlashProps<chip::type>::PAGE_SIZE };
	enum { START_ADDRESS = 0x08000000UL };
	enum { SIZE = PAGE_COUNT * PAGE_SIZE };

	struct Options
	{
		static bool Locked()   { return !(FLASH->CR & FLASH_CR_OPTWRE); }
		static void Lock()     { FLASH->CR &= ~FLASH_CR_OPTWRE; }
		static void Unlock()   { FLASH->OPTKEYR = KEY1; FLASH->OPTKEYR = KEY2; }
	};

	static bool Locked()   { return FLASH->CR & FLASH_CR_LOCK; }
	static void Lock()     { FLASH->CR |= FLASH_CR_LOCK; }
	static void Unlock()   { FLASH->KEYR = KEY1; FLASH->KEYR = KEY2; }

	static bool ErasePage(uint32_t addr);
	static bool MassErase();
#if defined(FLASH_OBR_RDPRT)
	static bool IsReadOutProtected() { return FLASH->OBR & FLASH_OBR_RDPRT; }
#elif defined(FLASH_OBR_LEVEL1_PROT)
	static bool IsReadOutProtected() { return FLASH->OBR & (FLASH_OBR_LEVEL1_PROT | FLASH_OBR_LEVEL2_PROT); }
#endif
	static bool ReadOutProtect();



	static bool WriteWord(uint32_t addr, uint32_t data);
	static bool WriteHalfword(uint32_t addr, uint32_t data);
	static WriteResult Write16(uint32_t addr, uint32_t data);
	static bool Write(uint32_t addr, const void* buf, uint32_t count);
	static uint32_t Read(uint32_t addr) { return *(uint32_t*)addr; }
	static void Read(uint32_t addr, void* buf, uint32_t count);
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


	static bool Pgerr()      { return FLASH->SR & FLASH_SR_PGERR; }
#if (defined STM32TPL_STM32F3XX)
	static bool Wrprterr()   { return FLASH->SR & FLASH_SR_WRPERR; }
#else
	static bool Wrprterr()   { return FLASH->SR & FLASH_SR_WRPRTERR; }
#endif
	static bool Busy()       { return FLASH->SR & FLASH_SR_BSY; }

	static void Delay();
	static void Wait();
	static bool Wait(uint32_t timeout);

	static bool PageErased(uint32_t addr)
	{
		const volatile uint32_t* addr_32 = reinterpret_cast<volatile uint32_t*>(addr);
		for (int i = 0; i < 128; i++)
			if (*addr_32++ != 0xFFFFFFFF)
				return false;
		return true;
	}
};

typedef Stm32Flash<> Flash;

template<class props>
void Stm32Flash<props>::Wait()
{
	while (Busy()) ;
	__DSB();
}

template<class props>
bool Stm32Flash<props>::Wait(uint32_t timeout)
{
	while (Busy())
	{
		if (!--timeout)
			return false;
		Delay();
	}
	__DSB();
	return true;
}

template<class props>
void Stm32Flash<props>::Delay()
{
	for (volatile int i = 0; i < 0xFF; i++) ;
}

template<class props>
bool Stm32Flash<props>::WriteWord(uint32_t addr, uint32_t data)
{
	TCritSect cs;
	volatile uint16_t* addr_16 = reinterpret_cast<volatile uint16_t*>(addr);

	uint32_t cr = FLASH->CR;
	FLASH->CR = FLASH_CR_PG;
	Wait();
	addr_16[0] = data;
	Wait();
	addr_16[1] = data >> 16;
	Wait();
	FLASH->CR = cr & ~FLASH_CR_PG;
	return (Read(addr) == data);
}

template<class props>
bool Stm32Flash<props>::WriteHalfword(uint32_t addr, uint32_t data)
{
	TCritSect cs;
	volatile uint16_t* addr_16 = reinterpret_cast<volatile uint16_t*>(addr);

	uint32_t cr = FLASH->CR;
	FLASH->CR = FLASH_CR_PG;
	Wait();
	addr_16[0] = data;
	Wait();
	FLASH->CR = cr & ~FLASH_CR_PG;
//	return (*(uint16_t*)addr == data);
	return true;
}

template<class props>
WriteResult Stm32Flash<props>::Write16(uint32_t addr, uint32_t data)
{
	TCritSect cs;
	volatile uint16_t* flashArray = reinterpret_cast<volatile uint16_t*>(addr);

	if (Locked())
		return wrLocked;

	FLASH->CR = FLASH_CR_PG;
	Wait();
	flashArray[0] = data;
	Wait();

	WriteResult ret =
		Pgerr() ? wrNotErased :
		Wrprterr() ? wrWriteProtected :
		wrOk;

#if (defined STM32TPL_STM32F3XX)  // clear errors, if any
	FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPERR;
#else
	FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
#endif
	FLASH->CR = 0;
	return ret;
}

template<class props>
bool Stm32Flash<props>::Write(uint32_t addr, const void* buf, uint32_t count)
{
	const uint8_t* src = reinterpret_cast<const uint8_t*>(buf);
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

//		if (!WriteHalfword(addr, w))
		if (Write16(addr, w) != wrOk)
			return false;
		addr += 2;
	}
	return true;
}

template<class props>
void Stm32Flash<props>::Read(uint32_t addr, void* buf, uint32_t count)
{
	const uint8_t* src = reinterpret_cast<const uint8_t*>(addr);
	memcpy(buf, src, count);
}

template<class props>
bool Stm32Flash<props>::ErasePage(uint32_t addr)
{
	TCritSect cs;
	Wait();
	FLASH->CR |= FLASH_CR_PER;
	FLASH->AR = addr;
	FLASH->CR |= FLASH_CR_STRT;
	return Wait(PAGE_ERASE_TIMEOUT);
}

template<class props>
bool Stm32Flash<props>::MassErase()
{
	TCritSect cs;
	Wait();
	FLASH->CR |= FLASH_CR_MER;
	FLASH->CR |= FLASH_CR_STRT;
	return Wait(MASS_ERASE_TIMEOUT);
}

template<class props>
bool Stm32Flash<props>::ReadOutProtect()
{
	TCritSect cs;
	Wait();
	Unlock();
	Options::Unlock();

	// erase options byte
	FLASH->CR |= FLASH_CR_OPTER;
	FLASH->CR |= FLASH_CR_STRT;
	bool ret = Wait(PAGE_ERASE_TIMEOUT);
	FLASH->CR &= ~FLASH_CR_OPTER;
	if (ret)
	{
		// program options byte
		FLASH->CR |= FLASH_CR_OPTPG;
		OB->RDP = 0x00;
		ret = Wait(PAGE_ERASE_TIMEOUT);
		FLASH->CR &= ~FLASH_CR_OPTPG;
		Options::Lock();
		Lock();
	}
	return ret;
}

} // namespace STM32

#endif // STM32TPL_STM32_FLASH_F1XX_H_INCLUDED
