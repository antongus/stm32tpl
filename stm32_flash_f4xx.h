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
 *  description  : FLASH module class template for STM32F4xx series.
 *  created on   : 03.08.2016
 *
 */

#ifndef STM32TPL_STM32_FLASH_F4XX_H_INCLUDED
#define STM32TPL_STM32_FLASH_F4XX_H_INCLUDED

#include "stm32.h"
#include <type_traits>

namespace STM32
{

/**
 * Program parallelism setting selection enum
 * Selects maximum bits that can be programmed in one step.
 */
enum ProgramWordWidth : uint32_t
{
	pw8bit   = (0UL << 8),   //!< 8bit  (VCC >= 1.8V)
	pw16bit  = (1UL << 8),   //!< 16bit (VCC >= 2.1V)
	pw32bit  = (2UL << 8),   //!< 32bit (VCC >= 2.7V)
	pw64bit  = (3UL << 8),   //!< 64bit (VCC >= 2.7V + VPP present)
};


/**
 * Default properties for STM32 Flash class template.
 */
struct FlashDefaultProps
{
	static const ProgramWordWidth pSize = pw32bit;
	enum { PAGE_ERASE_TIMEOUT = 0xFFFF };
	enum { MASS_ERASE_TIMEOUT = 0xFFFFFF };
};


/**
 * STM32 Flash class template.
 */
template<class props = FlashDefaultProps> class FlashController;


template <class props>
class FlashController
{
public:
	enum { START_ADDRESS = 0x08000000UL };

	/**
	 * Options structure
	 */
	struct Options
	{
		enum
		{
			OPTKEY1 = 0x08192A3B,
			OPTKEY2 = 0x4C5D6E7F,
		};
		static bool Locked()   { return !(FLASH->OPTCR & FLASH_OPTCR_OPTLOCK); }
		static void Lock()     { FLASH->OPTCR &= ~FLASH_OPTCR_OPTLOCK; }
		static void Unlock()   { FLASH->OPTKEYR = OPTKEY1; FLASH->OPTKEYR = OPTKEY2; }
	};

	/// Check if FLASH->CR locked
	static bool Locked()   { return FLASH->CR & FLASH_CR_LOCK; }
	/// Lock access to FLASH->CR
	static void Lock()     { FLASH->CR |= FLASH_CR_LOCK; }
	/// unlock writes to FLASH->CR register
	static void Unlock()   { FLASH->KEYR = KEY1; FLASH->KEYR = KEY2; }

	static bool EraseSector(uint32_t sector);
	static bool MassErase();

	static void Read(uint32_t addr, void* buf, uint32_t count);
	static bool Write(uint32_t addr, const void* buf, uint32_t count);

	static bool IsReadOutProtected();
	static bool ReadOutProtect();
private:
	enum
	{
		KEY1 = 0x45670123,
		KEY2 = 0xCDEF89AB
	};

	enum { PAGE_ERASE_TIMEOUT = props::PAGE_ERASE_TIMEOUT };
	enum { MASS_ERASE_TIMEOUT = props::MASS_ERASE_TIMEOUT };
	static const ProgramWordWidth pSize = props::pSize;

	IORegister<FLASH_BASE + 0x14 + 1, uint8_t> optCrByte0;
	static IORegister<FLASH_BASE + 0x14 + 1, uint8_t> rdpByte;
	enum {
		rdpLevelNone = 0xAA,
		rdpLevelOne = 1,
		rdpLevelTwo = 0x55,
	};

	static bool Pgerr()      { return FLASH->SR & (FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR); }
	static bool Wrprterr()   { return FLASH->SR & FLASH_SR_WRPERR; }
	static bool Busy()       { return FLASH->SR & FLASH_SR_BSY; }
	static void Start()      { FLASH->CR |= FLASH_CR_STRT; }

	static void Delay();
	static void Wait();
	static bool Wait(uint32_t timeout);

	template <typename T>
	static bool Write(uint32_t addr, T data);
};

typedef FlashController<> Flash;

template<class props>
void FlashController<props>::Wait()
{
	while (Busy()) ;
	__DSB();
}

template<class props>
bool FlashController<props>::IsReadOutProtected()
{
	if (rdpByte == rdpLevelNone)
		return false;
	return true;
}

template<class props>
bool FlashController<props>::ReadOutProtect()
{
	Unlock();
	Options::Unlock();

	FLASH->SR = 0
			| FLASH_SR_EOP
			| FLASH_SR_PGAERR
			| FLASH_SR_PGPERR
			| FLASH_SR_PGSERR
			| FLASH_SR_WRPERR // clear errors, if any
			;


	if (!IsReadOutProtected())
	{
		rdpByte = rdpLevelOne;
//		if (FLASH_OB_RDPConfig(OB_RDP_Level_1) == FLASH_COMPLETE)
//		{
//			/* Generate System Reset to load the new option byte values */
//			FLASH_OB_Launch();
//		}
	}
	return true;
}

template<class props>
bool FlashController<props>::Wait(uint32_t timeout)
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
void FlashController<props>::Delay()
{
	for (volatile int i = 0; i < 0xFF; i++) ;
}

template<class props>
void FlashController<props>::Read(uint32_t addr, void* buf, uint32_t count)
{
	const uint8_t* src = reinterpret_cast<const uint8_t*>(addr);
	memcpy(buf, src, count);
}

/**
 * Write one portion of data to flash. Portion width depends on template parameter T.
 * T could be uint8_t, uint16_t, uint32_t
 * @param addr - address to write
 * @param data - data to write
 * @return true on success, false otherwise
 */
template<class props>
template <typename T>
bool FlashController<props>::Write(uint32_t addr, T data)
{
	static_assert(std::is_integral<T>::value, "Only integral types allowed");

	volatile T* ptr = reinterpret_cast<volatile T*>(addr);

	TCritSect cs;

	Wait();
	FLASH->CR = pSize | FLASH_CR_PG;
	*ptr = data;
	Wait();

	bool ret = !(Pgerr() || Wrprterr());

	FLASH->SR = 0
			| FLASH_SR_EOP
			| FLASH_SR_PGAERR
			| FLASH_SR_PGPERR
			| FLASH_SR_PGSERR
			| FLASH_SR_WRPERR // clear errors, if any
			;
	FLASH->CR = 0;
	return ret;
}

/**
 * Write block to flash
 * @param addr - address to write
 * @param buf - buffer to write
 * @param count - byte count to write
 * @return true on success, false otherwise
 */
template<class props>
bool FlashController<props>::Write(uint32_t addr, const void* buf, uint32_t count)
{
	if (Locked())
		Unlock();

	if (Locked())
		return false;

	const uint8_t* src = reinterpret_cast<const uint8_t*>(buf);
	const uint8_t* const end = src + count;

	bool ret = true;

	switch(pSize)
	{
	default:
	case pw8bit:   // byte access - simplest case.
		while (src < end)
		{
			if (!Write(addr++, *src++))
			{
				ret = false;
				break;
			}
		}
		break;

	case pw16bit:  // half-word access
		while (src < end)
		{
			// calculate half-word-aligned address
			uint32_t alignedAddr = addr & ~1UL;

			// read data from this address
			volatile uint16_t* ptr = reinterpret_cast<volatile uint16_t*>(alignedAddr);
			uint16_t data = *ptr;

			if ((addr & 1) == 0) // address is aligned - take lower byte
			{
				data &= ~(0xFF << 0);
				data |= *src << 0;
				++src;
			}
			if (src < end)      // source end not reached yet - take higher byte
			{
				data &= ~(0xFF << 8);
				data |= *src << 8;
				++src;
			}

			// write modified half-word data back
			if (!Write(alignedAddr, data))
			{
				ret = false;
				break;
			}
			addr = alignedAddr + 2;
		}
		break;

	case pw32bit:  // word access
		while (src < end)
		{
			// calculate word-aligned address
			uint32_t alignedAddr = addr & ~3UL;

			// read data from this address
			volatile uint32_t* ptr = reinterpret_cast<volatile uint32_t*>(alignedAddr);
			uint32_t data = *ptr;

			// calculate count of bytes to place into current word
			uint32_t byteCount = 4 - (addr & 3UL);

			// modify bytes in the word
			uint32_t shift = (4 - byteCount) * 8;
			for (auto i = 0U; i < byteCount; ++i)
			{
				uint32_t byte = *src++;
				data &= ~(0xFF << shift);
				data |= byte << shift;
				shift += 8;
				if (src >= end)
					break;
			}

			// write modified data back
			if (!Write(alignedAddr, data))
			{
				ret = false;
				break;
			}

			addr += byteCount;
		}
		break;
	}
	Lock();
	return ret;
}

/**
 * Erase sector.
 * @param sector - sector to erase
 * @return true on success, false otherwise
 */
template<class props>
bool FlashController<props>::EraseSector(uint32_t sector)
{
	if (sector > 11)
		return false;

	if (Locked())
		Unlock();

	TCritSect cs;
	Wait();
	FLASH->CR = 0
			| pSize          // select parallelism
			| FLASH_CR_SER   // Sector erase
			| (sector << 3)  // set sector
			;
	Start();
	bool ret = Wait(PAGE_ERASE_TIMEOUT);
	Lock();
	return ret;
}

/**
 * Erase entire chip flash.
 * @return true on success, false otherwise
 */
template<class props>
bool FlashController<props>::MassErase()
{
	TCritSect cs;
	Wait();
	FLASH->CR = pSize | FLASH_CR_MER;
	Start();
	bool ret = Wait(MASS_ERASE_TIMEOUT);
	Lock();
	return ret;
}

} // namespace STM32

#endif // STM32TPL_STM32_FLASH_F4XX_H_INCLUDED
