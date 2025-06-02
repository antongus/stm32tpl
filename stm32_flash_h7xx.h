/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *  Visit https://github.com/antongus/stm32tpl for new versions
 *
 *  Copyright (c) 2011-2025 Anton B. Gusev
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
 *  file         : stm32_flash_h7xxx.h
 *  description  : FLASH module class template for STM32H7xx series.
 *
 */

#ifndef STM32TPL_STM32_FLASH_H7XX_H_INCLUDED
#define STM32TPL_STM32_FLASH_H7XX_H_INCLUDED

#include "stm32.h"
#if __has_include("scmRTOS.h")
#  include "scmRTOS.h"
using CritSect = TCritSect;
#else
#  include "CritSect.h"
#endif
#include <type_traits>
#include <cstring>

namespace STM32
{

/**
 * Program parallelism setting selection enum
 * Selects maximum bits that can be programmed in one step.
 */
enum ProgramWordWidth : uint32_t
{
	pw8bit   = (0UL << FLASH_CR_PSIZE_Pos),   //!< 8bit
	pw16bit  = (1UL << FLASH_CR_PSIZE_Pos),   //!< 16bit
	pw32bit  = (2UL << FLASH_CR_PSIZE_Pos),   //!< 32bit
	pw64bit  = (3UL << FLASH_CR_PSIZE_Pos),   //!< 64bit
};


/**
 * Default properties for STM32 Flash class template.
 */
struct FlashDefaultProps
{
	static constexpr auto bankNum  {1};             //!< bank number: 1/2
	static const ProgramWordWidth pSize = pw32bit;
	static constexpr auto PAGE_ERASE_TIMEOUT {0xFFFFu};
	static constexpr auto MASS_ERASE_TIMEOUT {0xFFFFFFu};
};


/**
 * STM32 Flash class template.
 */
template<class props = FlashDefaultProps> class FlashController;


template <class props>
class FlashController
{
public:
	static constexpr auto bankNum  {props::bankNum == 1 ? 1 : 2};             //!< bank number: 1/2
	static constexpr auto START_ADDRESS { bankNum == 1 ? 0x08000000UL : 0x08100000UL };

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

	/// Check if FLASH->CR1 locked
	static bool isLocked() { return CRx() & FLASH_CR_LOCK; }
	/// Lock access to FLASH->CR1
	static void lock()   { CRx() |= FLASH_CR_LOCK; }
	/// unlock writes to FLASH->CR1 register
	static void unlock()
	{
		auto& keyr = bankNum == 1 ? FLASH->KEYR1 : FLASH->KEYR2;
		keyr = KEY1;
		keyr = KEY2;
	}

	static bool eraseSector(uint32_t sector);
	static bool eraseBank();

	static void read(uint32_t addr, void* buf, uint32_t count);
	static bool write(uint32_t addr, const void* buf, uint32_t count);

	static bool isReadOutProtected();
	static bool readOutProtect();
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

	/// CR1/CR2 depending on selected bank
	static inline auto& CRx() { return bankNum == 1 ? FLASH->CR1 : FLASH->CR2; }
	/// SR1/SR2 depending on selected bank
	static inline auto& SRx() { return bankNum == 1 ? FLASH->SR1 : FLASH->SR2; }

	static bool isPgerr()     { return SRx() & (FLASH_SR_OPERR | FLASH_SR_PGSERR); }
	static bool isWrprterr()  { return SRx() & FLASH_SR_WRPERR; }
	static bool isBusy()      { return SRx() & FLASH_SR_QW; }
	static bool isQw()        { return SRx() & FLASH_SR_QW; }
	static void start()       { CRx() |= FLASH_CR_START; }

	static void delay();
	static void wait();
	static bool wait(uint32_t timeout);

	template <typename T>
	static bool write(uint32_t addr, T data);
};

typedef FlashController<> Flash;

template<class props>
void FlashController<props>::wait()
{
	while (isBusy()) ;
	__DSB();
}

template<class props>
bool FlashController<props>::isReadOutProtected()
{
	if (rdpByte == rdpLevelNone)
		return false;
	return true;
}

template<class props>
bool FlashController<props>::readOutProtect()
{
	unlock();
	Options::Unlock();

	SRx() = 0
		| FLASH_SR_EOP
		| FLASH_SR_OPERR
		| FLASH_SR_PGSERR
		| FLASH_SR_WRPERR // clear errors, if any
		;


	if (!isReadOutProtected())
	{
		rdpByte = rdpLevelOne;
//		if (FLASH_OB_RDPConfig(OB_RDP_Level_1) == FLASH_COMPLETE)  // TODO
//		{
//			/* Generate System Reset to load the new option byte values */
//			FLASH_OB_Launch();
//		}
	}
	return true;
}

template<class props>
bool FlashController<props>::wait(uint32_t timeout)
{
	while (isBusy())
	{
		if (!--timeout)
			return false;
		delay();
	}
	__DSB();
	return true;
}

template<class props>
void FlashController<props>::delay()
{
	for (volatile int i = 0; i < 0xFF; i++) ;
}

template<class props>
void FlashController<props>::read(uint32_t addr, void* buf, uint32_t count)
{
	auto src = reinterpret_cast<const uint8_t*>(addr);
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
bool FlashController<props>::write(uint32_t addr, T data)
{
	static_assert(std::is_integral<T>::value, "Only integral types allowed");

	volatile T* ptr = reinterpret_cast<volatile T*>(addr);

	CritSect cs;

	wait();
	CRx() = pSize | FLASH_CR_PG;
    __ISB();
    __DSB();

    *ptr = data;
	for(auto i = 0; i < 1000; ++i)
		if (isQw()) break;
	while(isQw()) {}
//	wait();

	bool ret = !(isPgerr() || isWrprterr());

	SRx() = 0
		| FLASH_SR_EOP
		| FLASH_SR_OPERR
		| FLASH_SR_PGSERR
		| FLASH_SR_WRPERR // clear errors, if any
		;
	CRx() &= ~FLASH_CR_PG;
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
bool FlashController<props>::write(uint32_t addr, const void* buf, uint32_t count)
{
	if (isLocked())
		unlock();

	if (isLocked())
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
			if (!write(addr++, *src++))
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
			if (!write(alignedAddr, data))
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
			if (!write(alignedAddr, data))
			{
				ret = false;
				break;
			}

			addr += byteCount;
		}
		break;
	}
	lock();
	return ret;
}

/**
 * Erase sector.
 * @param sector - sector to erase
 * @return true on success, false otherwise
 */
template<class props>
bool FlashController<props>::eraseSector(uint32_t sector)
{
	if (sector > 7)
		return false;

	if (isLocked())  // unlock only if locked!
		unlock();

	CritSect cs;
	wait();
	CRx() = 0
			| pSize
			| FLASH_CR_SER   // Sector erase
			| (sector << FLASH_CR_SNB_Pos)  // set sector
			;
	start();
	bool ret = wait(PAGE_ERASE_TIMEOUT);
	lock();
	return ret;
}

/**
 * Erase entire flash bank. (Loooong!)
 * @return true on success, false otherwise
 */
template<class props>
bool FlashController<props>::eraseBank()
{
	CritSect cs;
	wait();
	CRx() = pSize | FLASH_CR_BER;
	start();
	bool ret = wait(MASS_ERASE_TIMEOUT);
	lock();
	return ret;
}

} // namespace STM32

#endif // STM32TPL_STM32_FLASH_H7XX_H_INCLUDED
