/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *  Visit https://github.com/antongus/stm32tpl for new versions
 *
 *  Copyright (c) 2011-2023 Anton B. Gusev
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
 *  file         : stm32_flash_l0xx.h
 *  description  : stm32l0 flash class
 */

#pragma once

#include "stm32.h"
#include <cstring>

namespace STM32
{

namespace detail
{

struct FlashPropsL011
{
	static constexpr auto pageSize   {128};
	static constexpr auto flashSize  {16 * 1024};
	static constexpr auto eepromSize {512};
};

}

/**
 * Flash module.
 */
template <class Props>
class FlashController
{
public:
	static constexpr auto startAddress {0x08000000ul};
	static constexpr auto pageSize   {Props::pageSize};
	static constexpr auto flashSize  {Props::flashSize};
	static constexpr auto eepromSize {Props::eepromSize};

	/**
	 * PECR register. Should be unlocked before any write/erase operation
	 */
	struct Pecr
	{
		static bool isLocked();
		static void lock();
		static void unlock();
	};

	/**
	 * Options structure
	 */
	struct Options
	{
		static bool isLocked();
		static void lock();
		static void unlock();
		static void launch();
		static bool erase(uint32_t index);
		static bool write(uint32_t index, uint16_t value);
	};

	/**
	 * Eeprom structure
	 */
	struct Eeprom
	{
		static constexpr auto baseAddr {DATA_EEPROM_BASE};

		static bool isLocked() { return Pecr::isLocked(); }
		static void lock()     { Pecr::lock(); }
		static void unlock()   { Pecr::unlock(); }
		static void read(uint32_t addr, void* buf, uint32_t count);
		static bool write(uint32_t addr, void const* data, uint32_t dataSize);
	};

	/**
	 * Flash access
	 */
	struct Flash
	{
		static bool isLocked();
		static void lock();
		static void unlock();
		static void read(uint32_t addr, void* buf, uint32_t count);
		static bool erase(uint32_t pageAddr);
		static bool write(uint32_t addr, uint32_t data);
		static bool write(uint32_t addr, const void* buf, uint32_t count);
	};

	static bool isProtected();
	static bool protect();

private:
	static constexpr auto KEY1 {0x89ABCDEFU};
	static constexpr auto KEY2 {0x02030405U};
	static constexpr auto OPTKEY1 {0xFBEAD9C8U};
	static constexpr auto OPTKEY2 {0x24252627U};
	static constexpr auto PRGKEY1 {0x8C9DAEBFU};
	static constexpr auto PRGKEY2 {0x13141516U};

	static constexpr auto rdpLevel0 {0xAA};
	static constexpr auto rdpLevel1 {0xBB};
	static constexpr auto rdpLevel2 {0xCC};

	static bool busy()       { return FLASH->SR & FLASH_SR_BSY; }
	static void wait()       { while (busy()){}  __DSB(); }
	static void clearSr()    { FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGAERR | FLASH_SR_NOTZEROERR | FLASH_SR_WRPERR; }
	static bool checkEop();
};

template<class Props>
bool FlashController<Props>::checkEop()
{
	wait();
	if (FLASH->SR & FLASH_SR_EOP)  // programmed successfully
	{
		FLASH->SR |= FLASH_SR_EOP;
		return true;
	}
	clearSr();
	return false;
}

template<class Props>
bool FlashController<Props>::isProtected()
{
	return ((FLASH->OPTR & FLASH_OPTR_RDPROT) != rdpLevel0);
}


template<class Props>
bool FlashController<Props>::protect()
{
	if (isProtected())
		return true;

	Pecr::unlock();
	if (Options::isLocked())
	{
		Options::unlock();
		if (Options::isLocked())
			return false;
	}

//	Options::erase(0);
	Options::write(0, rdpLevel1);
	Options::launch();

	return true;  // should never got here
}


template<class Props>
void FlashController<Props>::Flash::read(uint32_t addr, void* buf, uint32_t count)
{
	auto src = reinterpret_cast<const uint8_t*>(addr);
	memcpy(buf, src, count);
}


template<class Props>
bool FlashController<Props>::Pecr::isLocked()
{
	return FLASH->PECR & FLASH_PECR_PELOCK;
}

template<class Props>
void FlashController<Props>::Pecr::lock()
{
	wait();
	FLASH->PECR |= FLASH_PECR_PELOCK;
}

/// check if option bytes is locked
template<class Props>
bool FlashController<Props>::Options::isLocked()
{
	return FLASH->PECR & FLASH_PECR_OPTLOCK;
}

/// Unlock option bytes.
/// PECR should be unlocked before unlocking option bytes
template<class Props>
void FlashController<Props>::Options::unlock()
{
	wait();
	FLASH->OPTKEYR = OPTKEY1;
	FLASH->OPTKEYR = OPTKEY2;
	__DSB();
}

/// Lock option bytes.
template<class Props>
void FlashController<Props>::Options::lock()
{
	FLASH->PECR |= FLASH_PECR_OPTLOCK;
}

/// Reload option bytes.
template<class Props>
void FlashController<Props>::Options::launch()
{
	FLASH->PECR |= FLASH_PECR_OBL_LAUNCH;
}

/**
 * Erase 32-bit options word
 * @param index - index of word to erase
 */
template<class Props>
bool FlashController<Props>::Options::erase(uint32_t index)
{
	FLASH->PECR |= FLASH_PECR_ERASE;
	auto optWords = reinterpret_cast<volatile uint32_t*>(OB_BASE);
	optWords[index] = 0;
	return checkEop();
}

/**
 * Write 16-bit word and its complement value to option word
 * @param index - index of options word
 * @param value - value to write
 */
template<class Props>
bool FlashController<Props>::Options::write(uint32_t index, uint16_t value)
{
	auto optWords = reinterpret_cast<volatile uint32_t*>(OB_BASE);
	auto word = (~static_cast<uint32_t>(value) << 16) | value;
	optWords[index] = word;
	return checkEop();
}

template<class Props>
void FlashController<Props>::Eeprom::read(uint32_t addr, void* buf, uint32_t count)
{
	auto src = reinterpret_cast<const uint8_t*>(addr);
	memcpy(buf, src, count);
}

/**
 * Write data to eeprom
 * @param addr - address to write
 * @param data pointer to data buffer
 * @param dataSize data size in bytes
 * @return true on success
 */
template<class Props>
bool FlashController<Props>::Eeprom::write(uint32_t addr, void const* data, uint32_t dataSize)
{
	auto ret {true};

	if (isLocked())
		unlock();
	FLASH->PECR |= FLASH_PECR_DATA;

	// if address and size is 4-byte aligned - use word access (32-bit)
	if ((addr & 0b11) == 0 && (dataSize & 0b11) == 0)
	{
		auto eeprom {reinterpret_cast<volatile uint32_t*>(addr)};
		auto words {reinterpret_cast<const uint32_t*>(data)};
		for (auto i = 0u; i < dataSize/4; ++i)
		{
			*eeprom++ = *words++;
			if (!checkEop())
			{
				ret = false;
				break;
			}
		}
	}
	else
	{
		auto eeprom {reinterpret_cast<volatile uint8_t*>(addr)};
		auto bytes {reinterpret_cast<const uint8_t*>(data)};
		for (auto i = 0u; i < dataSize; ++i)
		{
			*eeprom++ = *bytes++;
			if (!checkEop())
			{
				ret = false;
				break;
			}
		}
	}

	FLASH->PECR &= ~FLASH_PECR_DATA;
	clearSr();
	lock();
	return ret;
}

template<class Props>
bool FlashController<Props>::Flash::isLocked()
{
	return FLASH->PECR & FLASH_PECR_PRGLOCK;
}

template<class Props>
void FlashController<Props>::Flash::lock()
{
	FLASH->PECR |= FLASH_PECR_PRGLOCK;
}

template<class Props>
void FlashController<Props>::Flash::unlock()
{
	wait();
	FLASH->PRGKEYR = PRGKEY1;
	FLASH->PRGKEYR = PRGKEY2;
	__DSB();
}

/**
 * Write one word to flash.
 * @param addr - address to write
 * @param data - data to write
 * @return true on success, false otherwise
 */
template<class Props>
bool FlashController<Props>::Flash::write(uint32_t addr, uint32_t data)
{
	*reinterpret_cast<volatile uint32_t*>(addr) = data;
	return checkEop();
}

/**
 * Write block to flash
 * @param addr - address to write
 * @param buf - buffer to write
 * @param count - byte count to write
 * @return true on success, false otherwise
 */
template<class Props>
bool FlashController<Props>::Flash::write(uint32_t addr, const void* buf, uint32_t count)
{
	if (Pecr::isLocked())
		Pecr::unlock();
	if (isLocked())
	{
		unlock();
		if (isLocked())
			return false;
	}

	auto src = reinterpret_cast<const uint8_t*>(buf);
	const auto end = src + count;
	bool ret = true;

	while (src < end)
	{
		// calculate word-aligned address
		auto const alignedAddr = addr & ~3u;

		// read data from this address
		auto word = *reinterpret_cast<volatile uint32_t*>(alignedAddr);

		// calculate count of bytes to place into current word (1..4)
		auto const byteCount = 4 - (addr & 3);

		// modify bytes in the word
		auto shift = (addr & 3) * 8;
		for (auto i = 0u; i < byteCount; ++i)
		{
			word &= ~(0xFF << shift);
			word |= *src << shift;
			shift += 8;
			if (++src >= end)
				break;
		}

		// write modified data back
		if (!write(alignedAddr, word))
		{
			ret = false;
			break;
		}

		addr += byteCount;
	}

	lock();
	return ret;
}


template<class Props>
bool FlashController<Props>::Flash::erase(uint32_t pageAddr)
{
	wait();
	FLASH->PECR |= FLASH_PECR_ERASE | FLASH_PECR_PROG;
	*reinterpret_cast<volatile uint32_t*>(pageAddr) = 0;
	return checkEop();
}

template<class Props>
void FlashController<Props>::Pecr::unlock()
{
	wait();
	if (isLocked())
	{
		FLASH->PEKEYR = KEY1;
		FLASH->PEKEYR = KEY2;
		__DSB();
	}
}

using FlashL011 = FlashController<detail::FlashPropsL011>;

} // namespace STM32
