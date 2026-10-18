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
	// Timeouts are counted in delay() iterations, ~4us per iteration at max clock (480MHz).
	// Sized for worst case (datasheet, lowest parallelism): word program 0.58ms,
	// sector erase 4s, bank erase 26s.
	static constexpr auto PROGRAM_TIMEOUT    {0x10000u};
	static constexpr auto PAGE_ERASE_TIMEOUT {0x200000u};
	static constexpr auto MASS_ERASE_TIMEOUT {0xFFFFFFu};
};


/**
 * STM32 Flash class template.
 * While a bank is being programmed/erased, reads from that bank (instruction
 * fetch, interrupt vectors, data) stall until the operation completes: run
 * program/erase code from RAM or from the other bank if interrupts, RTOS or
 * watchdog must stay alive meanwhile (RM0399 sec.4.3.7).
 * The application must configure FLASH_ACR (LATENCY, WRHIGHFREQ) according to
 * the flash interface clock and voltage scale before any program/erase
 * operation: use configACR<axiClockHz, vos>() below, hardware does not verify
 * the settings (RM0399 table 16).
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
		static bool Locked()   { return FLASH->OPTCR & FLASH_OPTCR_OPTLOCK; }
		static void Lock()     { FLASH->OPTCR |= FLASH_OPTCR_OPTLOCK; }
		static void Unlock()
		{
			// Unlocking an already unlocked register locks it until next system reset (RM0399)
			if (!Locked())
				return;
			FLASH->OPTKEYR = OPTKEY1;
			FLASH->OPTKEYR = OPTKEY2;
		}
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

	/**
	 * Power voltage scaling ranges (RM0399 table 16).
	 */
	enum VoltageScale
	{
		vos3, //!< 0.95..1.05 V, most conservative
		vos2, //!< 1.05..1.15 V
		vos1, //!< 1.15..1.26 V
		vos0, //!< 1.26..1.40 V
	};

	/**
	 * Set FLASH_ACR (LATENCY, WRHIGHFREQ) for the given flash AXI interface
	 * clock and voltage scale (RM0399 table 16). Must be called before any
	 * program/erase operation. On 480MHz parts the AXI clock is SYSCLK/2.
	 * When raising the AXI clock, call before the RCC change; when lowering -
	 * after it (RM0399 sec.4.3.8).
	 * @tparam axiClockHz - flash AXI interface clock in Hz
	 * @tparam vos - power voltage scaling range
	 * @return true when new settings are confirmed by readback, false otherwise
	 */
	template <uint32_t axiClockHz, VoltageScale vos = vos3>
	static bool configACR()
	{
		constexpr uint32_t MHz = 1000000UL;
		constexpr uint32_t f = axiClockHz;

		static_assert(checkAcrParams(axiClockHz, vos),
			"AXI clock out of range for selected voltage scale (RM0399 table 16)");

		uint32_t ws;
		uint32_t wr;
		switch (vos)
		{
		case vos2:
			if (f > 165*MHz)      { ws = 3; wr = 2; }
			else if (f > 110*MHz) { ws = 2; wr = 1; }
			else if (f > 55*MHz)  { ws = 1; wr = 1; }
			else                  { ws = 0; wr = 0; }
			break;
		case vos1:
			if (f > 210*MHz)      { ws = 3; wr = 2; }
			else if (f > 185*MHz) { ws = 2; wr = 2; }
			else if (f > 140*MHz) { ws = 2; wr = 1; }
			else if (f > 70*MHz)  { ws = 1; wr = 1; }
			else                  { ws = 0; wr = 0; }
			break;
		case vos0:
			if (f > 225*MHz)      { ws = 4; wr = 2; }
			else if (f > 210*MHz) { ws = 3; wr = 2; }
			else if (f > 185*MHz) { ws = 2; wr = 2; }
			else if (f > 140*MHz) { ws = 2; wr = 1; }
			else if (f > 70*MHz)  { ws = 1; wr = 1; }
			else                  { ws = 0; wr = 0; }
			break;
		default: // vos3
			if (f > 180*MHz)      { ws = 4; wr = 2; }
			else if (f > 135*MHz) { ws = 3; wr = 2; }
			else if (f > 90*MHz)  { ws = 2; wr = 1; }
			else if (f > 45*MHz)  { ws = 1; wr = 1; }
			else                  { ws = 0; wr = 0; }
			break;
		}

		const uint32_t acr = (FLASH->ACR & ~(FLASH_ACR_LATENCY | FLASH_ACR_WRHIGHFREQ))
			| (ws << FLASH_ACR_LATENCY_Pos)
			| (wr << FLASH_ACR_WRHIGHFREQ_Pos);

		FLASH->ACR = acr;

		return (FLASH->ACR & (FLASH_ACR_LATENCY | FLASH_ACR_WRHIGHFREQ))
			== (acr & (FLASH_ACR_LATENCY | FLASH_ACR_WRHIGHFREQ));
	}

	static bool eraseSector(uint32_t sector);
	static bool eraseBank();

	static void read(uint32_t addr, void* buf, uint32_t count);
	static bool write(uint32_t addr, const void* buf, uint32_t count);

	static bool isReadOutProtected();
	static bool readOutProtect();
private:
	/// AXI clock frequency must be defined and fit the voltage scale range (RM0399 table 16)
	static constexpr bool checkAcrParams(uint32_t axiClockHz, VoltageScale vos)
	{
		constexpr uint32_t MHz = 1000000UL;
		switch (vos)
		{
		case vos0: return axiClockHz > 0 && axiClockHz <= 240*MHz;
		case vos1: return axiClockHz > 0 && axiClockHz <= 225*MHz;
		case vos2: return axiClockHz > 0 && axiClockHz <= 225*MHz;
		default:   return axiClockHz > 0 && axiClockHz <= 225*MHz; // vos3
		}
	}

	enum
	{
		KEY1 = 0x45670123,
		KEY2 = 0xCDEF89AB
	};

	enum { PROGRAM_TIMEOUT = props::PROGRAM_TIMEOUT };
	enum { PAGE_ERASE_TIMEOUT = props::PAGE_ERASE_TIMEOUT };
	enum { MASS_ERASE_TIMEOUT = props::MASS_ERASE_TIMEOUT };
	static const ProgramWordWidth pSize = props::pSize;

	enum {
		rdpLevelNone = 0xAA,
		rdpLevelOne  = 0x01, //!< any value but 0xAA/0xCC means level 1
		rdpLevelTwo  = 0xCC,
	};

	static uint32_t rdp() { return FLASH->OPTSR_CUR & FLASH_OPTSR_RDP; }

	/// CR1/CR2 depending on selected bank
	static inline auto& CRx() { return bankNum == 1 ? FLASH->CR1 : FLASH->CR2; }
	/// SR1/SR2 depending on selected bank
	static inline auto& SRx() { return bankNum == 1 ? FLASH->SR1 : FLASH->SR2; }

	static bool isBusy()      { return SRx() & FLASH_SR_QW; }
	static void start()       { CRx() |= FLASH_CR_START; }

	// error flags reported for program/erase operations (rc_w1: writing 1 clears)
	enum {
		progErr  = FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR | FLASH_SR_INCERR | FLASH_SR_OPERR,
		eraseErr = FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR | FLASH_SR_OPERR
	};
	static bool isProgErr()     { return SRx() & progErr; }
	static void clearProgErr()  { SRx() = progErr; }
	static bool isEraseErr()    { return SRx() & eraseErr; }
	static void clearEraseErr() { SRx() = eraseErr; }

	static void delay();
	static void wait();
	static bool wait(uint32_t timeout);

	static bool writeFlashWord(uint32_t alignedAddr, const uint32_t* data);
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
	if (rdp() == (rdpLevelNone << FLASH_OPTSR_RDP_Pos))
		return false;
	return true;
}

/**
 * Set readout protection level 1.
 * New protection value becomes effective after system reset.
 * @return true if protection is set (also when it was set before the call),
 * false on timeout or option change error
 */
template<class props>
bool FlashController<props>::readOutProtect()
{
	if (isReadOutProtected())
		return true;

	Options::Unlock();
	FLASH->OPTCCR = FLASH_OPTCCR_CLR_OPTCHANGEERR;

	uint32_t opt = FLASH->OPTSR_CUR & ~FLASH_OPTSR_RDP;
	opt |= rdpLevelOne << FLASH_OPTSR_RDP_Pos;
	FLASH->OPTSR_PRG = opt;

	FLASH->OPTCR |= FLASH_OPTCR_OPTSTART;

	uint32_t timeout = PAGE_ERASE_TIMEOUT;
	while (FLASH->OPTSR_CUR & FLASH_OPTSR_OPT_BUSY)
	{
		if (!--timeout)
			return false;
		delay();
	}

	Options::Lock();

	if (FLASH->OPTSR_CUR & FLASH_OPTSR_OPTCHANGEERR)
		return false;

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
 * Program one 256-bit flash word at 32-byte aligned address.
 * Programming is triggered by the last write that completes the 256-bit write
 * buffer, so the whole flash word must be written at once (RM0399 sec.4.6.4).
 * @param alignedAddr - 32-byte aligned flash address
 * @param data - 8 words (32 bytes) to program
 * @return true on success, false otherwise
 */
template<class props>
bool FlashController<props>::writeFlashWord(uint32_t alignedAddr, const uint32_t* data)
{
	CritSect cs;

	wait();
	clearProgErr(); // clearing PGSERR/INCERR is mandatory before a write operation
	CRx() = pSize | FLASH_CR_PG;
	__DSB();

	auto ptr = reinterpret_cast<volatile uint32_t*>(alignedAddr);
	for (auto i = 0; i < 8; i++)
		ptr[i] = data[i];

	// last write triggers programming and raises QW; a rejected write never raises it
	for (auto i = 0; i < 1000; i++)
		if (isBusy())
			break;

	bool ret = wait(PROGRAM_TIMEOUT) && !isProgErr();

	clearProgErr();
	CRx() &= ~FLASH_CR_PG;
	return ret;
}

/**
 * Write block to flash. Target area must be erased: new bytes are merged with
 * the existing content of each 256-bit flash word, bits can only be cleared.
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

	while (src < end)
	{
		const uint32_t wordAddr = addr & ~31UL;

		// copy current flash word content, overwrite bytes being written
		uint32_t word[8];
		memcpy(word, reinterpret_cast<const void*>(wordAddr), sizeof(word));

		auto dst = reinterpret_cast<uint8_t*>(word);
		for (auto offset = addr - wordAddr; offset < 32 && src < end; offset++)
			dst[offset] = *src++;

		ret = writeFlashWord(wordAddr, word);
		if (!ret)
			break;

		addr = wordAddr + 32;
	}

	lock();
	return ret;
}

/**
 * Erase sector in the bank selected by props::bankNum.
 * @param sector - sector number within the selected bank (0..7)
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
	clearEraseErr();
	CRx() = 0
			| pSize
			| FLASH_CR_SER   // Sector erase
			| (sector << FLASH_CR_SNB_Pos)  // set sector
			;
	start();
	bool ret = wait(PAGE_ERASE_TIMEOUT) && !isEraseErr();
	lock();
	return ret;
}

/**
 * Erase entire bank selected by props::bankNum (one bank only; both banks
 * need one call per bank instance). (Loooong!)
 * @return true on success, false otherwise
 */
template<class props>
bool FlashController<props>::eraseBank()
{
	if (isLocked())  // unlock only if locked!
		unlock();

	CritSect cs;
	wait();
	clearEraseErr();
	CRx() = pSize | FLASH_CR_BER;
	start();
	bool ret = wait(MASS_ERASE_TIMEOUT) && !isEraseErr();
	lock();
	return ret;
}

} // namespace STM32

#endif // STM32TPL_STM32_FLASH_H7XX_H_INCLUDED
