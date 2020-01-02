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
 *  file         : mram.h
 *  description  : serial MRAM memory class.
 *                 tested with:
 *                    MR25H10 (Everspin)
 *
 */

#ifndef STM32TPL_MRAM_H_INCLUDED
#define STM32TPL_MRAM_H_INCLUDED

#include <cstdint>
#include "pin.h"
#include "stm32_spi.h"

/**
 * Sample MRAM properties structure
 */
struct SampleMramProps
{
	static constexpr bool lockSpi { false };           //!< set to true if SPI port is shared
	static constexpr unsigned nopsAfterSelect { 1 };
	static constexpr unsigned nopsBeforeDeselect { 4 };
};

template<class props = SampleMramProps>
class MramCore
{
public:
	/// Command codes
	enum class Command : uint8_t
	{
		WREN            = 0x06, ///< write enable
		WRDI            = 0x04, ///< write disable
		RDSR            = 0x05, ///< read status register
		WRSR            = 0x01, ///< write status register
		READ            = 0x03, ///< read data
		WRITE           = 0x02, ///< write data
		SLEEP           = 0xB9, ///< enter sleep mode
		WAKE            = 0xAB, ///< leave sleep mode
	};

	/// Status bits
	enum StatusBits : uint8_t
	{
		STATUS_WEL     = 1U << 1,  ///< write enable latch (1 after WREN, 0 after WRDI)
		STATUS_BP0     = 1U << 2,  ///< block protect bits (non-volatile)
		STATUS_BP1     = 1U << 3,  ///< block protect bits (non-volatile)
		STATUS_SRWD    = 1U << 7,  ///< status register write disable (non-volatile)
	};

	MramCore(STM32::SPI::SpiBase& spiref)
		: spi_(spiref)
		{ }

	void read(uint32_t addr, void *buf, size_t len);
	void write(uint32_t addr, void const *buf, size_t len);
	bool verify(uint32_t addr, void const *buf, size_t len);
	void writeProtect(bool on);

	void writeEnable()    { command(Command::WREN); deselect(); }
	void writeDisable()   { command(Command::WRDI); deselect(); }
	void sleep()          { command(Command::SLEEP); deselect(); }
	void wake()           { command(Command::WAKE); deselect(); }

protected:
	virtual void doSelect() = 0;
	virtual void doDeselect() = 0;
	virtual void doWriteProtect(bool) = 0;

private:
	STM32::SPI::SpiBase& spi_;

	static constexpr bool lockSpi { props::lockSpi };
	static constexpr unsigned nopsAfterSelect { props::nopsAfterSelect };
	static constexpr unsigned nopsBeforeDeselect { props::nopsBeforeDeselect };

	void select();
	void deselect();
	void command(Command cmd);
	void command(Command cmd, uint32_t addr);
	uint8_t readStatus();
	void writeStatus(uint8_t value);
};

/**
 * MRAM chip.
 * It is actually MramCore<> descendant + CS and WP pin.
 * Made separate class to optimize multi-chip configurations.
 */
template<class props>
class MramChip : public props::CoreType
{
public:
	using CoreType = typename props::CoreType;
	MramChip(STM32::SPI::SpiBase& spiref)
		: CoreType(spiref)
		{
			CS::Mode(OUTPUT);
			CS::Off();
			WP::Mode(OUTPUT);
			WP::Off();
		}

protected:
	using CS = typename props::CS;
	using WP = typename props::WP;
	void doSelect() override   { CS::On(); };
	void doDeselect() override { CS::Off(); };
	void doWriteProtect(bool on) override { WP::On(on); };
};

template<class props>
void MramCore<props>::select()
{
	if (lockSpi)
		spi_.Lock();
	doSelect();
	for (auto i = 0U; i < nopsAfterSelect; i++)
		__asm__ __volatile__ ("nop");
}

template<class props>
void MramCore<props>::deselect()
{
	if (lockSpi)
		spi_.Unlock();
	for (auto i = 0U; i < nopsBeforeDeselect; i++)
		__asm__ __volatile__ ("nop");
	doDeselect();
}

template<class props>
void MramCore<props>::command(Command cmd)
{
	select();
	spi_.Rw(static_cast<uint8_t>(cmd));
}

template<class props>
void MramCore<props>::command(Command cmd, uint32_t addr)
{
	command(cmd);
	spi_.Rw(addr >> 16);
	spi_.Rw(addr >> 8);
	spi_.Rw(addr);
}

template<class props>
uint8_t MramCore<props>::readStatus()
{
	command(Command::RDSR);
	uint8_t res = spi_.Rw();
	deselect();
	return res;
}

template<class props>
void MramCore<props>::writeStatus(uint8_t value)
{
	writeEnable();
	command(Command::WRSR);
	spi_.Rw(value);
	deselect();
}

template<class props>
void MramCore<props>::writeProtect(bool on)
{
	doWriteProtect(on);
}

template<class props>
void MramCore<props>::read(uint32_t addr, void *buf, size_t len)
{
	command(Command::READ, addr);
	uint8_t *p = reinterpret_cast<uint8_t *>(buf);
	while (len--)
		*p++ = spi_.Rw();

	deselect();
}

template<class props>
void MramCore<props>::write(uint32_t addr, void const *buf, size_t len)
{
	writeEnable();
	command(Command::WRITE, addr);

	// send data itself
	uint8_t const *p = reinterpret_cast<uint8_t const *>(buf);
	while (len--)
		spi_.Rw(*p++);
	// done
	deselect();
}

template<class props>
bool MramCore<props>::verify(uint32_t addr, void const *buf, size_t len)
{
	bool ret = true;
	command(Command::READ, addr);
	spi_.Rw();  // dummy byte
	uint8_t const *p = reinterpret_cast<uint8_t const *>(buf);
	while (len--)
	{
		if (*p++ != spi_.Rw())
		{
			ret = false;
			break;
		}
	}
	deselect();
	return ret;
}

#endif // STM32TPL_MRAM_H_INCLUDED
