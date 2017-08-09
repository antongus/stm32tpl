/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 2017 Anton B. Gusev aka AHTOXA
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
 *  description  : MRAM memory class.
 *                 tested with:
 *                    MR25H10 (Everspin)
 *
 */

#ifndef STM32TPL_MRAM_H_INCLUDED
#define STM32TPL_MRAM_H_INCLUDED

#include <stdint.h>
#include "pin.h"
#include "stm32_spi.h"

struct SampleMramProps
{
	enum
	{
		SIZE_MB = 8,                  // 4, 8, 16 (MBytes)
		SHARE_PORT = false,           // set to true if SPI port is shared with other devices
		NOPS_AFTER_SELECT = 1,
		NOPS_BEFORE_DESELECT = 4
	};
	using CS = Pin<'A', 1, 'L'>;
	using WP = DummyPinOn;
};

template<class props = SampleMramProps>
class MramCore
{
public:
	/**
	 * Commands
	 */
	enum class Command : uint8_t
	{
		/// protection commands
		WREN            = 0x06, ///< write enable
		WRDI            = 0x04, ///< write disable

		/// status
		RDSR            = 0x05, ///< read status register
		WRSR            = 0x01, ///< write status register

		/// read/write
		READ            = 0x03, ///< read data
		WRITE           = 0x02, ///< write data

		/// miscellaneous.
		SLEEP           = 0xB9, ///< enter sleep mode
		WAKE            = 0xAB, ///< leave sleep mode
	};

	/**
	 * Status register
	 */
	struct Status
	{
		union
		{
			uint8_t byte;
			struct
			{
				uint8_t		                      :1;  ///< don't care
				uint8_t		WEL                   :1;  ///< write enable.
				uint8_t		BP                    :2;  ///< block protect bits (non-volatile)
				uint8_t		                      :3;  ///< don't care
				uint8_t		SRWD                  :1;  ///< status register write disable (non-volatile)
			}__attribute__ ((packed))
			bits;
		};
		Status() {}
		Status(const uint8_t val) { byte = val; }
		operator uint8_t() const { return byte; }
		Status operator=(const uint8_t val) { byte = val; return val; }
	};

	enum
	{
		SHARE_PORT = props::SHARE_PORT,
		NOPS_AFTER_SELECT = props::NOPS_AFTER_SELECT,
		NOPS_BEFORE_DESELECT = props::NOPS_BEFORE_DESELECT
	};

	MramCore(STM32::SPI::SpiBase& spiref)
		: spi_(spiref)
		{ }

	void read(uint32_t addr, void *buf, size_t len);
	void write(uint32_t addr, void const *buf, size_t len);
	bool verify(uint32_t addr, void const *buf, size_t len);
	Status readStatus();
protected:
	virtual void doSelect() = 0;
	virtual void doDeselect() = 0;
private:
	STM32::SPI::SpiBase& spi_;

	struct Locker
	{
		Locker(STM32::SPI::SpiBase& spi) : spi_(spi) { if (SHARE_PORT) spi_.Lock(); }
		~Locker() { if (SHARE_PORT) spi_.Unlock(); }
	private:
		STM32::SPI::SpiBase& spi_;
	};

	void select();
	void deselect();
	void command(Command cmd);
	void commandAndAddr(Command cmd, uint32_t addr);
	void writeEnable();
	bool writeStatus(uint8_t value);
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
	static void writeProtect(bool on)  { WP::On(on); };
};

template<class props>
void MramCore<props>::select()
{
	doSelect();
	for (int i = 0; i < NOPS_AFTER_SELECT; i++)
		__asm__ __volatile__ ("nop");
}

template<class props>
void MramCore<props>::deselect()
{
	for (int i = 0; i < NOPS_BEFORE_DESELECT; i++)
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
void MramCore<props>::commandAndAddr(Command cmd, uint32_t addr)
{
	select();
	spi_.Rw(static_cast<uint8_t>(cmd));
	spi_.Rw(addr >> 16);
	spi_.Rw(addr >> 8);
	spi_.Rw(addr);
}

template<class props>
void MramCore<props>::writeEnable()
{
	command(Command::WREN);
	deselect();
}

template<class props>
typename MramCore<props>::Status
MramCore<props>::readStatus()
{
	MramCore<props>::Status res;
	deselect();
	select();
	spi_.Rw(static_cast<uint8_t>(Command::RDSR));
	res = spi_.Rw();
	deselect();
	return res;
}

template<class props>
bool MramCore<props>::writeStatus(uint8_t value)
{
	Locker locker(spi_);
	bool ret = writeEnable();
	if (ret)
	{
		command(Command::WRSR);
		spi_.Rw(value);
		deselect();
	}
	return ret;
}

template<class props>
void MramCore<props>::read(uint32_t addr, void *buf, size_t len)
{
	Locker locker(spi_);
	commandAndAddr(Command::READ, addr);
	spi_.Rw();  // dummy byte
	uint8_t *p = reinterpret_cast<uint8_t *>(buf);
	while (len--)
		*p++ = spi_.Rw();

	deselect();
}

template<class props>
void MramCore<props>::write(uint32_t addr, void const *buf, size_t len)
{
	Locker locker(spi_);
	writeEnable();
	commandAndAddr(Command::WRITE, addr);

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
	Locker locker(spi_);
	bool ret = true;
	commandAndAddr(Command::READ, addr);
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
