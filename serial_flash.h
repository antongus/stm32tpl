/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 2014 Anton B. Gusev aka AHTOXA
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
 *  file         : serial_flash.h
 *  description  : serial flash memory class.
 *                 tested with:
 *                    MX25L6406E (Macronix)
 *                    IS25LP064 (Integrated Silicon Solution, Inc.)
 *
 */

#ifndef IS25LP_H_INCLUDED
#define IS25LP_H_INCLUDED

#include <stdint.h>
#include "pin.h"
#include "stm32_spi.h"

#include "textstream.h"

extern TextStream& uart;

namespace  // private types
{

/**
 * all IS25 opcodes
 */
enum IS25Command
{
	/// read commands
	READ                             = 0x03, ///< normal read, up to 50MHz,  needs 0 dummy bytes
	FAST_READ                        = 0x0B, ///< fast read, up to 133MHz, needs dummy cycles

	/// program commands
	PP                               = 0x02, ///< page program (write 1..256 byte of data)

	/// erase commands
	ERASE_SECTOR                     = 0x20, ///< sector erase, (4KB)
	ERASE_BLOCK32K                   = 0x52, ///< block erase, 32KB
	ERASE_BLOCK64K                   = 0xD8, ///< block erase, 64KB
	ERASE_CHIP                       = 0x60, ///< chip erase
	ERASE_CHIP2                      = 0xC7, ///< chip erase

	/// protection commands
	WREN                             = 0x06, ///< write enable
	WRDI                             = 0x04, ///< write disable

	// status
	RDSR                             = 0x05, ///< read status register
	WRSR                             = 0x01, ///< write status register

	// misc.
	RDJDID                           = 0x9F  ///< read JEDEC ID
};

/**
 * Status register
 */
struct Is25LpStatus
{
	union
	{
		uint8_t byte;
		struct
		{
			uint16_t		WIP                   :1;  ///< write in progress
			uint16_t		WEL                   :1;  ///< write enable.
			uint16_t		BP                    :4;  ///< block protect bits (non-volatile)
			uint16_t		QE                    :1;  ///< quad enable
			uint16_t		SRWD                  :1;  ///< status register write disable (non-volatile)
		}__attribute__ ((packed))
		bits;
	};
	Is25LpStatus() {}
	Is25LpStatus(uint8_t val) { byte = val; }
    operator uint8_t() const
    {
        return byte;
    }

    uint8_t operator = (const uint8_t val)
	{
    	byte = val;
		return val;
	}

};

} // anonymous namespace


struct SampleIs25LpProps
{
	enum
	{
		SIZE_MB = 8,                  // 4, 8, 16 (MBytes)
		SHARE_PORT = false,
		DEBUG_LOG = true,
		NOPS_AFTER_SELECT = 1,
		NOPS_BEFORE_DESELECT = 4
	};
	using CS = Pin<'A', 1, 'L'>;
};

template<class props = SampleIs25LpProps>
class Is25LpCore
{
public:
	enum
	{
		SIZE_MB = props::SIZE_MB,              // size in MBytes
		SIZE = SIZE_MB * 1024 * 1024UL,        // size in bytes
		SECTOR_SIZE = 4096UL,                  // sector size (minimal erasable unit)
		SECTOR_MASK = ~(SECTOR_SIZE - 1),      // mask to get sector address from any address
		SECTOR_COUNT = SIZE / SECTOR_SIZE,     // sector count (2048)
		PAGE_SIZE = 256UL,                     // page size (maximum writable unit)
		PAGE_MASK = ~(PAGE_SIZE - 1),          // mask to get page address from any address
		PAGE_COUNT = SIZE / PAGE_SIZE          // 32768 (0x8000)
	};

	enum
	{
		SHARE_PORT = props::SHARE_PORT,
		DEBUG_LOG = props::DEBUG_LOG,
		NOPS_AFTER_SELECT = props::NOPS_AFTER_SELECT,
		NOPS_BEFORE_DESELECT = props::NOPS_BEFORE_DESELECT
	};

	Is25LpCore(STM32::SPI::SpiBase& spiref)
		: spi_(spiref)
		{ }

	bool Read(uint32_t addr, void *buf, uint32_t len);
	bool Write(uint32_t addr, void const *buf, uint32_t len);
	bool Verify(uint32_t addr, void const *buf, uint32_t len);
	bool IsSectorErased(uint32_t addr);
	bool IsPageErased(uint32_t addr);
	bool CheckErased(uint32_t addr, uint32_t count);
	bool EraseSector(uint32_t addr);
	bool FullErase();
	void ProtectSector(uint32_t addr);
	void UnprotectSector(uint32_t addr);
	void GlobalProtect();
	void GlobalUnprotect();
	Is25LpStatus ReadStatus();
	uint32_t ReadJedecId();
	uint32_t PageAddress(uint32_t addr) { return addr & PAGE_MASK; }
	uint32_t SectorAddress(uint32_t addr) { return addr & SECTOR_MASK; }
protected:
	virtual void DoSelect(void) = 0;
	virtual void DoDeselect(void) = 0;
private:
	STM32::SPI::SpiBase& spi_;

	void Lock(){ if (SHARE_PORT) spi_.Lock(); }
	void Unlock(){ if (SHARE_PORT) spi_.Unlock(); }
	void Select(void);
	void Deselect(void);
	bool Command(uint8_t cmd);
	bool CommandAndAddr(uint8_t cmd, uint32_t addr);
	bool WriteEnable();
	bool Wait(uint32_t ticks = 210);
	bool WriteStatus(uint8_t value);
};

template<class props>
class Is25LpChip : public Is25LpCore<props>
{
public:
	Is25LpChip(STM32::SPI::SpiBase& spiref)
		: Is25LpCore<props>(spiref)
		{
			CS::Direct(OUTPUT);
			CS::Off();
		}
protected:
	typedef class props::CS CS;
	void DoSelect(void) { CS::On(); };
	void DoDeselect(void) { CS::Off(); };
};

template<class props>
void Is25LpCore<props>::Select()
{
	DoSelect();
	for (int i = 0; i < NOPS_AFTER_SELECT; i++)
		__asm__ __volatile__ ("nop");
}

template<class props>
void Is25LpCore<props>::Deselect()
{
	for (int i = 0; i < NOPS_BEFORE_DESELECT; i++)
		__asm__ __volatile__ ("nop");
	DoDeselect();
}

template<class props>
bool Is25LpCore<props>::Command(uint8_t cmd)
{
	if (!Wait()) return false;
	Select();
	spi_.Rw(cmd);
	return true;
}

template<class props>
bool Is25LpCore<props>::CommandAndAddr(uint8_t cmd, uint32_t addr)
{
	if (!Wait()) return false;
	Select();
	spi_.Rw(cmd);
	spi_.Rw(addr >> 16);
	spi_.Rw(addr >> 8);
	spi_.Rw(addr);
	return true;
}

template<class props>
bool Is25LpCore<props>::Wait(uint32_t ticks)
{
	// Page write time        : typical: 1.4ms, max :    5ms
	// 4Kb  sector erase time : typical:  75ms, max :  200ms
	// 64Kb block erase time  : typical:  0.7s, max :    2s
	// full erase time        : typical:   50s, max :   80s
	for (uint32_t i = 0; i < ticks; i++)
	{
		if (!(ReadStatus().bits.WIP)) // write in progress?
			return true;
		OS::sleep(1);
	}

	if (DEBUG_LOG)
		uart << "\r\n at25 : wait failed!";
	return false;
}


template<class props>
bool Is25LpCore<props>::WriteEnable()
{
	bool ret = Command(WREN);
	Deselect();
	return ret;
}

template<class props>
void Is25LpCore<props>::GlobalProtect()
{
	WriteStatus(0x7F);  // do not write 1 to SRWD (because it can't be set back to 0 if WP pin not tied to VCC)
}

template<class props>
void Is25LpCore<props>::GlobalUnprotect()
{
	WriteStatus(0);
}

template<class props>
Is25LpStatus Is25LpCore<props>::ReadStatus()
{
	Is25LpStatus res;
	Select();
	spi_.Rw(RDSR);
	res = spi_.Rw();                  // first byte
	Deselect();
	return res;
}

template<class props>
uint32_t Is25LpCore<props>::ReadJedecId()
{
	uint32_t res = 0;
	Lock();
	if (Command(RDJDID))
	{
		res = spi_.Rw();
		res <<= 8;
		res |= spi_.Rw();
		res <<= 8;
		res |= spi_.Rw();
	}
	Deselect();
	Unlock();
	return res;
}

template<class props>
bool Is25LpCore<props>::WriteStatus(uint8_t value)
{
	Lock();
	bool ret = WriteEnable();
	if (ret)
	{
		Command(WRSR);
		spi_.Rw(value);
		Deselect();
	}
	Unlock();
	return ret;
}

template<class props>
bool Is25LpCore<props>::Read(uint32_t addr, void *buf, uint32_t len)
{
	Lock();

	bool ret = CommandAndAddr(FAST_READ, addr);
	if (ret)
	{
		spi_.Rw();  // dummy byte
		uint8_t *p = reinterpret_cast<uint8_t *>(buf);
		while (len--)
			*p++ = spi_.Rw();

		Deselect();
	}

	Unlock();
	return ret;
}

template<class props>
bool Is25LpCore<props>::Write(uint32_t addr, void const *buf, uint32_t len)
{
	Lock();

	bool ret = WriteEnable() && CommandAndAddr(PP, addr);
	if (ret)
	{
		// send data itself
		uint8_t const *p = reinterpret_cast<uint8_t const *>(buf);
		while (len--)
			spi_.Rw(*p++);
		// done
		Deselect();

		// wait for write operation finishes (max time : 6us)
		ret = Wait(7);
	}
	else
		Deselect();

	Unlock();
	return ret;
}

template<class props>
bool Is25LpCore<props>::Verify(uint32_t addr, void const *buf, uint32_t len)
{
	Lock();
	bool ret = CommandAndAddr(FAST_READ, addr);
	if (ret)
	{
		spi_.Rw();  // dummy byte
		uint8_t const *p = reinterpret_cast<uint8_t const *>(buf);
		while (len--)
			if (*p++ != spi_.Rw())
			{
				ret = false;
				break;
			}

		Deselect();
	}
	Unlock();
	return ret;
}


template<class props>
bool Is25LpCore<props>::CheckErased(uint32_t addr, uint32_t count)
{
	Lock();
	bool ret = CommandAndAddr(FAST_READ, addr);
	if (ret)
	{
		spi_.Rw();  // dummy byte

		for (uint32_t i = 0; i < count; i++)
			if (spi_.Rw() != 0xFF)
			{
				ret = false;
				break;
			}
	}
	Deselect();
	return ret;
}

template<class props>
bool Is25LpCore<props>::IsSectorErased(uint32_t addr)
{
	return CheckErased(addr, SECTOR_SIZE);
}

template<class props>
bool Is25LpCore<props>::IsPageErased(uint32_t addr)
{
	return CheckErased(addr, PAGE_SIZE);
}

template<class props>
bool Is25LpCore<props>::EraseSector(uint32_t addr)
{
	Lock();
	bool ret = WriteEnable() && CommandAndAddr(ERASE_SECTOR, addr);
	Deselect();

	// wait for 4K erase operation finishes (max time : 300us)
	ret = ret && Wait(310);
	Unlock();
	return ret;
}

template<class props>
bool Is25LpCore<props>::FullErase()
{
	Lock();
	bool ret = WriteEnable() && Command(ERASE_CHIP);
	Deselect();

	// wait for chip erase operation finishes (max time : 80s)
	ret = ret && Wait(82 * 1000);
	Unlock();
	return ret;
}


#endif // IS25LP_H_INCLUDED
