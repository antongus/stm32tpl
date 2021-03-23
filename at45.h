/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *  Visit https://github.com/antongus/stm32tpl for new versions
 *
 *  Copyright (c) 2011-2021 Anton B. Gusev
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
 *  file         : at45.h
 *  description  : AT45 dataflash interface.
 *
 */

#pragma once

#include "pin.h"
#include "stm32_spi.h"

/**
 * all AT45 opcodes
 */
enum At45Commands : uint8_t
{
	BLOCK_ERASE                      = 0x50, ///< erase 512 pages
	MAIN_MEMORY_PAGE_READ            = 0x52, ///< main memory page read
	MM_PAGE_TO_B1_XFER               = 0x53, ///< main memory page to buffer 1 transfer
	BUFFER_1_READ                    = 0x54, ///< Buffer 1 read
	MM_PAGE_TO_B2_XFER               = 0x55, ///< main memory page to buffer 2 transfer
	BUFFER_2_READ                    = 0x56, ///< buffer 2 read
	STATUS_READ                      = 0x57, ///< status register
	AUTO_PAGE_REWRITE_THROUGH_B1     = 0x58, ///< auto page rewrite through buffer 1
	AUTO_PAGE_REWRITE_THROUGH_B2     = 0x59, ///< auto page rewrite through buffer 2
	MM_PAGE_TO_B1_COMP               = 0x60, ///< main memory page compare to buffer 1
	MM_PAGE_TO_B2_COMP               = 0x61, ///< main memory page compare to buffer 2
	ARRAY_READ                       = 0x68, ///< start continuous array read
	PAGE_ERASE                       = 0x81, ///< erase a 264 byte page
	MM_PAGE_PROG_THROUGH_B1          = 0x82, ///< main memory page program through buffer 1
	B1_TO_MM_PAGE_PROG_WITH_ERASE    = 0x83, ///< buffer 1 to main memory page program with built-in erase
	BUFFER_1_WRITE                   = 0x84, ///< Buffer 1 write
	MM_PAGE_PROG_THROUGH_B2          = 0x85, ///< main memory page program through buffer 2
	B2_TO_MM_PAGE_PROG_WITH_ERASE    = 0x86, ///< buffer 2 to main memory page program with built-in erase
	BUFFER_2_WRITE                   = 0x87, ///< buffer 2 write
	B1_TO_MM_PAGE_PROG_WITHOUT_ERASE = 0x88, ///< buffer 1 to main memory page program without built-in erase
	B2_TO_MM_PAGE_PROG_WITHOUT_ERASE = 0x89, ///< buffer 2 to main memory page program without built-in erase
};

/**
 * all AT45 devices
 */
enum At45Variant
{
	AT45DB011B,
	AT45DB021B,
	AT45DB041B,
	AT45DB081B,
	AT45DB161B,
	AT45DB321B,
	AT45DB642B
};

/**
 * Chip traits
 */
template<At45Variant variant> struct At45Traits;

template<>
struct At45Traits<AT45DB161B>
{
	static constexpr uint32_t pageCount     {4096};
	static constexpr uint32_t pagesPerBlock    {8};
	static constexpr uint32_t pageSize       {528};
	static constexpr uint32_t pageBits        {10};
};

template<>
struct At45Traits<AT45DB642B>
{
	static constexpr uint32_t pageCount     {8192};
	static constexpr uint32_t pagesPerBlock    {8};
	static constexpr uint32_t pageSize      {1056};
	static constexpr uint32_t pageBits        {11};
};

/**
 * Sample At45 properties structure
 */
struct SampleAt45Props
{
	static constexpr At45Variant variant {AT45DB161B};  //!< chip model
	using CS = Pin<'A', 1, 'L'>;                        //!< chip select pin
	using RST = DummyPinOn;                             //!< reset pin (use DummyPinOn if WP not used)
	using WP = DummyPinOn;                              //! write protect pin (use DummyPinOn if WP not used)
	static constexpr auto lockSpi {false};              //!< set to true if SPI port is shared
	static constexpr auto nopsAfterSelect {1};          //!< how many nop's insert after select
	static constexpr auto nopsBeforeDeselect {4};       //!< how many nop's insert before de-select
};

template<typename Props>
class At45Chip
{
private:
	using Traits = At45Traits<Props::variant>;

public:
	static constexpr auto pageCount     {Traits::pageCount};
	static constexpr auto pagesPerBlock {Traits::pagesPerBlock};
	static constexpr auto pageSize      {Traits::pageSize};
	static constexpr auto pageBits      {Traits::pageBits};

	At45Chip(STM32::SPI::SpiBase& spiref)
		: m_spi(spiref)
		, m_mutex()
	{
		CS::Off();
		CS::Mode(OUTPUT);
		RST::Off();
		RST::Mode(OUTPUT);
		WP::On();
		WP::Mode(OUTPUT);
	}
	void lock()				{ m_mutex.lock(); }
	void unlock()			{ m_mutex.unlock(); }
	uint8_t status();
	void writeProtect(bool value);
	void reset();

	bool read(uint32_t page, uint16_t offset, void* data, uint32_t len);
	bool write(uint32_t page, uint16_t offset, const void* data, uint32_t len, int buf = 1);
	bool bufWrite(int buf, uint16_t offset, const void* data, uint32_t len);
	bool bufRead(int buf, uint16_t offset, void* data, uint32_t len);
	bool bufSave(int buf, uint32_t page);
	bool bufLoad(int buf, uint32_t page);
	bool erasePage(uint32_t page);
	bool eraseBlock(uint32_t block);
	bool fullErase();

private:
	using CS = typename Props::CS;
	using RST = typename Props::RST;
	using WP = typename Props::WP;
	static constexpr auto lockSpi {Props::lockSpi};
	static constexpr auto nopsAfterSelect {Props::nopsAfterSelect};
	static constexpr auto nopsBeforeDeselect {Props::nopsBeforeDeselect};

	STM32::SPI::SpiBase& m_spi;
	OS::TMutex m_mutex;
	void select();
	void deselect();
	void latch();
	bool waitReady();
	bool sendCommand(uint8_t command, uint32_t page, uint16_t offset = 0);
};


template<typename Props>
void At45Chip<Props>::select()
{
	if (lockSpi)
		m_spi.Lock();
	CS::On();
	for (auto i = 0; i < nopsAfterSelect; i++)
		__asm__ __volatile__ ("nop");
}

template<typename Props>
void At45Chip<Props>::deselect()
{
	for (auto i = 0; i < nopsBeforeDeselect; i++)
		__asm__ __volatile__ ("nop");
	CS::Off();
	if (lockSpi)
		m_spi.Unlock();
}

template<typename Props>
void At45Chip<Props>::latch()
{
	for (auto i = 0; i < nopsBeforeDeselect; i++)
		__asm__ __volatile__ ("nop");
	CS::Off();
	CS::On();
	for (auto i = 0; i < nopsAfterSelect; i++)
		__asm__ __volatile__ ("nop");
}

/// read chip status
template<typename Props>
uint8_t At45Chip<Props>::status()
{
	select();
	m_spi.Rw(STATUS_READ);
	auto status = m_spi.Rw(0xFF);
	deselect();
	return status;
}

/**
 * Wait for chip ready, then latch
 * @return true on success
 */
template<typename Props>
bool At45Chip<Props>::waitReady()
{
	select();
	for (auto i = 0; i < 500000; ++i)
	{
		m_spi.Rw(STATUS_READ);
		if ((m_spi.Rw(0xFF) & 0x80))
		{
			latch();
			return true;
		}
	}
	deselect();
	return false;
}

/**
 * Start operation: select, wait for chip ready, then latch
 * @return true on success
 */
template<typename Props>
bool At45Chip<Props>::sendCommand(uint8_t command, uint32_t page, uint16_t offset)
{
	if (!waitReady())
		return false;
	if (page >= pageCount)
		return false;
	m_spi.Rw(command);
	m_spi.Rw(page >> (16 - pageBits));
	m_spi.Rw(page << (pageBits - 8) | offset >> 8);
	m_spi.Rw(offset);
	return true;
}

template<typename Props>
void At45Chip<Props>::writeProtect(bool value)
{
	WP::On(value);
}

template<typename Props>
void At45Chip<Props>::reset()
{
	RST::On();
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	RST::Off();
}

/**
 * read data from memory
 * @param page - start page
 * @param offset - offset in page
 * @param data - pointer to buffer where data stored
 * @param len - data length
 * @return
 */
template<typename Props>
bool At45Chip<Props>::read(uint32_t page, uint16_t offset, void *data, uint32_t len)
{
	if (!sendCommand(ARRAY_READ, page, offset)) return false;

	// write 4 don't care bytes (32 bits)
	m_spi.Rw(0xFF);
	m_spi.Rw(0xFF);
	m_spi.Rw(0xFF);
	m_spi.Rw(0xFF);

	uint8_t *p = (uint8_t *)data;
	while (len--)
		*p++ = m_spi.Rw(0xFF);

	deselect();
	return true;
}

/**
 * Write through buffer.
 * data stored to buffer then flashed.
 * @param page - page to save
 * @param offset - offset in page
 * @param data - data to save
 * @param len data length
 * @param buf - buffer number (1/2)
 * @return true on success
 */
template<typename Props>
bool At45Chip<Props>::write(uint32_t page, uint16_t offset, const void *data, uint32_t len, int buf)
{
	auto cmd = buf == 2 ? MM_PAGE_PROG_THROUGH_B2 : MM_PAGE_PROG_THROUGH_B1;
	if (!sendCommand(cmd, page, offset)) return false;

	auto p = reinterpret_cast<const uint8_t *>(data);
	while (len--)
		m_spi.Rw(*p++);

	deselect();
	return true;
}

template<typename Props>
bool At45Chip<Props>::eraseBlock(uint32_t block)
{
	if (!sendCommand(BLOCK_ERASE, block)) return false;
	deselect();
	return true;
}

template<typename Props>
bool At45Chip<Props>::erasePage(uint32_t page)
{
	if (!sendCommand(PAGE_ERASE, page)) return false;
	deselect();
	return true;
}

template<typename Props>
bool At45Chip<Props>::fullErase()
{
	bool res;
	for (uint32_t i=0; i < pageCount / pagesPerBlock; i++)
	{
		if ((res = eraseBlock(i)))
			return res;
	}
	return true;
}


template<typename Props>
bool At45Chip<Props>::bufWrite(int buf, uint16_t offset, const void *data, uint32_t len)
{
	auto cmd = buf == 2 ? BUFFER_2_WRITE : BUFFER_1_WRITE;
	if (!sendCommand(cmd, 0, offset)) return false;

	auto p = reinterpret_cast<const uint8_t *>(data);
	while (len--)
		m_spi.Rw(*p++);

	deselect();
	return true;
}

template<typename Props>
bool At45Chip<Props>::bufRead(int buf, uint16_t offset, void *data, uint32_t len)
{
	auto cmd = buf == 2 ? BUFFER_2_READ : BUFFER_1_READ;
	if (!sendCommand(cmd, 0, offset)) return false;
	m_spi.Rw(0); // dummy byte

	auto p = reinterpret_cast<uint8_t *>(data);
	while (len--)
		*p++ = m_spi.Rw(0xFF);

	deselect();
	return true;
}

template<typename Props>
bool At45Chip<Props>::bufSave(int buf, uint32_t page)
{
	auto cmd = buf == 2 ? B2_TO_MM_PAGE_PROG_WITH_ERASE : B1_TO_MM_PAGE_PROG_WITH_ERASE;
	if (!sendCommand(cmd, page)) return false;
	deselect();
	return true;
}

/**
 * Load page content to buffer
 * @param buf  buffer number (1/2)
 * @param page page number
 * @return true on success
 */
template<typename Props>
bool At45Chip<Props>::bufLoad(int buf, uint32_t page)
{
	if (!sendCommand())
		return false;

	m_spi.Rw(buf == 2 ? MM_PAGE_TO_B2_XFER : MM_PAGE_TO_B1_XFER);
	m_spi.Rw(page >> (16 - pageBits));
	m_spi.Rw(page << (pageBits - 8));
	m_spi.Rw(0xFF);

	deselect();
	return true;
}
