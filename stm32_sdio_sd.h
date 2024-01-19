/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *  Visit https://github.com/antongus/stm32tpl for new versions
 *
 *  Copyright (c) 2011-2024 Anton B. Gusev
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
 *  file         : stm32_sdio_sd.h
 *  description  : STM32 SDIO-connected SD card (4-bit mode only)
 */

#pragma once

#include "stm32.h"
#include "pin.h"
#include "stm32_dma.h"
#include "sd_card_regs.h"
#include <scmRTOS.h>

namespace STM32
{
namespace SD
{

static constexpr auto SDIO_BUS_FREQ        {48'000'000UL};
// SDIO_CK frequency = SDIO_BUS_FREQ / [DIVISOR + 2].
static constexpr auto SDIO_DIVISOR_400KHZ  {SDIO_BUS_FREQ / 400'000 - 2};
static constexpr auto SDIO_DIVISOR_25MHZ   {0};


enum class SdioBusWidth : uint32_t
{
	OneBit    = 0,
	FourBits  = (1UL << 11),
	EightBit  = (2UL << 11),
	Mask      = (3UL << 11)
};

enum class CardBusWidth : uint32_t
{
	OneBit = 0,
	FourBits = 2
};

enum Command : uint32_t
{
	CMD0_GO_IDLE_STATE                = 0,
	CMD2_ALL_SEND_CID                 = 2,
	CMD3_SET_RELATIVE_ADDR            = 3,
	CMD4_SET_DSR                      = 4,
	CMD7_SELECT_DESELECT_CARD         = 7,
	CMD8_SEND_IF_COND                 = 8,
	CMD9_SEND_CSD                     = 9,
	CMD10_SEND_CID                    = 10,
	CMD11_READ_DAT_UNTIL_STOP         = 11,
	CMD12_STOP_TRANSMISSION           = 12,
	CMD13_SEND_STATUS                 = 13,
	CMD_HS_BUSTEST_READ               = 14,
	CMD_GO_INACTIVE_STATE             = 15,
	CMD16_SET_BLOCKLEN                = 16,
	CMD17_READ_SINGLE_BLOCK           = 17,
	CMD18_READ_MULT_BLOCK             = 18,
	CMD_HS_BUSTEST_WRITE              = 19,
	CMD_WRITE_DAT_UNTIL_STOP          = 20,
	CMD23_SET_BLOCK_COUNT             = 23,          // MMC only
	CMD24_WRITE_SINGLE_BLOCK          = 24,
	CMD25_WRITE_MULT_BLOCK            = 25,
	CMD_PROG_CID                      = 26,
	CMD_PROG_CSD                      = 27,
	CMD_SET_WRITE_PROT                = 28,
	CMD_CLR_WRITE_PROT                = 29,
	CMD_SEND_WRITE_PROT               = 30,
	CMD_SD_ERASE_GRP_START            = 32,
	CMD_SD_ERASE_GRP_END              = 33,
	CMD_ERASE_GRP_START               = 35,
	CMD_ERASE_GRP_END                 = 36,
	CMD_ERASE                         = 38,
	CMD_FAST_IO                       = 39,
	CMD_GO_IRQ_STATE                  = 40,
	CMD_LOCK_UNLOCK                   = 42,
	CMD55_APP_CMD                     = 55,
	CMD_GEN_CMD                       = 56,
	CMD_NO_CMD                        = 64,

	// SD-card specific commands (must be preceded by CMD55_APP_CMD command)
	ACMD6_SET_BUS_WIDTH               = 6,
	ACMD13_SD_STATUS                  = 13,
	ACMD22_SEND_NUM_WR_BLOCKS         = 22,
	ACMD23_SET_WR_BLK_ERASE_COUNT     = 23,
	ACMD41_SD_SEND_OP_COND            = 41,
	ACMD42_SET_CLR_CARD_DETECT        = 42,
	ACMD51_SEND_SCR                   = 51
};

enum CommandFlags
{
	CMD_NO_RESPONSE                   = 0,           ///< no response
	CMD_SHORT_RESPONSE                = 1UL << 6,    ///< short response
	CMD_LONG_RESPONSE                 = 3UL << 6,    ///< long response
	CMD_WAITINT                       = 1UL << 8,    ///< disable command timeout and wait for an interrupt request
	CMD_WAITPEND                      = 1UL << 9,    ///< wait for the end of data transfer before sending a command
	CMD_ENABLE_CPSM                   = 1UL << 10,   ///< command path state machine (CPSM) enable
	CMD_SDIOSuspend                   = 1UL << 11,   ///< SDIO suspend command
	CMD_ENCMDcompl                    = 1UL << 12,   ///< Enable CMD completion
	CMD_nIEN                          = 1UL << 13,   ///< not Interrupt Enable
	CMD_ATACMD                        = 1UL << 14,   ///< send CMD61
	CMD_COMMAND_MASK                  = 0x3F
};

enum class SdioError : uint32_t
{
	None                 = 0,
	WaitTimeout          = 1,
	CommandCTimeout      = 2,
	CrcFail              = 3,
	InvalidCommand       = 4,
	R1Error              = 5,
	NotSupported         = 6,
	BadVoltageRange      = 7,
	R6Error              = 8,
	CardLocked           = 9,
	DataRead             = 10,
	DmaTimeout           = 11,
	DmaTransfer          = 12,
	BusWidthUnsupported  = 13,
	UnalignedBuffer      = 14,
};

enum class CardState
{
	NoCard,
	Error,
	Ready
};

enum CardFlags
{
	cfVer20               = (1UL << 0),
	cfSDHC                = (1UL << 1)
};

enum R6Error
{
	SD_R6_GENERAL_UNKNOWN_ERROR     = 0x00002000UL,
	SD_R6_ILLEGAL_CMD               = 0x00004000UL,
	SD_R6_COM_CRC_FAILED            = 0x00008000UL
};

enum DataTransferDirection
{
	DATA_TRANSFER_WRITE      = 0,
	DATA_TRANSFER_READ       = SDIO_DCTRL_DTDIR
};

enum DataBlockSize
{
	DATA_BLOCK_1B      = 0 << 4,
	DATA_BLOCK_2B      = (1UL << 4),
	DATA_BLOCK_4B      = (2UL << 4),
	DATA_BLOCK_8B      = (3UL << 4),
	DATA_BLOCK_16B     = (4UL << 4),
	DATA_BLOCK_32B     = (5UL << 4),
	DATA_BLOCK_64B     = (6UL << 4),
	DATA_BLOCK_128B    = (7UL << 4),
	DATA_BLOCK_256B    = (8UL << 4),
	DATA_BLOCK_512B    = (9UL << 4),
	DATA_BLOCK_1024B   = (10UL << 4),
	DATA_BLOCK_2048B   = (11UL << 4),
	DATA_BLOCK_4096B   = (12UL << 4),
	DATA_BLOCK_8192B   = (13UL << 4),
	DATA_BLOCK_16384B  = (14UL << 4)
};

enum
{
	SD_VOLTAGE_WINDOW_SD            = 0x80100000UL,
	ACMD41_ARG_SDSC                 = (0UL << 30),
	ACMD41_ARG_SDHC                 = (1UL << 30),

	ACMD41_RESP_SDHC                = (1UL << 30), // card capacity status (0-SDSC, 1-SDHC)
	ACMD41_RESP_INIT_DONE           = (1UL << 31)
};

/**
 * Internal things
 */
namespace
{
/**
 * SDIO pins
 */
struct Pins
{
	using D0  = Pin<'C',  8, 'H', PIN_SPEED_25MHZ>;
	using D1  = Pin<'C',  9, 'H', PIN_SPEED_25MHZ>;
	using D2  = Pin<'C', 10, 'H', PIN_SPEED_25MHZ>;
	using D3  = Pin<'C', 11, 'H', PIN_SPEED_25MHZ>;
	using CK  = Pin<'C', 12, 'H', PIN_SPEED_25MHZ>;
	using CMD = Pin<'D',  2, 'H', PIN_SPEED_25MHZ>;

	static void init()
	{
		D0::Alternate(ALT_FUNC_SDIO);
		D1::Alternate(ALT_FUNC_SDIO);
		D2::Alternate(ALT_FUNC_SDIO);
		D3::Alternate(ALT_FUNC_SDIO);
		CMD::Alternate(ALT_FUNC_SDIO);
		CK::Alternate(ALT_FUNC_SDIO);

		D0::Mode(ALT_OUTPUT);
		D1::Mode(ALT_OUTPUT);
		D2::Mode(ALT_OUTPUT);
		D3::Mode(ALT_OUTPUT);
		CMD::Mode(ALT_OUTPUT);
		CK::Mode(ALT_OUTPUT);
	}
};

enum class DmaDirection : uint32_t
{
	Receive = DMA::DMA_CR_DIR_PERITH_TO_MEM,
	Transmit = DMA::DMA_CR_DIR_MEM_TO_PERITH
};

} // anonymous namespace

enum SdioResult
{
	srOk,
	srDCrcFail,
	srDTimeout,
	srRxOverr,
	srTxUnderr,
	srStBitErr
};


/**
 * Sample Props for SdioSdCard class template.
 * User should make a copy of this structure in his project,
 * and adopt it for project requirements.
 */
struct SampleSdioSdCardProps
{
	// interrupt priority settings for interrupts used
	static constexpr auto sdioInterruptPrioGroup {1};
	static constexpr auto sdioInterruptSubprio {1};
	static constexpr auto dmaInterruptPrioGroup {2};
	static constexpr auto dmaInterruptSubprio {2};

	static constexpr auto commandTimeout {20'000'000u}; //!< counter for command completion loop (~2 sec)

	static void powerOn(){}    //!< power on sd card
	static void powerOff() {}  //!< power off sd card
	static bool isPowerOn() { return true; } //!< report power state

	static constexpr auto debugTrace {true};

	template <typename ... Args>
	static void trace(const char* , const Args& ... )
	{
	}
};

template<class Props /* = SampleSdioSdCardProps*/>
class SdioSdCard
{
public:
	static bool init();
	static void cyclePower();
	static void report();
	static CardState getState() { return m_state; };
	static SdioError readBuffer(void *buf, uint32_t sector, uint32_t count);
	static SdioError writeBuffer(const void *buf, uint32_t sector, uint32_t count);

	static uint32_t  getSectorCount() { return m_csd.getSectorCount(); }
	static uint32_t  getBlockSize()   { return m_csd.getBlockLen(); }
	static uint32_t  getSize()        { return m_csd.getSize(); }

	static SdioError readCardStatus(CardStatus& status);

	static void lock() { m_mutex.lock(); }
	static void unlock() { m_mutex.unlock(); }

	// Interrupt handlers
	static INLINE void sdioInterruptHandler();
	static INLINE void dmaInterruptHandler();

private:
	using DmaStream = DMA::Dma2Channel3;
	//using DmaStream = DMA::Dma2Stream6;  -- alternate stream
	static constexpr auto DMA_IRQn {DMA2_Stream3_IRQn};
	static constexpr auto SDIO_DMA_CHANNEL {DMA::DMA_CR_CHSEL_CH4};

	static inline CardState m_state {CardState::NoCard};

	static inline uint32_t m_cardFlags {0};
	static inline OS::TMutex m_mutex;
	static inline OS::TEventFlag m_dmaFlag;
	static inline OS::TEventFlag m_sdioFlag;
	static inline CIDRegister m_cid;
	static inline CSDRegister m_csd;
	static inline uint32_t m_rca;
	static inline volatile SdioResult m_sdioResult;

	static bool isV20() { return m_cardFlags & cfVer20; }
	static bool isSDHC() { return m_cardFlags & cfSDHC; }
	static void disableDataTransfer() { SDIO->DCTRL = 0; }
	static void setSdioBusWidth(SdioBusWidth width);
	static void setSdioDivisor(uint32_t divisor) { SDIO->CLKCR = (SDIO->CLKCR & ~SDIO_CLKCR_CLKDIV) | divisor; }

	static SdioError command(uint32_t command, uint32_t arg = 0);

	static SdioError waitReady();
	static SdioError identifyCard();
	static SdioError initializeCard();
	static SdioError selectDeselectCard();
	static SdioError switchTo4bitBus();
	static SdioError setBlockLen(uint32_t len);
	static SdioError stopTransmission();
	static SdioError setCardBusWidth(CardBusWidth width);
	static SdioError readSCR(SCRRegister *scr);

	static void enableSdioInterrupts();
	static void disableSdioInterrupts() { SDIO->MASK = 0; }
	static void enableDataTransfer(DataTransferDirection dir, uint32_t len, DataBlockSize bSize);
	static void startDMA(void *buf, DmaDirection direction);
	static void stopDMA();

	// call the external std::fmt-like debug print function, specified in Props.
	// (can be fully disabled by specifying Props::debugTrace {false})
	template <typename ... Args>
	static void trace(const char* format, const Args& ... args)
	{
		(void)format;
		if constexpr (Props::debugTrace)
			Props::trace(format, args...);
	}
};

static bool failed(SdioError err) { return err != SdioError::None;  }
static bool success(SdioError err) { return err == SdioError::None; }

static const char* desc(SdioError err) {
	switch (err)
	{
	case SdioError::None : return "None";
	case SdioError::WaitTimeout : return "WaitTimeout";
	case SdioError::CommandCTimeout : return "CommandCTimeout";
	case SdioError::CrcFail : return "CrcFail";
	case SdioError::InvalidCommand : return "InvalidCommand";
	case SdioError::R1Error : return "R1Error";
	case SdioError::NotSupported : return "NotSupported";
	case SdioError::BadVoltageRange : return "BadVoltageRange";
	case SdioError::R6Error : return "R6Error";
	case SdioError::CardLocked : return "CardLocked";
	case SdioError::DataRead : return "DataRead";
	case SdioError::DmaTimeout : return "DmaTimeout";
	case SdioError::DmaTransfer : return "DmaTransfer";
	case SdioError::BusWidthUnsupported : return "BusWidthUnsupported";
	case SdioError::UnalignedBuffer : return "UnalignedBuffer";
	}
	return "?";
}

/**
 * Send command, wait for response and check response
 */
template<class Props>
SdioError SdioSdCard<Props>::command(uint32_t command, uint32_t arg)
{
	// clear command-related flags
	SDIO->ICR = SDIO_ICR_CCRCFAILC | SDIO_ICR_CTIMEOUTC | SDIO_ICR_CMDRENDC | SDIO_ICR_CMDSENTC;
	SDIO->ARG = arg;
	SDIO->CMD = command | CMD_ENABLE_CPSM;

	auto waitStatus = [](uint32_t mask) -> uint32_t {
		auto flags {0u};
		for (auto i = 0u; i < Props::commandTimeout; ++i)
		{
			if (flags = SDIO->STA & mask; flags != 0)
				break;
		}
		SDIO->ICR = 0x5FF;  // clear flags
		return flags;
	};

	auto responseFlags = command & SDIO_CMD_WAITRESP;
	if (!responseFlags) // command without response
	{
		// wait for CMDSENT or CTIMEOUT
		auto status = waitStatus(SDIO_STA_CTIMEOUT | SDIO_STA_CMDSENT);
		if (!status)
		{
			trace("\r\nSD cmd({}): no confirmation", command);
			return SdioError::WaitTimeout;
		}
		return status & SDIO_STA_CMDSENT ? SdioError::None : SdioError::CommandCTimeout;
	}

	// command with response: wait for CMDREND or CTIMEOUT or CCRCFAIL
	command &= CMD_COMMAND_MASK;  // strip flags
	auto status = waitStatus(SDIO_STA_CMDREND | SDIO_STA_CTIMEOUT | SDIO_STA_CCRCFAIL);
	if (!status)
	{
		trace("\r\nSD cmd({}): no response", command);
		return SdioError::WaitTimeout;
	}

	if (status & SDIO_STA_CTIMEOUT)
	{
		trace("\r\nSD cmd({}): command timeout", command);
		return SdioError::CommandCTimeout;
	}

	// crc failure is ok for ACMD41 (R3 response)
	if (status & SDIO_STA_CCRCFAIL && command != ACMD41_SD_SEND_OP_COND)
	{
		trace("\r\nSD cmd({}): crc failed", command);
		return SdioError::CrcFail;
	}

	if (responseFlags == CMD_LONG_RESPONSE || command == ACMD41_SD_SEND_OP_COND)  // no additional checks needed
		return SdioError::None;

	if (SDIO->RESPCMD != command)
	{
		trace("\r\nSD cmd({}): bad RESPCMD", command);
		return SdioError::InvalidCommand;
	}

	if (command == CMD8_SEND_IF_COND) // R7 response
	{
		if ((SDIO->RESP1 & 0xFF) != 0xAA)
		{
			trace("\r\nSD cmd({}): bad response pattern", command);
			return SdioError::InvalidCommand;
		}
		return SdioError::None;
	}

	if (command == CMD3_SET_RELATIVE_ADDR)
	{
		auto respR6 {SDIO->RESP1};
		if (respR6 & (SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_ILLEGAL_CMD | SD_R6_COM_CRC_FAILED))
		{
			trace("\r\nSD cmd({}): R6 error", command);
			return SdioError::R6Error;
		}
		m_rca = respR6 & 0xFFFF0000;
		return SdioError::None;
	}

	// all other commands: R1 response
	if (SDIO->RESP1 & CardStatus::CS_ERROR_MASK)
	{
		trace("\r\nSD cmd({}): error flags ({:0x}) set", command, SDIO->RESP1 & CardStatus::CS_ERROR_MASK);
		return SdioError::R1Error;
	}
	return SdioError::None;
}

template<class Props>
SdioError SdioSdCard<Props>::identifyCard()
{
	m_cardFlags = 0;

	auto goIdle = [&](auto attempts) {
		auto err {SdioError::None};
		for (auto i = 0; i < attempts; ++i)
		{
			if (err = command(CMD0_GO_IDLE_STATE); success(err))
				break;
		}
		return err;
	};

	// issue CMD0 GO_IDLE_STATE command (software reset)
	if (auto err = goIdle(20); failed(err))
	{
		return err;
	}

	// try to verify operating conditions (voltage) with new (v 2.0) CMD8 command.
	 // 0x100 (3.3v) + AA (pattern)
	if (auto err = command(CMD8_SEND_IF_COND | CMD_SHORT_RESPONSE, 0x1AA); success(err))
	{
		m_cardFlags |= cfVer20;
	}
	else if (err = command(CMD0_GO_IDLE_STATE); failed(err))
	{
		trace("\r\nSD: unsupported card");
		return err;
	}

	for (auto tries = 0; tries < 10000; ++tries)  // about 2 second
	{
		if (auto err = command(CMD55_APP_CMD | CMD_SHORT_RESPONSE); failed(err))
			break;

		auto arg = SD_VOLTAGE_WINDOW_SD | (isV20() ? ACMD41_ARG_SDHC : ACMD41_ARG_SDSC);
		if (auto err = command(ACMD41_SD_SEND_OP_COND | CMD_SHORT_RESPONSE, arg); failed(err))
			break;

		if (auto r1 = SDIO->RESP1; r1 & ACMD41_RESP_INIT_DONE)
		{
			if (r1 & ACMD41_RESP_SDHC)
			{
				m_cardFlags |= cfSDHC;
			}
			return SdioError::None;
		}
	}
	trace("\r\nSD: error setting voltage range");
	return SdioError::BadVoltageRange;
}

template<class Props>
SdioError SdioSdCard<Props>::initializeCard()
{
	if (auto err = command(CMD2_ALL_SEND_CID | CMD_LONG_RESPONSE); failed(err))
		return err;

	// got CID
	uint32_t arr[4];
	arr[0] = SDIO->RESP4;
	arr[1] = SDIO->RESP3;
	arr[2] = SDIO->RESP2;
	arr[3] = SDIO->RESP1;

	m_cid.fill(arr);

	if (auto err = command(CMD3_SET_RELATIVE_ADDR | CMD_SHORT_RESPONSE); failed(err))
		return err;

	if (auto err = command(CMD9_SEND_CSD | CMD_LONG_RESPONSE, m_rca); failed(err))
		return err;

	// got CSD
	arr[0] = SDIO->RESP4;
	arr[1] = SDIO->RESP3;
	arr[2] = SDIO->RESP2;
	arr[3] = SDIO->RESP1;

	m_csd.fill(arr);

	return SdioError::None;
}

template<class Props>
SdioError SdioSdCard<Props>::selectDeselectCard()
{
	if (auto err = command(CMD7_SELECT_DESELECT_CARD | CMD_SHORT_RESPONSE, m_rca); failed(err))
		return err;

	CardStatus cs(SDIO->RESP1);
	if (cs.bits.CARD_IS_LOCKED)
		return SdioError::CardLocked;
	return SdioError::None;
}

template<class Props>
SdioError SdioSdCard<Props>::readCardStatus(CardStatus& status)
{
	if (auto err = command(CMD13_SEND_STATUS | CMD_SHORT_RESPONSE, m_rca); failed(err))
		return err;

	status = SDIO->RESP1;
	return SdioError::None;
}

template<class Props>
SdioError SdioSdCard<Props>::setBlockLen(uint32_t len)
{
	return command(CMD16_SET_BLOCKLEN | CMD_SHORT_RESPONSE, len);
}

template<class Props>
SdioError SdioSdCard<Props>::stopTransmission()
{
	return command(CMD12_STOP_TRANSMISSION | CMD_SHORT_RESPONSE);
}

template<class Props>
void SdioSdCard<Props>::setSdioBusWidth(SdioBusWidth width)
{
	SDIO->CLKCR = (SDIO->CLKCR & ~static_cast<uint32_t>(SdioBusWidth::Mask)) | static_cast<uint32_t>(width);
}

template<class Props>
void SdioSdCard<Props>::enableDataTransfer(DataTransferDirection dir, uint32_t len, DataBlockSize bSize)
{
	// prepare to send/receive data
	SDIO->DTIMER = 0xFFFFFFFF;
	SDIO->DLEN = len;
	const uint32_t clearMask = ~(SDIO_DCTRL_DBLOCKSIZE | SDIO_DCTRL_DTEN | SDIO_DCTRL_DTDIR | SDIO_DCTRL_DTMODE);
	SDIO->DCTRL = (SDIO->DCTRL & clearMask)
			| SDIO_DCTRL_DTEN  // enable
			| dir              // direction
			| bSize            // block size
			| SDIO_DCTRL_DMAEN // enable DMA
			;
}

template<class Props>
SdioError SdioSdCard<Props>::readSCR(SCRRegister *scr)
{
	if (auto err = setBlockLen(8); failed(err))
		return err;

	if (auto err = command(CMD55_APP_CMD | CMD_SHORT_RESPONSE, m_rca); failed(err))
		return err;

	enableDataTransfer(DATA_TRANSFER_READ, 9, DATA_BLOCK_8B);

	if (auto err = command(ACMD51_SEND_SCR | CMD_SHORT_RESPONSE); failed(err))
		return err;

	auto index = 1;
	while (!(SDIO->STA & (SDIO_STA_RXOVERR | SDIO_STA_DCRCFAIL | SDIO_STA_DTIMEOUT | SDIO_STA_DBCKEND | SDIO_STA_STBITERR)))
	{
		if (SDIO->STA & SDIO_STA_RXDAVL)
		{
			if (index >= 0)
				scr->words[index--] = __builtin_bswap32(SDIO->FIFO);
		}
	}

	if (SDIO->STA & SDIO_STA_DTIMEOUT)
		return SdioError::DataRead;

	if (SDIO->STA & SDIO_STA_DCRCFAIL)
		return SdioError::DataRead;

	if (SDIO->STA & SDIO_STA_RXOVERR)
		return SdioError::DataRead;

	if (SDIO->STA & SDIO_STA_STBITERR)
		return SdioError::DataRead;

	return SdioError::None;
}

template<class Props>
SdioError SdioSdCard<Props>::setCardBusWidth(CardBusWidth width)
{
	if (auto err = command(CMD55_APP_CMD | CMD_SHORT_RESPONSE, m_rca); failed(err))
		return err;

	if (auto err = command(ACMD6_SET_BUS_WIDTH | CMD_SHORT_RESPONSE, static_cast<uint32_t>(width)); failed(err))
		return err;

	return SdioError::None;
}

template<class Props>
SdioError SdioSdCard<Props>::switchTo4bitBus()
{
	// This block is unnecessary because of every SD card must support 4-bit bus.
#if 0
	// read SCR to check if the card supports wide bus
	SCRRegister scr;
	if ((err = readSCR(&scr)))
		return err;

	// if not - bail out
	if (!(scr.bits.SD_BUS_WIDTHS & scr.SD_BUS_WIDTH_4BIT))
		return ERROR_WIDE_BUS_UNSUPPORTED;
#endif

	// switch card to 4-bit mode
	if (auto err = setCardBusWidth(CardBusWidth::FourBits); failed(err))
		return err;

	// switch SDIO interface to 4-bit mode
	setSdioBusWidth(SdioBusWidth::FourBits);
	return SdioError::None;
}

template<class Props>
void SdioSdCard<Props>::report()
{

	if (m_state != CardState::Ready)
	{
		trace ("\r\n no card detected");
		return;
	}

	trace ("\r\n {}card detected", isSDHC() ? "SDHC " : "");
	// CID data:
	trace ("\r\n Manufacturer : {:02x}", m_cid.manufacturerId);
	trace ("\r\n OEM_ID       : {:04x}", m_cid.oemId);
	trace ("\r\n Product Name : {}", m_cid.productName);
	trace ("\r\n OEM_ID       : {:x}", m_cid.serial);

	// CSD data:
	trace ("\r\n SD version   : {}", (isV20() ? "v2.0" : "1.0"));
	trace ("\r\n CSD version  : {}", (m_csd.isV20() ? "2.0" : "1.0"));
	trace ("\r\n blockLen     : {}", getBlockSize());
	trace ("\r\n sector count : {}", getSectorCount());
	trace ("\r\n size         : {} KB", getSize());
}

template<class Props>
void SdioSdCard<Props>::cyclePower()
{
	if (Props::isPowerOn())
	{
		Props::powerOff();
	}
	Props::powerOn();
}

template<class Props>
bool SdioSdCard<Props>::init()
{
	if (!Props::isPowerOn())
	{
		Props::powerOn();
	}

	// reset SDIO
	RCC->APB2RSTR |= RCC_APB2RSTR_SDIORST; __DSB();
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SDIORST; __DSB();

	Pins::init();

	RCC->APB2ENR |= RCC_APB2ENR_SDIOEN; __DSB();
	DmaStream::EnableClocks();

	// initialize SDIO hardware
	setSdioDivisor(SDIO_DIVISOR_400KHZ);
	SDIO->CLKCR |= SDIO_CLKCR_CLKEN;   // enable clock
	SDIO->POWER = SDIO_POWER_PWRCTRL;  // power on

	// Enable SDIO IRQ
	NVIC_SetPriority(SDIO_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
			Props::sdioInterruptPrioGroup, Props::sdioInterruptSubprio));
	NVIC_EnableIRQ(SDIO_IRQn);

	// Enable DMA IRQ
	NVIC_SetPriority(DMA_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
			Props::dmaInterruptPrioGroup, Props::dmaInterruptSubprio));
	NVIC_EnableIRQ(DMA_IRQn);

	m_state = CardState::Error;

	// identify card
	if (auto err = identifyCard(); failed(err))
	{
		trace("\r\nSD: identifyCard() error: {}", desc(err));
		return false;
	}

	if (auto err = initializeCard(); failed(err))
	{
		trace("\r\nSD: initializeCard() error: {}", desc(err));
		return false;
	}
	setSdioDivisor(SDIO_DIVISOR_25MHZ);

	// try to select card
	if (auto err = selectDeselectCard(); failed(err))
	{
		trace("\r\nSD: selectDeselectCard() error: {}", desc(err));
		return false;
	}

	// switch to wide bus
	if (auto err = switchTo4bitBus(); failed(err))
	{
		trace("\r\nSD: switchTo4bitBus() error: {}", desc(err));
		return false;
	}

	if (!isSDHC())
	{
		if (auto err = setBlockLen(512); failed(err))
		{
			trace("\r\nSD: error in setBlockLen(): {}", desc(err));
			return false;
		}
	}

	m_state = CardState::Ready;
	return true;
}

template<class Props>
void SdioSdCard<Props>::startDMA(void *buf, DmaDirection direction)
{
	// config for word-aligned buffer
//	const uint32_t dmaConfigWords = 0
//			| DMA::DMA_CR_MINC               // memory increment mode
//			| DMA::DMA_CR_MSIZE_32_BIT       // memory size
//			| DMA::DMA_CR_PSIZE_32_BIT       // peripheral size
//			| DMA::DMA_CR_PRIO_HIGHEST       // priority
//			| DMA::DMA_CR_TCIE               // transfer-complete interrupt enable
//			| DMA::DMA_CR_PFCTRL             // flow controller is peripheral
//			| DMA::DMA_CR_MBURST_INCR4       //
//			| DMA::DMA_CR_PBURST_INCR4       //
//			| SDIO_DMA_CHANNEL               // select channel
//			;

	// config for byte-aligned buffer
	const uint32_t dmaConfigBytes = 0
			| DMA::DMA_CR_MINC               // memory increment mode
			| DMA::DMA_CR_MSIZE_8_BIT        // memory size
			| DMA::DMA_CR_PSIZE_32_BIT       // peripheral size
			| DMA::DMA_CR_PRIO_HIGHEST       // priority
			| DMA::DMA_CR_TCIE               // transfer-complete interrupt enable
			| DMA::DMA_CR_PFCTRL             // flow controller is peripheral
			| DMA::DMA_CR_MBURST_INCR16      //
			| DMA::DMA_CR_PBURST_INCR4       //
			| SDIO_DMA_CHANNEL               // select channel
			;

	DmaStream::IFCR = DmaStream::DMA_MASK_ALL;

	DmaStream::Stream->FCR |= DMA_SxFCR_DMDIS | DMA_SxFCR_FTH;  // enable FIFO
	DmaStream::Stream->PAR = (uint32_t)&SDIO->FIFO ;
	DmaStream::Stream->MAR = (uint32_t)buf;
	DmaStream::Stream->NDTR = 0;             // count is controlled by peripheral (DMA_CR_PFCTRL)

//	DmaStream::Stream->CR = direction | (((uintptr_t)buf & 0x3) ? dmaConfigBytes : dmaConfigWords);
	DmaStream::Stream->CR = static_cast<uint32_t>(direction) | dmaConfigBytes;

	DmaStream::Enable();
}

template<class Props>
void SdioSdCard<Props>::stopDMA()
{
	// not needed, SDIO disables DMA by itself.
//	DmaStream::Disable();
}

template<class Props>
void SdioSdCard<Props>::enableSdioInterrupts()
{
	SDIO->MASK = 0
			| SDIO_MASK_DCRCFAILIE    // data CRC fail
			| SDIO_MASK_DTIMEOUTIE    // data timeout
			| SDIO_MASK_TXUNDERRIE    // TX underrun
			| SDIO_MASK_RXOVERRIE     // RX overrun
			| SDIO_MASK_STBITERRIE    // start bit error
			| SDIO_MASK_DATAENDIE     // data end
			;
}

template<class Props>
SdioError SdioSdCard<Props>::waitReady()
{
	for (auto timeout = 0; timeout < 0x5555; timeout++)
	{
		CardStatus status;
		if (auto err = readCardStatus(status); failed(err))
			return err;

		if (status.bits.READY_FOR_DATA)
			break;
	}
	return SdioError::None;
}

template<class Props>
SdioError SdioSdCard<Props>::readBuffer(void *buf, uint32_t sector, uint32_t count)
{
	// Disable data path
	disableDataTransfer();

	// wait for card ready
	if (auto err = waitReady(); failed(err))
		return err;

	m_dmaFlag.clear();
	m_sdioFlag.clear();

	// convert to byte address for non-SDHC card
	if (!isSDHC())
		sector *= 512;

	// Enable data path
	enableDataTransfer(DATA_TRANSFER_READ, 512 * count, DATA_BLOCK_512B);
	// enable SDIO interrupts
	enableSdioInterrupts();
	// start DMA
	startDMA(buf, DmaDirection::Receive);
	// send read command
	if (count == 1)
	{
		if (auto err = command(CMD17_READ_SINGLE_BLOCK | CMD_SHORT_RESPONSE, sector); failed(err))
			return err;
	}
	else
	{
		if (auto err = command(CMD18_READ_MULT_BLOCK | CMD_SHORT_RESPONSE, sector); failed(err))
			return err;
	}

	auto ret = m_dmaFlag.wait(500 + count * 100);
	stopDMA();
	if (!ret)
		return SdioError::DmaTimeout;

	ret = m_sdioFlag.wait(500);
	if (!ret)
		return SdioError::DmaTimeout;

	if (m_sdioResult != srOk)
		return SdioError::DmaTransfer;

	if (count > 1)
		stopTransmission();

	return SdioError::None;
}

template<class Props>
SdioError SdioSdCard<Props>::writeBuffer(const void *buf, uint32_t sector, uint32_t count)
{
	// wait for card ready
	if (auto err = waitReady(); failed(err))
		return err;

	m_dmaFlag.clear();
	m_sdioFlag.clear();

	// convert to byte address for non-SDHC card
	if (!isSDHC())
		sector *= 512;

	// send write command
	if (count == 1)
	{
		if (auto err = command(CMD24_WRITE_SINGLE_BLOCK | CMD_SHORT_RESPONSE, sector); failed(err))
			return err;
	}
	else
	{
		if (auto err = command(CMD25_WRITE_MULT_BLOCK | CMD_SHORT_RESPONSE, sector); failed(err))
			return err;
	}
	// Enable data path
	enableDataTransfer(DATA_TRANSFER_WRITE, 512 * count, DATA_BLOCK_512B);
	// enable SDIO interrupts
	enableSdioInterrupts();
	// start DMA
	startDMA(const_cast<void *>(buf), DmaDirection::Transmit);
	auto ret = m_dmaFlag.wait(500 + count * 100);
	stopDMA();
	if (!ret)
		return SdioError::DmaTimeout;

	ret = m_sdioFlag.wait(500);
	if (!ret)
		return SdioError::DmaTimeout;

	if (m_sdioResult != srOk)
		return SdioError::DmaTransfer;

	if (count > 1)
		stopTransmission();

	return SdioError::None;
}

template<class Props>
void SdioSdCard<Props>::sdioInterruptHandler()
{
	OS::TISRW ISR;

	uint32_t status = SDIO->STA;

	if (status & SDIO_STA_DATAEND)
		m_sdioResult = srOk;
	else if (status & SDIO_STA_DCRCFAIL)
		m_sdioResult = srDCrcFail;
	else if (status & SDIO_STA_STBITERR)
		m_sdioResult = srStBitErr;
	else if (status & SDIO_STA_DTIMEOUT)
		m_sdioResult = srDTimeout;
	else if (status & SDIO_STA_RXOVERR)
		m_sdioResult = srRxOverr;
	else if (status & SDIO_STA_TXUNDERR)
		m_sdioResult = srTxUnderr;

	// clear flags
	SDIO->ICR = 0
			| SDIO_ICR_DATAENDC
			| SDIO_ICR_DCRCFAILC
			| SDIO_ICR_STBITERRC
			| SDIO_ICR_DTIMEOUTC
			| SDIO_ICR_RXOVERRC
			| SDIO_ICR_TXUNDERRC
			;
	// disable interrupts
	disableSdioInterrupts();

	m_sdioFlag.signal_isr();
}

template<class Props>
void SdioSdCard<Props>::dmaInterruptHandler()
{
	if (DmaStream::ISR & DmaStream::DMA_MASK_TCIF)
	{
		OS::TISRW ISR;
		// clear interrupt flags
		DmaStream::IFCR = DmaStream::DMA_MASK_ALL;
		// flag to upper level
		m_dmaFlag.signal_isr();
	}
}


} // namespace SD
} // namespace STM32
