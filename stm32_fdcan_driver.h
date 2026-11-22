/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *  Visit https://github.com/antongus/stm32tpl for new versions
 *
 *  Copyright (c) 2026 Anton B. Gusev
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
 *  file         : stm32_fdcan_driver.h
 *  description  : STM32G4 FDCAN module driver, classic CAN 2.0B frames only
 *                 (CCCR.FDOE = 0, BRS off). Standard and extended identifiers.
 *
 *  Unlike bxCAN (stm32_can.h), FDCAN keeps Tx buffers, Rx FIFOs and
 *  acceptance filters in a separate message RAM instead of registers.
 *  On STM32G4 the message RAM layout is fixed by hardware (RM0440 §44.3.6,
 *  "It is not necessary to configure each of the sections"):
 *
 *    offset(byte)  size(word)  content
 *         0x0000         28    standard ID filter list (28 x 1 word)
 *         0x0070         16    extended ID filter list (8 x 2 words)
 *         0x00B0         54    Rx FIFO 0 (3 x 18 words)
 *         0x0188         54    Rx FIFO 1 (3 x 18 words)
 *         0x0260          6    Tx event FIFO (3 x 2 words)
 *         0x0278         54    Tx FIFO (3 x 18 words)
 *
 *  Total 212 words (0x350 bytes) per instance; FDCAN2 message RAM starts at
 *  SRAMCAN_BASE + 0x350 (RM0440: "start address of FDCANn = end address of
 *  FDCANn-1 + 4"). An element still occupies 18 words, only the first 4 are
 *  used by classic frames.
 *
 *  Access to configuration registers (NBTP, RXGFC, XIDAM, CCCR.TEST/MON/DAR)
 *  requires INIT = 1 and CCE = 1 simultaneously (RM0440 §44.3.4). CCE is
 *  cleared by hardware automatically when INIT is cleared.
 */

#pragma once

#include "stm32.h"
#include "pin.h"
#include <cstddef>

#if !defined(STM32TPL_STM32G4XX)
#  error "stm32_fdcan_driver.h: this module supports STM32G4 family only (FDCAN)"
#endif

namespace STM32
{

namespace FDCANx
{

/**
 * FDCAN peripheral number
 */
enum FdCanNum
{
	FDCAN_1,
#if defined(FDCAN2)
	FDCAN_2,
#endif
#if defined(FDCAN3)
	FDCAN_3,
#endif
};

/**
 * FDCAN kernel clock source (RCC CCIPR.FDCANSEL). Common for all instances.
 */
enum ClockSource
{
	CLK_SRC_HSE  = 0,   //!< HSE clock selected as FDCAN kernel clock
	CLK_SRC_PLLQ = 1,   //!< PLL "Q" clock selected as FDCAN kernel clock
	CLK_SRC_PCLK = 2,   //!< PCLK clock selected as FDCAN kernel clock
};

/**
 * Filter element type (RM0440 Table 417/421, SFT/EFT[1:0])
 */
enum FilterType
{
	ftRange,         //!< range filter: id1 <= rx_id <= id2
	ftDualId,        //!< match id1 or id2
	ftMask,          //!< classic filter: id1 = filter pattern, id2 = mask
};

/**
 * Filter element action on match (RM0440 Table 418/422, SFEC/EFEC[2:0])
 */
enum FilterAction
{
	faDisable = 0,   //!< filter element disabled
	faStoreFifo0,    //!< store in Rx FIFO 0 if filter matches
	faStoreFifo1,    //!< store in Rx FIFO 1 if filter matches
	faReject,        //!< reject message if filter matches
};

/**
 * Operating mode (call SetMode() in init mode, RM0440 §44.3.4:
 * TEST/MON/DAR bits can only be set while INIT and CCE are both set)
 */
enum Mode
{
	modeNormal,             //!< normal operation
	modeLoopbackInternal,   //!< internal loopback: Tx connected to Rx, bus output stays recessive
	modeLoopbackExternal,   //!< external loopback: frames go to the bus and are received back (via transceiver);
	                        //!< auto retransmission is disabled (DAR), so a frame is sent once even without ACK
	modeSilent,             //!< bus monitor: receive only, never drives the bus dominant, never ACKs
};

/**
 * CAN message (classic frame, up to 8 data bytes)
 */
struct Message
{
	uint32_t id;           //!< 11-bit (std) or 29-bit (ext) identifier
	bool     ext;          //!< true - extended (29 bit) identifier
	bool     rtr;          //!< remote transmission request
	uint8_t  len;          //!< data length, 0..8
	uint8_t  filterIndex;  //!< index of matched filter element (receive only)
	uint16_t timestamp;    //!< timestamp counter value, bit times (receive only)
	union
	{
		uint8_t  data[8];
		uint32_t data32[2];   //!< little-endian word view: matches message RAM T2/T3 layout
	};

	Message(): id(0), ext(false), rtr(false), len(0), filterIndex(0), timestamp(0), data() {}
};

/**
 * Bit timing parameters (values in time quanta, not "minus one")
 */
struct BitTiming
{
	uint32_t brp;            //!< prescaler 1..512
	uint32_t tseg1;          //!< Prop_Seg + Phase_Seg1, 1..256
	uint32_t tseg2;          //!< Phase_Seg2, 1..128
	uint32_t sjw;            //!< synchronization jump width, 1..tseg2
	bool     valid;

	constexpr uint32_t Nbtp() const
	{
		// NBTP.NSJW in CMSIS is the NTSJW field of RM0440 (same position)
		return ((sjw   - 1) << FDCAN_NBTP_NSJW_Pos)
		     | ((brp   - 1) << FDCAN_NBTP_NBRP_Pos)
		     | ((tseg1 - 1) << FDCAN_NBTP_NTSEG1_Pos)
		     | ((tseg2 - 1) << FDCAN_NBTP_NTSEG2_Pos);
	}
	constexpr uint32_t QuantaPerBit() const { return 1 + tseg1 + tseg2; }
	constexpr uint32_t SamplePointPermille() const { return (1 + tseg1) * 1000 / QuantaPerBit(); }
};

/**
 *  Bit timing calculation (compile time).
 *  Exact division of the kernel clock only; 8..256 quanta per bit;
 *  sample point as close as possible to spPermille (default 87.5%, CiA
 *  recommendation for CAN 2.0); SJW = Phase_Seg2. On equal sample point
 *  error the prescaler found first (smaller brp, more quanta per bit) wins.
 */
constexpr BitTiming CalcBitTiming(uint32_t kernelClock, uint32_t bitrate, uint32_t spPermille = 875)
{
	BitTiming best { 0, 0, 0, 0, false };
	uint32_t bestErr = 0xFFFFFFFFUL;

	for (uint32_t brp = 1; bitrate != 0 && brp <= 512; ++brp)
	{
		const uint32_t div = brp * bitrate;
		if (kernelClock % div)
			continue;
		const uint32_t ntq = kernelClock / div;   // 1 + tseg1 + tseg2
		if (ntq < 8)
			break;                                // ntq decreases as brp grows
		if (ntq > 256)
			continue;

		const uint32_t sp    = (ntq * spPermille + 500) / 1000;  // quanta up to sample point, incl. Sync_Seg
		const uint32_t tseg1 = sp - 1;
		const uint32_t tseg2 = ntq - sp;
		if (tseg1 < 1 || tseg1 > 256 || tseg2 < 1 || tseg2 > 128)
			continue;

		const uint32_t spPm = sp * 1000 / ntq;
		const uint32_t err  = spPm > spPermille ? spPm - spPermille : spPermille - spPm;
		if (err < bestErr)
		{
			bestErr = err;
			best = BitTiming { brp, tseg1, tseg2, tseg2, true };
		}
	}
	return best;
}

/**
 *  Message RAM element and section types (RM0440 Table 414/415).
 *  Element size is 18 words; classic frames use R0, R1, DATA[0..1].
 */
struct MsgRamElement
{
	volatile uint32_t R0;        //!< ESI[31] XTD[30] RTR[29] ID[28:0]
	volatile uint32_t R1;        //!< DLC[19:16]; Rx: FIDX[30:24] timestamp[15:0]; Tx: EFC[23] FDF[21] BRS[20]
	volatile uint32_t DATA[16];  //!< 64 bytes (only 8 used by classic frames)
};

struct ExtFilterElement
{
	volatile uint32_t F0;        //!< EFEC[31:29] EFID1[28:0]
	volatile uint32_t F1;        //!< EFT[31:30]   EFID2[28:0]
};

struct MsgRam_TypeDef
{
	volatile uint32_t     STD_FILTER[28];  //!< 0x000
	ExtFilterElement      EXT_FILTER[8];   //!< 0x070
	MsgRamElement         RX_FIFO0[3];     //!< 0x0B0
	MsgRamElement         RX_FIFO1[3];     //!< 0x188
	volatile uint32_t     TX_EVENT[6];     //!< 0x260
	MsgRamElement         TX_BUFFER[3];    //!< 0x278
};

static_assert(offsetof(MsgRam_TypeDef, EXT_FILTER) == 0x070, "FDCAN message RAM layout");
static_assert(offsetof(MsgRam_TypeDef, RX_FIFO0)   == 0x0B0, "FDCAN message RAM layout");
static_assert(offsetof(MsgRam_TypeDef, RX_FIFO1)   == 0x188, "FDCAN message RAM layout");
static_assert(offsetof(MsgRam_TypeDef, TX_EVENT)   == 0x260, "FDCAN message RAM layout");
static_assert(offsetof(MsgRam_TypeDef, TX_BUFFER)  == 0x278, "FDCAN message RAM layout");
static_assert(sizeof(MsgRam_TypeDef)               == 0x350, "FDCAN message RAM size");

enum : unsigned
{
	// section element counts are derived from the struct above (single
	// source of truth); the static_asserts pin the resulting layout down
	STD_FILTERS_COUNT = sizeof(MsgRam_TypeDef::STD_FILTER) / sizeof(uint32_t),
	EXT_FILTERS_COUNT = sizeof(MsgRam_TypeDef::EXT_FILTER) / sizeof(ExtFilterElement),
	RXFIFO_COUNT      = sizeof(MsgRam_TypeDef::RX_FIFO0)   / sizeof(MsgRamElement),
	TXBUFFER_COUNT    = sizeof(MsgRam_TypeDef::TX_BUFFER)  / sizeof(MsgRamElement),
	MSG_RAM_SIZE      = sizeof(MsgRam_TypeDef),   // asserted to 0x350 below (RM0440 §44.3.6)
};

/**
 * FDCAN pins selection (RX_TX). FDCANn_RX and FDCANn_TX are independent
 * signals (see MixPins below), so RX and TX may also be mixed across
 * variants.
 */
enum Remap
{
	REMAP_A11_A12,      //!< FDCAN1: RX = PA11, TX = PA12
	REMAP_B8_B9,        //!< FDCAN1: RX = PB8,  TX = PB9
	REMAP_D0_D1,        //!< FDCAN1: RX = PD0,  TX = PD1
	REMAP_B12_B13,      //!< FDCAN2: RX = PB12, TX = PB13
	REMAP_B5_B6,        //!< FDCAN2: RX = PB5,  TX = PB6
	REMAP_A8_A15,       //!< FDCAN3: RX = PA8,  TX = PA15
	REMAP_B3_B4,        //!< FDCAN3: RX = PB3,  TX = PB4
};

/**
 *  FDCAN pins selector. All FDCAN functions are AF9 on STM32G4.
 */
template<FdCanNum num, Remap remap> struct FdCanPins;

template<> struct FdCanPins<FDCAN_1, REMAP_A11_A12>   //!< FDCAN1_RX = PA11, FDCAN1_TX = PA12
{
	using RxPin = Pin<'A', 11>;
	using TxPin = Pin<'A', 12>;
	static const PinAltFunction ALT_FUNC_CANx = ALT_FUNC_9;
};

template<> struct FdCanPins<FDCAN_1, REMAP_B8_B9>   //!< FDCAN1_RX = PB8, FDCAN1_TX = PB9
{
	using RxPin = Pin<'B', 8>;
	using TxPin = Pin<'B', 9>;
	static const PinAltFunction ALT_FUNC_CANx = ALT_FUNC_9;
};

template<> struct FdCanPins<FDCAN_1, REMAP_D0_D1>   //!< FDCAN1_RX = PD0, FDCAN1_TX = PD1
{
	using RxPin = Pin<'D', 0>;
	using TxPin = Pin<'D', 1>;
	static const PinAltFunction ALT_FUNC_CANx = ALT_FUNC_9;
};

#if defined(FDCAN2)
template<> struct FdCanPins<FDCAN_2, REMAP_B5_B6>   //!< FDCAN2_RX = PB5, FDCAN2_TX = PB6
{
	using RxPin = Pin<'B', 5>;
	using TxPin = Pin<'B', 6>;
	static const PinAltFunction ALT_FUNC_CANx = ALT_FUNC_9;
};

template<> struct FdCanPins<FDCAN_2, REMAP_B12_B13>   //!< FDCAN2_RX = PB12, FDCAN2_TX = PB13
{
	using RxPin = Pin<'B', 12>;
	using TxPin = Pin<'B', 13>;
	static const PinAltFunction ALT_FUNC_CANx = ALT_FUNC_9;
};
#endif

#if defined(FDCAN3)
template<> struct FdCanPins<FDCAN_3, REMAP_A8_A15>   //!< FDCAN3_RX = PA8, FDCAN3_TX = PA15
{
	using RxPin = Pin<'A', 8>;
	using TxPin = Pin<'A', 15>;
	static const PinAltFunction ALT_FUNC_CANx = ALT_FUNC_9;
};

template<> struct FdCanPins<FDCAN_3, REMAP_B3_B4>   //!< FDCAN3_RX = PB3, FDCAN3_TX = PB4
{
	using RxPin = Pin<'B', 3>;
	using TxPin = Pin<'B', 4>;
	static const PinAltFunction ALT_FUNC_CANx = ALT_FUNC_9;
};
#endif

/**
 * FDCAN traits. Used internally.
 */
template<FdCanNum num> struct FdCanTraits;

template<> struct FdCanTraits<FDCAN_1>
{
	static const uint32_t FDCANx_BASE = FDCAN1_BASE;
	static const uint32_t RAM_BASE    = SRAMCAN_BASE;
	static const IRQn     IT0_IRQn    = FDCAN1_IT0_IRQn;
	static const IRQn     IT1_IRQn    = FDCAN1_IT1_IRQn;
};

#if defined(FDCAN2)
template<> struct FdCanTraits<FDCAN_2>
{
	static const uint32_t FDCANx_BASE = FDCAN2_BASE;
	static const uint32_t RAM_BASE    = SRAMCAN_BASE + 1 * MSG_RAM_SIZE;
	static const IRQn     IT0_IRQn    = FDCAN2_IT0_IRQn;
	static const IRQn     IT1_IRQn    = FDCAN2_IT1_IRQn;
};
#endif

#if defined(FDCAN3)
template<> struct FdCanTraits<FDCAN_3>
{
	static const uint32_t FDCANx_BASE = FDCAN3_BASE;
	static const uint32_t RAM_BASE    = SRAMCAN_BASE + 2 * MSG_RAM_SIZE;
	static const IRQn     IT0_IRQn    = FDCAN3_IT0_IRQn;
	static const IRQn     IT1_IRQn    = FDCAN3_IT1_IRQn;
};
#endif

/**
 *  Custom pin combination: any FDCANn_RX pin with any FDCANn_TX pin of the
 *  same instance. There is no peripheral-level remap on FDCAN: each pin
 *  selects its signal by its own AF register (AF9), so RX and TX may be
 *  mixed freely, e.g. FDCAN1_RX = PA11 with FDCAN1_TX = PB9.
 */
template<typename Rx, typename Tx> struct MixPins
{
	using RxPin = Rx;
	using TxPin = Tx;
	static const PinAltFunction ALT_FUNC_CANx = ALT_FUNC_9;
};

/**
 * Sample properties for FDCAN template
 */
struct SampleFdCanProps
{
	static const FdCanNum    NUMBER  = FDCAN_1;
	static const Remap       REMAP   = REMAP_A11_A12;
	static const ClockSource CLK_SRC = CLK_SRC_HSE;
	static const uint32_t    CLK_FREQ  = HSE_VALUE;
	static const uint32_t    BIT_RATE  = 500000;
};

/**
 *  FDCAN driver (static functions only).
 *  Sample properties:
 *  @code
 *  struct MyCanProps
 *  {
 *  	static const FDCANx::FdCanNum    NUMBER   = FDCANx::FDCAN_1;
 *  	static const FDCANx::Remap       REMAP    = FDCANx::REMAP_A11_A12;
 *  	static const FDCANx::ClockSource CLK_SRC  = FDCANx::CLK_SRC_HSE;
 *  	static const uint32_t            CLK_FREQ  = HSE_VALUE;   // kernel clock, must match CLK_SRC
 *  	static const uint32_t            BIT_RATE  = 500000;
 *  };
 *  typedef FDCANx::FdCanModule<MyCanProps> MyCan;
 *  @endcode
 *  Non-standard pin combination (same FDCAN instance): pass a pin set
 *  instead of using REMAP:
 *  @code
 *  typedef FDCANx::FdCanModule<MyCanProps,
 *          FDCANx::MixPins<Pin<'A', 11>, Pin<'B', 9>>> MyCanMixed;
 *  @endcode
 */
template<typename props = SampleFdCanProps,
         typename Pins = FdCanPins<props::NUMBER, props::REMAP>>
class FdCanModule
{
public:
	static const FdCanNum NUMBER = props::NUMBER;

	using Traits = FdCanTraits<NUMBER>;
	using RxPin  = typename Pins::RxPin;
	using TxPin  = typename Pins::TxPin;

	static const IRQn IT0_IRQn = Traits::IT0_IRQn;
	static const IRQn IT1_IRQn = Traits::IT1_IRQn;
	/// legacy (stm32_can.h compatible) name: interrupt line 0
	static const IRQn CanModuleIRQn = Traits::IT0_IRQn;

	static FDCAN_GlobalTypeDef *const FDCANx;

protected:
	static const uint32_t FDCANx_BASE = Traits::FDCANx_BASE;
	static const uint32_t RAM_BASE    = Traits::RAM_BASE;

	// message element bit masks (RM0440 Table 414/415)
	enum : uint32_t
	{
		EL_ID   = 0x1FFFFFFFUL,
		EL_XTD  = (1UL << 30),
		EL_RTR  = (1UL << 29),
		EL_DLC  = (0xFUL << 16),
	};

	INLINE static volatile MsgRam_TypeDef *MsgRam()
	{
		return reinterpret_cast<volatile MsgRam_TypeDef *>(RAM_BASE);
	}

public:
	static void InitPins()
	{
		TxPin::Alternate(Pins::ALT_FUNC_CANx);
		RxPin::Alternate(Pins::ALT_FUNC_CANx);
		TxPin::Mode(ALT_OUTPUT);
		RxPin::Mode(ALT_INPUT_PULLUP);   // keeps RX recessive with a disconnected transceiver
	}

	static void DeinitPins()
	{
		TxPin::Mode(INPUT);
		RxPin::Mode(INPUT);
	}

	/// Note: FDCAN clock enable/reset bits are shared by all FDCAN instances
	INLINE static void EnableClocks()  { RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;  __DSB(); }
	INLINE static void DisableClocks() { RCC->APB1ENR1 &= ~RCC_APB1ENR1_FDCANEN; __DSB(); }
	INLINE static void Reset()         { RCC->APB1RSTR1 |= RCC_APB1RSTR1_FDCANRST;
	                                     RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_FDCANRST; }

	INLINE static void SelectClockSource(ClockSource src)
	{
		RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_FDCANSEL) |
		              (static_cast<uint32_t>(src) << RCC_CCIPR_FDCANSEL_Pos);
		__DSB();
	}

	/// Resume FDCAN kernel clock after stop request (CCCR.CSR)
	INLINE static bool ExitSleepMode(uint32_t timeout = 0xFFFF)
	{
		FDCANx->CCCR &= ~FDCAN_CCCR_CSR_Msk;
		for (uint32_t i = 0; i < timeout; ++i)
		{
			if (!(FDCANx->CCCR & FDCAN_CCCR_CSA_Msk))
				return true;
		}
		return false;
	}

	/// Set INIT and CCE: entry to init mode with configuration access
	INLINE static bool EnterInitMode(uint32_t timeout = 0xFFFF)
	{
		if (!ExitSleepMode(timeout))   // INIT can be set only with the kernel clock running
			return false;
		FDCANx->CCCR |= FDCAN_CCCR_INIT_Msk;
		for (uint32_t i = 0; i < timeout; ++i)
		{
			if (FDCANx->CCCR & FDCAN_CCCR_INIT_Msk)
			{
				FDCANx->CCCR |= FDCAN_CCCR_CCE_Msk;
				for (uint32_t j = 0; j < timeout; ++j)
				{
					if (FDCANx->CCCR & FDCAN_CCCR_CCE_Msk)
						return true;
				}
				return false;
			}
		}
		return false;
	}

	/// Clear INIT (CCE is cleared by hardware): leave init mode, start bus activity
	INLINE static bool ExitInitMode(uint32_t timeout = 0xFFFF)
	{
		FDCANx->CCCR &= ~FDCAN_CCCR_INIT_Msk;
		for (uint32_t i = 0; i < timeout; ++i)
		{
			if (!(FDCANx->CCCR & FDCAN_CCCR_INIT_Msk))
				return true;
		}
		return false;
	}

	INLINE static bool IsInitMode() { return FDCANx->CCCR & FDCAN_CCCR_INIT_Msk; }

	/**
	 * Full initialization: pins, clocks, kernel clock source, bit timing,
	 * message RAM clear, filter list sizes, both interrupt lines enabled.
	 * Call from thread context. Filters are not configured afterwards:
	 * with the default RXGFC (ANFS/ANFE = 0) all messages are accepted
	 * into Rx FIFO 0 until ConfigureFilter()/ConfigureExtendedFilter()
	 * are used.
	 * @return false if the clock/bit rate pair has no exact bit timing
	 *         solution or init mode acknowledge timed out
	 */
	static bool Init()
	{
		constexpr BitTiming timing = CalcBitTiming(props::CLK_FREQ, props::BIT_RATE);
		if (!timing.valid)
			return false;

		InitPins();
		EnableClocks();
		SelectClockSource(props::CLK_SRC);

		if (!EnterInitMode())
			return false;

		// message RAM contents are undefined after power-on;
		// zeroing disables all filter elements (SFEC/EFEC = 000)
		volatile uint32_t *ram = reinterpret_cast<volatile uint32_t *>(RAM_BASE);
		for (unsigned i = 0; i < MSG_RAM_SIZE / 4; ++i)
			ram[i] = 0;

		FDCANx->NBTP  = timing.Nbtp();
		FDCANx->XIDAM = EL_ID;   // extended AND-mask filter (ftRange EFT=00) uses full 29-bit mask
		FDCANx->RXGFC = (STD_FILTERS_COUNT << FDCAN_RXGFC_LSS_Pos)
		              | (EXT_FILTERS_COUNT << FDCAN_RXGFC_LSE_Pos);
		// disable protocol exception handling: ISO 11898-1:2015 behaviour,
		// prevents error frames on reserved-bit sequences
		FDCANx->CCCR |= FDCAN_CCCR_PXHD_Msk;
		FDCANx->ILE   = FDCAN_ILE_EINT0_Msk | FDCAN_ILE_EINT1_Msk;

		return ExitInitMode();
	}

	static void Deinit()
	{
		if (EnterInitMode())
			ExitInitMode();
		DisableClocks();   // shared bit: check that no other FDCAN instance is running
		DeinitPins();
	}

	/**
	 * Operating mode. Must be called in init mode (EnterInitMode() without
	 * ExitInitMode()); MON/TEST/DAR cannot be set in normal mode.
	 */
	INLINE static void SetMode(Mode mode)
	{
		uint32_t cccr = FDCANx->CCCR;
		cccr &= ~(FDCAN_CCCR_MON_Msk | FDCAN_CCCR_TEST_Msk | FDCAN_CCCR_DAR_Msk);
		switch (mode)
		{
		case modeLoopbackInternal: cccr |= FDCAN_CCCR_TEST_Msk;                     break;
		case modeLoopbackExternal: cccr |= FDCAN_CCCR_TEST_Msk | FDCAN_CCCR_DAR_Msk; break;
		case modeSilent:           cccr |= FDCAN_CCCR_MON_Msk;                      break;
		default:                                                                break;
		}
		FDCANx->CCCR = cccr;
		FDCANx->TEST = (mode == modeLoopbackInternal) ? FDCAN_TEST_LBCK_Msk : 0UL;
	}

	/**
	 * Configure standard (11 bit) ID filter element.
	 * Filter list is scanned from element 0; first match wins.
	 * Single-word write is atomic, can be called in normal mode.
	 */
	INLINE static void ConfigureFilter(unsigned number,
			uint32_t id1,
			uint32_t id2,
			FilterType type,
			FilterAction action = faStoreFifo0)
	{
		if (number >= STD_FILTERS_COUNT)
			return;
		// RM0440 Table 417: SFT[31:30] | SFEC[29:27] | SFID1[26:16] | SFID2[10:0]
		MsgRam()->STD_FILTER[number] =
				(static_cast<uint32_t>(type)   << 30)
		              | (static_cast<uint32_t>(action) << 27)
		              | ((id1 & 0x7FFUL) << 16)
		              | (id2 & 0x7FFUL);
	}

	/**
	 * Configure extended (29 bit) filter element.
	 * For ftMask: id1 = filter pattern, id2 = mask.
	 * Two-word element: disabled first, so a partially updated element
	 * cannot accept wrong messages (EFEC = 000 during update).
	 */
	INLINE static void ConfigureExtendedFilter(unsigned number,
			uint32_t id1,
			uint32_t id2,
			FilterType type,
			FilterAction action = faStoreFifo0)
	{
		if (number >= EXT_FILTERS_COUNT)
			return;
		volatile ExtFilterElement &el = MsgRam()->EXT_FILTER[number];
		el.F0 = 0;   // EFEC = 000: element disabled while updating
		el.F1 = (static_cast<uint32_t>(type) << 30) | (id2 & EL_ID);
		el.F0 = (static_cast<uint32_t>(action) << 29) | (id1 & EL_ID);   // EFEC[31:29], EFID1[28:0]
	}

	/// Disable filter element (SFEC = 000)
	INLINE static void DisableFilter(unsigned number)
	{
		if (number < STD_FILTERS_COUNT)
			MsgRam()->STD_FILTER[number] = 0;
	}

	/// Disable extended filter element (EFEC = 000)
	INLINE static void DisableExtendedFilter(unsigned number)
	{
		if (number < EXT_FILTERS_COUNT)
			MsgRam()->EXT_FILTER[number].F0 = 0;
	}

	/**
	 * Global filter: behaviour for messages matching no filter element.
	 * @param stdAction/faStoreFifo0, faStoreFifo1, faReject - for 11-bit frames
	 * @param extAction  same, for 29-bit frames
	 */
	INLINE static void SetNonMatchingAction(FilterAction stdAction, FilterAction extAction)
	{
		const auto an = [](FilterAction a) -> uint32_t
		{
			switch (a)
			{
			case faStoreFifo1: return 1UL;   // ANFS/ANFE: 00 FIFO0, 01 FIFO1, 1x reject
			case faReject:     return 2UL;
			default:           return 0UL;
			}
		};
		FDCANx->RXGFC = (FDCANx->RXGFC & ~(FDCAN_RXGFC_ANFS_Msk | FDCAN_RXGFC_ANFE_Msk))
		              | (an(stdAction) << FDCAN_RXGFC_ANFS_Pos)
		              | (an(extAction) << FDCAN_RXGFC_ANFE_Pos);
	}

	/// Reject remote frames (RTR) with standard / extended identifier
	INLINE static void RejectRemoteFrames(bool rejectStd, bool rejectExt)
	{
		const uint32_t bits = (rejectStd ? FDCAN_RXGFC_RRFS_Msk : 0UL)
		                    | (rejectExt ? FDCAN_RXGFC_RRFE_Msk : 0UL);
		FDCANx->RXGFC = (FDCANx->RXGFC & ~(FDCAN_RXGFC_RRFS_Msk | FDCAN_RXGFC_RRFE_Msk)) | bits;
	}

	/**
	 * Put message to the next free Tx FIFO element and request transmission.
	 * @return false if Tx FIFO is full
	 */
	static bool Send(const Message &msg)
	{
		const uint32_t txfqs = FDCANx->TXFQS;
		if (txfqs & FDCAN_TXFQS_TFQF_Msk)
			return false;
		const unsigned pi = (txfqs & FDCAN_TXFQS_TFQPI_Msk) >> FDCAN_TXFQS_TFQPI_Pos;

		volatile MsgRamElement &el = MsgRam()->TX_BUFFER[pi];
		// RM0440 Table 414: identifier occupies ID[28:0] as is for extended
		// frames; standard identifier has to be written to ID[28:18]
		uint32_t r0 = msg.ext
				? (msg.id & EL_ID) | EL_XTD
				: (msg.id & 0x7FFUL) << 18;
		if (msg.rtr)
			r0 |= EL_RTR;

		const uint32_t len = msg.len > 8 ? 8 : msg.len;
		el.R0      = r0;
		el.DATA[0] = msg.data32[0];
		el.DATA[1] = msg.data32[1];
		el.R1      = len << 16;   // DLC written last; FDF = 0, BRS = 0, EFC = 0
		FDCANx->TXBAR = 1UL << pi;   // add buffer for transmission (requires CCE = 0)
		return true;
	}

	/// true while at least one Tx buffer request is pending
	INLINE static bool TxBusy()         { return FDCANx->TXBRP != 0; }
	INLINE static uint32_t TxBusyMask() { return FDCANx->TXBRP; }
	/// Tx FIFO free elements count
	INLINE static unsigned TxFreeCount()
	{
		return (FDCANx->TXFQS & FDCAN_TXFQS_TFFL_Msk) >> FDCAN_TXFQS_TFFL_Pos;
	}
	/// cancel a pending Tx buffer request
	INLINE static void CancelSend(unsigned buffer)
	{
		FDCANx->TXBCR = 1UL << buffer;
	}
	/// read and clear "transmission occurred" flags (TXBTO is read-cleared)
	INLINE static uint32_t GetTxOccurredMask() { return FDCANx->TXBTO; }

	/**
	 * Get and acknowledge oldest message from Rx FIFO 0.
	 * @return false if FIFO is empty
	 */
	INLINE static bool Recv(Message &msg) { return RecvFifo(0, msg); }

	/// Get and acknowledge oldest message from Rx FIFO 0 or 1
	static bool RecvFifo(unsigned fifo, Message &msg)
	{
		volatile uint32_t &rxfs = fifo ? FDCANx->RXF1S : FDCANx->RXF0S;
		volatile uint32_t &rxfa = fifo ? FDCANx->RXF1A : FDCANx->RXF0A;

		const uint32_t s = rxfs;
		if ((s & FDCAN_RXF0S_F0FL_Msk) == 0)
			return false;
		const unsigned gi = fifo
				? ((s & FDCAN_RXF1S_F1GI_Msk) >> FDCAN_RXF1S_F1GI_Pos)
				: ((s & FDCAN_RXF0S_F0GI_Msk) >> FDCAN_RXF0S_F0GI_Pos);

		volatile MsgRamElement &el = fifo ? MsgRam()->RX_FIFO1[gi] : MsgRam()->RX_FIFO0[gi];
		const uint32_t r0 = el.R0;
		const uint32_t r1 = el.R1;
		msg.data32[0] = el.DATA[0];
		msg.data32[1] = el.DATA[1];

		msg.ext = (r0 & EL_XTD) != 0;
		msg.rtr = (r0 & EL_RTR) != 0;
		msg.id  = msg.ext ? (r0 & EL_ID) : ((r0 >> 18) & 0x7FFUL);
		const uint32_t len = (r1 & EL_DLC) >> 16;
		msg.len         = static_cast<uint8_t>(len > 8 ? 8 : len);
		msg.filterIndex = static_cast<uint8_t>((r1 >> 24) & 0x7FUL);   // bit 31: non-matching frame flag
		msg.timestamp   = static_cast<uint16_t>(r1);

		rxfa = gi;   // release element (write get index)
		return true;
	}

	/// Rx FIFO fill level (0..3)
	INLINE static unsigned RxFifoCount(unsigned fifo = 0)
	{
		return (fifo ? FDCANx->RXF1S : FDCANx->RXF0S) & FDCAN_RXF0S_F0FL_Msk;
	}

	INLINE static uint32_t GetErrorCounters()   { return FDCANx->ECR; }
	INLINE static uint32_t GetProtocolStatus()  { return FDCANx->PSR; }
	INLINE static bool     IsBusOff()           { return FDCANx->PSR & FDCAN_PSR_BO_Msk; }

	/// raw IR register read / clear (flags are cleared by writing 1)
	INLINE static uint32_t GetInterruptFlags()            { return FDCANx->IR; }
	INLINE static void     ClearInterruptFlags(uint32_t mask) { FDCANx->IR = mask; }
	/// read and clear pending interrupt flags from the given set
	INLINE static uint32_t TakeInterrupts(uint32_t mask)
	{
		const uint32_t ir = FDCANx->IR & mask;
		FDCANx->IR = ir;
		return ir;
	}

	/// Route interrupt groups to line 1 (default: everything on line 0).
	/// Mask is a combination of FDCAN_ILS_* group bits.
	INLINE static void RouteInterruptsToLine1(uint32_t ilsMask) { FDCANx->ILS |= ilsMask; }

	struct RxFifo0Interrupt
	{
		static const uint32_t FLAGS = FDCAN_IR_RF0N_Msk | FDCAN_IR_RF0F_Msk | FDCAN_IR_RF0L_Msk;
		static void Enable()    { FDCANx->IE |= FDCAN_IE_RF0NE_Msk | FDCAN_IE_RF0LE_Msk; }
		static void Disable()   { FDCANx->IE &= ~(FDCAN_IE_RF0NE_Msk | FDCAN_IE_RF0LE_Msk); }
		static bool IsEnabled() { return FDCANx->IE & FDCAN_IE_RF0NE_Msk; }
		static void Clear()     { FDCANx->IR = FLAGS; }
	};

	struct RxFifo1Interrupt
	{
		static const uint32_t FLAGS = FDCAN_IR_RF1N_Msk | FDCAN_IR_RF1F_Msk | FDCAN_IR_RF1L_Msk;
		static void Enable()    { FDCANx->IE |= FDCAN_IE_RF1NE_Msk | FDCAN_IE_RF1LE_Msk; }
		static void Disable()   { FDCANx->IE &= ~(FDCAN_IE_RF1NE_Msk | FDCAN_IE_RF1LE_Msk); }
		static bool IsEnabled() { return FDCANx->IE & FDCAN_IE_RF1NE_Msk; }
		static void Clear()     { FDCANx->IR = FLAGS; }
	};

	/// "transmission occurred" interrupt for all Tx FIFO elements
	struct TxCompleteInterrupt
	{
		static const uint32_t FLAGS = FDCAN_IR_TC_Msk;
		static void Enable()
		{
			FDCANx->TXBTIE = (1UL << TXBUFFER_COUNT) - 1;
			FDCANx->IE |= FDCAN_IE_TCE_Msk;
		}
		static void Disable()   { FDCANx->IE &= ~FDCAN_IE_TCE_Msk; FDCANx->TXBTIE = 0; }
		static bool IsEnabled() { return FDCANx->IE & FDCAN_IE_TCE_Msk; }
		static void Clear()     { FDCANx->IR = FLAGS; }
	};

	/// bus-off interrupt
	struct BusOffInterrupt
	{
		static const uint32_t FLAGS = FDCAN_IR_BO_Msk;
		static void Enable()    { FDCANx->IE |= FDCAN_IE_BOE_Msk; }
		static void Disable()   { FDCANx->IE &= ~FDCAN_IE_BOE_Msk; }
		static bool IsEnabled() { return FDCANx->IE & FDCAN_IE_BOE_Msk; }
		static void Clear()     { FDCANx->IR = FLAGS; }
	};
};

template<typename props, typename Pins>
FDCAN_GlobalTypeDef *const FdCanModule<props, Pins>::FDCANx =
		reinterpret_cast<FDCAN_GlobalTypeDef *const>(FdCanModule<props, Pins>::FDCANx_BASE);

} // namespace FDCANx
} // namespace STM32
