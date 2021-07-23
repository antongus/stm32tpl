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
 *  file         : stm32_spi.h
 *  description  : STM32 SPI module class
 *  created on   : 01.03.2010
 *
 */

#ifndef STM32TPL_STM32_SPI_H_INCLUDED
#define STM32TPL_STM32_SPI_H_INCLUDED

#include "stm32.h"
#include "pin.h"
#include "stm32_dma.h"
#include <scmRTOS.h>

#include <type_traits>

namespace STM32
{
namespace SPI
{

/**
 * Enumeration for all SPI devices in system
 */
enum SpiNum
{
	SPI_1
#if (defined RCC_APB1ENR_SPI2EN)
	,SPI_2
#endif
#if (defined RCC_APB1ENR_SPI3EN)
	,SPI_3
#endif
};

/**
*  SPI divisors
*/
enum Divisor
{
	SPI_DIV_2	= (0 << 3),  //!< divisor = 2
	SPI_DIV_4	= (1 << 3),  //!< divisor = 4
	SPI_DIV_8	= (2 << 3),  //!< divisor = 8
	SPI_DIV_16	= (3 << 3),  //!< divisor = 16
	SPI_DIV_32	= (4 << 3),  //!< divisor = 32
	SPI_DIV_64	= (5 << 3),  //!< divisor = 64
	SPI_DIV_128	= (6 << 3),  //!< divisor = 128
	SPI_DIV_256	= (7 << 3)   //!< divisor = 256
};

/**
*  SPI remap states
*/
enum Remap
{
	REMAP_NONE = 0,        //!< no remap
	REMAP_FULL,            //!< remap
	REMAP_2,               //!< remap #2
};

/**
*  SPI InitialCPOL (Clock Polarity) enum
*/
enum Cpol
{
	CPOL_L = 0,              //!< Clock Polarity LOW
	CPOL_H = SPI_CR1_CPOL    //!< Clock Polarity HIGH
};

/**
*  SPI InitialCPHA (Clock Phase) enum
*/
enum Cpha
{
	CPHA_1 = 0,               //!< Clock Phase 1
	CPHA_2 = SPI_CR1_CPHA     //!< Clock Phase 2
};



/**
*  SPI base class.
*  Implements common SPI functions.
*  Template implementations are derived from it.
*/
class SpiBase
{
public:
	SPI_TypeDef *const SPIx;

	SpiBase(SPI_TypeDef *const spix)
		: SPIx(spix)
		, mutex_()
	{ }

	/**
	 * Read/write function.
	 */
	uint8_t Rw(uint8_t out = 0xFF)
	{
		auto ptr = reinterpret_cast<volatile uint8_t*>(&SPIx->DR);
		*ptr = out;
		while (!(SPIx->SR & SPI_SR_RXNE)) ;
		return *ptr;
	}

	void Lock()                   { mutex_.lock(); }
	void Unlock()                 { mutex_.unlock(); }
	bool TryLock()                { return mutex_.try_lock(); }

	void Enable()                 { SPIx->CR1 |= SPI_CR1_SPE; }
	void Disable()                { SPIx->CR1 &= ~SPI_CR1_SPE; }
	void SetDivisor(Divisor div)  { SPIx->CR1 = (SPIx->CR1 & ~SPI_DIV_256) | div; }
	Divisor GetDivisor()          { return (Divisor)(SPIx->CR1 & SPI_DIV_256); }
	void SetCpol(Cpol pol)        { SPIx->CR1 = (SPIx->CR1 & ~SPI_CR1_CPOL) | pol; }
	Cpol GetCpol()                { return (Cpol)(SPIx->CR1 & SPI_CR1_CPOL); }
	void SetCpha(Cpha pha)        { SPIx->CR1 = (SPIx->CR1 & ~SPI_CR1_CPHA) | pha; }
	Cpha GetCpha()                { return (Cpha)(SPIx->CR1 & SPI_CR1_CPHA); }
	void WaitTxDone()
	{
		for (uint32_t div = GetDivisor() >> 3; div; div >>= 1)
			__asm__ __volatile__ ("nop");
	}
	SpiBase& operator=(uint8_t val) { Rw(val); return *this; }
	operator uint8_t() { return Rw(); }
	virtual void BufRw(uint8_t * rxBuf, uint8_t const* txBuf, size_t cnt) = 0;
private:
	OS::TMutex mutex_;
};

/**
*  SPI Pins selector.
*/
template<SpiNum spiNum, Remap remap = REMAP_NONE> struct SpiPins;

template<> struct SpiPins<SPI_1>
{
	typedef Pin<'A', 5> PinSCK;
	typedef Pin<'A', 6> PinMISO;
	typedef Pin<'A', 7> PinMOSI;
#if (defined STM32TPL_F2xxF4xx) || (defined STM32TPL_STM32L1XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_SPI1;
#elif (defined STM32TPL_STM32L0XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_0;
#elif (defined STM32TPL_STM32F3XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_5;
#endif
};

template<> struct SpiPins<SPI_1, REMAP_FULL>
{
	typedef Pin<'B', 3> PinSCK;
	typedef Pin<'B', 4> PinMISO;
	typedef Pin<'B', 5> PinMOSI;
#if (defined STM32TPL_F2xxF4xx) || (defined STM32TPL_STM32L1XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_SPI1;
#elif (defined STM32TPL_STM32L0XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_0;
#elif (defined STM32TPL_STM32F3XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_5;
#endif
};

template<> struct SpiPins<SPI_1, REMAP_2>
{
#if (defined STM32TPL_STM32F3XX)
	typedef Pin<'C', 7> PinSCK;
	typedef Pin<'C', 8> PinMISO;
	typedef Pin<'C', 9> PinMOSI;
#else
	typedef Pin<'E', 13> PinSCK;
	typedef Pin<'E', 14> PinMISO;
	typedef Pin<'E', 15> PinMOSI;
#endif
#if (defined STM32TPL_F2xxF4xx) || (defined STM32TPL_STM32L1XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_SPI1;
#elif (defined STM32TPL_STM32L0XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_0;
#elif (defined STM32TPL_STM32F3XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_5;
#endif
};

#if (defined RCC_APB1ENR_SPI2EN)
template<> struct SpiPins<SPI_2>
{
#if (defined STM32TPL_STM32F3XX)
	typedef Pin<'B', 10> PinSCK;
#else
	typedef Pin<'B', 13> PinSCK;
#endif
	typedef Pin<'B', 14> PinMISO;
	typedef Pin<'B', 15> PinMOSI;
#if (defined STM32TPL_F2xxF4xx) || (defined STM32TPL_STM32L1XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_SPI2;
#elif (defined STM32TPL_STM32L0XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_0;
#elif (defined STM32TPL_STM32F3XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_5;
#endif
};

#if (defined STM32TPL_STM32F3XX)
template<> struct SpiPins<SPI_2, REMAP_FULL>
{
	typedef Pin<'A', 8> PinSCK;
	typedef Pin<'A', 9> PinMISO;
	typedef Pin<'A', 10> PinMOSI;
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_5;
};
#endif

#endif  // #if (defined RCC_APB1ENR_SPI2EN)

#if (defined RCC_APB1ENR_SPI3EN)
template<> struct SpiPins<SPI_3>
{
	typedef Pin<'B', 3> PinSCK;
	typedef Pin<'B', 4> PinMISO;
	typedef Pin<'B', 5> PinMOSI;
#if (defined STM32TPL_F2xxF4xx) || (defined STM32TPL_STM32L1XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_SPI3;
#elif (defined STM32TPL_STM32F3XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_6;
#endif
};

template<> struct SpiPins<SPI_3, REMAP_FULL>
{
	typedef Pin<'C', 10> PinSCK;
	typedef Pin<'C', 11> PinMISO;
	typedef Pin<'C', 12> PinMOSI;
#if (defined STM32TPL_F2xxF4xx) || (defined STM32TPL_STM32L1XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_SPI3;
#elif (defined STM32TPL_STM32F3XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_6;
#endif
};

#if (defined STM32TPL_STM32F3XX)
template<> struct SpiPins<SPI_3, REMAP_2>
{
	typedef Pin<'A', 1> PinSCK;
	typedef Pin<'A', 2> PinMISO;
	typedef Pin<'A', 3> PinMOSI;
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_6;
};
#endif

#endif // #if (defined RCC_APB1ENR_SPI3EN)

namespace
{
/**
*  SPI Traits. Used internally.
*/
template <SpiNum spiNum> struct SpiTraits;

template<> struct SpiTraits<SPI_1>
{
	static const IRQn SPIx_IRQn  = SPI1_IRQn;
	enum
	{
		SPIx_BASE               = SPI1_BASE,
#if (defined AFIO_MAPR_SPI1_REMAP)
		SPIx_REMAP              = AFIO_MAPR_SPI1_REMAP,
#else
		SPIx_REMAP              = 0,
#endif
		BUS_FREQ                = chip::APB2_FREQ
	};

#if (defined STM32TPL_F2xxF4xx)
	enum { RX_DMA_CHANNEL = DMA::DMA_CR_CHSEL_CH3 };
	enum { TX_DMA_CHANNEL = DMA::DMA_CR_CHSEL_CH3 };

	typedef DMA::Dma2Channel2 RxDmaStream;
	typedef DMA::Dma2Channel3 TxDmaStream;
#elif (defined STM32TPL_STM32L0XX)
	typedef DMA::Dma1Channel2 RxDmaStream;
	typedef DMA::Dma1Channel3 TxDmaStream;
	static const RxDmaStream::ChannelSelection CH_SEL_SPIx_RX = RxDmaStream::ChannelSelection::CH_SEL_SPI1_RX;
	static const TxDmaStream::ChannelSelection CH_SEL_SPIx_TX = TxDmaStream::ChannelSelection::CH_SEL_SPI1_TX;
#else
	typedef DMA::Dma1Channel2 RxDmaStream;
	typedef DMA::Dma1Channel3 TxDmaStream;
#endif

	INLINE static void EnableClocks()  { RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  __DSB(); }
	INLINE static void DisableClocks() { RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN; __DSB(); }

};

#if (defined RCC_APB1ENR_SPI2EN)
template<> struct SpiTraits<SPI_2>
{
	static const IRQn SPIx_IRQn  = SPI2_IRQn;
	enum
	{
		SPIx_BASE               = SPI2_BASE,
		SPIx_REMAP              = 0,
		BUS_FREQ                = chip::APB1_FREQ
	};
#if (defined STM32TPL_F2xxF4xx)
	enum { RX_DMA_CHANNEL = DMA::DMA_CR_CHSEL_CH0 };
	enum { TX_DMA_CHANNEL = DMA::DMA_CR_CHSEL_CH0 };

	typedef DMA::Dma1Channel3 RxDmaStream;
	typedef DMA::Dma1Channel4 TxDmaStream;
#elif (defined STM32TPL_STM32L0XX)
	typedef DMA::Dma1Channel4 RxDmaStream;
	typedef DMA::Dma1Channel5 TxDmaStream;
	static const RxDmaStream::ChannelSelection CH_SEL_SPIx_RX = RxDmaStream::ChannelSelection::CH_SEL_SPI2_RX;
	static const TxDmaStream::ChannelSelection CH_SEL_SPIx_TX = TxDmaStream::ChannelSelection::CH_SEL_SPI2_TX;
#else
	typedef DMA::Dma1Channel4 RxDmaStream;
	typedef DMA::Dma1Channel5 TxDmaStream;
#endif

	INLINE static void EnableClocks()  { RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;  __DSB(); }
	INLINE static void DisableClocks() { RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN; __DSB(); }
};
#endif

#if (defined RCC_APB1ENR_SPI3EN)
template<> struct SpiTraits<SPI_3>
{
	static const IRQn SPIx_IRQn  = SPI3_IRQn;
	enum
	{
		SPIx_BASE               = SPI3_BASE,
#if (defined AFIO_MAPR_SPI3_REMAP)
		SPIx_REMAP              = AFIO_MAPR_SPI3_REMAP,
#else
		SPIx_REMAP              = 0,
#endif
		BUS_FREQ                = chip::APB1_FREQ
	};

#if (defined STM32TPL_F2xxF4xx)
	enum { RX_DMA_CHANNEL = DMA::DMA_CR_CHSEL_CH0 };
	enum { TX_DMA_CHANNEL = DMA::DMA_CR_CHSEL_CH0 };

	typedef DMA::Dma1Channel2 RxDmaStream;
	typedef DMA::Dma1Channel5 TxDmaStream;
#else
	typedef DMA::Dma2Channel1 RxDmaStream;
	typedef DMA::Dma2Channel2 TxDmaStream;
#endif

	INLINE static void EnableClocks()  { RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;  __DSB(); }
	INLINE static void DisableClocks() { RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN; __DSB(); }
};
#endif

} // anonymous namespace

namespace detail
{

template <class Props>
struct DmaStrategyPolling
{
	using DmaStream = typename SpiTraits<Props::NUMBER>::RxDmaStream;
	INLINE static void waitDmaDone()
	{
		while (!(DmaStream::ISR & DmaStream::DMA_MASK_TCIF)) ;
	}
	void initDmaInterrupt() {};
	void deinitDmaInterrupt() {};
};

template <class Props>
struct DmaStrategyInterrupt
{
	using DmaStream = typename SpiTraits<Props::NUMBER>::RxDmaStream;
	INLINE void waitDmaDone()
	{
		flag.wait();
	}
	void initDmaInterrupt()
	{
	    // Enable RX DMA IRQ
#if (!defined STM32TPL_STM32L0XX)
	    NVIC_SetPriority(DmaStream::DMAChannel_IRQn,
	    		NVIC_EncodePriority(NVIC_GetPriorityGrouping(), Props::IrqPrioGroup, Props::IrqSubPrio));
#else
	    NVIC_SetPriority(DmaStream::DMAChannel_IRQn, Props::IrqSubPrio);
#endif
	    NVIC_EnableIRQ(DmaStream::DMAChannel_IRQn);
	};
	INLINE void IrqHandler()
	{
		if (DmaStream::ISR & DmaStream::DMA_MASK_TCIF )
		{
			DmaStream::IFCR = DmaStream::DMA_MASK_TCIF;
			flag.signal_isr();
		}
	}
	void deinitDmaInterrupt()
	{
	    NVIC_DisableIRQ(DmaStream::DMAChannel_IRQn);
	};
private:
	OS::TEventFlag flag;
};

// alias template for conditional select DMA handling strategy depending on properties
template <class Props>
using DmaStrategy = std::conditional<Props::UseInterrupt,
		DmaStrategyInterrupt<Props>,
		DmaStrategyPolling<Props>>;

} // namespace detail


/**
 * Sample properties for SPI template
 */
struct SampleSpiProps
{
	static const SpiNum   NUMBER           = SPI_1;
	static const Remap    REMAP            = REMAP_NONE;
	static const Divisor  InitialDivisor   = SPI_DIV_32;
	static const Cpol     InitialCPOL      = CPOL_L;
	static const Cpha     InitialCPHA      = CPHA_1;
	static const bool     UseInterrupt     = false;
	static const uint32_t IrqPrioGroup     = 2;
	static const uint32_t IrqSubPrio       = 2;
};

/**
*  SPI template class.
*/
template<typename props, typename pins = SpiPins<props::NUMBER, props::REMAP>>
class Spi
		: public SpiBase
		, public detail::DmaStrategy<props>::type
{
public:
	static const SpiNum   NUMBER    = props::NUMBER;
	static const Remap    REMAP     = props::REMAP;
private:
	static const Divisor  InitialDivisor   = props::InitialDivisor;
	static const Cpol     InitialCPOL      = props::InitialCPOL;
	static const Cpha     InitialCPHA      = props::InitialCPHA;
	static const bool     UseInterrupt     = props::UseInterrupt;

	typedef SpiTraits<NUMBER> Traits;
	typedef typename pins::PinSCK SCK;
	typedef typename pins::PinMISO MISO;
	typedef typename pins::PinMOSI MOSI;
	INLINE static void EnableClocks()   { Traits::EnableClocks(); }
	INLINE static void DisableClocks()  { Traits::DisableClocks(); }

	static const IRQn SPIx_IRQn  = Traits::SPIx_IRQn;
#if (!defined STM32TPL_STM32F1XX)
	static const PinAltFunction ALT_FUNC_SPIx = pins::ALT_FUNC_SPIx;
#endif
	typedef typename Traits::RxDmaStream RxDmaStream;
	typedef typename Traits::TxDmaStream TxDmaStream;
#if (defined STM32TPL_F2xxF4xx)
	enum { RX_DMA_CHANNEL   = Traits::RX_DMA_CHANNEL };
	enum { TX_DMA_CHANNEL   = Traits::TX_DMA_CHANNEL };
#elif (defined STM32TPL_STM32L0XX)
	static const typename RxDmaStream::ChannelSelection CH_SEL_SPIx_RX = Traits::CH_SEL_SPIx_RX;
	static const typename TxDmaStream::ChannelSelection CH_SEL_SPIx_TX = Traits::CH_SEL_SPIx_TX;
#endif

	using detail::DmaStrategy<props>::type::waitDmaDone;
	using detail::DmaStrategy<props>::type::initDmaInterrupt;
	using detail::DmaStrategy<props>::type::deinitDmaInterrupt;

	enum
	{
		SPIx_BASE               = Traits::SPIx_BASE,
		SPIx_REMAP              = Traits::SPIx_REMAP,
		BUS_FREQ                = Traits::BUS_FREQ
	};
public:
	void HwInit();
	void HwDeinit();

	Spi()
		: SpiBase(reinterpret_cast<SPI_TypeDef *const>(SPIx_BASE))
	{
		HwInit();
	}

	void SetActive(bool active)
	{
		active ? HwInit() : HwDeinit();
	}

	void BufRw(uint8_t * rxBuf, uint8_t const* txBuf, size_t cnt) override;
};

template<typename props, typename pins>
void Spi<props, pins>::HwInit()
{
#if (defined STM32TPL_STM32F1XX)
	if (REMAP)  // remap module if needed
		AFIO->MAPR |= SPIx_REMAP;
#endif

	EnableClocks();    // enable SPI module clock

	// configure pins
#if (defined STM32TPL_STM32F1XX)
	SCK::Mode(ALT_OUTPUT);
	MOSI::Mode(ALT_OUTPUT);
	MISO::Mode(INPUTPULLED);
#else
	SCK::Alternate(ALT_FUNC_SPIx);
	MOSI::Alternate(ALT_FUNC_SPIx);
	MISO::Alternate(ALT_FUNC_SPIx);

	SCK::Mode(ALT_OUTPUT);
	MOSI::Mode(ALT_OUTPUT);
	MISO::Mode(ALT_OUTPUT);
#endif

	// configure SPI
	SPIx->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD;
#if (defined SPI_CR2_FRXTH)
	SPIx->CR2 = SPI_CR2_FRXTH;
#else
	SPIx->CR2 = 0;
#endif
	SPIx->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | InitialDivisor | InitialCPHA | InitialCPOL;

	initDmaInterrupt();
}

template<typename props, typename pins>
void Spi<props, pins>::HwDeinit()
{
	RxDmaStream::DisableClocks();
	TxDmaStream::DisableClocks();
	deinitDmaInterrupt();

	SPIx->CR2 = 0;             // turn off SPI
	SPIx->CR1 = 0;

#if (defined STM32TPL_STM32F1XX)
	if (REMAP)                 // turn off remap
		AFIO->MAPR &= ~SPIx_REMAP;
#endif

	DisableClocks();           // disable SPI module clock

	SCK::Mode(INPUT);          // configure pins as inputs
	MOSI::Mode(INPUT);
	MISO::Mode(INPUT);
}

template<typename props, typename pins>
void Spi<props, pins>::BufRw(uint8_t * rxBuf, uint8_t const* txBuf, size_t cnt)
{
	bool const tx = txBuf != nullptr;
	bool const rx = rxBuf != nullptr;
	uint8_t txDummy = 0xFF;
	uint8_t rxDummy;

	RxDmaStream::EnableClocks();
	TxDmaStream::EnableClocks();
#if (defined STM32TPL_STM32L0XX)
	RxDmaStream::SelectChannel(CH_SEL_SPIx_RX);
	TxDmaStream::SelectChannel(CH_SEL_SPIx_TX);
#endif

	// clear all interrupts on RX DMA channel
	RxDmaStream::IFCR = RxDmaStream::DMA_MASK_ALL;

	// RX DMA stream : from DR to rxBuf
	RxDmaStream::PAR = (uint32_t)&SPIx->DR;
	RxDmaStream::MAR = rx ? (uint32_t)rxBuf : (uint32_t)&rxDummy;
	RxDmaStream::NDTR = cnt;
	RxDmaStream::CR = 0
			| DMA::DMA_CR_DIR_PERITH_TO_MEM  // From peripheral to memory
			| (rx ? DMA::DMA_CR_MINC : 0)    // Memory increment mode
			| DMA::DMA_CR_MSIZE_8_BIT        // Memory size
			| DMA::DMA_CR_PSIZE_8_BIT        // Peripheral size
			| DMA::DMA_CR_PRIO_HIGH          // priority
#if (defined STM32TPL_F2xxF4xx)
			| RX_DMA_CHANNEL                 // select channel (only for F4xx devices)
#endif
			;


	// clear all interrupts on TX DMA channel
	TxDmaStream::IFCR = TxDmaStream::DMA_MASK_ALL;	// TX DMA stream : from txBuf to DR

	// TX DMA stream : from txBuf to DR
	TxDmaStream::PAR = (uint32_t)&SPIx->DR;
	TxDmaStream::MAR = tx ? (uint32_t)txBuf : (uint32_t)&txDummy;
	TxDmaStream::NDTR = cnt;
	TxDmaStream::CR = 0
			| DMA::DMA_CR_DIR_MEM_TO_PERITH  // From memory to peripheral
			| (tx ? DMA::DMA_CR_MINC : 0)    // Memory increment mode
			| DMA::DMA_CR_MSIZE_8_BIT        // Memory size
			| DMA::DMA_CR_PSIZE_8_BIT        // Peripheral size
			| DMA::DMA_CR_PRIO_HIGH          // priority
#if (defined STM32TPL_F2xxF4xx)
			| TX_DMA_CHANNEL                 // select channel (only for F4xx devices)
#endif
			;

	// enable DMA channels
	RxDmaStream::Enable();
	TxDmaStream::Enable();

	// enable SPI DMA transfer
	SPIx->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

	// wait for RX DMA done
	waitDmaDone();

	// disable DMA channels
	TxDmaStream::Disable();
	RxDmaStream::Disable();

	// disable SPI DMA transfer
	SPIx->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
}

} // namespace SPI

} // namespace STM32

#endif // STM32TPL_STM32_SPI_H_INCLUDED
