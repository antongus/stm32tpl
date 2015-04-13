/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 2010-2015 Anton B. Gusev aka AHTOXA
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
#include "stm32_dma.h"
#include <scmRTOS.h>

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
	REMAP_NONE = 0,      //!< no remap
	REMAP_FULL           //!< remap
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
		SPIx->DR = out;
		while (!(SPIx->SR & SPI_SR_RXNE)) ;
		return SPIx->DR;
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
	virtual void BufRw(uint8_t * rxbuf, uint8_t const* txBuf, size_t cnt) = 0;
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
#if (defined F2xxF4xx)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_SPI1;
#elif (defined STM32L0XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_0;
#endif
};

template<> struct SpiPins<SPI_1, REMAP_FULL>
{
	typedef Pin<'B', 3> PinSCK;
	typedef Pin<'B', 4> PinMISO;
	typedef Pin<'B', 5> PinMOSI;
#if (defined F2xxF4xx)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_SPI1;
#elif (defined STM32L0XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_0;
#endif
};

#if (defined RCC_APB1ENR_SPI2EN)
template<> struct SpiPins<SPI_2>
{
	typedef Pin<'B', 13> PinSCK;
	typedef Pin<'B', 14> PinMISO;
	typedef Pin<'B', 15> PinMOSI;
#if (defined F2xxF4xx)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_SPI2;
#elif (defined STM32L0XX)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_0;
#endif
};
#endif

#if (defined RCC_APB1ENR_SPI3EN)
template<> struct SpiPins<SPI_3>
{
	typedef Pin<'B', 3> PinSCK;
	typedef Pin<'B', 4> PinMISO;
	typedef Pin<'B', 5> PinMOSI;
#if (defined F2xxF4xx)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_SPI3;
#endif
};

template<> struct SpiPins<SPI_3, REMAP_FULL>
{
	typedef Pin<'C', 10> PinSCK;
	typedef Pin<'C', 11> PinMISO;
	typedef Pin<'C', 12> PinMOSI;
#if (defined F2xxF4xx)
	static const PinAltFunction ALT_FUNC_SPIx = ALT_FUNC_SPI3;
#endif
};
#endif

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

#if (defined F2xxF4xx)
	enum { RX_DMA_CHANNEL = DMA::DMA_CR_CHSEL_CH3 };
	enum { TX_DMA_CHANNEL = DMA::DMA_CR_CHSEL_CH3 };

	typedef DMA::Dma2Channel2 RxDmaStream;
	typedef DMA::Dma2Channel3 TxDmaStream;
#elif (defined STM32L0XX)
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
#if (defined F2xxF4xx)
	enum { RX_DMA_CHANNEL = DMA::DMA_CR_CHSEL_CH0 };
	enum { TX_DMA_CHANNEL = DMA::DMA_CR_CHSEL_CH0 };

	typedef DMA::Dma1Channel3 RxDmaStream;
	typedef DMA::Dma1Channel4 TxDmaStream;
#elif (defined STM32L0XX)
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

#if (defined F2xxF4xx)
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
};

/**
*  SPI template class.
*/
template<typename props>
class Spi: public SpiBase
{
public:
	static const SpiNum   NUMBER    = props::NUMBER;
	static const Remap    REMAP     = props::REMAP;
private:
	static const Divisor  InitialDivisor   = props::InitialDivisor;
	static const Cpol     InitialCPOL      = props::InitialCPOL;
	static const Cpha     InitialCPHA      = props::InitialCPHA;

	typedef SpiTraits<NUMBER> Traits;
	typedef SpiPins<NUMBER, REMAP> pins;
	typedef typename pins::PinSCK SCK;
	typedef typename pins::PinMISO MISO;
	typedef typename pins::PinMOSI MOSI;
	INLINE static void EnableClocks()   { Traits::EnableClocks(); }
	INLINE static void DisableClocks()  { Traits::DisableClocks(); }

	static const IRQn SPIx_IRQn  = Traits::SPIx_IRQn;
#if (!defined STM32F1XX)
	static const PinAltFunction ALT_FUNC_SPIx = pins::ALT_FUNC_SPIx;
#endif
	typedef typename Traits::RxDmaStream RxDmaStream;
	typedef typename Traits::TxDmaStream TxDmaStream;
#if (defined F2xxF4xx)
	enum { RX_DMA_CHANNEL   = Traits::RX_DMA_CHANNEL };
	enum { TX_DMA_CHANNEL   = Traits::TX_DMA_CHANNEL };
#elif (defined STM32L0XX)
	static const typename RxDmaStream::ChannelSelection CH_SEL_SPIx_RX = Traits::CH_SEL_SPIx_RX;
	static const typename TxDmaStream::ChannelSelection CH_SEL_SPIx_TX = Traits::CH_SEL_SPIx_TX;
#endif

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

	void BufRw(uint8_t * rxbuf, uint8_t const* txBuf, size_t cnt) override;
};

template<typename props>
void Spi<props>::HwInit()
{
#if (defined STM32F1XX)
	if (REMAP)  // remap module if needed
		AFIO->MAPR |= SPIx_REMAP;
#endif

	EnableClocks();    // enable SPI module clock

	// configure pins
#if (defined STM32F1XX)
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
	SPIx->CR2 = 0;
	SPIx->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | InitialDivisor | InitialCPHA | InitialCPOL;
}

template<typename props>
void Spi<props>::HwDeinit()
{
	SPIx->CR2 = 0;             // turn off SPI
	SPIx->CR1 = 0;

#if (defined STM32F1XX)
	if (REMAP)                 // turn off remap
		AFIO->MAPR &= ~SPIx_REMAP;
#endif

	DisableClocks();           // disable SPI module clock

	SCK::Mode(INPUT);          // configure pins as inputs
	MOSI::Mode(INPUT);
	MISO::Mode(INPUT);
}

template<typename props>
void Spi<props>::BufRw(uint8_t * rxBuf, uint8_t const* txBuf, size_t cnt)
{
	RxDmaStream::EnableClocks();
	TxDmaStream::EnableClocks();
#if (defined STM32L0XX)
	RxDmaStream::SelectChannel(CH_SEL_SPIx_RX);
	TxDmaStream::SelectChannel(CH_SEL_SPIx_TX);
#endif

	// clear all interrupts on RX DMA channel
	RxDmaStream::IFCR = RxDmaStream::DMA_MASK_ALL;

	// RX DMA stream : from DR to rxBuf
	RxDmaStream::PAR = (uint32_t)&SPIx->DR;
	RxDmaStream::MAR = (uint32_t)rxBuf;
	RxDmaStream::NDTR = cnt;
	RxDmaStream::CR = 0
			| DMA::DMA_CR_DIR_PERITH_TO_MEM  // From peripheral to memory
			| DMA::DMA_CR_MINC               // Memory increment mode
			| DMA::DMA_CR_MSIZE_8_BIT        // Memory size
			| DMA::DMA_CR_PSIZE_8_BIT        // Peripheral size
			| DMA::DMA_CR_PRIO_HIGH          // priority
#if (defined F2xxF4xx)
			| RX_DMA_CHANNEL                 // select channel (only for F4xx devices)
#endif
			;


	// TX DMA stream : from txBuf to DR
	TxDmaStream::PAR = (uint32_t)&SPIx->DR;
	TxDmaStream::MAR = (uint32_t)txBuf;
	TxDmaStream::NDTR = cnt;
	TxDmaStream::CR = 0
			| DMA::DMA_CR_DIR_MEM_TO_PERITH  // From memory to peripheral
			| DMA::DMA_CR_MINC               // Memory increment mode
			| DMA::DMA_CR_MSIZE_8_BIT        // Memory size
			| DMA::DMA_CR_PSIZE_8_BIT        // Peripheral size
			| DMA::DMA_CR_PRIO_HIGH          // priority
#if (defined F2xxF4xx)
			| TX_DMA_CHANNEL                 // select channel (only for F4xx devices)
#endif
			;

	// enable DMA channels
	RxDmaStream::Enable();
	TxDmaStream::Enable();

	// enable SPI DMA transfer
	SPIx->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

	while (!(RxDmaStream::ISR & RxDmaStream::DMA_MASK_TCIF)) ;

	// disable DMA channels
	TxDmaStream::Disable();
	RxDmaStream::Disable();

	// disable SPI DMA transfer
	SPIx->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
}

} // namespace SPI

} // namespace STM32

#endif // STM32TPL_STM32_SPI_H_INCLUDED
