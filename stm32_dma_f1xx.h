/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 2011-2014 Anton B. Gusev aka AHTOXA
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
 *  file         : stm32_dma_f1xx.h
 *  description  : DMA module class template for STM32F1xx series.
 *  created on   : 08.11.2011
 *
 */

#ifndef STM32TPL_STM32_DMA_F1XX_H_INCLUDED
#define STM32TPL_STM32_DMA_F1XX_H_INCLUDED

#include "stm32.h"
#include <stddef.h>

namespace STM32
{

namespace DMA
{

#if defined(RCC_AHBENR_DMA2EN)
#	define HAS_DMA2
#endif

/**
 * Enumeration for all DMA channels
 */
enum DmaChannelNum
{
	DMA1_CH1,
	DMA1_CH2,
	DMA1_CH3,
	DMA1_CH4,
	DMA1_CH5,
	DMA1_CH6,
	DMA1_CH7

#ifdef HAS_DMA2
	,
	DMA2_CH1,
	DMA2_CH2,
	DMA2_CH3,
	DMA2_CH4,
	DMA2_CH5
#endif
};

namespace
{

/**
 * DMA module registers.
 */
typedef struct
{
	volatile uint32_t ISR;    // DMA interrupt status register
	volatile uint32_t IFCR;   // DMA interrupt flag clear register
#if (defined STM32TPL_STM32L0XX)
	uint32_t unused[40];
	volatile uint32_t CSELR;  // DMA channel selection register (offset: 0xA8)
#endif
} DMAx_TypeDef;

/**
 * DMA Channel registers structure.
 */
typedef struct
{
	volatile uint32_t CCR;
	volatile uint32_t CNDTR;
	volatile uint32_t CPAR;
	volatile uint32_t CMAR;
} DMAx_Channel_TypeDef;

/**
 * DMA traits. Used internally.
 * Declares some constants, based on DMA number.
 */
template <int dmaNo> struct DmaTraits
{
	enum
	{
		DMA_NO = 1,
		DMAx_BASE = DMA1_BASE,
		RCC_AHBENR_DMAxEN = RCC_AHBENR_DMA1EN
	};
};

#ifdef HAS_DMA2
template<> struct DmaTraits<2>
{
	enum
	{
		DMA_NO = 2,
		DMAx_BASE = DMA2_BASE,
		RCC_AHBENR_DMAxEN = RCC_AHBENR_DMA2EN
	};
};
#endif

/**
 * DMA channel traits. Used internally.
 * Declares some constants, based on DMA channel number.
 */
template<DmaChannelNum chNum> struct DmaChannelTraits;

template<> struct DmaChannelTraits<DMA1_CH1>
{
	static const IRQn DMAChannel_IRQn = DMA1_Channel1_IRQn;
	enum
	{
		DMA_NO = 1,
		CHANNEL_NO = 1
	};
#if (defined STM32TPL_STM32L0XX)
	enum { CSELR_SHIFT = (CHANNEL_NO-1)*4 };
	enum class ChannelSelection : uint32_t  // possible CSELR values
	{
		CH_SEL_ADC             = (0x00 << CSELR_SHIFT),
		CH_SEL_TIM2_CH3        = (0x08 << CSELR_SHIFT),
		CH_SEL_AES_IN          = (0x0B << CSELR_SHIFT),
		CH_SEL_MASK            = (0x0F << CSELR_SHIFT),
	};
#endif
};


template<> struct DmaChannelTraits<DMA1_CH2>
{
#if (defined STM32TPL_STM32L0XX) || (defined STM32TPL_STM32F0XX)
	static const IRQn DMAChannel_IRQn = DMA1_Channel2_3_IRQn;
#else
	static const IRQn DMAChannel_IRQn = DMA1_Channel2_IRQn;
#endif

	enum
	{
		DMA_NO = 1,
		CHANNEL_NO = 2
	};
#if (defined STM32TPL_STM32L0XX)
	enum { CSELR_SHIFT = (CHANNEL_NO-1)*4 };
	enum class ChannelSelection : uint32_t  // possible CSELR values
	{
		CH_SEL_ADC             = (0x00 << CSELR_SHIFT),
		CH_SEL_SPI1_RX         = (0x01 << CSELR_SHIFT),
		CH_SEL_USART1_TX       = (0x03 << CSELR_SHIFT),
		CH_SEL_LPUART1_TX      = (0x05 << CSELR_SHIFT),
		CH_SEL_I2C1_TX         = (0x06 << CSELR_SHIFT),
		CH_SEL_TIM2_UP         = (0x08 << CSELR_SHIFT),
		CH_SEL_TIM6_UPDAC_CH1  = (0x09 << CSELR_SHIFT),
		CH_SEL_AES_OUT         = (0x0B << CSELR_SHIFT),
		CH_SEL_MASK            = (0x0F << CSELR_SHIFT),
	};
#endif
};

template<> struct DmaChannelTraits<DMA1_CH3>
{
#if (defined STM32TPL_STM32L0XX) || (defined STM32TPL_STM32F0XX)
	static const IRQn DMAChannel_IRQn = DMA1_Channel2_3_IRQn;
#else
	static const IRQn DMAChannel_IRQn = DMA1_Channel3_IRQn;
#endif
	enum
	{
		DMA_NO = 1,
		CHANNEL_NO = 3
	};
#if (defined STM32TPL_STM32L0XX)
	enum { CSELR_SHIFT = (CHANNEL_NO-1)*4 };
	enum class ChannelSelection : uint32_t  // possible CSELR values
	{
		CH_SEL_SPI1_TX         = (0x01 << CSELR_SHIFT),
		CH_SEL_USART1_RX       = (0x03 << CSELR_SHIFT),
		CH_SEL_LPUART1_RX      = (0x05 << CSELR_SHIFT),
		CH_SEL_I2C1_RX         = (0x06 << CSELR_SHIFT),
		CH_SEL_TIM2_CH2        = (0x08 << CSELR_SHIFT),
		CH_SEL_AES_OUT         = (0x0B << CSELR_SHIFT),
		CH_SEL_MASK            = (0x0F << CSELR_SHIFT),
	};
#endif
};

template<> struct DmaChannelTraits<DMA1_CH4>
{
#if (defined STM32TPL_STM32L0XX) || (defined STM32TPL_STM32F0XX)
	static const IRQn DMAChannel_IRQn = DMA1_Channel4_5_6_7_IRQn;
#else
	static const IRQn DMAChannel_IRQn = DMA1_Channel4_IRQn;
#endif
	enum
	{
		DMA_NO = 1,
		CHANNEL_NO = 4
	};
#if (defined STM32TPL_STM32L0XX)
	enum { CSELR_SHIFT = (CHANNEL_NO-1)*4 };
	enum class ChannelSelection : uint32_t  // possible CSELR values
	{
		CH_SEL_SPI2_RX         = (0x02 << CSELR_SHIFT),
		CH_SEL_USART1_TX       = (0x03 << CSELR_SHIFT),
		CH_SEL_USART2_TX       = (0x04 << CSELR_SHIFT),
		CH_SEL_I2C2_TX         = (0x07 << CSELR_SHIFT),
		CH_SEL_TIM2_CH4        = (0x08 << CSELR_SHIFT),
		CH_SEL_MASK            = (0x0F << CSELR_SHIFT),
	};
#endif
};

template<> struct DmaChannelTraits<DMA1_CH5>
{
#if (defined STM32TPL_STM32L0XX) || (defined STM32TPL_STM32F0XX)
	static const IRQn DMAChannel_IRQn = DMA1_Channel4_5_6_7_IRQn;
#else
	static const IRQn DMAChannel_IRQn = DMA1_Channel5_IRQn;
#endif
	enum
	{
		DMA_NO = 1,
		CHANNEL_NO = 5
	};
#if (defined STM32TPL_STM32L0XX)
	enum { CSELR_SHIFT = (CHANNEL_NO-1)*4 };
	enum class ChannelSelection : uint32_t  // possible CSELR values
	{
		CH_SEL_SPI2_TX         = (0x02 << CSELR_SHIFT),
		CH_SEL_USART1_RX       = (0x03 << CSELR_SHIFT),
		CH_SEL_USART2_RX       = (0x04 << CSELR_SHIFT),
		CH_SEL_I2C2_RX         = (0x07 << CSELR_SHIFT),
		CH_SEL_TIM2_CH1        = (0x08 << CSELR_SHIFT),
		CH_SEL_AES_IN          = (0x0B << CSELR_SHIFT),
		CH_SEL_MASK            = (0x0F << CSELR_SHIFT),
	};
#endif
};

template<> struct DmaChannelTraits<DMA1_CH6>
{
#if (defined STM32TPL_STM32L0XX) || (defined STM32TPL_STM32F0XX)
	static const IRQn DMAChannel_IRQn = DMA1_Channel4_5_6_7_IRQn;
#else
	static const IRQn DMAChannel_IRQn = DMA1_Channel6_IRQn;
#endif
	enum
	{
		DMA_NO = 1,
		CHANNEL_NO = 6
	};
#if (defined STM32TPL_STM32L0XX)
	enum { CSELR_SHIFT = (CHANNEL_NO-1)*4 };
	enum class ChannelSelection : uint32_t  // possible CSELR values
	{
		CH_SEL_SPI2_RX         = (0x02 << CSELR_SHIFT),
		CH_SEL_USART2_RX       = (0x04 << CSELR_SHIFT),
		CH_SEL_LPUART1_RX      = (0x05 << CSELR_SHIFT),
		CH_SEL_I2C1_TX         = (0x06 << CSELR_SHIFT),
		CH_SEL_MASK            = (0x0F << CSELR_SHIFT),
	};
#endif
};

template<> struct DmaChannelTraits<DMA1_CH7>
{
#if (defined STM32TPL_STM32L0XX) || (defined STM32TPL_STM32F0XX)
	static const IRQn DMAChannel_IRQn = DMA1_Channel4_5_6_7_IRQn;
#else
	static const IRQn DMAChannel_IRQn = DMA1_Channel7_IRQn;
#endif
	enum
	{
		DMA_NO = 1,
		CHANNEL_NO = 7
	};
#if (defined STM32TPL_STM32L0XX)
	enum { CSELR_SHIFT = (CHANNEL_NO-1)*4 };
	enum class ChannelSelection : uint32_t  // possible CSELR values
	{
		CH_SEL_SPI2_TX         = (0x02 << CSELR_SHIFT),
		CH_SEL_USART2_TX       = (0x04 << CSELR_SHIFT),
		CH_SEL_LPUART1_TX      = (0x05 << CSELR_SHIFT),
		CH_SEL_I2C1_RX         = (0x06 << CSELR_SHIFT),
		CH_SEL_TIM2_CH2_CH4    = (0x08 << CSELR_SHIFT),
		CH_SEL_MASK            = (0x0F << CSELR_SHIFT),
	};
#endif
};

#ifdef HAS_DMA2
template<> struct DmaChannelTraits<DMA2_CH1>
{
	static const IRQn DMAChannel_IRQn = DMA2_Channel1_IRQn;
	enum
	{
		DMA_NO = 2,
		CHANNEL_NO = 1
	};
};

template<> struct DmaChannelTraits<DMA2_CH2>
{
	static const IRQn DMAChannel_IRQn = DMA2_Channel2_IRQn;
	enum
	{
		DMA_NO = 2,
		CHANNEL_NO = 2
	};
};

template<> struct DmaChannelTraits<DMA2_CH3>
{
	static const IRQn DMAChannel_IRQn = DMA2_Channel3_IRQn;
	enum
	{
		DMA_NO = 2,
		CHANNEL_NO = 3
	};
};

template<> struct DmaChannelTraits<DMA2_CH4>
{
#if (defined STM32F10X_CL) || (defined STM32TPL_STM32L1XX)
	static const IRQn DMAChannel_IRQn = DMA2_Channel4_IRQn;
#else
	static const IRQn DMAChannel_IRQn = DMA2_Channel4_5_IRQn;
#endif
	enum
	{
		DMA_NO = 2,
		CHANNEL_NO = 4
	};
};

template<> struct DmaChannelTraits<DMA2_CH5>
{
#if (defined STM32F10X_CL) || (defined STM32TPL_STM32L1XX)
	static const IRQn DMAChannel_IRQn = DMA2_Channel5_IRQn;
#else
	static const IRQn DMAChannel_IRQn = DMA2_Channel4_5_IRQn;
#endif
	enum
	{
		DMA_NO = 2,
		CHANNEL_NO = 5
	};
};
#endif

/**
 * Helper template to calculate interrupt mask from DMA channel number
 */
template <int chan> struct DmaIntMask
{
	enum
	{
		DMA_MASK_ALL     = 0x0F << (4 * (chan - 1)),
		DMA_MASK_GIF     = 0x01 << (4 * (chan - 1)),
		DMA_MASK_TCIF    = 0x02 << (4 * (chan - 1)),
		DMA_MASK_HTIF    = 0x04 << (4 * (chan - 1)),
		DMA_MASK_TEIF    = 0x08 << (4 * (chan - 1))
	};
};

} // namespace

/**
 * CCR register masks
 */
enum
{
	DMA_CR_EN                = (1 << 0),    // DMA channel enable
	DMA_CR_TCIE              = (1 << 1),    // transfer complete interrupt enable
	DMA_CR_HTIE              = (1 << 2),    // half-transfer interrupt enable
	DMA_CR_TEIE              = (1 << 3),    // transfer error interrupt enable
	DMA_CR_DIR               = (1 << 4),    // direction flag:
	DMA_CR_DIR_PERITH_TO_MEM = (0 << 4),    // read from peripheral
	DMA_CR_DIR_MEM_TO_PERITH = (1 << 4),    // read from memory
	DMA_CR_CIRC              = (1 << 5),    // circular mode
	DMA_CR_PINC              = (1 << 6),    // peripheral increment mode
	DMA_CR_MINC              = (1 << 7),    // memory increment mode

	DMA_CR_PSIZE             = (3 << 8),    // peripheral size
	DMA_CR_PSIZE_8_BIT       = (0 << 8),
	DMA_CR_PSIZE_16_BIT      = (1 << 8),
	DMA_CR_PSIZE_32_BIT      = (2 << 8),

	DMA_CR_MSIZE             = (3 << 10),    // memory size
	DMA_CR_MSIZE_8_BIT       = (0 << 10),
	DMA_CR_MSIZE_16_BIT      = (1 << 10),
	DMA_CR_MSIZE_32_BIT      = (2 << 10),

	DMA_CR_PRIO              = (3 << 12),    // channel priority level
	DMA_CR_PRIO_LOW          = (0 << 12),
	DMA_CR_PRIO_MED          = (1 << 12),
	DMA_CR_PRIO_HIGH         = (2 << 12),
	DMA_CR_PRIO_HIGHEST      = (3 << 12),

	DMA_CR_MEM2MEM           = (1 << 14)
};


template<DmaChannelNum chNum>
class DmaChannel
{
private:
	typedef DmaChannelTraits<chNum> ChannelTraits;
public:
	/**
	 * Channel-specific constants
	 */
	static const IRQn DMAChannel_IRQn = ChannelTraits::DMAChannel_IRQn;
	enum
	{
		DMA_NO                   = ChannelTraits::DMA_NO,
		CHANNEL_NO               = ChannelTraits::CHANNEL_NO,
		DMAx_BASE                = DmaTraits<DMA_NO>::DMAx_BASE,
		CHANNEL_BASE             = DMAx_BASE + 0x08 + 0x14 * (CHANNEL_NO - 1),
		RCC_AHBENR_DMAxEN        = DmaTraits<DMA_NO>::RCC_AHBENR_DMAxEN
	};

	/**
	 * Channel-specific masks
	 */
	enum
	{
		DMA_MASK_ALL     = DmaIntMask<CHANNEL_NO>::DMA_MASK_ALL,    //<< mask for all flags
		DMA_MASK_GIF     = DmaIntMask<CHANNEL_NO>::DMA_MASK_GIF,    //<< mask for global interrupt flag
		DMA_MASK_TCIF    = DmaIntMask<CHANNEL_NO>::DMA_MASK_TCIF,   //<< mask for transfer complete interrupt flag
		DMA_MASK_HTIF    = DmaIntMask<CHANNEL_NO>::DMA_MASK_HTIF,   //<< mask for half-transfer interrupt flag
		DMA_MASK_TEIF    = DmaIntMask<CHANNEL_NO>::DMA_MASK_TEIF    //<< mask for transfer error interrupt flag
	};

#if (defined STM32TPL_STM32L0XX)
	using ChannelSelection = typename ChannelTraits::ChannelSelection;
	static void SelectChannel(ChannelSelection cs)
	{
		DMA->CSELR =
				(DMA->CSELR & ~static_cast<uint32_t>(ChannelSelection::CH_SEL_MASK))
				| static_cast<uint32_t>(cs);
	}
#endif

	/// Constructor.
	DmaChannel() {}

	static IOStruct<DMAx_BASE, DMAx_TypeDef> DMA;
	static IOStruct<CHANNEL_BASE, DMAx_Channel_TypeDef> Stream;

	// DMA registers
	static IORegister<DMAx_BASE + offsetof(DMAx_TypeDef, ISR)> ISR;
	static IORegister<DMAx_BASE + offsetof(DMAx_TypeDef, IFCR)> IFCR;

	// Channel registers
	static IORegister<CHANNEL_BASE + offsetof(DMAx_Channel_TypeDef, CCR)>   CR;
	static IORegister<CHANNEL_BASE + offsetof(DMAx_Channel_TypeDef, CNDTR)> NDTR;
	static IORegister<CHANNEL_BASE + offsetof(DMAx_Channel_TypeDef, CPAR)>  PAR;
	static IORegister<CHANNEL_BASE + offsetof(DMAx_Channel_TypeDef, CMAR)>  MAR;

	static void EnableClocks()     { RCC->AHBENR |= RCC_AHBENR_DMAxEN;  __DSB(); }
	static void DisableClocks()    { RCC->AHBENR &= ~RCC_AHBENR_DMAxEN; __DSB(); }
	static void Enable()           { CR |= DMA_CR_EN; }
	static void Disable()          { CR &= ~DMA_CR_EN; }
	static bool Enabled()          { return CR & DMA_CR_EN; }
};

typedef DmaChannel<DMA1_CH1> Dma1Channel1;
typedef DmaChannel<DMA1_CH2> Dma1Channel2;
typedef DmaChannel<DMA1_CH3> Dma1Channel3;
typedef DmaChannel<DMA1_CH4> Dma1Channel4;
typedef DmaChannel<DMA1_CH5> Dma1Channel5;
typedef DmaChannel<DMA1_CH6> Dma1Channel6;
typedef DmaChannel<DMA1_CH7> Dma1Channel7;

#ifdef HAS_DMA2
typedef DmaChannel<DMA2_CH1> Dma2Channel1;
typedef DmaChannel<DMA2_CH2> Dma2Channel2;
typedef DmaChannel<DMA2_CH3> Dma2Channel3;
typedef DmaChannel<DMA2_CH4> Dma2Channel4;
typedef DmaChannel<DMA2_CH5> Dma2Channel5;
#endif

}  // namespace DMA

}  // namespace STM32

#endif // STM32TPL_STM32_DMA_F1XX_H_INCLUDED
