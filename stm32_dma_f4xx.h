/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 2013-2014 Anton B. Gusev aka AHTOXA
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
 *  file         : stm32_dma_f4xx.h
 *  description  : DMA module class template for STM32F4xx series.
 *  created on   : 07.06.2013
 *
 */

#ifndef STM32TPL_STM32_DMA_F4XX_H_INCLUDED
#define STM32TPL_STM32_DMA_F4XX_H_INCLUDED

#include "stm32.h"

namespace STM32
{

namespace DMA
{

/**
 * Enumeration for all DMA Streams
 */
enum DmaChannelNum
{
	DMA1_CH0,
	DMA1_CH1,
	DMA1_CH2,
	DMA1_CH3,
	DMA1_CH4,
	DMA1_CH5,
	DMA1_CH6,
	DMA1_CH7,

	DMA2_CH0,
	DMA2_CH1,
	DMA2_CH2,
	DMA2_CH3,
	DMA2_CH4,
	DMA2_CH5,
	DMA2_CH6,
	DMA2_CH7
};

namespace
{

/**
 * DMA module registers.
 */
typedef struct
{
	volatile uint32_t LISR;   // DMA low interrupt status register
	volatile uint32_t HISR;   // DMA high interrupt status register
	volatile uint32_t LIFCR;  // DMA low interrupt flag clear register
	volatile uint32_t HIFCR;  // DMA high interrupt flag clear register
} DMAx_TypeDef;

/**
 * DMA Stream registers structure.
 */
typedef struct
{
	volatile uint32_t CR;     // DMA stream configuration register
	volatile uint32_t NDTR;   // DMA stream number of data register
	volatile uint32_t PAR;    // DMA stream peripheral address register
	volatile uint32_t MAR;    // DMA stream memory 0 address register
	volatile uint32_t M1AR;   // DMA stream memory 1 address register
	volatile uint32_t FCR;    // DMA stream FIFO control register
}DMAx_Stream_TypeDef;

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
		RCC_AHBENR_DMAxEN = RCC_AHB1ENR_DMA1EN
	};
};

template<> struct DmaTraits<2>
{
	enum
	{
		DMA_NO = 2,
		DMAx_BASE = DMA2_BASE,
		RCC_AHBENR_DMAxEN = RCC_AHB1ENR_DMA2EN
	};
};

/**
 * DMA stream traits. Used internally.
 * Declares some constants, based on DMA stream number.
 */
template<DmaChannelNum chNum> struct DmaStreamTraits;

template<> struct DmaStreamTraits<DMA1_CH0>
{
	static const IRQn DMAChannel_IRQn = DMA1_Stream0_IRQn;
	enum
	{
		DMA_NO = 1,
		CHANNEL_NO = 0,
		MASK_SHIFT = 0
	};
};

template<> struct DmaStreamTraits<DMA1_CH1>
{
	static const IRQn DMAChannel_IRQn = DMA1_Stream1_IRQn;
	enum
	{
		DMA_NO = 1,
		CHANNEL_NO = 1,
		MASK_SHIFT = 6
	};
};

template<> struct DmaStreamTraits<DMA1_CH2>
{
	static const IRQn DMAChannel_IRQn = DMA1_Stream2_IRQn;
	enum
	{
		DMA_NO = 1,
		CHANNEL_NO = 2,
		MASK_SHIFT = 16
	};
};

template<> struct DmaStreamTraits<DMA1_CH3>
{
	static const IRQn DMAChannel_IRQn = DMA1_Stream3_IRQn;
	enum
	{
		DMA_NO = 1,
		CHANNEL_NO = 3,
		MASK_SHIFT = 22
	};
};

template<> struct DmaStreamTraits<DMA1_CH4>
{
	static const IRQn DMAChannel_IRQn = DMA1_Stream4_IRQn;
	enum
	{
		DMA_NO = 1,
		CHANNEL_NO = 4,
		MASK_SHIFT = 0
	};
};

template<> struct DmaStreamTraits<DMA1_CH5>
{
	static const IRQn DMAChannel_IRQn = DMA1_Stream5_IRQn;
	enum
	{
		DMA_NO = 1,
		CHANNEL_NO = 5,
		MASK_SHIFT = 6
	};
};

template<> struct DmaStreamTraits<DMA1_CH6>
{
	static const IRQn DMAChannel_IRQn = DMA1_Stream6_IRQn;
	enum
	{
		DMA_NO = 1,
		CHANNEL_NO = 6,
		MASK_SHIFT = 16
	};
};

template<> struct DmaStreamTraits<DMA1_CH7>
{
	static const IRQn DMAChannel_IRQn = DMA1_Stream7_IRQn;
	enum
	{
		DMA_NO = 1,
		CHANNEL_NO = 7,
		MASK_SHIFT = 22
	};
};

template<> struct DmaStreamTraits<DMA2_CH0>
{
	static const IRQn DMAChannel_IRQn = DMA2_Stream0_IRQn;
	enum
	{
		DMA_NO = 2,
		CHANNEL_NO = 0,
		MASK_SHIFT = 0
	};
};

template<> struct DmaStreamTraits<DMA2_CH1>
{
	static const IRQn DMAChannel_IRQn = DMA2_Stream1_IRQn;
	enum
	{
		DMA_NO = 2,
		CHANNEL_NO = 1,
		MASK_SHIFT = 6
	};
};

template<> struct DmaStreamTraits<DMA2_CH2>
{
	static const IRQn DMAChannel_IRQn = DMA2_Stream2_IRQn;
	enum
	{
		DMA_NO = 2,
		CHANNEL_NO = 2,
		MASK_SHIFT = 16
	};
};

template<> struct DmaStreamTraits<DMA2_CH3>
{
	static const IRQn DMAChannel_IRQn = DMA2_Stream3_IRQn;
	enum
	{
		DMA_NO = 2,
		CHANNEL_NO = 3,
		MASK_SHIFT = 22
	};
};

template<> struct DmaStreamTraits<DMA2_CH4>
{
	static const IRQn DMAChannel_IRQn = DMA2_Stream4_IRQn;
	enum
	{
		DMA_NO = 2,
		CHANNEL_NO = 4,
		MASK_SHIFT = 0
	};
};

template<> struct DmaStreamTraits<DMA2_CH5>
{
	static const IRQn DMAChannel_IRQn = DMA2_Stream5_IRQn;
	enum
	{
		DMA_NO = 2,
		CHANNEL_NO = 5,
		MASK_SHIFT = 6
	};
};

template<> struct DmaStreamTraits<DMA2_CH6>
{
	static const IRQn DMAChannel_IRQn = DMA2_Stream6_IRQn;
	enum
	{
		DMA_NO = 2,
		CHANNEL_NO = 6,
		MASK_SHIFT = 16
	};
};

template<> struct DmaStreamTraits<DMA2_CH7>
{
	static const IRQn DMAChannel_IRQn = DMA2_Stream7_IRQn;
	enum
	{
		DMA_NO = 2,
		CHANNEL_NO = 7,
		MASK_SHIFT = 22
	};
};

template <bool IsHi> struct LoHiSelector {
	enum { OFFSET   = 0 };
};

template <> struct LoHiSelector<true> {
	enum { OFFSET   = 4 };
};


} // namespace


/**
 * CR register masks
 */
enum
{
	DMA_CR_EN                       = (1 << 0),     // DMA stream enable
	DMA_CR_DMEIE                    = (1 << 1),     // direct mode error interrupt enable
	DMA_CR_TEIE                     = (1 << 2),     // transfer error interrupt enable
	DMA_CR_HTIE                     = (1 << 3),     // half transfer interrupt enable
	DMA_CR_TCIE                     = (1 << 4),     // transfer complete interrupt enable
	DMA_CR_PFCTRL                   = (1 << 5),     // peripheral flow controller. (The flow controller is DMA(0)/peripheral(1))

	DMA_CR_DIR                      = (3 << 6),     // direction flag:
		DMA_CR_DIR_PERITH_TO_MEM    = (0 << 6),     // peripheral-to-memory
		DMA_CR_DIR_MEM_TO_PERITH    = (1 << 6),     // memory-to-peripheral
		DMA_CR_DIR_MEM_TO_MEM       = (2 << 6),     // memory-to-memory

	DMA_CR_CIRC                     = (1 << 8),     // circular mode
	DMA_CR_PINC                     = (1 << 9),     // peripheral increment mode
	DMA_CR_MINC                     = (1 << 10),    // memory increment mode

	DMA_CR_PSIZE                    = (3 << 11),    // peripheral size
		DMA_CR_PSIZE_8_BIT          = (0 << 11),
		DMA_CR_PSIZE_16_BIT         = (1 << 11),
		DMA_CR_PSIZE_32_BIT         = (2 << 11),

	DMA_CR_MSIZE                    = (3 << 13),    // memory size
		DMA_CR_MSIZE_8_BIT          = (0 << 13),
		DMA_CR_MSIZE_16_BIT         = (1 << 13),
		DMA_CR_MSIZE_32_BIT         = (2 << 13),

	DMA_CR_PINCOS                   = (3 << 15),    // peripheral increment offset size (0 - as PSIZE, 1 - 32bit)

	DMA_CR_PRIO                     = (3 << 16),    // stream priority level
		DMA_CR_PRIO_LOW             = (0 << 16),
		DMA_CR_PRIO_MED             = (1 << 16),
		DMA_CR_PRIO_HIGH            = (2 << 16),
		DMA_CR_PRIO_HIGHEST         = (3 << 16),

	DMA_CR_DBM                      = (1 << 18),    // double buffer mode
	DMA_CR_CT                       = (1 << 19),    // current target (0 = Mem0, 1 = Mem1)

	DMA_CR_PBURST                   = (3 << 21),    // peripheral burst transfer configuration
		DMA_CR_PBURST_SINGLE        = (0 << 21),
		DMA_CR_PBURST_INCR4         = (1 << 21),
		DMA_CR_PBURST_INCR8         = (2 << 21),
		DMA_CR_PBURST_INCR16        = (3 << 21),

	DMA_CR_MBURST                   = (3 << 23),    // memory burst transfer configuration
		DMA_CR_MBURST_SINGLE        = (0 << 23),
		DMA_CR_MBURST_INCR4         = (1 << 23),
		DMA_CR_MBURST_INCR8         = (2 << 23),
		DMA_CR_MBURST_INCR16        = (3 << 23),

	DMA_CR_CHSEL                    = (7 << 25),    // channel selection
		DMA_CR_CHSEL_CH0            = (0 << 25),
		DMA_CR_CHSEL_CH1            = (1 << 25),
		DMA_CR_CHSEL_CH2            = (2 << 25),
		DMA_CR_CHSEL_CH3            = (3 << 25),
		DMA_CR_CHSEL_CH4            = (4 << 25),
		DMA_CR_CHSEL_CH5            = (5 << 25),
		DMA_CR_CHSEL_CH6            = (6 << 25),
		DMA_CR_CHSEL_CH7            = (7 << 25)
};

template<DmaChannelNum chNum>
class DmaStream
{
private:
	typedef DmaStreamTraits<chNum> StreamTraits;
public:
	/**
	 * Stream-specific constants
	 */
	static const IRQn DMAChannel_IRQn = StreamTraits::DMAChannel_IRQn;
	enum
	{
		DMA_NO                   = StreamTraits::DMA_NO,
		CHANNEL_NO               = StreamTraits::CHANNEL_NO,
		DMAx_BASE                = DmaTraits<DMA_NO>::DMAx_BASE,
		CHANNEL_BASE             = DMAx_BASE + 0x10 + 0x18 * CHANNEL_NO,
		RCC_AHBENR_DMAxEN        = DmaTraits<DMA_NO>::RCC_AHBENR_DMAxEN,
		LO_HI_OFFSET             = LoHiSelector<(CHANNEL_NO > 3)>::OFFSET
	};

	/**
	 * Stream-specific masks
	 */
	enum
	{
		MASK_SHIFT       = StreamTraits::MASK_SHIFT,
		DMA_MASK_ALL     = 0x3D << MASK_SHIFT,
		DMA_MASK_FEIF    = 0x01 << MASK_SHIFT,    // FIFO error
		DMA_MASK_DMEIF   = 0x04 << MASK_SHIFT,    // direct mode error
		DMA_MASK_TEIF    = 0x08 << MASK_SHIFT,    // transfer error
		DMA_MASK_HTIF    = 0x10 << MASK_SHIFT,    // half transfer
		DMA_MASK_TCIF    = 0x20 << MASK_SHIFT     // transfer complete
	};

	/// Constructor.
	DmaStream() {}

	static IOStruct<DMAx_BASE, DMAx_TypeDef> DMA;
	static IOStruct<CHANNEL_BASE, DMAx_Stream_TypeDef> Stream;

	// DMA registers
	static IORegister<DMAx_BASE + offsetof(DMAx_TypeDef, LISR) + LO_HI_OFFSET> ISR;
	static IORegister<DMAx_BASE + offsetof(DMAx_TypeDef, LIFCR) + LO_HI_OFFSET> IFCR;

	// Stream registers
	static IORegister<CHANNEL_BASE + offsetof(DMAx_Stream_TypeDef, CR)>   CR;
	static IORegister<CHANNEL_BASE + offsetof(DMAx_Stream_TypeDef, FCR)>  FCR;
	static IORegister<CHANNEL_BASE + offsetof(DMAx_Stream_TypeDef, NDTR)> NDTR;
	static IORegister<CHANNEL_BASE + offsetof(DMAx_Stream_TypeDef, PAR)>  PAR;
	static IORegister<CHANNEL_BASE + offsetof(DMAx_Stream_TypeDef, MAR)>  MAR;
	static IORegister<CHANNEL_BASE + offsetof(DMAx_Stream_TypeDef, M1AR)> M1AR;

	static void EnableClocks()     { RCC->AHB1ENR |= RCC_AHBENR_DMAxEN;  __DSB(); }
	static void DisableClocks()    { RCC->AHB1ENR &= ~RCC_AHBENR_DMAxEN; __DSB(); }
	static void Enable()           { CR |= DMA_CR_EN; }
	static void Disable()          { CR &= ~DMA_CR_EN; }
	static bool Enabled()          { return CR & DMA_CR_EN; }
};

typedef DmaStream<DMA1_CH0> Dma1Channel0;
typedef DmaStream<DMA1_CH1> Dma1Channel1;
typedef DmaStream<DMA1_CH2> Dma1Channel2;
typedef DmaStream<DMA1_CH3> Dma1Channel3;
typedef DmaStream<DMA1_CH4> Dma1Channel4;
typedef DmaStream<DMA1_CH5> Dma1Channel5;
typedef DmaStream<DMA1_CH6> Dma1Channel6;
typedef DmaStream<DMA1_CH7> Dma1Channel7;

typedef DmaStream<DMA2_CH0> Dma2Channel0;
typedef DmaStream<DMA2_CH1> Dma2Channel1;
typedef DmaStream<DMA2_CH2> Dma2Channel2;
typedef DmaStream<DMA2_CH3> Dma2Channel3;
typedef DmaStream<DMA2_CH4> Dma2Channel4;
typedef DmaStream<DMA2_CH5> Dma2Channel5;
typedef DmaStream<DMA2_CH6> Dma2Channel6;
typedef DmaStream<DMA2_CH7> Dma2Channel7;

}  // namespace DMA

}  // namespace STM32

#endif // STM32TPL_STM32_DMA_F4XX_H_INCLUDED
