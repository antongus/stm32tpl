/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *  Visit https://github.com/antongus/stm32tpl for new versions
 *
 *  Copyright (c) 2011-2023 Anton B. Gusev
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
 *  file         : mm32_adc.h
 *  description  : MM32 ADC module
 */

#pragma once

#include "mm32.h"

namespace MM32
{

namespace ADC
{

struct ADCx_TypeDef
{
	volatile uint32_t ADDATA;        //!< data register
	volatile uint32_t ADCFG;         //!< configuration register
	volatile uint32_t ADCR;          //!< control register
	volatile uint32_t ADCHS;         //!< channel select register
	volatile uint32_t ADCMPR;        //!< window compare register
	volatile uint32_t ADSTA;         //!< status register
	volatile uint32_t ADDR[16];      //!< data registers (channels 0..9, 14..15)
	volatile uint32_t ADSTA_EXT;     //!< extended status
};

/**
 * ADC bit definitions
 */
enum Bits
{
	/// ADC_ADCFG, configuration register
	ADC_ADCFG_ADEN          = 1UL << 0,    //!< ADC enable
	ADC_ADCFG_ADWEN         = 1UL << 1,    //!< A/D window comparator enabled
	ADC_ADCFG_TSEN          = 1UL << 2,    //!< Temperature sensor enabled
	ADC_ADCFG_VSEN          = 1UL << 3,    //!< Internal voltage sensor enabled

	ADC_ADCFG_ADCPRE_POS    = 4,           //!< ADC prescaler (3 bit)
	ADC_ADCFG_ADCPRE_POS2   = 14,          //!< ADC prescaler (1 bit, lsb)
	ADC_ADCFG_ADCPRE        = 0x7 << ADC_ADCFG_ADCPRE_POS | 1 << ADC_ADCFG_ADCPRE_POS2,

	ADC_ADCFG_RSL_POS       = 7,    //!< ADC resolution (3 bit)
	ADC_ADCFG_RSL           = 7UL << ADC_ADCFG_RSL_POS,
	ADC_ADCFG_RSL_12BIT     = 0UL << ADC_ADCFG_RSL_POS,
	ADC_ADCFG_RSL_11BIT     = 1UL << ADC_ADCFG_RSL_POS,
	ADC_ADCFG_RSL_10BIT     = 2UL << ADC_ADCFG_RSL_POS,
	ADC_ADCFG_RSL_9BIT      = 3UL << ADC_ADCFG_RSL_POS,
	ADC_ADCFG_RSL_8BIT      = 4UL << ADC_ADCFG_RSL_POS,

	ADC_ADCFG_SAMCTL_POS    = 10,    //!< sample time (4 bit)
	ADC_ADCFG_SAMCTL        = 0xFUL << ADC_ADCFG_SAMCTL_POS,
	ADC_ADCFG_SAMCTL_1_5    = 0UL  << ADC_ADCFG_SAMCTL_POS,
	ADC_ADCFG_SAMCTL_7_5    = 1UL  << ADC_ADCFG_SAMCTL_POS,
	ADC_ADCFG_SAMCTL_13_5   = 2UL  << ADC_ADCFG_SAMCTL_POS,
	ADC_ADCFG_SAMCTL_28_5   = 3UL  << ADC_ADCFG_SAMCTL_POS,
	ADC_ADCFG_SAMCTL_41_5   = 4UL  << ADC_ADCFG_SAMCTL_POS,
	ADC_ADCFG_SAMCTL_55_5   = 5UL  << ADC_ADCFG_SAMCTL_POS,
	ADC_ADCFG_SAMCTL_71_5   = 6UL  << ADC_ADCFG_SAMCTL_POS,
	ADC_ADCFG_SAMCTL_239_5  = 7UL  << ADC_ADCFG_SAMCTL_POS,
	ADC_ADCFG_SAMCTL_2_5    = 8UL  << ADC_ADCFG_SAMCTL_POS,
	ADC_ADCFG_SAMCTL_3_5    = 9UL  << ADC_ADCFG_SAMCTL_POS,
	ADC_ADCFG_SAMCTL_4_5    = 10UL << ADC_ADCFG_SAMCTL_POS,
	ADC_ADCFG_SAMCTL_5_5    = 11UL << ADC_ADCFG_SAMCTL_POS,
	ADC_ADCFG_SAMCTL_6_5    = 12UL << ADC_ADCFG_SAMCTL_POS,

	// ADC_ADCR, control register
	ADC_ADCR_ADIE           = 1UL << 0,    //!< ADC interrupt enable
	ADC_ADCR_ADWIE          = 1UL << 1,    //!< ADC window comparator interrupt enable
	ADC_ADCR_TRGEN          = 1UL << 2,    //!< External trigger enable
	ADC_ADCR_DMAEN          = 1UL << 3,    //!< DMA enable

	ADC_ADCR_TRGSEL_POS1    = 4,           //!< External trigger selection, bits[18:17,6:4]
	ADC_ADCR_TRGSEL_POS2    = 17,
	ADC_ADCR_TRGSEL         = (7UL << ADC_ADCR_TRGSEL_POS1) | (3UL << ADC_ADCR_TRGSEL_POS2),
	ADC_ADCR_TRGSEL_TIM1_CC1    = 0UL << ADC_ADCR_TRGSEL_POS1,
	ADC_ADCR_TRGSEL_TIM1_CC2    = 1UL << ADC_ADCR_TRGSEL_POS1,
	ADC_ADCR_TRGSEL_TIM1_CC3    = 2UL << ADC_ADCR_TRGSEL_POS1,
	ADC_ADCR_TRGSEL_TIM2_CC2    = 3UL << ADC_ADCR_TRGSEL_POS1,
	ADC_ADCR_TRGSEL_TIM3_TRGO   = 4UL << ADC_ADCR_TRGSEL_POS1,
	ADC_ADCR_TRGSEL_TIM1_CC45   = 5UL << ADC_ADCR_TRGSEL_POS1,
	ADC_ADCR_TRGSEL_TIM3_CC1    = 6UL << ADC_ADCR_TRGSEL_POS1,
	ADC_ADCR_TRGSEL_TIM3_EXTI11 = 7UL << ADC_ADCR_TRGSEL_POS1,
	ADC_ADCR_TRGSEL_TIM1_TRGO   = 1UL << ADC_ADCR_TRGSEL_POS2,
	ADC_ADCR_TRGSEL_TIM2_CC1    = 1UL << ADC_ADCR_TRGSEL_POS2 | 3UL << ADC_ADCR_TRGSEL_POS1,
	ADC_ADCR_TRGSEL_TIM3_CC4    = 1UL << ADC_ADCR_TRGSEL_POS2 | 4UL << ADC_ADCR_TRGSEL_POS1,
	ADC_ADCR_TRGSEL_TIM2_TRGO   = 1UL << ADC_ADCR_TRGSEL_POS2 | 5UL << ADC_ADCR_TRGSEL_POS1,
	ADC_ADCR_TRGSEL_TIM2_EXTI15 = 1UL << ADC_ADCR_TRGSEL_POS2 | 7UL << ADC_ADCR_TRGSEL_POS1,
	ADC_ADCR_TRGSEL_TIM1_CC4    = 2UL << ADC_ADCR_TRGSEL_POS2,
	ADC_ADCR_TRGSEL_TIM1_CC5    = 2UL << ADC_ADCR_TRGSEL_POS2 | 1UL << ADC_ADCR_TRGSEL_POS1,

	ADC_ADCR_ADST           = 1UL << 8,    //!< ADC start

	ADC_ADCR_ADMD_POS       = 9,           //!< ADC mode (2 bit)
	ADC_ADCR_ADMD           = 3UL << ADC_ADCR_ADMD_POS,
	ADC_ADCR_ADMD_SINGLE    = 0UL << ADC_ADCR_ADMD_POS,  //!< single channel (one time)
	ADC_ADCR_ADMD_SEQ       = 1UL << ADC_ADCR_ADMD_POS,  //!< sequence of channels (one time)
	ADC_ADCR_ADMD_CONT      = 2UL << ADC_ADCR_ADMD_POS,  //!< continous sequence (cycle)

	ADC_ADCR_ALIGN_RIGHT    = 0UL << 11,    //!< data alignment
	ADC_ADCR_ALIGN_LEFT     = 1UL << 11,    //!< data alignment
};


/**
 * ADC peripheral
 */
struct Adc
{
	static IOStruct<ADC1_BASE, ADCx_TypeDef> ADCx;

	static constexpr auto vRefIntChannel {15};
	static constexpr auto tSensorChannel {14};

	static void enableClocks()  { RCC->APB2ENR |= RCC_APB2ENR_ADCEN; __DSB(); }
	static void disableClocks() { RCC->APB2ENR &= ~RCC_APB2ENR_ADCEN; __DSB(); }

	static void enable()        { ADCx->ADCFG |= ADC_ADCFG_ADEN; }
	static void disable()       { ADCx->ADCFG &= ~ADC_ADCFG_ADEN; }

	static void start() { ADCx->ADCR |= ADC_ADCR_ADST; }

	static void setSequence(uint32_t mask) { ADCx->ADCHS = mask; }
	static void setSampleTime(uint32_t value)
	{
		ADCx->ADCFG = (ADCx->ADCFG & ~ADC_ADCFG_SAMCTL) | (value & ADC_ADCFG_SAMCTL);
	}
	static void setPrescaler(uint32_t value)
	{
		value = (value & 0b1110) << (ADC_ADCFG_ADCPRE_POS - 1) | (value & 0b1  << ADC_ADCFG_ADCPRE_POS2);
		ADCx->ADCFG = (ADCx->ADCFG & ~ADC_ADCFG_ADCPRE) | value;
	}

	/// setup ADC registers
	static void setupAdc()
	{
		ADCx->ADCFG |= ADC_ADCFG_VSEN;
		ADCx->ADCR |= ADC_ADCR_ADMD_SEQ;
	}

};

} // namespace ADC
} // namespace MM32
