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
 *  file         : stm32_tim.h
 *  description  : STM32 TIM module class
 *  created on   : 21.11.2014
 *
 */


#ifndef STM32_TIM_H_INCLUDED
#define STM32_TIM_H_INCLUDED

#include "stm32.h"

namespace STM32
{

namespace TIM
{

/**
                :   T1  T2-5    T6-7    T8  T9-11   T12-14  T15-17
--------------------------------------------------------------------
STM32F10X_LD_VL :   x   2-4     x       -	-       -       x
STM32F10X_MD_VL :   x   2-4     x       -	-       -       x
STM32F10X_HD_VL :   x   x       x       -	-       x       x
STM32F10X_LD    :   x   2-3     -       -   -       -       -
STM32F10X_MD    :   x   2-4     -       -   -       -       -
STM32F10X_HD    :   x   x       x       x   -       -       -
STM32F10X_CL    :   x   x       x       x   -       -       -
STM32F10X_XL    :   x   x       x       x   x       x       -
STM32F2XX       :   x   x*      x       x   x       x       -
STM32F40_41xxx  :   x   x*      x       x   x       x       -
STM32F427_437xx :   x   x*      x       x   x       x       -
STM32F429_439xx :   x   x*      x       x   x       x       -
STM32F401xx     :
---------------------------------------------------------------------
  * - TIM2 & TIM5 are 32 bit timers

T1, T8 - Advanced control timers
T6, T7 - Basic timers
all other - General purpose timers

*/

enum TimerNum
{
	TIM_1,
	TIM_2, TIM_3, TIM_4, TIM_5,
	TIM_6, TIM_7,
	TIM_8,
	TIM_9, TIM_10, TIM_11,
	TIM_12, TIM_13, TIM_14,
	TIM_15, TIM_16, TIM_17,
};

enum TimerType
{
	Advanced,
	Basic,
	General
};

namespace
{

template<TimerNum num> struct TimerTraits;

template<> struct TimerTraits<TIM_1>
{
	static const uint32_t TIMx_BASE = TIM1_BASE;
	static const TimerType timerType = Advanced;
	static const uint32_t ccModulesCount = 4;
	static const bool dmaCapable = true;
	static const bool canRunDown = true;
	static const IRQn TIMx_IRQn = TIM1_UP_IRQn;
	static void EnableClocks()   { RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; }
	static void DisableClocks()  { RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN; }
};

template<> struct TimerTraits<TIM_2>
{
	static const uint32_t TIMx_BASE = TIM2_BASE;
	static const TimerType timerType = General;
	static const uint32_t ccModulesCount = 4;
	static const bool dmaCapable = true;
	static const bool canRunDown = true;
	static const IRQn TIMx_IRQn = TIM2_IRQn;
	static void EnableClocks()   { RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; }
	static void DisableClocks()  { RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN; }
};

template<> struct TimerTraits<TIM_3>
{
	static const uint32_t TIMx_BASE = TIM3_BASE;
	static const TimerType timerType = General;
	static const uint32_t ccModulesCount = 4;
	static const bool dmaCapable = true;
	static const bool canRunDown = true;
	static const IRQn TIMx_IRQn = TIM3_IRQn;
	static void EnableClocks()   { RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; }
	static void DisableClocks()  { RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN; }
};

#if defined (RCC_APB1ENR_TIM4EN)
template<> struct TimerTraits<TIM_4>
{
	static const uint32_t TIMx_BASE = TIM4_BASE;
	static const TimerType timerType = General;
	static const uint32_t ccModulesCount = 4;
	static const bool dmaCapable = true;
	static const bool canRunDown = true;
	static const IRQn TIMx_IRQn = TIM4_IRQn;
	static void EnableClocks()   { RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; }
	static void DisableClocks()  { RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN; }
};
#endif

#if defined(RCC_APB1ENR_TIM5EN)
template<> struct TimerTraits<TIM_5>
{
	static const uint32_t TIMx_BASE = TIM5_BASE;
	static const TimerType timerType = General;
	static const uint32_t ccModulesCount = 4;
	static const bool dmaCapable = true;
	static const bool canRunDown = true;
	static const IRQn TIMx_IRQn = TIM5_IRQn;
	static void EnableClocks()   { RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; }
	static void DisableClocks()  { RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN; }
};
#endif


template<> struct TimerTraits<TIM_8>
{
	static const uint32_t TIMx_BASE = TIM8_BASE;
	static const TimerType timerType = Advanced;
	static const uint32_t ccModulesCount = 4;
	static const bool dmaCapable = true;
	static const bool canRunDown = true;
};

} // anon namespace

namespace
{
template <int ch> struct CCRxSelector;

template <> struct CCRxSelector<1> {
	static constexpr uint32_t ccmrOffset = offsetof(TIM_TypeDef, CCMR1);
	static constexpr uint32_t ccrOffset = offsetof(TIM_TypeDef, CCR1);
};
template <> struct CCRxSelector<2> {
	static constexpr uint32_t ccmrOffset = offsetof(TIM_TypeDef, CCMR1);
	static constexpr uint32_t ccrOffset = offsetof(TIM_TypeDef, CCR2);
};
template <> struct CCRxSelector<3> {
	static constexpr uint32_t ccmrOffset = offsetof(TIM_TypeDef, CCMR2);
	static constexpr uint32_t ccrOffset = offsetof(TIM_TypeDef, CCR3);
};
template <> struct CCRxSelector<4> {
	static constexpr uint32_t ccmrOffset = offsetof(TIM_TypeDef, CCMR2);
	static constexpr uint32_t ccrOffset = offsetof(TIM_TypeDef, CCR4);
};


} // anon namespace

struct SampleTimerInitProps
{
	enum
	{
		BUS_FREQ              = 72000000UL,
		FREQUENCY             = 1000000UL,
		PRESCALER             = 72,
		PERIOD                = 0xFFFF,
		INTERRUPT_PRIOGROUP   = 2,
		INTERRUPT_SUBPRIO     = 2,
	};

};

template <TimerNum timerNum>
class Timer
{
public:
	static const TimerNum NUMBER        = timerNum;
	typedef TimerTraits<timerNum> Traits;
	static const uint32_t TIMx_BASE       = Traits::TIMx_BASE;
	static const IRQn TIMx_IRQn           = Traits::TIMx_IRQn;
	static const TimerType timerType      = Traits::timerType;
	static const uint32_t ccModulesCount  = Traits::ccModulesCount;
	static const bool dmaCapable          = Traits::dmaCapable;
	static const bool canRunDown          = Traits::canRunDown;

	static IOStruct<TIMx_BASE, TIM_TypeDef> TIMx;

	static void Enable()   { TIMx->CR1 |= TIM_CR1_CEN; }
	static void Disable()  { TIMx->CR1 &= ~TIM_CR1_CEN; }
	static void EnableClocks()   { Traits::EnableClocks(); }
	static void DisableClocks()  { Traits::DisableClocks(); }

	static uint16_t Count() { return TIMx->CNT; }

	/**
	 * Capture/Compare channel template
	 */
	template <int num>
	struct CCModule
	{
		/**
		 * CCMR register flags
		 */
		enum : uint16_t
		{
			CCMR_SHIFT = ((num - 1) & 1) * 8,        // number of bits to shift for this module
			CCMR_MASK = 0xFF << CCMR_SHIFT,          // mask for all CCMR bits of this module

			CCMR_CCS                = (3 << 0) << CCMR_SHIFT,    // capture/compare selection
				CCMR_CCS_OUTPUT     = (0 << 0) << CCMR_SHIFT,    // output
				CCMR_CCS_IC1        = (1 << 0) << CCMR_SHIFT,    // input, mapped on corresponding input
				CCMR_CCS_IC1_CROSS  = (2 << 0) << CCMR_SHIFT,    // input, mapped on adjacent input (1<->2 3<->4)
				CCMR_CCS_IC1_TRC    = (3 << 0) << CCMR_SHIFT,    // input, mapped on TRC input

			CCMR_OCFE               = (1 << 2) << CCMR_SHIFT,    // output compare fast enable
			CCMR_OCPE               = (1 << 3) << CCMR_SHIFT,    // output compare preload enable

			CCMR_OCM                = (7 << 4) << CCMR_SHIFT,    // output compare mode
				CCMR_OCM_FROZEN     = (0 << 4) << CCMR_SHIFT,    // frozen
				CCMR_OCM_SET        = (1 << 4) << CCMR_SHIFT,    // set channel to active level on match
				CCMR_OCM_RESET      = (2 << 4) << CCMR_SHIFT,    // set channel to inactive level on match
				CCMR_OCM_TOGGLE     = (3 << 4) << CCMR_SHIFT,    // toggle channel on match
				CCMR_OCM_FORCE_OFF  = (4 << 4) << CCMR_SHIFT,    // force channel to inactive level
				CCMR_OCM_FORCE_ON   = (5 << 4) << CCMR_SHIFT,    // force channel to active level
				CCMR_OCM_PWM1       = (6 << 4) << CCMR_SHIFT,    // PWM mode 1 (up:(inactive < match < active), down:(active > match > inactive))
				CCMR_OCM_PWM2       = (7 << 4) << CCMR_SHIFT,    // PWM mode 2 (up:(active < match < inactive), down:(inactive > match > active))

			CCMR_OCCE               = (1 << 7) << CCMR_SHIFT     // output compare clear enable (by high level on ETRF input)
		};

		/**
		 * CCER register flags
		 */
		enum : uint16_t
		{
			CCER_SHIFT = (num - 1) * 4,            // number of bits to shift for this module
			CCER_CCxE =  (1 << 0) << CCER_SHIFT,   // enable
			CCER_CCxP =  (1 << 1) << CCER_SHIFT    // input/output polarity
		};

		/**
		 * Module flags
		 */
		enum : uint16_t
		{
			SR_CCxIF      = 1 << num,         // module interrupt flag
			SR_CCxOF      = 1 << (num + 8),   // capture module overflow interrupt flag
			DIER_CCxIE    = 1 << num,         // module interrupt enable flag
			DIER_CCxDE    = 1 << (num + 8),   // module DMA enable flag
		};

		static IORegister<TIMx_BASE + CCRxSelector<num>::ccmrOffset, uint16_t> CCMRx;
		static IORegister<TIMx_BASE + CCRxSelector<num>::ccrOffset, uint16_t> CCRx;

		static void Enable()              { TIMx->CCER |= CCER_CCxE; }
		static void Disable()             { TIMx->CCER &= ~CCER_CCxE; }
		static void EnableDma()           { TIMx->DIER |= DIER_CCxDE; }
		static void DisableDma()          { TIMx->DIER &= ~DIER_CCxDE; }
		static void EnableInterrupt()     { TIMx->DIER |= DIER_CCxIE; }
		static void DisableInterrupt()    { TIMx->DIER &= ~DIER_CCxIE; }
		static bool Interrupted()         { return TIMx->SR & SR_CCxIF; }
		static void ClearInterrupt()      { TIMx->SR = ~SR_CCxIF; }
	};

	typedef CCModule<1> Ch1;
	typedef CCModule<2> Ch2;
	typedef CCModule<3> Ch3;
	typedef CCModule<4> Ch4;
};

} // namespace TIM
} // namespace STM32

#endif // STM32_TIM_H_INCLUDED
