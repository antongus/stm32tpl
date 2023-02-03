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
 *  file         : mm32_tim.h
 *  description  : MM32 TIM module
 */

#pragma once

#include "mm32.h"

namespace MM32
{

namespace TIM
{

enum TimerNum
{
#if defined (RCC_APB2ENR_TIM1EN)
	TIM_1,
#endif
#if defined (RCC_APB1ENR_TIM2EN)
	TIM_2,
#endif
#if defined (RCC_APB1ENR_TIM3EN)
	TIM_3,
#endif
#if defined (RCC_APB1ENR_TIM4EN)
	TIM_4,
#endif
#if defined(RCC_APB1ENR_TIM5EN)
	TIM_5,
#endif
#if defined(RCC_APB1ENR_TIM6EN)
	TIM_6,
#endif
#if defined(RCC_APB1ENR_TIM7EN)
	TIM_7,
#endif
#if defined(RCC_APB2ENR_TIM8EN)
	TIM_8,
#endif
#if defined(RCC_APB2ENR_TIM9EN)
	TIM_9,
#endif
#if defined(RCC_APB2ENR_TIM10EN)
	TIM_10,
#endif
#if defined(RCC_APB2ENR_TIM11EN)
	TIM_11,
#endif
#if defined(RCC_APB1ENR_TIM12EN)
	TIM_12,
#endif
#if defined(RCC_APB1ENR_TIM13EN)
	TIM_13,
#endif
#if defined(RCC_APB2ENR_TIM14EN)
	TIM_14,
#endif
#if defined(RCC_APB2ENR_TIM15EN)
	TIM_15,
#endif
#if defined(RCC_APB2ENR_TIM16EN)
	TIM_16,
#endif
#if defined(RCC_APB2ENR_TIM17EN)
	TIM_17,
#endif
#if defined(RCC_APB2ENR_TIM19EN)
	TIM_19,
#endif
#if defined(RCC_APB2ENR_TIM21EN)
	TIM_21,
#endif
#if defined(RCC_APB2ENR_TIM22EN)
	TIM_22,
#endif
};

enum class TimerType
{
	Advanced,
	General,
	Basic,
};

namespace
{

struct TIMx_TypeDef
{
	volatile uint32_t CR1;     //!< control register 1,              Offset: 0x00
	volatile uint32_t CR2;     //!< control register 2,              Offset: 0x04
	volatile uint32_t SMCR;    //!< slave Mode Control register,     Offset: 0x08
	volatile uint32_t DIER;    //!< DMA/interrupt enable register,   Offset: 0x0C
	volatile uint32_t SR;      //!< status register,                 Offset: 0x10
	volatile uint32_t EGR;     //!< event generation register,       Offset: 0x14
	volatile uint32_t CCMR1;   //!< capture/compare mode register 1, Offset: 0x18
	volatile uint32_t CCMR2;   //!< capture/compare mode register 2, Offset: 0x1C
	volatile uint32_t CCER;    //!< capture/compare enable register, Offset: 0x20
	volatile uint32_t CNT;     //!< counter register,                Offset: 0x24
	volatile uint32_t PSC;     //!< prescaler register,              Offset: 0x28
	volatile uint32_t ARR;     //!< auto-reload register,            Offset: 0x2C
	volatile uint32_t RCR;     //!< repetition counter register,     Offset: 0x30
	volatile uint32_t CCR1;    //!< capture/compare register 1,      Offset: 0x34
	volatile uint32_t CCR2;    //!< capture/compare register 2,      Offset: 0x38
	volatile uint32_t CCR3;    //!< capture/compare register 3,      Offset: 0x3C
	volatile uint32_t CCR4;    //!< capture/compare register 4,      Offset: 0x40
	volatile uint32_t BDTR;    //!< break and dead-time register,    Offset: 0x44
	volatile uint32_t DCR;     //!< DMA control register,            Offset: 0x48
	volatile uint32_t DMAR;    //!< DMA address for full transfer,   Offset: 0x4C
	         uint32_t pad;     //!<                                  Offset: 0x50
	volatile uint32_t CCMR3;   //!< capture/compare mode register 3, Offset: 0x54
	volatile uint32_t CCR5;    //!< capture/compare register 5,      Offset: 0x58
};


template<TimerNum num> struct TimerTraits;

#if defined (RCC_APB2ENR_TIM1EN)
template<> struct TimerTraits<TIM_1>
{
	using CounterType = uint16_t;

	static constexpr auto TIMx_BASE         {TIM1_BASE};
	static constexpr auto timerType         {TimerType::Advanced};
	static constexpr auto ccModulesCount    {4};
	static constexpr auto dmaCapable        {true};
	static constexpr auto canRunDown        {true};
	static constexpr auto complementaryOut  {true};
	static constexpr auto TIMx_IRQn         {TIM1_BRK_UP_TRG_COM_IRQn};

	static void enableClocks()   { RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; }
	static void disableClocks()  { RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN; }
};
#endif

#if defined (RCC_APB1ENR_TIM2EN)
template<> struct TimerTraits<TIM_2>
{
	using CounterType = uint16_t;
	static constexpr auto TIMx_BASE         {TIM2_BASE};
	static constexpr auto timerType         {TimerType::General};
	static constexpr auto ccModulesCount    {4};
	static constexpr auto dmaCapable        {true};
	static constexpr auto canRunDown        {true};
	static constexpr auto complementaryOut  {false};
	static constexpr auto TIMx_IRQn         {TIM2_IRQn};

	static void enableClocks()   { RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; }
	static void disableClocks()  { RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN; }
};
#endif

#if defined (RCC_APB1ENR_TIM3EN)
template<> struct TimerTraits<TIM_3>
{
	using CounterType = uint16_t;
	static constexpr auto TIMx_BASE         {TIM3_BASE};
	static constexpr auto timerType         {TimerType::General};
	static constexpr auto ccModulesCount    {4};
	static constexpr auto dmaCapable        {true};
	static constexpr auto canRunDown        {true};
	static constexpr auto complementaryOut  {false};
	static constexpr auto TIMx_IRQn         {TIM3_IRQn};

	static void enableClocks()   { RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; }
	static void disableClocks()  { RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN; }
};
#endif

#if defined(RCC_APB2ENR_TIM14EN)
template<> struct TimerTraits<TIM_14>
{
	using CounterType = uint16_t;
	static constexpr auto TIMx_BASE         {TIM14_BASE};
	static constexpr auto timerType         {TimerType::Basic};
	static constexpr auto ccModulesCount    {1};
	static constexpr auto dmaCapable        {true};
	static constexpr auto canRunDown        {false};
	static constexpr auto complementaryOut  {false};
	static constexpr auto TIMx_IRQn         {TIM14_IRQn};

	static void enableClocks()   { RCC->APB2ENR |= RCC_APB2ENR_TIM14EN; }
	static void disableClocks()  { RCC->APB2ENR &= ~RCC_APB2ENR_TIM14EN; }
};
#endif

#if defined(RCC_APB2ENR_TIM16EN)
template<> struct TimerTraits<TIM_16>
{
	using CounterType = uint16_t;
	static constexpr auto TIMx_BASE         {TIM16_BASE};
	static constexpr auto timerType         {TimerType::Basic};
	static constexpr auto ccModulesCount    {1};
	static constexpr auto dmaCapable        {true};
	static constexpr auto canRunDown        {false};
	static constexpr auto complementaryOut  {true};
	static constexpr auto TIMx_IRQn         {TIM16_IRQn};

	static void enableClocks()   { RCC->APB2ENR |= RCC_APB2ENR_TIM16EN; }
	static void disableClocks()  { RCC->APB2ENR &= ~RCC_APB2ENR_TIM16EN; }
};
#endif

#if defined(RCC_APB2ENR_TIM17EN)
template<> struct TimerTraits<TIM_17>
{
	using CounterType = uint16_t;
	static constexpr auto TIMx_BASE         {TIM17_BASE};
	static constexpr auto timerType         {TimerType::Basic};
	static constexpr auto ccModulesCount    {1};
	static constexpr auto dmaCapable        {true};
	static constexpr auto canRunDown        {false};
	static constexpr auto complementaryOut  {true};
	static constexpr auto TIMx_IRQn         {TIM17_IRQn};

	static void enableClocks()   { RCC->APB2ENR |= RCC_APB2ENR_TIM17EN; }
	static void disableClocks()  { RCC->APB2ENR &= ~RCC_APB2ENR_TIM17EN; }
};
#endif

} // anon namespace

namespace
{
template <int ch> struct CCRxSelector;

template <> struct CCRxSelector<1> {
	static constexpr uint32_t ccmrOffset = offsetof(TIMx_TypeDef, CCMR1);
	static constexpr uint32_t ccrOffset = offsetof(TIMx_TypeDef, CCR1);
};
template <> struct CCRxSelector<2> {
	static constexpr uint32_t ccmrOffset = offsetof(TIMx_TypeDef, CCMR1);
	static constexpr uint32_t ccrOffset = offsetof(TIMx_TypeDef, CCR2);
};
template <> struct CCRxSelector<3> {
	static constexpr uint32_t ccmrOffset = offsetof(TIMx_TypeDef, CCMR2);
	static constexpr uint32_t ccrOffset = offsetof(TIMx_TypeDef, CCR3);
};
template <> struct CCRxSelector<4> {
	static constexpr uint32_t ccmrOffset = offsetof(TIMx_TypeDef, CCMR2);
	static constexpr uint32_t ccrOffset = offsetof(TIMx_TypeDef, CCR4);
};


} // anon namespace

template <TimerNum timerNum>
struct Timer
{
	static constexpr auto NUMBER      {timerNum};

	using Traits = TimerTraits<timerNum>;

	using CounterType = typename Traits::CounterType;
	static constexpr auto TIMx_BASE         {Traits::TIMx_BASE};
	static constexpr auto timerType         {Traits::timerType};
	static constexpr auto ccModulesCount    {Traits::ccModulesCount};
	static constexpr auto dmaCapable        {Traits::dmaCapable};
	static constexpr auto canRunDown        {Traits::canRunDown};
	static constexpr auto complementaryOut  {Traits::complementaryOut};
	static constexpr auto TIMx_IRQn         {Traits::TIMx_IRQn};

	static IOStruct<TIMx_BASE, TIMx_TypeDef> TIMx;

	static void enable()   { TIMx->CR1 |= TIM_CR1_CEN; }
	static void disable()  { TIMx->CR1 &= ~TIM_CR1_CEN; }
	static void enableClocks()   { Traits::enableClocks(); }
	static void disableClocks()  { Traits::disableClocks(); }

	struct UpdateInterrupt
	{
		static void enable()    { TIMx->DIER |= TIM_DIER_UIE; }
		static void disable()   { TIMx->DIER &= ~TIM_DIER_UIE; }
		static bool triggered() { return TIMx->SR & TIM_SR_UIF; }
		static void clear()     { TIMx->SR = ~TIM_SR_UIF; }
	};

	static CounterType count() { return TIMx->CNT; }

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
		static IORegister<TIMx_BASE + CCRxSelector<num>::ccrOffset, CounterType> CCRx;

		static void enable()              { TIMx->CCER |= CCER_CCxE; }
		static void disable()             { TIMx->CCER &= ~CCER_CCxE; }
		struct Dma
		{
			static void enable()           { TIMx->DIER |= DIER_CCxDE; }
			static void disable()          { TIMx->DIER &= ~DIER_CCxDE; }
		};
		struct Interrupt
		{
			static void enable()     { TIMx->DIER |= DIER_CCxIE; }
			static void disable()    { TIMx->DIER &= ~DIER_CCxIE; }
			static bool enabled()    { return TIMx->DIER & DIER_CCxIE; }
			static bool triggered()  { return TIMx->SR & SR_CCxIF; }
			static void clear()      { TIMx->SR = ~SR_CCxIF; }
		};
	};

	using Ch1 = CCModule<1>;
	using Ch2 = CCModule<2>;
	using Ch3 = CCModule<3>;
	using Ch4 = CCModule<4>;
};

} // namespace TIM
} // namespace MM32
