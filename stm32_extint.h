/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *  Visit https://github.com/antongus/stm32tpl for new versions
 *
 *  Copyright (c) 2011-2022 Anton B. Gusev
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
 *  file         : stm32_extint.h
 *  description  : External interrupt utility struct
 *
 *  USAGE :
 *    // specialize ExtInt type (pin, edge):
 *    using TheExtInt = ExtInt<Pin<'A', 4>, Edge::Falling>;
 *
 *    // calculate priority
 *    auto priority = NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0);
 *
 *    // initialize external interrupt (edges, interrupt priority)
 *    TheExtInt::init(priority);
 *
 *    // clear interrupt flag:
 *    TheExtInt::Interrupt::clear();
 *
 *    // enable interrupt
 *    TheExtInt::Interrupt::enable();
 *
 */

#pragma once

#include "pin.h"
#include <cstdint>

namespace STM32
{

enum class Edge
{
	Falling,
	Rising,
	Both
};

template <typename PinType, Edge edges = Edge::Falling>
struct ExtInt
{
	static constexpr auto port   {PinType::port_no};
	static constexpr auto pin    {PinType::pin};
	static constexpr auto mask   {PinType::mask};
	static constexpr auto shift  {(pin & 0x03) << 2};
	static constexpr auto idx    {pin >> 2};  // index in EXTICR[] array

#if (defined RCC_APB2ENR_SYSCFGEN)
	static void enableClocks() { RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; __DSB(); }
	static void disableClocks() { RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN; __DSB(); }
#endif

	static void init(uint32_t priority)
	{
#if (defined RCC_APB2ENR_SYSCFGEN)
		enableClocks();
#endif

		EXTI->EMR &= ~mask;       // disable event on this line
		EXTI->IMR &= ~mask;       // disable interrupt on this line

		if constexpr (edges == Edge::Falling)
		{
			EXTI->RTSR &= ~mask;
			EXTI->FTSR |= mask;
		}
		if constexpr (edges == Edge::Rising)
		{
			EXTI->RTSR |= mask;
			EXTI->FTSR &= ~mask;
		}
		if constexpr (edges == Edge::Both)
		{
			EXTI->RTSR |= mask;
			EXTI->FTSR |= mask;
		}

		SYSCFG->EXTICR[idx] &= ~(0x0FUL << shift);
		SYSCFG->EXTICR[idx] |= port << shift;

		NVIC_SetPriority(irq, priority);
		NVIC_EnableIRQ(irq);
	}

	struct Interrupt
	{
		static void enable()    { EXTI->IMR |= mask; }
		static void disable()   { EXTI->IMR &= ~mask; }
		static bool isEnabled() { return EXTI->IMR & mask; }
		static bool isFired()   { return EXTI->PR & mask; }
		static void clear()     { EXTI->PR = mask; }
	};


	static constexpr IRQn_Type irqn()
	{
		IRQn_Type ret {EXTI15_10_IRQn};
		switch (PinType::pin)
		{
		case 0 : ret = EXTI0_IRQn; break;
		case 1 : ret = EXTI1_IRQn; break;
		case 2 : ret = EXTI2_IRQn; break;
		case 3 : ret = EXTI3_IRQn; break;
		case 4 : ret = EXTI4_IRQn; break;
		case 5 : ret = EXTI9_5_IRQn; break;
		case 6 : ret = EXTI9_5_IRQn; break;
		case 7 : ret = EXTI9_5_IRQn; break;
		case 8 : ret = EXTI9_5_IRQn; break;
		case 9 : ret = EXTI9_5_IRQn; break;
		default : ret = EXTI15_10_IRQn; break;
		}
		return ret;
	}
	static constexpr IRQn_Type irq {irqn()};
};

} // namespace STM32
