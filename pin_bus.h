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
 *  file         : pin_bus.h
 *  description  : GPIO bus manipulation class template for STM32 series.
 *
 *  USAGE:
 *  1. Declare Pins for bus:
 *     using D0 = Pin<'E', 0>;
 *     using D1 = Pin<'E', 1>;
 *     using D2 = Pin<'E', 2>;
 *     using D3 = Pin<'E', 3>;
 *     using D4 = Pin<'E', 4>;
 *     using D5 = Pin<'E', 5>;
 *     using D6 = Pin<'E', 6>;
 *     using D7 = Pin<'E', 7>;
 *
 *  2. Declare bus class:
 *     using TestBus = STM32TPL::Bus<D0, D1, D2, D3, D4, D5, D6, D7>;
 *
 *  3. Use the bus:
 *     TestBus::mode(OUTPUT);
 *     auto value = TestBus::get();
 *     TestBus::set(value + 1);
 */

#ifndef STM32TPL_PIN_BUS_H_INCLUDED
#define STM32TPL_PIN_BUS_H_INCLUDED

#include "pin.h"

namespace STM32TPL
{

namespace detail
{

template <typename Pin, typename... Pins>
void setPinMode(direction dir)
{
	Pin::Mode(dir);
	if constexpr (sizeof...(Pins) > 0)
		setPinMode<Pins...>(dir);
}

template <typename Pin, typename... Pins>
void setPinValue(uint32_t value)
{
	Pin::On(value & 1);
	if constexpr (sizeof...(Pins) > 0)
		setPinValue<Pins...>(value >> 1);
}

template <typename Pin, typename... Pins>
uint32_t getPinValue()
{
	uint32_t value;
	if constexpr (sizeof...(Pins) > 0)
	{
		value = getPinValue<Pins...>();
		value <<= 1;
	}
	if (Pin::Signalled())
		value |= 1;
	return value;
}
} // namespace details

template <typename... Pins>
class Bus
{
public:
	static void mode(direction dir)
	{
		detail::setPinMode<Pins...>(dir);
	}
	static void set(uint32_t value)
	{
		detail::setPinValue<Pins...>(value);
	}
	static uint32_t get()
	{
		return detail::getPinValue<Pins...>();
	}
};

} // namespace STM32TPL

#endif // STM32TPL_PIN_BUS_H_INCLUDED
