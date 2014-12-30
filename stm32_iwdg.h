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
 *  file         : stm32_iwdg.h
 *  description  : STM32 IWDG module class
 *  created on   : 2014-DEC-30
 *
 */


#ifndef STM32TPL_STM32_IWDG_H_INCLUDED
#define STM32TPL_STM32_IWDG_H_INCLUDED

#include "stm32.h"

namespace STM32
{

class Iwdg
{
public:
	static void Reload() { IWDG->KR = RELOAD; }

	template<uint32_t period>
	static void Start()
	{
		static_assert(CalcPrescaler(period) <= MAX_PRESCALER, "IWDG period too big");
		IWDG->KR = UNLOCK;
		IWDG->PR = CalcPrescaler(period);
		IWDG->RLR = CalcReload(period);
		Reload();
		IWDG->KR = START;
	}
private:
	static const uint32_t LSI_FREQ = 40000;     // LSI frequency
	static const uint32_t MAX_RELOAD = 0xFFF;   // Max reload value (12 bits)
	static const uint32_t MAX_PRESCALER = 6;    // Max prescaler value

	// Convert milliseconds to LSI timer ticks
	static constexpr uint32_t Ms2Ticks(uint32_t ms) { return ms * LSI_FREQ / 1000; }

	// Convert prescaler to divider
	static constexpr uint32_t Prescaler2Divider(uint32_t prescaler) { return 1UL << (prescaler + 2); }

	// Recursive constexpr function to calculate prescaler for given interval value (in milliseconds).
	static constexpr uint32_t CalcPrescaler(uint32_t ms, uint32_t prescaler = 0)
	{
		return Ms2Ticks(ms) / Prescaler2Divider(prescaler) > MAX_RELOAD ?
				CalcPrescaler(ms, prescaler + 1) :
				prescaler;
	}

	// Calculate divider by given reload value.
	static constexpr uint32_t CalcDivider(uint32_t ms) { return Prescaler2Divider(CalcPrescaler(ms)); }

	// Calculate reload value for given interval value (in milliseconds).
	static constexpr uint32_t CalcReload(uint32_t ms)
	{
		return Ms2Ticks(ms)/CalcDivider(ms) < MAX_RELOAD ? Ms2Ticks(ms)/CalcDivider(ms) : MAX_RELOAD;
	}

	enum : uint16_t
	{
		START   = 0xCCCC,
		RELOAD  = 0xAAAA,
		UNLOCK  = 0x5555
	};
};

} // namespace STM32




#endif // STM32TPL_STM32_IWDG_H_INCLUDED
