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
 *  file         : pin.h
 *  description  : GPIO pin manipulation class template for STM32 series.
 *                 Inspired by AVR macros from Askold Volkov
 *
 *  USAGE: see corresponding include file pin_stm32xxx.h
 *
 */

#ifndef STM32TPL_PIN_H_INCLUDED
#define STM32TPL_PIN_H_INCLUDED

#if (defined STM32F2XX) || (defined STM32F4XX) || (defined STM32F40_41xxx) || (defined STM32F427_437xx) || (defined STM32F429_439xx) || (defined STM32F401xx)
#  include "pin_stm32F4xx.h"
#elif (defined STM32L051xx) || (defined STM32L052xx) || (defined STM32L053xx) || (defined STM32L061xx) || (defined STM32L062xx) || (defined STM32L063xx)
#  include "pin_stm32L0xx.h"
#elif (defined STM32L100xB) || (defined STM32L100xBA) || (defined STM32L100xC) || \
    (defined STM32L151xB) || (defined STM32L151xBA) || (defined STM32L151xC) || (defined STM32L151xCA) || (defined STM32L151xD) || (defined STM32L151xDX) || (defined STM32L151xE) || \
    (defined STM32L152xB) || (defined STM32L152xBA) || (defined STM32L152xC) || (defined STM32L152xCA) || (defined STM32L152xD) || (defined STM32L152xDX) || (defined STM32L152xE) || \
    (defined STM32L162xC) || (defined STM32L162xCA) || (defined STM32L162xD) || (defined STM32L162xDX) || (defined STM32L162xE)
#  define STM32TPL_PIN_STM32L1XX
#  include "pin_stm32F4xx.h"
#elif (defined STM32F030x6) || (defined STM32F030x8) || (defined STM32F030xC) || (defined STM32F031x6) || \
    (defined STM32F038xx) || (defined STM32F042x6) || (defined STM32F048x6) || (defined STM32F051x8) || \
    (defined STM32F058xx) || (defined STM32F070x6) || (defined STM32F070xB) || (defined STM32F071xB) || \
    (defined STM32F072xB) || (defined STM32F078xx) || (defined STM32F091xC) || (defined STM32F098xx)
#  define STM32TPL_PIN_STM32F0XX
#  include "pin_stm32L0xx.h"
#elif (defined STM32F756xx) || (defined STM32F746xx) || (defined STM32F745xx) || (defined STM32F765xx) || (defined STM32F767xx) \
|| (defined STM32F769xx) || (defined STM32F777xx) || (defined STM32F779xx) || (defined STM32F722xx) || (defined STM32F723xx) \
|| (defined STM32F732xx) || (defined STM32F733xx)
#  include "pin_stm32F7xx.h"
#else
#  include "pin_stm32F1xx.h"
#endif

/**
 * Dummy pin.
 * Used when some pin is optional and not present in current configuration
 * (i.e DE pin in USART or WP pin in serial flash).
 */
template <bool latched = true>
struct DummyPin
{
	static void On() { }
	static void Off() { }
	static void Cpl() { }
	static void Mode(direction) { }
	static int Latched() { return latched; }
};

using DummyPinOn = DummyPin<true>;
using DummyPinOff = DummyPin<false>;

#endif // STM32TPL_PIN_H_INCLUDED
