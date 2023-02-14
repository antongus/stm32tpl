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
 *
 *  file         : mm32_gpio.h
 *  description  : GPIO pin manipulation class template for MM32F03xx.
 *                 Inspired by AVR macros from Askold Volkov
 *
 * USAGE:
 *
 *   I. Declare typedef for pin:
 * using PA5 = Pin<'A', 5, 'H'>;    // PA5, active level = high
 * typedef Pin<'A', 6> PA6;         // PA6, active level = high ('H' is default parameter)
 * typedef Pin<'B', 12, 'L'> PB12;  // PB12, active level = low
 *
 *   II. Set pin mode:
 * PA5::Mode(OUTPUT);       // configure PA5 as output (push-pull, 50MHz)
 * PA5::Direct(OUTPUT);     // the same.
 * PA6::Mode(INPUT);        // configure PA6 as input floating (use object and "." notation)
 * PB12::Mode(OUTPUT);      // configure PB12 as output
 * TX::Mode(ALT_OUTPUT);    // configure TX as alternate output push-pull
 *                          // (see "direction" enum for list of all pin modes)
 *
 *   III. Manipulate pin:
 * PA5::On();               // switch PA5 to active state (H)
 * PB12::On();              // switch PB12 to active state (L)
 * PA5::Off();              // switch PA5 to inactive state (L)
 * PB12::Cpl();             // invert PB12 output
 *
 *   IV. Check pin state:
 * if (PA5::Signalled())     // returns non-zero if pin input = active state (H for PA5)
 * if (PB12::Latched())      // returns non-zero if pin output = active state (L for PB12)
 *
 *   V. Use pin registers uniformly:
 * locked = PA5::GPIOx->LCKR & PA5::mask; // read PA5 lock state.
 *
 *   It is also possible to declare object of defined type:
 * PA5 PinA5;
 * In this case, one can use "dot" notation to call object functions, i.e.
 * PinA5.On();
 * PB12.Mode(INPUT);
 * Note : using objects instead of types can (in some cases) increase memory consumption.
 *
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace {

struct GPIOxTypeDef
{
	volatile uint32_t CRL;
	volatile uint32_t CRH;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t BRR;
	volatile uint32_t LCKR;
	         uint32_t reserved;
	volatile uint32_t AFR[2];
};

enum
{
	pGPIOA_BASE      = 0x48000000UL,
	pGPIOB_BASE      = 0x48000400UL,
	pGPIOC_BASE      = 0x48000800UL,
	pGPIOD_BASE      = 0x48000C00UL,
};

constexpr auto makePinCfg(unsigned mode, unsigned cfg)
{
	return (cfg << 2) | mode;
}

}

enum direction
{
	ANALOGINPUT = 			makePinCfg(0, 0),
	INPUT =					makePinCfg(0, 1),
	INPUTPULLED =			makePinCfg(0, 2),

	OUTPUT =				makePinCfg(1, 0),
	OUTPUT_OD =				makePinCfg(1, 1),
	ALT_OUTPUT =			makePinCfg(1, 2),
	ALT_OUTPUT_OD =			makePinCfg(1, 3)
};

/**
 * PinAltFunction enumeration. Defines all possible arguments for Alternate() function.
 */
enum PinAltFunction
{
	ALT_FUNC_0         = 0,
	ALT_FUNC_1         = 1,
	ALT_FUNC_2         = 2,
	ALT_FUNC_3         = 3,
	ALT_FUNC_4         = 4,
	ALT_FUNC_5         = 5,
	ALT_FUNC_6         = 6,
	ALT_FUNC_7         = 7,
};

template<char port> struct port_gpio_t;

template<> struct port_gpio_t<'A'>
{
	enum { GPIOx_BASE = pGPIOA_BASE };
};

template<> struct port_gpio_t<'B'>
{
	enum { GPIOx_BASE = pGPIOB_BASE };
};

template<> struct port_gpio_t<'C'>
{
	enum { GPIOx_BASE = pGPIOC_BASE };
};

template<> struct port_gpio_t<'D'>
{
	enum { GPIOx_BASE = pGPIOD_BASE };
};

template<char port, int pin_no, char activestate = 'H'> struct Pin;

template<char port, int pin_no, char activestate>
struct Pin
{
	enum { GPIOx_BASE = port_gpio_t<port>::GPIOx_BASE };
	static struct
	{
		GPIOxTypeDef* operator-> () { return (GPIOxTypeDef*)GPIOx_BASE; }
	}GPIOx;


	static struct{
		uint32_t operator=(uint32_t value){
			pin_no < 8 ? GPIOx->CRL = value :
			GPIOx->CRH = value;
			return value;
		}
		void operator|=(uint32_t value){
			pin_no < 8 ? GPIOx->CRL |= value :
			GPIOx->CRH |= value;
		}
		void operator&=(uint32_t value){
			pin_no < 8 ? GPIOx->CRL &= value :
			GPIOx->CRH &= value;
		}
		operator uint32_t(){
			return pin_no < 8 ? GPIOx->CRL :
			GPIOx->CRH;
		}
	}CRx;

	static const int pin = pin_no;
	static const int port_no = port-'A';
	static const uint32_t shift = pin_no;
	static const uint32_t shift_x2 = pin * 2;
	static const uint32_t shift_x4 = (pin % 8) * 4;
	static const uint32_t mask = 1UL << shift;
	static const uint32_t mask_x2 = 3UL << shift_x2;
	static const uint32_t mask_x4 = 0xFUL << shift_x4;

	static void Set() { GPIOx->BSRR = mask; }
	static void Clr() { GPIOx->BRR = mask; }

	static void On(bool set_on=true)
	{
		(activestate == 'H') == set_on ? Set() : Clr();
	}
	static void Off() { On(false); }
	static void Cpl() { GPIOx->ODR ^= mask; }

	static void Mode(direction dir)
	{
		CRx = (CRx & ~mask_x4) | (dir << shift_x4);
	}
	static void Direct(direction dir) { Mode(dir); }
	static void PullUp() { Set(); }
	static void PullDown() { Clr(); }

	static void Alternate(PinAltFunction val)
	{
		GPIOx->AFR[pin / 8] = (GPIOx->AFR[pin / 8] & ~mask_x4) | (val << shift_x4);
	}

	static int Latched()
	{
		int ret = GPIOx->ODR & mask;
		return activestate == 'L' ? !ret : ret;
	}

	static int Signalled()
	{
		int ret = GPIOx->IDR & mask;
		return activestate == 'L' ? !ret : ret;
	}

};
