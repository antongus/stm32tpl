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
 *  file         : pin_stm32L0xx.h
 *  description  : GPIO pin manipulation class template for STM32L0xx and STM32F0xx.
 *                 Inspired by AVR macros from Askold Volkov
 *
 * USAGE:
 *
 *   I. Declare typedef for pin:
 * typedef Pin<'A', 5, 'H', PIN_SPEED_HIGH> PA5;    // PA5, active level = high, pin speed = 50MHZ
 * typedef Pin<'A', 6> PA6;               // PA6, default active level ('H'), default pin speed (PIN_SPEED_VERY_LOW)
 * typedef Pin<'B', 12, 'L'> PB12;        // PB12, active level = low, default pin speed (PIN_SPEED_VERY_LOW)
 *
 *   II. Set pin mode:
 * PA5::Mode(OUTPUT);                     // configure PA5 as output (push-pull, 100MHz)
 * PA5::Direct(OUTPUT);                   // the same.
 * PA6::Mode(INPUT);                      // configure PA6 as input floating
 * PB12::Mode(OUTPUT);                    // configure PB12 as output
 * TX::Mode(ALT_OUTPUT);                  // configure TX as alternate output push-pull
 *                                        // (see PinConfig enum for list of all pin modes)
 *
 *   III. Select pin alternate function:
 * TX::Alternate(ALT_FUNC_USART1);        // turn P15 alternate function on
 *                                        // (see PinAltFunction enum for list of all alternate functions)
 *
 *   IV. Manipulate pin:
 * PA5::On();                             // switch PA5 to active state (H)
 * PB12::On();                            // switch PB12 to active state (L)
 * PA5::Off();                            // switch PA5 to inactive state (L)
 * PB12::Cpl();                           // invert PB12 output
 *
 *   V. Check pin state:
 * if (PA5::Signalled())                  // returns non-zero if pin input = active state (H for PA5)
 * if (PB12::Latched())                   // returns non-zero if pin output = active state (L for PB12)
 *
 *   VI. Use pin registers uniformly:
 * locked = PA5::GPIOx->LCKR & PA5::mask; // read PA5 lock state.
 *
 *   It is also possible to declare object of defined type:
 * PA5 PinA5;
 * In this case, one can use "dot" notation to call object functions, i.e.
 * PinA5.On();
 * PB12.Mode(INPUT);
 * Note : using objects instead of types can (in some cases) increase memory consumption.
 */

#ifndef STM32TPL_PIN_STM32L0XX_H_INCLUDED
#define STM32TPL_PIN_STM32L0XX_H_INCLUDED

#include <cstdint>
#include <cstddef>

#ifndef INLINE
#define INLINE __attribute__((__always_inline__)) inline
#endif

namespace {

typedef struct
{
  volatile uint32_t MODER;    ///!< GPIO port mode register
  volatile uint32_t OTYPER;   ///!< GPIO port output type register
  volatile uint32_t OSPEEDR;  ///!< GPIO port output speed register
  volatile uint32_t PUPDR;    ///!< GPIO port pull-up/pull-down register
  volatile uint32_t IDR;      ///!< GPIO port input data register
  volatile uint32_t ODR;      ///!< GPIO port output data register
  volatile uint32_t BSRR;     ///!< GPIO port bit set/reset low register
  volatile uint32_t LCKR;     ///!< GPIO port configuration lock register
  volatile uint32_t AFR[2];   ///!< GPIO alternate function registers
} GPIOxTypeDef;

#ifdef STM32TPL_PIN_STM32F0XX
enum
{
	pGPIOA_BASE      = 0x48000000UL,
	pGPIOB_BASE      = 0x48000400UL,
	pGPIOC_BASE      = 0x48000800UL,
	pGPIOD_BASE      = 0x48000C00UL,
	pGPIOE_BASE      = 0x48001000UL,
	pGPIOF_BASE      = 0x48001400UL,
};
#else // by default STM32L00x assumed
enum
{
	pGPIOA_BASE      = 0x50000000UL,
	pGPIOB_BASE      = 0x50000400UL,
	pGPIOC_BASE      = 0x50000800UL,
	pGPIOD_BASE      = 0x50000C00UL,
	pGPIOH_BASE      = 0x50001C00UL
};
#endif

}

/**
 * PinConfig enumeration. Defines all possible pin configuration variants.
 */
enum PinConfig
{
	INPUT,                   ///!< input floating
	INPUT_PULLUP,            ///!< input pull-up
	INPUT_PULLDOWN,          ///!< input pull-down
	ANALOG,                  ///!< analog

	OUTPUT,                  ///!< output push-pull
	OUTPUT_OD,               ///!< output open-drain
	OUTPUT_OD_PULLUP,        ///!< output open-drain pull-up
	OUTPUT_OD_PULLDOWN,      ///!< output open-drain pull-down
	ALT_INPUT,               ///!< alternate function input
	ALT_INPUT_PULLUP,        ///!< alternate function input with pull-up
	ALT_INPUT_PULLDOWN,      ///!< alternate function input with pull-down
	ALT_OUTPUT,              ///!< alternate function output push-pull
	ALT_OUTPUT_PULLUP,       ///!< alternate function output push-pull pull-up
	ALT_OUTPUT_PULLDOWN,     ///!< alternate function output push-pull pull-down
	ALT_OUTPUT_OD,           ///!< alternate function output open-drain
	ALT_OUTPUT_OD_PULLUP,    ///!< alternate function output open-drain pull-up
	ALT_OUTPUT_OD_PULLDOWN   ///!< alternate function output open-drain pull-down
};

typedef PinConfig direction;


/**
 * PinMode enumeration. Defines all possible values for MODER register.
 */
enum PinMode
{
	MODER_INPUT  = 0,
	MODER_OUTPUT  = 1,
	MODER_ALTERNATE  = 2,
	MODER_ANALOG  = 3,
	MODER_MASK  = 3
};

/**
 * PullUpMode enumeration. Defines all possible values for PUPDR register.
 */
enum PullUpMode
{
	PUPDR_NONE  = 0,
	PUPDR_PULLUP = 1,
	PUPDR_PULLDOWN = 2,
	PUPDR_MASK = 3
};

/**
 * PinSpeed enumeration. Defines all possible values for OSPEEDR register.
 */
enum PinSpeed
{
	PIN_SPEED_VERY_LOW = 0,
	PIN_SPEED_LOW = 1,
	PIN_SPEED_HIGH = 2,
	PIN_SPEED_VERY_HIGH = 3,
	PIN_SPEED_MASK = 3
};

/**
 * PinOutputType enumeration. Defines all possible values for OTYPER register.
 */
enum PinOutputType
{
	PUSH_PULL = 0,
	OPEN_DRAIN = 1
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

#ifdef STM32TPL_PIN_STM32F0XX
template<> struct port_gpio_t<'E'>
{
	enum { GPIOx_BASE = pGPIOE_BASE };
};

template<> struct port_gpio_t<'F'>
{
	enum { GPIOx_BASE = pGPIOF_BASE };
};

#else
template<> struct port_gpio_t<'H'>
{
	enum { GPIOx_BASE = pGPIOH_BASE };
};
#endif

template<
	char port,
	int pin_no,
	char activestate = 'H',
	PinSpeed speed = PIN_SPEED_VERY_HIGH
	> struct Pin;

template<char port, int pin_no, char activestate, PinSpeed speed>
struct Pin
{
	static const uint32_t pin = pin_no;
	static const uint32_t port_no = port-'A';
	static const uint32_t shift = pin;
	static const uint32_t shift_x2 = pin * 2;
	static const uint32_t shift_x4 = (pin % 8) * 4;
	static const uint32_t mask = 1UL << shift;
	static const uint32_t mask_x2 = 3UL << shift_x2;
	static const uint32_t mask_x4 = 0xFUL << shift_x4;
	static const uint32_t clearmask = 1UL << (pin + 16);
	enum { GPIOx_BASE = port_gpio_t<port>::GPIOx_BASE };

	static struct
	{
		GPIOxTypeDef* operator-> () { return (GPIOxTypeDef*)GPIOx_BASE; }
	}GPIOx;

	INLINE static void On(bool set_on=true)
	{
		(activestate == 'H') == set_on ? GPIOx->BSRR = mask : GPIOx->BSRR = clearmask;
	}
	INLINE static void Off() { On(false); }
	INLINE static void Cpl() { GPIOx->ODR ^= mask; }

	INLINE static void Mode(PinConfig cfg)
	{
		switch(cfg)
		{
		case  INPUT:
			SetMode(MODER_INPUT);
			PullNone();
			break;

		case  INPUT_PULLUP:
			SetMode(MODER_INPUT);
			PullUp();
			break;

		case INPUT_PULLDOWN:
			SetMode(MODER_INPUT);
			PullDown();
			break;

		case ANALOG:
			SetMode(MODER_ANALOG);
			PullNone();
			break;

		case OUTPUT:
			SetMode(MODER_OUTPUT);
			SetSpeed(speed);
			SetPushPull();
			PullNone();
			break;

		case OUTPUT_OD:
			SetMode(MODER_OUTPUT);
			SetSpeed(speed);
			SetOpenDrain();
			PullNone();
			break;

		case OUTPUT_OD_PULLUP:
			SetMode(MODER_OUTPUT);
			SetSpeed(speed);
			SetOpenDrain();
			PullUp();
			break;

		case OUTPUT_OD_PULLDOWN:
			SetMode(MODER_OUTPUT);
			SetSpeed(speed);
			SetOpenDrain();
			PullDown();
			break;

		case ALT_INPUT:
			SetMode(MODER_ALTERNATE);
			PullNone();
			break;

		case ALT_INPUT_PULLUP:
			SetMode(MODER_ALTERNATE);
			PullUp();
			break;

		case ALT_INPUT_PULLDOWN:
			SetMode(MODER_ALTERNATE);
			PullDown();
			break;

		case ALT_OUTPUT:
			SetMode(MODER_ALTERNATE);
			SetSpeed(speed);
			SetPushPull();
			PullNone();
			break;

		case ALT_OUTPUT_PULLUP:
			SetMode(MODER_ALTERNATE);
			SetSpeed(speed);
			SetPushPull();
			PullUp();
			break;

		case ALT_OUTPUT_PULLDOWN:
			SetMode(MODER_ALTERNATE);
			SetSpeed(speed);
			SetPushPull();
			PullDown();
			break;

		case ALT_OUTPUT_OD:
			SetMode(MODER_ALTERNATE);
			SetSpeed(speed);
			SetOpenDrain();
			PullNone();
			break;

		case ALT_OUTPUT_OD_PULLUP:
			SetMode(MODER_ALTERNATE);
			SetSpeed(speed);
			SetOpenDrain();
			PullUp();
			break;

		case ALT_OUTPUT_OD_PULLDOWN:
			SetMode(MODER_ALTERNATE);
			SetSpeed(speed);
			SetOpenDrain();
			PullDown();
			break;
		}
	}
	INLINE static void Direct(PinConfig cfg) { Mode(cfg); }

	/**
	 * Set Pull-UP/Pull-Down mode.
	 */
	INLINE static void SetPullUp(PullUpMode mode)
	{
		GPIOx->PUPDR = (GPIOx->PUPDR & ~(PUPDR_MASK << shift_x2)) | (mode << shift_x2);
	}
	INLINE static void PullUp()   { SetPullUp(PUPDR_PULLUP); }
	INLINE static void PullDown() { SetPullUp(PUPDR_PULLDOWN); }
	INLINE static void PullNone() { SetPullUp(PUPDR_NONE); }

	INLINE static void SetSpeed(PinSpeed sp)
	{
		GPIOx->OSPEEDR = (GPIOx->OSPEEDR & ~mask_x2) | (sp << shift_x2);
	}

	INLINE static void SetMode(PinMode mode)
	{
		GPIOx->MODER = (GPIOx->MODER & ~mask_x2) | (mode << shift_x2);
	}

	INLINE static void SetOutputType(PinOutputType val)
	{
		GPIOx->OTYPER = (GPIOx->OTYPER & ~mask) | (val << shift);
	}
	INLINE static void SetPushPull() { SetOutputType(PUSH_PULL); }
	INLINE static void SetOpenDrain() { SetOutputType(OPEN_DRAIN);}

	INLINE static void Alternate(PinAltFunction val)
	{
		GPIOx->AFR[pin / 8] = (GPIOx->AFR[pin / 8] & ~mask_x4) | (val << shift_x4);
	}

	INLINE static int Latched()
	{
		int ret = GPIOx->ODR & mask;
		return activestate == 'L' ? !ret : ret;
	}

	INLINE static int Signalled()
	{
		int ret = GPIOx->IDR & mask;
		return activestate == 'L' ? !ret : ret;
	}

};


#endif // STM32TPL_PIN_STM32L0XX_H_INCLUDED
