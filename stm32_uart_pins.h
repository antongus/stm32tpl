/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 2010-2014 Anton B. Gusev aka AHTOXA
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
 *  @file         : stm32_uart_pins.h
 *  @description  : STM32 UART pins class template.
 *  created on    : 09.11.2010
 *
 */

#ifndef STM32TPL_STM32_UART_PINS_H_INCLUDED
#define STM32TPL_STM32_UART_PINS_H_INCLUDED

#include "stm32.h"

namespace STM32
{
namespace UART
{
typedef uint32_t Baudrate;


/**
 * define count of UART modules
 */
#if (defined RCC_APB1ENR_UART8EN)
#	define UART_COUNT 8
#elif (defined RCC_APB1ENR_UART7EN)
#	define UART_COUNT 7
#elif (defined RCC_APB2ENR_USART6EN)
#	define UART_COUNT 6
#elif (defined RCC_APB1ENR_UART5EN)
#	define UART_COUNT 5
#elif (defined RCC_APB1ENR_UART4EN)
#	define UART_COUNT 4
#elif (defined RCC_APB1ENR_USART3EN)
#	define UART_COUNT 3
#else
#	define UART_COUNT 2
#endif

/**
*  @brief enum for UART device selection
*/
enum UartNum
{
	UART_1
	,UART_2
#if (UART_COUNT > 2)
	,UART_3
#endif
#if (UART_COUNT > 3)
	,UART_4
#endif
#if (UART_COUNT > 4)
	,UART_5
#endif
#if (UART_COUNT > 5)
	,UART_6
#endif
#if (UART_COUNT > 6)
	,UART_7
#endif
#if (UART_COUNT > 7)
	,UART_8
#endif
};

/**
*  @brief enum for remap state
*/
enum Remap { REMAP_NONE, REMAP_FULL, REMAP_PARTIAL };

/**
*  @brief UART pins selection template. Used internally.
*/
template<UartNum uartNum, Remap remapped = REMAP_NONE> struct UartPins;

template<> struct UartPins<UART_1>
{
	typedef Pin<'A', 9> PinTX;
	typedef Pin<'A', 10> PinRX;
};

template<> struct UartPins<UART_1, REMAP_FULL>
{
	typedef Pin<'B', 6> PinTX;
	typedef Pin<'B', 7> PinRX;
};

template<> struct UartPins<UART_2>
{
	typedef Pin<'A', 2> PinTX;
	typedef Pin<'A', 3> PinRX;
};

template<> struct UartPins<UART_2, REMAP_FULL>
{
	typedef Pin<'D', 5> PinTX;
	typedef Pin<'D', 6> PinRX;
};

#if (UART_COUNT > 2)
template<> struct UartPins<UART_3>
{
	typedef Pin<'B', 10> PinTX;
	typedef Pin<'B', 11> PinRX;
};

template<> struct UartPins<UART_3, REMAP_PARTIAL>
{
	typedef Pin<'C', 10> PinTX;
	typedef Pin<'C', 11> PinRX;
};

template<> struct UartPins<UART_3, REMAP_FULL>
{
	typedef Pin<'D', 8> PinTX;
	typedef Pin<'D', 9> PinRX;
};
#endif

#if (UART_COUNT > 3)
template<> struct UartPins<UART_4>
{
	typedef Pin<'C', 10> PinTX;
	typedef Pin<'C', 11> PinRX;
};
#endif

#if (UART_COUNT > 4)
template<> struct UartPins<UART_5>
{
	typedef Pin<'C', 12> PinTX;
	typedef Pin<'D', 2> PinRX;
};
#endif

#if (UART_COUNT > 5)
template<> struct UartPins<UART_6>
{
	typedef Pin<'C', 6> PinTX;
	typedef Pin<'C', 7> PinRX;
};

template<> struct UartPins<UART_6, REMAP_FULL>
{
	typedef Pin<'G', 14> PinTX;
	typedef Pin<'G', 9> PinRX;
};
#endif


/**
*  UART traits template.
*/
template<UartNum uartNum> struct UartTraits;

template<> struct UartTraits<UART_1>
{
	static const IRQn USARTx_IRQn  = USART1_IRQn;
	enum
	{
		USARTx_BASE               = USART1_BASE,
#if (defined AFIO_MAPR_USART1_REMAP)
		USARTx_REMAP              = AFIO_MAPR_USART1_REMAP,
#else
		USARTx_REMAP              = 0,
#endif
		USARTx_REMAP_PARTIAL      = 0,
		BUS_FREQ                  = chip::APB2_FREQ
	};
#if (defined F2xxF4xx)
	static const PinAltFunction ALT_FUNC_USARTx = ALT_FUNC_USART1;
#endif
	INLINE static void EnableClocks() { RCC->APB2ENR |= RCC_APB2ENR_USART1EN; }
	INLINE static void DisableClocks() { RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN; }
};

template<> struct UartTraits<UART_2>
{
	static const IRQn USARTx_IRQn  = USART2_IRQn;
	enum
	{
		USARTx_BASE               = USART2_BASE,
#if (defined AFIO_MAPR_USART2_REMAP)
		USARTx_REMAP              = AFIO_MAPR_USART2_REMAP,
#else
		USARTx_REMAP              = 0,
#endif
		USARTx_REMAP_PARTIAL      = 0,
		BUS_FREQ                  = chip::APB1_FREQ
	};
#if (defined F2xxF4xx)
	static const PinAltFunction ALT_FUNC_USARTx = ALT_FUNC_USART2;
#endif
	INLINE static void EnableClocks() { RCC->APB1ENR |= RCC_APB1ENR_USART2EN; }
	INLINE static void DisableClocks() { RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN; }
};

#if (UART_COUNT > 2)
template<> struct UartTraits<UART_3>
{
	static const IRQn USARTx_IRQn  = USART3_IRQn;
	enum
	{
		USARTx_BASE               = USART3_BASE,
#if (defined AFIO_MAPR_USART3_REMAP)
		USARTx_REMAP              = AFIO_MAPR_USART3_REMAP,
		USARTx_REMAP_PARTIAL      = AFIO_MAPR_USART3_REMAP_0,
#else
		USARTx_REMAP              = 0,
		USARTx_REMAP_PARTIAL      = 0,
#endif
		BUS_FREQ                  = chip::APB1_FREQ
	};
#if (defined F2xxF4xx)
	static const PinAltFunction ALT_FUNC_USARTx = ALT_FUNC_USART3;
#endif
	INLINE static void EnableClocks() { RCC->APB1ENR |= RCC_APB1ENR_USART3EN; }
	INLINE static void DisableClocks() { RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN; }
};
#endif

#if (UART_COUNT > 3)
template<> struct UartTraits<UART_4>
{
	static const IRQn USARTx_IRQn  = UART4_IRQn;
	enum
	{
		USARTx_BASE               = UART4_BASE,
		USARTx_REMAP              = 0,
		USARTx_REMAP_PARTIAL      = 0,
		BUS_FREQ                  = chip::APB1_FREQ
	};
#if (defined F2xxF4xx)
	static const PinAltFunction ALT_FUNC_USARTx = ALT_FUNC_UART4;
#endif
	INLINE static void EnableClocks() { RCC->APB1ENR |= RCC_APB1ENR_UART4EN; }
	INLINE static void DisableClocks() { RCC->APB1ENR &= ~RCC_APB1ENR_UART4EN; }
};
#endif

#if (UART_COUNT > 4)
template<> struct UartTraits<UART_5>
{
	static const IRQn USARTx_IRQn  = UART5_IRQn;
	enum
	{
		USARTx_BASE               = UART5_BASE,
		USARTx_REMAP              = 0,
		USARTx_REMAP_PARTIAL      = 0,
		BUS_FREQ                  = chip::APB1_FREQ
	};
#if (defined F2xxF4xx)
	static const PinAltFunction ALT_FUNC_USARTx = ALT_FUNC_UART5;
#endif
	INLINE static void EnableClocks() { RCC->APB1ENR |= RCC_APB1ENR_UART5EN; }
	INLINE static void DisableClocks() { RCC->APB1ENR &= ~RCC_APB1ENR_UART5EN; }
};
#endif

#if (UART_COUNT > 5)
template<> struct UartTraits<UART_6>
{
	static const IRQn USARTx_IRQn  = USART6_IRQn;
	enum
	{
		USARTx_BASE               = USART6_BASE,
		USARTx_REMAP              = 0,
		USARTx_REMAP_PARTIAL      = 0,
		BUS_FREQ                  = chip::APB2_FREQ
	};
#if (defined F2xxF4xx)
	static const PinAltFunction ALT_FUNC_USARTx = ALT_FUNC_USART6;
#endif
	INLINE static void EnableClocks() { RCC->APB2ENR |= RCC_APB2ENR_USART6EN; }
	INLINE static void DisableClocks() { RCC->APB1ENR &= ~RCC_APB2ENR_USART6EN; }
};
#endif

/**
 * Dummy DE pin.
 * Used as DE pin when no DE pin needed.
 */
struct DummyDE
{
	INLINE static void On() { }
	INLINE static void Off() { }
	INLINE static void Cpl() { }
	INLINE static void Mode(direction) { }
	INLINE static int Latched() { return true; }
};

} // namespace UART
} // namespace STM32


#endif // STM32TPL_STM32_UART_PINS_H_INCLUDED
