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
 *  @file         : stm32_uart.h
 *  @description  : STM32 UART text stream class template.
 *  created on    : 09.11.2010
 *
 */

#ifndef STM32TPL_STM32_UART_H_INCLUDED
#define STM32TPL_STM32_UART_H_INCLUDED

#include "stm32.h"
#include "textstream.h"
#include <scmRTOS.h>

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

namespace
{
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
#else
		USARTx_REMAP              = 0,
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
		BUS_FREQ                  = chip::APB2_FREQ
	};
#if (defined F2xxF4xx)
	static const PinAltFunction ALT_FUNC_USARTx = ALT_FUNC_USART6;
#endif
	INLINE static void EnableClocks() { RCC->APB2ENR |= RCC_APB2ENR_USART6EN; }
	INLINE static void DisableClocks() { RCC->APB1ENR &= ~RCC_APB2ENR_USART6EN; }
};
#endif


} // namespace


// dummy structure for no DE pin.
struct NoDePin
{
	INLINE static void On() { }
	INLINE static void Off() { }
	INLINE static void Cpl() { }
	INLINE static void Mode(direction) { }
	INLINE static int Latched() { return true; }
};



/**
 * sample UART properties structure
 */
struct SampleUartProps
{
	static const UartNum uartNum = UART_1;
	static const Remap remap = REMAP_NONE;
	enum
	{
		BAUDRATE = 115200,
		RX_BUF_SIZE = 128,
		TX_BUF_SIZE = 128,
		UART_INTERRUPT_PRIOGROUP = 2,
		UART_INTERRUPT_SUBPRIO = 2
	};
	typedef NoDePin PinDE;
};


/**
*  UART main template.
*/
template<typename props = SampleUartProps>
class Uart
	: virtual public TextStream
{
private:
	typedef UartTraits<props::uartNum> Traits;
public:
	static const UartNum uartNum = props::uartNum;
	static const Remap remap = props::remap;
	static const IRQn USARTx_IRQn  = Traits::USARTx_IRQn;
	enum
	{
		BAUDRATE = props::BAUDRATE,
		RX_BUF_SIZE = props::RX_BUF_SIZE,
		TX_BUF_SIZE = props::TX_BUF_SIZE,
		UART_INTERRUPT_PRIOGROUP = props::UART_INTERRUPT_PRIOGROUP,
		UART_INTERRUPT_SUBPRIO = props::UART_INTERRUPT_SUBPRIO
	};
private:
	OS::channel<char, RX_BUF_SIZE, uint32_t> rxChannel_;
	OS::channel<char, TX_BUF_SIZE, uint32_t> txChannel_;
	OS::TEventFlag txDone_;

	typedef UartPins<props::uartNum, props::remap> Pins;
	typedef typename Pins::PinTX TX;
	typedef typename Pins::PinRX RX;
	typedef typename props::PinDE DE;

	enum { USARTx_BASE      = Traits::USARTx_BASE };
	enum { USARTx_REMAP     = Traits::USARTx_REMAP };
	enum { BUS_FREQ         = Traits::BUS_FREQ };
#if (defined F2xxF4xx)
	static const PinAltFunction ALT_FUNC_USARTx = Traits::ALT_FUNC_USARTx;
#endif

	inline void EnableTxInterrupt()  { USARTx->CR1 |= USART_CR1_TXEIE; }
	inline void DisableTxInterrupt() { USARTx->CR1 &= ~USART_CR1_TXEIE; }
	inline void EnableTcInterrupt()  { USARTx->CR1 |= USART_CR1_TCIE; }
	inline void DisableTcInterrupt() { USARTx->CR1 &= ~USART_CR1_TCIE; }
public:
	static IOStruct<USARTx_BASE, USART_TypeDef> USARTx;
	Uart();

	INLINE static void EnableClocks()       { Traits::EnableClocks(); }
	INLINE static void DisableClocks()      { Traits::DisableClocks(); }
	INLINE static void Enable()             { USARTx->CR1 |= USART_CR1_UE; }
	INLINE static void Disable()            { USARTx->CR1 &= ~USART_CR1_UE; }
	INLINE static void StartTx()            { DE::On(); }
	INLINE static void EndTx()              { DE::Off(); }

	void SetBaudrate(Baudrate value) { USARTx->BRR = (BUS_FREQ + value/2) / value; }
	Baudrate GetBaudrate() { return BUS_FREQ / USARTx->BRR; }

	void PutChar(char ch);
	int GetChar(int timeout = 0);
	int Keypressed() { return rxChannel_.get_count(); }
	virtual int CanSend() { return true; };
	virtual int TxEmpty() { return false; };

	void SendBuffer(const void* buf, size_t size);
	bool ReadBuffer(char* const buf, size_t size) { return rxChannel_.read(buf, size); }
	INLINE void UartIrqHandler();
};

template<class props>
Uart<props>::Uart()
	: TextStream()
	, rxChannel_()
	, txChannel_()
	, txDone_()
{
#if (!defined F2xxF4xx)
	if (remap == REMAP_FULL)        // remap pins if needed
		AFIO->MAPR |= USARTx_REMAP;
#endif

	EnableClocks();                 // enable UART module clock

#if (!defined F2xxF4xx)             // configure pins
	TX::Mode(ALT_OUTPUT);
	RX::Mode(INPUTPULLED);
	RX::PullUp();
#else
	TX::Alternate(ALT_FUNC_USARTx);
	RX::Alternate(ALT_FUNC_USARTx);
	TX::Mode(ALT_OUTPUT);
	RX::Mode(ALT_INPUT_PULLUP);
#endif

	DE::Mode(OUTPUT);
	DE::Off();

	USARTx->CR1 = 0
			| USART_CR1_RE      // receive enable
			| USART_CR1_TE      // transmit enable
			| USART_CR1_RXNEIE  // RX not empty interrupt enable
			;
	USARTx->CR2 = 0; // 1 stop
	USARTx->CR3 = 0; // no flow control

	SetBaudrate(BAUDRATE);

	Enable();        // enable USART

	NVIC_SetPriority(USARTx_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), UART_INTERRUPT_PRIOGROUP, UART_INTERRUPT_SUBPRIO));
	NVIC_EnableIRQ(USARTx_IRQn);
}

template<class props>
void Uart<props>::PutChar(char ch)
{
	txChannel_.push(ch);
	EnableTxInterrupt();
}

template<class props>
int Uart<props>::GetChar(int timeout)
{
	char ch = 0;
	if (rxChannel_.pop(ch, timeout))
		return ch;
	return -1;
}

template<class props>
void Uart<props>::SendBuffer(const void* buf, size_t size)
{
	const char* ptr = reinterpret_cast<const char*>(buf);
	txChannel_.write(ptr, size);
	EnableTxInterrupt();
}

template<class props>
void Uart<props>::UartIrqHandler()
{
	uint16_t status = USARTx->SR;
	uint8_t data = USARTx->DR;

	// RX NOT EMPTY INTERRUPT
	if (status & USART_SR_RXNE)
	{
		if (rxChannel_.get_free_size())
			rxChannel_.push(data);
	}

	// TX EMPTY INTERRUPT
	if ((status & USART_SR_TXE) && (USARTx->CR1 & USART_CR1_TXEIE))
	{
		if (txChannel_.get_count())
		{
			char ch = 0;
			txChannel_.pop(ch);
			if (!DE::Latched())
			{
				DE::On();
				__asm__ __volatile__ ("nop");
				__asm__ __volatile__ ("nop");
				__asm__ __volatile__ ("nop");
				__asm__ __volatile__ ("nop");
			}
			USARTx->DR = ch;
		}
		else
		{
			DisableTxInterrupt();
			EnableTcInterrupt();
		}
	}

	// TRANSMIT COMPLETE INTERRUPT
	if ((status & USART_SR_TC) && (USARTx->CR1 & USART_CR1_TCIE))
	{
		// clear interrupt
		USARTx->SR &= ~USART_SR_TC;
		// disable it
		DisableTcInterrupt();
		// turn off transmitter
		DE::Off();
		// and flag transmission done
		txDone_.signal_isr();
	}
}

} // namespace UART
} // namespace STM32

#endif // STM32TPL_STM32_UART_H_INCLUDED
