/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 2013-2014 Anton B. Gusev aka AHTOXA
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
 *  @file         : stm32_uart_dbg.h
 *  @description  : STM32 UART (debug version, no interrupts used).
 *  created on    : 2015-JAN-16.
 *
 */

#ifndef STM32TPL_STM32_UART_DBG_H_INCLUDED
#define STM32TPL_STM32_UART_DBG_H_INCLUDED

#include "stm32.h"
#include "stm32_uart_pins.h"
#include "textstream.h"

namespace STM32
{
namespace UART
{

/**
 * sample UART+DMA properties structure
 */
struct SampleUartDbgProps
{
	static const UartNum uartNum = UART_1;
	static const Remap remap = REMAP_NONE;
	enum { BAUDRATE = 115200 };
	typedef DummyDE PinDE;
};


/**
*  UART main template.
*/
template<typename props = SampleUartDbgProps>
class UartDbg
	: public TextStream
{
private:
	typedef UartTraits<props::uartNum> Traits;
public:
	static const UartNum uartNum = props::uartNum;
	static const Remap remap = props::remap;
	enum { BAUDRATE = props::BAUDRATE };

private:
	typedef UartPins<props::uartNum, props::remap> Pins;
	typedef typename Pins::PinTX TX;
	typedef typename Pins::PinRX RX;
	typedef typename props::PinDE DE;

	enum { USARTx_BASE            = Traits::USARTx_BASE };
	enum { USARTx_REMAP           = Traits::USARTx_REMAP };
	enum { USARTx_REMAP_PARTIAL   = Traits::USARTx_REMAP_PARTIAL };
	enum { BUS_FREQ               = Traits::BUS_FREQ };
#if (defined F2xxF4xx)
	static const PinAltFunction ALT_FUNC_USARTx = Traits::ALT_FUNC_USARTx;
#endif

public:
	static IOStruct<USARTx_BASE, USART_TypeDef> USARTx;

	UartDbg();

	INLINE static void EnableClocks()    { Traits::EnableClocks(); }
	INLINE static void DisableClocks()   { Traits::DisableClocks(); }
	INLINE static void Enable()          { USARTx->CR1 |= USART_CR1_UE; }
	INLINE static void Disable()         { USARTx->CR1 &= ~USART_CR1_UE; }
	INLINE static void StartTx()         { DE::On(); }
	INLINE static void EndTx()           { DE::Off(); }

	INLINE static void SetBaudrate(Baudrate value)   { USARTx->BRR = (BUS_FREQ + value/2) / value; }
	INLINE static Baudrate GetBaudrate()             { return BUS_FREQ / USARTx->BRR; }

	virtual void PutChar(char ch) override;
	virtual int GetChar(int timeout = 0) override;
	virtual int Keypressed() override { return USARTx->SR & USART_SR_RXNE; }
	virtual int CanSend() override { return true; };
	virtual int TxEmpty() override { return true; };
};

template<typename props>
UartDbg<props>::UartDbg()
	: TextStream()
{
#if (!defined F2xxF4xx)
	if (remap == REMAP_FULL)        // remap pins if needed
		AFIO->MAPR |= USARTx_REMAP;
	else if (remap == REMAP_PARTIAL)
		AFIO->MAPR |= USARTx_REMAP_PARTIAL;
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
			;
	USARTx->CR2 = 0; // 1 stop
	USARTx->CR3 = 0;

	SetBaudrate(BAUDRATE);

	Enable();        // Enable USART
}

template<class props>
void UartDbg<props>::PutChar(char ch)
{
	while(!(USARTx->SR & USART_SR_TXE)) ;
	USARTx->DR = ch;
}

template<class props>
int UartDbg<props>::GetChar(int timeout)
{
	for(;;)
	{
		if (Keypressed())
			return USARTx->DR;
		if (timeout && !--timeout)
			return -1;
	}
}

} // namespace UART
} // namespace STM32


#endif // STM32TPL_STM32_UART_DBG_H_INCLUDED
