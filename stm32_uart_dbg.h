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
#include "stm32_uart_driver.h"
#include "textstream.h"

namespace STM32
{
namespace UART
{

/**
 * sample debug UART properties structure
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
	, public UartDriver<props::uartNum>
{
public:
	static const UartNum uartNum = props::uartNum;
	static const Remap remap = props::remap;
	enum { BAUDRATE = props::BAUDRATE };
	using Driver = UartDriver<uartNum>;
	using Driver::USARTx;

	UartDbg();

	INLINE static void StartTx()         { DE::On(); }
	INLINE static void EndTx()           { DE::Off(); }

	virtual void PutChar(char ch) override;
	virtual int GetChar(int timeout = 0) override;
	virtual int Keypressed() override  { return Driver::Status() & USART_FLAG_RXNE; }
	virtual int CanSend() override { return true; };
	virtual int TxEmpty() override { return true; };
private:
	using Pins = UartPins<props::uartNum, props::remap>;
	using DE = typename props::PinDE;
};

template<typename props>
UartDbg<props>::UartDbg()
{
#if (defined STM32F1XX)
	if (remap == REMAP_FULL)        // remap pins if needed
		AFIO->MAPR |= Driver::USARTx_REMAP;
	else if (remap == REMAP_PARTIAL)
		AFIO->MAPR |= Driver::USARTx_REMAP_PARTIAL;
#endif

	Driver::EnableClocks();         // enable UART module clock
	Pins::Init();                   // configure pins
	DE::Mode(OUTPUT);
	DE::Off();

	USARTx->CR1 = 0
			| USART_CR1_RE      // receive enable
			| USART_CR1_TE      // transmit enable
			;
	USARTx->CR2 = 0; // 1 stop
	USARTx->CR3 = 0;

	Driver::SetBaudrate(BAUDRATE);

	Driver::Enable();             // Enable USART
}

template<class props>
void UartDbg<props>::PutChar(char ch)
{
	while (!(Driver::Status() & USART_FLAG_TXE)) ;
	Driver::WriteData(ch);
}

template<class props>
int UartDbg<props>::GetChar(int timeout)
{
	for(;;)
	{
		if (Keypressed())
			return Driver::ReadData();
		if (timeout && !--timeout)
			return -1;
	}
}

} // namespace UART
} // namespace STM32


#endif // STM32TPL_STM32_UART_DBG_H_INCLUDED
