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
#include "stm32_uart_pins.h"
#include "textstream.h"
#include <scmRTOS.h>

namespace STM32
{
namespace UART
{

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
	typedef DummyDE PinDE;
};


/**
*  UART main template.
*/
template<typename props = SampleUartProps>
class Uart
	: public TextStream
	, public UartBase
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

	enum { USARTx_BASE            = Traits::USARTx_BASE };
	enum { USARTx_REMAP           = Traits::USARTx_REMAP };
	enum { USARTx_REMAP_PARTIAL   = Traits::USARTx_REMAP_PARTIAL };
	enum { BUS_FREQ               = Traits::BUS_FREQ };
#if (!defined STM32F1XX)
	static const PinAltFunction ALT_FUNC_USARTx = Pins::ALT_FUNC_USARTx;
#endif
public:
//	static IOStruct<USARTx_BASE, USARTx_TypeDef> USARTx;
	Uart();

	INLINE static void EnableClocks()    { Traits::EnableClocks(); }
	INLINE static void DisableClocks()   { Traits::DisableClocks(); }
	INLINE static void StartTx()         { DE::On(); }
	INLINE static void EndTx()           { DE::Off(); }

	INLINE void SetBaudrate(Baudrate value)   { USARTx->BRR = (BUS_FREQ + value/2) / value; }
	INLINE Baudrate GetBaudrate()             { return BUS_FREQ / USARTx->BRR; }

	virtual void PutChar(char ch) override;
	virtual int GetChar(int timeout = 0) override;
	virtual int Keypressed() override { return rxChannel_.get_count(); }
	virtual int CanSend() override { return txChannel_.get_free_size(); };
	virtual int TxEmpty() override { return txChannel_.get_count() == 0; };

	void SendBuffer(const void* buf, size_t size);
	bool ReceiveBuffer(void* buf, size_t count, timeout_t timeout);

	INLINE void UartIrqHandler();
};

template<class props>
Uart<props>::Uart()
	: TextStream()
	, UartBase(reinterpret_cast<USARTx_TypeDef *const>(USARTx_BASE))
	, rxChannel_()
	, txChannel_()
	, txDone_()
{
#if (defined STM32F1XX)
	if (remap == REMAP_FULL)        // remap pins if needed
		AFIO->MAPR |= USARTx_REMAP;
	else if (remap == REMAP_PARTIAL)
		AFIO->MAPR |= USARTx_REMAP_PARTIAL;
#endif

	EnableClocks();                 // enable UART module clock

#if (defined STM32F1XX)             // configure pins
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

	Enable();        // Enable USART

#if (!defined STM32L0XX)
	NVIC_SetPriority(USARTx_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), UART_INTERRUPT_PRIOGROUP, UART_INTERRUPT_SUBPRIO));
#else
	NVIC_SetPriority(USARTx_IRQn, UART_INTERRUPT_SUBPRIO);
#endif
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

template<typename props>
bool Uart<props>::ReceiveBuffer(void* buf, size_t count, timeout_t timeout)
{
	char* ptr = reinterpret_cast<char*>(buf);
	return rxChannel_.read(ptr, count, timeout);
}

template<class props>
void Uart<props>::UartIrqHandler()
{
#if (defined STM32L0XX)
	uint32_t status = ReadStatus();
	uint8_t data = ReadData();

	// RX NOT EMPTY INTERRUPT
	if (status & USART_ISR_RXNE)
	{
		if (rxChannel_.get_free_size())
			rxChannel_.push(data);
	}

	// TX EMPTY INTERRUPT
	if ((status & USART_ISR_TXE) && (USARTx->CR1 & USART_CR1_TXEIE))
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
			USARTx->TDR = ch;
		}
		else
		{
			DisableTxInterrupt();
			EnableTcInterrupt();
		}
	}

	// TRANSMIT COMPLETE INTERRUPT
	if ((status & USART_ISR_TC) && (USARTx->CR1 & USART_CR1_TCIE))
	{
		// clear interrupt
		USARTx->ICR = USART_ICR_TCCF;
		// disable it
		DisableTcInterrupt();
		// turn off transmitter
		DE::Off();
		// and flag transmission done
		txDone_.signal_isr();
	}
#else
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
#endif
}

} // namespace UART
} // namespace STM32

#endif // STM32TPL_STM32_UART_H_INCLUDED
