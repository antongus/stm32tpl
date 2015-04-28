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
 *  @file         : stm32_uart_dma.h
 *  @description  : STM32 UART with DMA text stream class template.
 *  created on    : 07.06.2013
 *
 */

#ifndef STM32TPL_STM32_UART_DMA_H_INCLUDED
#define STM32TPL_STM32_UART_DMA_H_INCLUDED

#include "stm32.h"
#include "stm32_uart_driver.h"
#include "textstream.h"
#include "stm32_dma.h"
#include <cstring>
#include <scmRTOS.h>

namespace STM32
{
namespace UART
{

namespace  // private declarations
{

/**
*  UART DMA traits template.
*/
template<UartNum uartNum> struct UartDmaTraits;

/**
*  DMA Traits for UART1.
*/
template<> struct UartDmaTraits<UART_1>
{
#if (defined F2xxF4xx)
	enum { RX_DMA_CHANNEL = DMA::DMA_CR_CHSEL_CH4 };
	enum { TX_DMA_CHANNEL = DMA::DMA_CR_CHSEL_CH4 };

	typedef DMA::Dma2Channel5 RxDmaStream;
	typedef DMA::Dma2Channel7 TxDmaStream;
#elif (defined STM32L0XX)
	typedef DMA::Dma1Channel3 RxDmaStream;
	typedef DMA::Dma1Channel2 TxDmaStream;
	static const RxDmaStream::ChannelSelection CH_SEL_USARTx_RX = RxDmaStream::ChannelSelection::CH_SEL_USART1_RX;
	static const TxDmaStream::ChannelSelection CH_SEL_USARTx_TX = TxDmaStream::ChannelSelection::CH_SEL_USART1_TX;
#else
	typedef DMA::Dma1Channel5 RxDmaStream;
	typedef DMA::Dma1Channel4 TxDmaStream;
#endif
};


/**
*  DMA Traits for UART2.
*/
template<> struct UartDmaTraits<UART_2>
{
#if (defined F2xxF4xx)
	enum { RX_DMA_CHANNEL = DMA::DMA_CR_CHSEL_CH4 };
	enum { TX_DMA_CHANNEL = DMA::DMA_CR_CHSEL_CH4 };

	typedef DMA::Dma1Channel5 RxDmaStream;
	typedef DMA::Dma1Channel6 TxDmaStream;
#elif (defined STM32L0XX)
	typedef DMA::Dma1Channel5 RxDmaStream;
	typedef DMA::Dma1Channel4 TxDmaStream;
	static const RxDmaStream::ChannelSelection CH_SEL_USARTx_RX = RxDmaStream::ChannelSelection::CH_SEL_USART2_RX;
	static const TxDmaStream::ChannelSelection CH_SEL_USARTx_TX = TxDmaStream::ChannelSelection::CH_SEL_USART2_TX;
#else
	typedef DMA::Dma1Channel6 RxDmaStream;
	typedef DMA::Dma1Channel7 TxDmaStream;
#endif
};

/**
*  DMA Traits for UART3.
*/
#if (UART_COUNT > 2)
template<> struct UartDmaTraits<UART_3>
{
#if (defined F2xxF4xx)
	enum { RX_DMA_CHANNEL = DMA::DMA_CR_CHSEL_CH4 };
	enum { TX_DMA_CHANNEL = DMA::DMA_CR_CHSEL_CH4 };

	typedef DMA::Dma1Channel1 RxDmaStream;
	typedef DMA::Dma1Channel3 TxDmaStream;
#else
	enum { RX_DMA_CHANNEL = 0 };
	enum { TX_DMA_CHANNEL = 0 };

	typedef DMA::Dma1Channel3 RxDmaStream;
	typedef DMA::Dma1Channel2 TxDmaStream;
#endif
};
#endif
} // namespace


/**
 * sample UART+DMA properties structure
 */
struct SampleUartDmaProps
{
	static const UartNum uartNum = UART_1;
	static const Remap remap = REMAP_NONE;
	enum
	{
		BAUDRATE = 115200,
		RX_BUF_SIZE = 128,
		UART_INTERRUPT_PRIOGROUP = 2,
		UART_INTERRUPT_SUBPRIO = 2,
		RXDMA_INTERRUPT_PRIOGROUP = 2,
		RXDMA_INTERRUPT_SUBPRIO = 2,
	};
	typedef DummyDE PinDE;
};


/**
*  UART main template.
*/
template<typename props = SampleUartDmaProps>
class UartDma
	: public TextStream
	, public UartDriver<props::uartNum>
{
private:
	typedef UartDmaTraits<props::uartNum> DmaTraits;
public:
	static const UartNum uartNum = props::uartNum;
	static const Remap remap = props::remap;
	enum
	{
		BAUDRATE                      = props::BAUDRATE,
		RX_BUF_SIZE                   = props::RX_BUF_SIZE,
		UART_INTERRUPT_PRIOGROUP      = props::UART_INTERRUPT_PRIOGROUP,
		UART_INTERRUPT_SUBPRIO        = props::UART_INTERRUPT_SUBPRIO,
		RXDMA_INTERRUPT_PRIOGROUP     = props::RXDMA_INTERRUPT_PRIOGROUP,
		RXDMA_INTERRUPT_SUBPRIO       = props::RXDMA_INTERRUPT_SUBPRIO,
	};
	using Driver = UartDriver<uartNum>;
	using Driver::USARTx;

	UartDma();

	INLINE static void StartTx()         { DE::On(); }
	INLINE static void EndTx()           { DE::Off(); }

	virtual void PutChar(char ch) override;
	virtual int GetChar(int timeout = 0) override;
	virtual int Keypressed() override { return rxChannel_.get_count(); }
	virtual int CanSend() override { return true; }
	virtual int TxEmpty() override { return !DE::Latched(); }
	virtual void Puts(const char * s) override;
	virtual void SendBuffer(const void* buf, size_t size) override;
	virtual bool ReceiveBuffer(void* buf, size_t count, int timeout) override;

	void Lock()                          { mutex_.lock(); }
	void Unlock()                        { mutex_.unlock(); }
	bool TryToLock()                     { return mutex_.try_lock(); }


	INLINE void UartIrqHandler();
	INLINE void RxDmaIrqHandler();

private:
	OS::TMutex mutex_;
	OS::TEventFlag txDmaDone_;
	OS::TEventFlag rxDmaDone_;
	OS::channel<char, RX_BUF_SIZE, uint32_t> rxChannel_;
	char rxBuf_[RX_BUF_SIZE];

	using Pins = UartPins<props::uartNum, props::remap>;
	using TX = typename Pins::PinTX;
	using RX = typename Pins::PinRX;
	using DE = typename props::PinDE;

#if (!defined STM32F1XX)
	static const PinAltFunction ALT_FUNC_USARTx = Pins::ALT_FUNC_USARTx;
#endif

	using RxDmaStream = typename DmaTraits::RxDmaStream;
	using TxDmaStream = typename DmaTraits::TxDmaStream;
#if (defined F2xxF4xx)
	enum { RX_DMA_CHANNEL   = DmaTraits::RX_DMA_CHANNEL };
	enum { TX_DMA_CHANNEL   = DmaTraits::TX_DMA_CHANNEL };
#elif (defined STM32L0XX)
	static const typename RxDmaStream::ChannelSelection CH_SEL_USARTx_RX = DmaTraits::CH_SEL_USARTx_RX;
	static const typename TxDmaStream::ChannelSelection CH_SEL_USARTx_TX = DmaTraits::CH_SEL_USARTx_TX;
#endif

	void InitRxDma();
	void InitTxDma();
	void AcceptBlock(char const* block, size_t size);
	void DisableRxDma();
	void EnableRxDma() { RxDmaStream::Enable(); }
};

template<typename props>
UartDma<props>::UartDma()
{
#if (defined STM32F1XX)
	if (remap == REMAP_FULL)        // remap pins if needed
		AFIO->MAPR |= Driver::USARTx_REMAP;
	else if (remap == REMAP_PARTIAL)
		AFIO->MAPR |= Driver::USARTx_REMAP_PARTIAL;
#endif

	Driver::EnableClocks();         // enable UART module clock

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
			| USART_CR1_IDLEIE  // idle interrupt enable
			;
	USARTx->CR2 = 0; // 1 stop
	USARTx->CR3 = USART_CR3_DMAR | USART_CR3_DMAT; // enable DMA for RX and TX

	Driver::SetBaudrate(BAUDRATE);

	Driver::Enable();             // Enable USART

	InitTxDma();

	InitRxDma();

	// Enable USART IRQ
#if (!defined STM32L0XX)
	NVIC_SetPriority(Driver::USARTx_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), UART_INTERRUPT_PRIOGROUP, UART_INTERRUPT_SUBPRIO));
#else
	NVIC_SetPriority(Driver::USARTx_IRQn, UART_INTERRUPT_SUBPRIO);
#endif
	NVIC_EnableIRQ(Driver::USARTx_IRQn);

	// Enable RX DMA IRQ
#if (!defined STM32L0XX)
	NVIC_SetPriority(RxDmaStream::DMAChannel_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), RXDMA_INTERRUPT_PRIOGROUP, RXDMA_INTERRUPT_SUBPRIO));
#else
	NVIC_SetPriority(RxDmaStream::DMAChannel_IRQn, RXDMA_INTERRUPT_SUBPRIO);
#endif
	NVIC_EnableIRQ(RxDmaStream::DMAChannel_IRQn);
}

template<class props>
void UartDma<props>::PutChar(char ch)
{
	SendBuffer(&ch, 1);
}

template<class props>
int UartDma<props>::GetChar(int timeout)
{
	char ch = 0;
	if (rxChannel_.pop(ch, timeout))
		return ch;
	return -1;
}

template<class props>
void UartDma<props>::Puts(const char * s)
{
	SendBuffer(s, strlen(s));
}

/**
 * RX DMA channel initialization.
 */
template<typename props>
void UartDma<props>::InitRxDma()
{
	RxDmaStream::EnableClocks();
#if (defined STM32L0XX)
	RxDmaStream::SelectChannel(CH_SEL_USARTx_RX);
#endif
#if (defined F2xxF4xx)
	RxDmaStream::FCR &= ~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);
	RxDmaStream::M1AR = (uint32_t)rxBuf_;
#endif
#if (defined STM32L0XX)
	RxDmaStream::PAR = (uint32_t)&USARTx->RDR;
#else
	RxDmaStream::PAR = (uint32_t)&USARTx->DR;
#endif
	RxDmaStream::MAR = (uint32_t)rxBuf_;
	RxDmaStream::NDTR = RX_BUF_SIZE;

	RxDmaStream::CR = 0
			| DMA::DMA_CR_DIR_PERITH_TO_MEM // from peripheral to memory
			| DMA::DMA_CR_MINC              // memory increment mode
			| DMA::DMA_CR_MSIZE_8_BIT       // memory size
			| DMA::DMA_CR_PSIZE_8_BIT       // peripheral size
			| DMA::DMA_CR_PRIO_HIGH         // priority
			| DMA::DMA_CR_CIRC              // circular mode
			| DMA::DMA_CR_HTIE              // half-transfer interrupt enable
			| DMA::DMA_CR_TCIE              // transfer-complete interrupt enable
#if (defined F2xxF4xx)
			| RX_DMA_CHANNEL                // select channel (only for F4xx devices)
#endif
			;

	RxDmaStream::Enable();
}

/**
 * TX DMA channel initialization.
 */
template<typename props>
void UartDma<props>::InitTxDma()
{
	TxDmaStream::EnableClocks();
#if (defined STM32L0XX)
	TxDmaStream::SelectChannel(CH_SEL_USARTx_TX);
#endif
#if (defined F2xxF4xx)
	TxDmaStream::FCR &= ~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);  // turn off FIFO
#endif
#if (defined STM32L0XX)
	RxDmaStream::PAR = (uint32_t)&USARTx->TDR;
#else
	TxDmaStream::PAR = (uint32_t)&USARTx->DR;                // peripheral address
#endif
	TxDmaStream::CR = 0
			| DMA::DMA_CR_DIR_MEM_TO_PERITH  // From memory to peripheral
			| DMA::DMA_CR_MINC               // Memory increment mode
			| DMA::DMA_CR_MSIZE_8_BIT        // Memory size
			| DMA::DMA_CR_PSIZE_8_BIT        // Peripheral size
			| DMA::DMA_CR_PRIO_HIGH          // priority
#if (defined F2xxF4xx)
			| TX_DMA_CHANNEL                 // select channel (only for F4xx devices)
#endif
			;
}

template<typename props>
void UartDma<props>::SendBuffer(const void* buf, size_t size)
{
	DE::On();     // Enable transmitter
	DisableRxDma();

	// Clear USART transmission complete flag
	Driver::TcInterrupt::Clear();
	// Enable UART TC interrupt
	USARTx->CR1 |= USART_CR1_TCIE;

	// clear all interrupts on TX DMA channel
	TxDmaStream::IFCR = TxDmaStream::DMA_MASK_ALL;
	// set memory address and size
	TxDmaStream::MAR = (uint32_t)buf;
	TxDmaStream::NDTR = size;
	TxDmaStream::Enable();

	// wait for transfer complete
	txDmaDone_.wait();

	// Disable TX DMA stream
	TxDmaStream::Disable();

	EnableRxDma();
	DE::Off();    // Disable transmitter
}

template<typename props>
bool UartDma<props>::ReceiveBuffer(void* buf, size_t count, int timeout)
{
	char* ptr = reinterpret_cast<char*>(buf);
	return rxChannel_.read(ptr, count, timeout);
}

template<typename props>
void UartDma<props>::AcceptBlock(char const* block, size_t size)
{
	uint32_t freeSize = rxChannel_.get_free_size();
	if (size > freeSize)
		size = freeSize;
	if (size)
		rxChannel_.write(block, size);
}

template<typename props>
void UartDma<props>::DisableRxDma()
{
	if (RxDmaStream::Enabled())
	{
#if (defined F2xxF4xx)
		// disable DMA TC interrupt because it fired when disabling DMA channel (F4 only)
		RxDmaStream::CR &= ~DMA::DMA_CR_TCIE;
#endif
		// turn off DMA stream
		RxDmaStream::Disable();
#if (defined F2xxF4xx)
		// wait for TC interrupt flag and clear it
		while (!(RxDmaStream::ISR & RxDmaStream::DMA_MASK_TCIF)) ;
		RxDmaStream::IFCR = RxDmaStream::DMA_MASK_TCIF;
		// re-enable DMA TC interrupt
		RxDmaStream::CR |= DMA::DMA_CR_TCIE;
#endif
	}
}

template<typename props>
void UartDma<props>::UartIrqHandler()
{
	uint32_t status = Driver::Status();

	// IDLE INTERRUPT
	if (status & USART_FLAG_IDLE)  // IDLEIE is always on, so not need to check this
	{
#if (defined STM32L0XX)
		Driver::ClearStatus(USART_FLAG_IDLE);
#else
		Driver::ReadData();
#endif
		uint32_t rxCount = RX_BUF_SIZE - RxDmaStream::NDTR;
		bool firstHalf = rxCount < RX_BUF_SIZE/2;
		rxCount %= RX_BUF_SIZE/2;
		if (rxCount)
		{
			AcceptBlock(firstHalf ? rxBuf_ : &rxBuf_[RX_BUF_SIZE/2], rxCount);
			DisableRxDma();
			RxDmaStream::NDTR = RX_BUF_SIZE;   // re-launch DMA stream
			EnableRxDma();
		}
	}

	// TRANSMIT COMPLETE INTERRUPT
	if ((status & USART_FLAG_TC) && (USARTx->CR1 & USART_CR1_TCIE))
	{
		// clear interrupt
		Driver::TcInterrupt::Clear();
		// disable it
		Driver::TcInterrupt::Disable();
		// and flag transmission done
		txDmaDone_.signal_isr();
	}
}

template<typename props>
void UartDma<props>::RxDmaIrqHandler()
{
	if (RxDmaStream::ISR & RxDmaStream::DMA_MASK_HTIF)
	{
		RxDmaStream::IFCR = RxDmaStream::DMA_MASK_HTIF;
		AcceptBlock(rxBuf_, RX_BUF_SIZE/2);
	}
	if (RxDmaStream::ISR & RxDmaStream::DMA_MASK_TCIF)
	{
		RxDmaStream::IFCR = RxDmaStream::DMA_MASK_TCIF;
		AcceptBlock(&rxBuf_[RX_BUF_SIZE/2], RX_BUF_SIZE/2);
	}
}

} // namespace UART
} // namespace STM32


#endif // STM32TPL_STM32_UART_DMA_H_INCLUDED
