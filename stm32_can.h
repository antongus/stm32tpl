/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 2010-2016 Anton B. Gusev aka AHTOXA
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
 *  @file         : stm32_can.h
 *  @description  : STM32 CAN module driver.
 *  created on    : 01.11.2016
 *
 */

#ifndef STM32TPL_STM32_CAN_H_INCLUDED
#define STM32TPL_STM32_CAN_H_INCLUDED

#include "stm32.h"
#include "pin.h"

namespace STM32
{

namespace CANx
{

/**
 * CAN pins remap enumeration
 */
enum Remap { REMAP_B8_B9, REMAP_A11_A12, REMAP_D0_D1 };

/**
 * Can pins structure
 */
template<Remap remapped> struct CanPins;

template<> struct CanPins<REMAP_B8_B9>
{
	using RxPin = Pin<'B', 8>;
	using TxPin = Pin<'B', 9>;
#if (!defined STM32TPL_STM32F1XX)
	static const PinAltFunction ALT_FUNC_CANx = ALT_FUNC_4;
#endif
};

template<> struct CanPins<REMAP_A11_A12>
{
	using RxPin = Pin<'A', 11>;
	using TxPin = Pin<'A', 12>;
#if (!defined STM32TPL_STM32F1XX)
	static const PinAltFunction ALT_FUNC_CANx = ALT_FUNC_4;
#endif
};

template<> struct CanPins<REMAP_D0_D1>
{
	using RxPin = Pin<'D', 0>;
	using TxPin = Pin<'D', 1>;
#if (!defined STM32TPL_STM32F1XX)
	static const PinAltFunction ALT_FUNC_CANx = ALT_FUNC_0;
#endif
};



/**
 * CAN hardware registers
 */
struct TxMailBoxRegs
{
	volatile uint32_t TIR;    //!< CAN TX mailbox identifier register
	volatile uint32_t TDTR;   //!< CAN mailbox data length control and time stamp register
	volatile uint32_t TDLR;   //!< CAN mailbox data low register
	volatile uint32_t TDHR;   //!< CAN mailbox data high register
};

struct FifoMailBox
{
	volatile uint32_t RIR;  //!< CAN receive FIFO mailbox identifier register
	volatile uint32_t RDTR; //!< CAN receive FIFO mailbox data length control and time stamp register
	volatile uint32_t RDLR; //!< CAN receive FIFO mailbox data low register
	volatile uint32_t RDHR; //!< CAN receive FIFO mailbox data high register
};

struct FilterBank
{
	volatile uint32_t FR1;    //!< CAN Filter bank register 1
	volatile uint32_t FR2;    //!< CAN Filter bank register 2
};

struct CANx_TypeDef
{
	volatile uint32_t MCR;    //!< CAN master control register
	volatile uint32_t MSR;    //!< CAN master status register
	volatile uint32_t TSR;    //!< CAN transmit status register
	volatile uint32_t RF0R;   //!< CAN receive FIFO 0 register
	volatile uint32_t RF1R;   //!< CAN receive FIFO 1 register
	volatile uint32_t IER;    //!< CAN interrupt enable register
	volatile uint32_t ESR;    //!< CAN error status register
	volatile uint32_t BTR;    //!< CAN bit timing register
	uint32_t RESERVED0[88];   //!< Reserved, 0x020 - 0x17F
	TxMailBoxRegs txMailBoxes[3]; //!< CAN TX MailBoxes
	FifoMailBox fifoMailBoxes[2]; //!< CAN FIFO MailBoxes
	uint32_t RESERVED1[12];   //!< Reserved, 0x1D0 - 0x1FF
	volatile uint32_t FMR;    //!< CAN filter master register
	volatile uint32_t FM1R;   //!< CAN filter mode register
	uint32_t RESERVED2;       //!< Reserved, 0x208
	volatile uint32_t FS1R;   //!< CAN filter scale register
	uint32_t RESERVED3;       //!< Reserved, 0x210
	volatile uint32_t FFA1R;  //!< CAN filter FIFO assignment register
	uint32_t RESERVED4;       //!< Reserved, 0x218
	volatile uint32_t FA1R;   //!< CAN filter activation register
	uint32_t RESERVED5[8];    //!< Reserved, 0x220-0x23F
	FilterBank filters[14];   //!< CAN Filter Registers
};

/// filter scale enum
enum FilterScale
{
	fs11bit,   //!< filter scale is 11 bit
	fs29bit    //!< filter scale is 29 bit
};

/// filter Mode enum
enum FilterMode
{
	fmMask,  //!< filter is in mask mode
	fmList   //!< filter is in list mode
};

/// filter FIFO enum (0, 1)
enum FilterFifo
{
	fifo0,    //!< FIFO 0 is assigned to filter
	fifo1,    //!< FIFO 1 is assigned to filter
};


/**
 * USART driver
 */
template<class props>
struct CanModule
{
	static const IRQn CanModuleIRQn  = CEC_CAN_IRQn;

	using Pins = CanPins<props::remapped>;
	using TxPin = typename Pins::TxPin;
	using RxPin = typename Pins::RxPin;

#if (!defined STM32TPL_STM32F1XX)
	static const PinAltFunction ALT_FUNC_CANx = Pins::ALT_FUNC_CANx;
#endif
	static void InitPins()
	{
#if (defined STM32TPL_STM32F1XX)
		TxPin::Mode(ALT_OUTPUT);
		RxPin::Mode(INPUTPULLED);
		RxPin::PullUp();
#else
		TxPin::Alternate(ALT_FUNC_CANx);
		RxPin::Alternate(ALT_FUNC_CANx);
		TxPin::Mode(ALT_OUTPUT);
		RxPin::Mode(ALT_INPUT_PULLUP);
#endif
	}
	static void DeinitPins()
	{
#if (defined STM32TPL_STM32F1XX)
		TxPin::Mode(ANALOGINPUT);
		RxPin::Mode(ANALOGINPUT);
		RxPin::PullUp();
#else
		TxPin::Alternate((PinAltFunction)0);
		RxPin::Alternate((PinAltFunction)0);
		TxPin::Mode(INPUT);
		RxPin::Mode(INPUT);
		RxPin::PullNone();
#endif
	}

	enum
	{
		CANx_BASE = 0x40006400UL,
	};
	static IOStruct<CANx_BASE, CANx_TypeDef> CANx;

	INLINE static void EnableClocks()  { RCC->APB1ENR |= RCC_APB1ENR_CANEN;  __DSB(); }
	INLINE static void DisableClocks() { RCC->APB1ENR &= ~RCC_APB1ENR_CANEN; __DSB(); }
	INLINE static void Reset()         { RCC->APB1RSTR |= RCC_APB1RSTR_CANRST; RCC->APB1RSTR &= ~RCC_APB1RSTR_CANRST; }
	INLINE static bool EnterInitMode(uint32_t timeout = 0xFFFFF)
	{
		CANx->MCR |= CAN_MCR_INRQ;
		for (uint32_t i = 0; i < timeout; ++i)
		{
			if (CANx->MSR & CAN_MSR_INAK)
				return true;
		}
		return false;
	}
	INLINE static bool ExitInitMode(uint32_t timeout = 0xFFFFF)
	{
		CANx->MCR &= ~CAN_MCR_INRQ;
		for (uint32_t i = 0; i < timeout; ++i)
		{
			if (!(CANx->MSR & CAN_MSR_INAK))
				return true;
		}
		return false;
	}
	INLINE static void EnterSleepMode() { CANx->MCR |= CAN_MCR_SLEEP; }
	INLINE static void ExitSleepMode()  { CANx->MCR &= ~CAN_MCR_SLEEP; }

	INLINE static void EnterFilterInitMode() { CANx->FMR |= CAN_FMR_FINIT; }
	INLINE static void ExitFilterInitMode()  { CANx->FMR &= ~CAN_FMR_FINIT; }

	INLINE static void ConfigureFilter(unsigned number,
			uint32_t id,
			uint32_t mask,
			FilterMode mode,
			FilterScale scale,
			FilterFifo fifo)
	{
		const uint32_t fMask = 1UL << number;
//		CANx->FA1R &= ~mask; // deactivate filter (not needed if in filter initialization mode)
		CANx->filters[number].FR1 = id;
		CANx->filters[number].FR2 = mask;
		CANx->FM1R = (CANx->FM1R & ~fMask) | (mode << number);   // set filter mode (mask/list)
		CANx->FS1R = (CANx->FS1R & ~fMask) | (scale << number);  // set filter scale (11 bit/29 bit)
		CANx->FFA1R = (CANx->FFA1R & ~fMask) | (fifo << number);  // assign FIFO for filter
		CANx->FA1R |= fMask;
	}

	template <FilterMode mode, FilterScale scale, FilterFifo fifo>
	INLINE static void ConfigureFilter(
			unsigned number,
			uint32_t id,
			uint32_t mask)
	{
		const uint32_t fMask = 1UL << number;
//		CANx->FA1R &= ~mask; // deactivate filter (not needed if in filter initialization mode)
		CANx->filters[number].FR1 = id;
		CANx->filters[number].FR2 = mask;
		CANx->FM1R = (CANx->FM1R & ~fMask) | (mode << number);   // set filter mode (mask/list)
		CANx->FS1R = (CANx->FS1R & ~fMask) | (scale << number);  // set filter scale (11 bit/29 bit)
		CANx->FFA1R = (CANx->FFA1R & ~fMask) | (fifo << number);  // assign FIFO for filter
		CANx->FA1R |= fMask;
	}

	INLINE static void DisableFilter(unsigned number)
	{
		const uint32_t fMask = 1UL << number;
		CANx->FA1R &= ~fMask;
	}

	struct TxEmptyInterrupt
	{
		static void Enable()    { CANx->IER |= CAN_IER_TMEIE; }
		static void Disable()   { CANx->IER &= ~CAN_IER_TMEIE; }
		static bool IsEnabled() { return CANx->IER & CAN_IER_TMEIE; }
	};

	struct RxFifo0NotEmptyInterrupt
	{
		static void Enable()    { CANx->IER |= CAN_IER_FMPIE0; }
		static void Disable()   { CANx->IER &= ~CAN_IER_FMPIE0; }
		static bool IsEnabled() { return CANx->IER & CAN_IER_FMPIE0; }
	};

};

} // namespace CANx
} // namespace STM32


#endif // STM32TPL_STM32_CAN_H_INCLUDED
