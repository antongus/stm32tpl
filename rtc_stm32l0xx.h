/**

 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 2015 Anton B. Gusev aka AHTOXA
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
 *  file         : rtc_stm32l0xx.h
 *  description  : RTC module for STM32L0xx series. Used by rtc.h.
 *  created on   : 2015-APR-30
 *
 */

#ifndef STM32TPL_RTC_STM32L0XX_H_INCLUDED
#define STM32TPL_RTC_STM32L0XX_H_INCLUDED

#include "stm32.h"

/**
 * local types
 */
namespace
{
/**
 * RTC Time Register
 */
union RTC_TR_Struct
{
	uint32_t word;
	struct
	{
		uint32_t sec : 7;
		uint32_t     : 1;
		uint32_t min : 7;
		uint32_t     : 1;
		uint32_t hrs : 6;
		uint32_t pm  : 1;
		uint32_t     : 9;
	}
	bits;
};
static_assert(sizeof(RTC_TR_Struct)==sizeof(uint32_t), "Bad RTC_TR_Struct size!");

/**
 * RTC Date Register
 */
union RTC_DR_Struct
{
	uint32_t word;
	struct
	{
		uint32_t day  : 6;
		uint32_t      : 2;
		uint32_t mon  : 5;
		uint32_t dow  : 3;
		uint32_t year : 8;
		uint32_t      : 8;
	}
	bits;
};
static_assert(sizeof(RTC_DR_Struct)==sizeof(uint32_t), "Bad RTC_TR_Struct size!");

} // anonymous namespace


template<bool use_lse = true>
class RtcModule
{
public:
	RtcModule();
	uint32_t ResetReason() { return resetFlags_; }
	static time_t ReadTime(void);
	static bool WriteTime(time_t t);
	static void DisableBdProtection() { PWR->CR |= PWR_CR_DBP; }
	static void EnableBdProtection()  { PWR->CR &= ~PWR_CR_DBP; }
private:
	uint32_t resetFlags_;
	enum { magicKey_ = 0x1970 };
	enum { WAIT_CYCLES = 20000 };

	static bool WaitSync();
	static void DisableWP()   { RTC->WPR = 0xCA; RTC->WPR = 0x53; }
	static void EnableWP()    { RTC->WPR = 0; }
	static bool EnterInitMode();
	static void LeaveInitMode();
	static constexpr uint8_t Int2Bcd(uint8_t value);
	static constexpr uint8_t Bcd2Int(uint8_t value);
};

template<bool use_lse>
RtcModule<use_lse>::RtcModule()
{
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;    // enable PWR clock
	resetFlags_ = RCC->CSR;               // remember reset reason
	RCC->CSR |= RCC_CSR_RMVF;             // clear reset flags

	if (RTC->BKP0R != magicKey_)          // RTC not initialized yet
	{
		DisableBdProtection();            // disable backup domain write protection

		RCC->CSR |= RCC_CSR_RTCRST;       // reset RTC and backup registers
		RCC->CSR &= ~RCC_CSR_RTCRST;

		if (use_lse)
		{
			RCC->CSR |= RCC_CSR_LSEON;                 // enable LSE
			while (!(RCC->CSR & RCC_CSR_LSERDY)) {}    // wait LSE ready
			RCC->CSR |= RCC_CSR_RTCSEL_LSE;            // select LSE as RTC clock source
		}
		else
		{
			RCC->CSR |= RCC_CSR_LSION;                 // enable LSI
			while (!(RCC->CSR & RCC_CSR_LSIRDY)) {}    // wait LSI ready
			RCC->CSR |= RCC_CSR_RTCSEL_LSI;            // select LSI as RTC clock source
		}

		RCC->CSR |= RCC_CSR_RTCEN;                     // enable RTC clock

		WaitSync();
		DisableWP();
		EnterInitMode();

		RTC->CR &= ~RTC_CR_FMT;       // 24 hour format

		// prescaler should be set in two separate writes
		RTC->PRER = 0xFFUL;           // sync prescaler
		RTC->PRER |= 0x7FUL << 16;    // async prescaler

		LeaveInitMode();
		EnableWP();

		WriteTime(1430438400); //  01 MAY 2015 00:00:00

		RTC->BKP0R = magicKey_;

		EnableBdProtection();
	}
	WaitSync();
}

template<bool use_lse>
bool RtcModule<use_lse>::WaitSync()
{
	DisableWP();
	RTC->ISR &= ~RTC_ISR_RSF;
	for (int i = WAIT_CYCLES; i; i--)
		if (RTC->ISR & RTC_ISR_RSF) break;
	EnableWP();
	return RTC->ISR & RTC_ISR_RSF;
}


template<bool use_lse>
bool RtcModule<use_lse>::EnterInitMode()
{
	RTC->ISR = 0xFFFFFFFF;  // avoid clearing flags by reading
	for (int i = WAIT_CYCLES; i; i--) // wait up to 2 RTCCLK clock cycles
		if (RTC->ISR & RTC_ISR_INITF) return true;
	return false;
}

template<bool use_lse>
void RtcModule<use_lse>::LeaveInitMode()
{
	RTC->ISR = ~RTC_ISR_INIT;
}

template<bool use_lse>
constexpr uint8_t RtcModule<use_lse>::Bcd2Int(uint8_t value)
{
	return (value & 0x0F) + (value >> 4) * 10;
}

template<bool use_lse>
constexpr uint8_t RtcModule<use_lse>::Int2Bcd(uint8_t value)
{
	return (value % 10) + ((value / 10) << 4);
}

template<bool use_lse>
time_t RtcModule<use_lse>::ReadTime()
{
	struct tm tim;
	RTC_TR_Struct TR;
	RTC_DR_Struct DR;

	TR.word = RTC->TR;
	tim.tm_sec = Bcd2Int(TR.bits.sec);
	tim.tm_min = Bcd2Int(TR.bits.min);
	tim.tm_hour = Bcd2Int(TR.bits.hrs);

	DR.word = RTC->DR;
	tim.tm_wday = DR.bits.dow - 1;  // Monday=0
	tim.tm_mday = Bcd2Int(DR.bits.day);
	tim.tm_mon = Bcd2Int(DR.bits.mon) - 1;  // January = 0
	tim.tm_year = Bcd2Int(DR.bits.year) + 100;    // year since 1900

//	return mktime(&tim);
	return TimeUtil::mktime(&tim);
}

template<bool use_lse>
bool RtcModule<use_lse>::WriteTime(time_t t)
{
	struct tm tim;

	DisableBdProtection();        // disable backup domain write protection

//	localtime_r(&t, &tim);
	TimeUtil::localtime(t, &tim);

	RTC_TR_Struct TR;
	RTC_DR_Struct DR;

	TR.bits.sec  = Int2Bcd(tim.tm_sec);
	TR.bits.min  = Int2Bcd(tim.tm_min);
	TR.bits.hrs  = Int2Bcd(tim.tm_hour);

	DR.bits.dow  = tim.tm_wday + 1;
	DR.bits.day  = Int2Bcd(tim.tm_mday);
	DR.bits.mon  = Int2Bcd(tim.tm_mon + 1);
	DR.bits.year = Int2Bcd(tim.tm_year % 100);

	DisableWP();

	bool ret = EnterInitMode();
	if (ret)
	{
		RTC->TR = TR.word;
		RTC->DR = DR.word;
	}
	LeaveInitMode();
	if (ret && !(RTC->CR & RTC_CR_BYPSHAD))
		ret = WaitSync();
	EnableWP();
	EnableBdProtection();
	return ret;
}

#endif // STM32TPL_RTC_STM32L0XX_H_INCLUDED
