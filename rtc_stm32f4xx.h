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
 *  file         : rtc_stm32f4xx.h
 *  description  : RTC module for stm32f4xx series. Used by rtc.h.
 *  created on   : 15.05.2013
 *
 */

#ifndef RTC_STM32F4XX_H_INCLUDED
#define RTC_STM32F4XX_H_INCLUDED

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
	}__attribute__ ((packed))
	bits;
};

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
	}__attribute__ ((packed))
	bits;
};

} // anonymous namespace


template<bool use_lse = true>
class RtcModule
{
private:
	uint32_t resetFlags_;
	enum { magicKey_ = 0x1970 };
	enum { WAIT_CYCLES = 20000 };

	bool waitSync();

	void disableWP()
	{
		RTC->WPR = 0xCA;
		RTC->WPR = 0x53;
	}
	void enableWP()
	{
		RTC->WPR = 0;
	}
	bool enterInitMode();
	void leaveInitMode();
	uint8_t int2bcd(uint8_t value);
	uint8_t bcd2int(uint8_t value);

	static PeriphBit<PWR_BASE, 8> PWR_CR_DBP_BIT;

protected:
	time_t readTime(void);
	bool writeTime(time_t t);
public:
	RtcModule();
	uint32_t reset_reason() { return resetFlags_; }
};

template<bool use_lse>
RtcModule<use_lse>::RtcModule()
{
	// enable PWR clock
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	// remember reset reason
	resetFlags_ = RCC->CSR;
	// clear reset flags
	RCC->CSR |= RCC_CSR_RMVF;

//	if (RTC->ISR & RTC_ISR_INITS == 0)  // RTC not initialized yet
	if (RTC->BKP0R != magicKey_) // RTC not initialized yet
	{
		// disable backup domain write protection
		PWR->CR |= PWR_CR_DBP;

		// reset Backup Domain
		RCC->BDCR |= RCC_BDCR_BDRST;
		RCC->BDCR &= ~RCC_BDCR_BDRST;

		// turn off LSE and LSE Bypass
		RCC->BDCR &= ~(RCC_BDCR_LSEON | RCC_BDCR_LSEBYP);

		if (use_lse)
		{
			// enable LSE
			RCC->BDCR |= RCC_BDCR_LSEON;
			// wait till LSE is ready
			while (!(RCC->BDCR & RCC_BDCR_LSERDY)) ;
			// select LSE as RTC clock source
			RCC->BDCR |= RCC_BDCR_RTCSEL_0;
		}
		else
		{
			// enable LSI
			RCC->CSR |= RCC_CSR_LSION;
			// wait till LSI is ready
			while (!(RCC->CSR & RCC_CSR_LSIRDY)) ;
			// select LSI as RTC clock source
			RCC->BDCR |= RCC_BDCR_RTCSEL_1;
		}

		// enable RTC clock
		RCC->BDCR |= RCC_BDCR_RTCEN;

		waitSync();
		disableWP();
		enterInitMode();

		RTC->CR &= ~RTC_CR_FMT;       // 24 hour format

		// prescaler should be set in two separate writes
		RTC->PRER = 0xFFUL;           // sync prescaler
		RTC->PRER |= 0x7FUL << 16;    // async prescaler

		leaveInitMode();
		enableWP();

		writeTime(1356998400); //  Tue, 01 JAN 2013 00:00:00

		RTC->BKP0R = magicKey_;
	}
	else
	{
		waitSync();
	}
}

template<bool use_lse>
bool RtcModule<use_lse>::waitSync()
{
	disableWP();
	RTC->ISR &= ~RTC_ISR_RSF;
	for (int i = WAIT_CYCLES; i; i--)
		if (RTC->ISR & RTC_ISR_RSF) break;
	enableWP();
	return RTC->ISR & RTC_ISR_RSF;
}


template<bool use_lse>
bool RtcModule<use_lse>::enterInitMode()
{
	RTC->ISR = 0xFFFFFFFF;  // avoid clearing flags by reading
	for (int i = WAIT_CYCLES; i; i--) // wait up to 2 RTCCLK clock cycles
		if (RTC->ISR & RTC_ISR_INITF) return true;
	return false;
}

template<bool use_lse>
void RtcModule<use_lse>::leaveInitMode()
{
	RTC->ISR = ~RTC_ISR_INIT;
}

template<bool use_lse>
uint8_t RtcModule<use_lse>::bcd2int(uint8_t value)
{
	return (value & 0x0F) +(value>>4)*10;
}

template<bool use_lse>
uint8_t RtcModule<use_lse>::int2bcd(uint8_t value)
{
	return (value % 10) + ((value/10)<<4);
}

template<bool use_lse>
time_t RtcModule<use_lse>::readTime()
{
	struct tm tim;
	RTC_TR_Struct TR;
	RTC_DR_Struct DR;

	TR.word = RTC->TR;
	tim.tm_sec = bcd2int(TR.bits.sec);
	tim.tm_min = bcd2int(TR.bits.min);
	tim.tm_hour = bcd2int(TR.bits.hrs);

	DR.word = RTC->DR;
	tim.tm_wday = DR.bits.dow - 1;  // Monday=0
	tim.tm_mday = bcd2int(DR.bits.day);
	tim.tm_mon = bcd2int(DR.bits.mon) - 1;  // January = 0
	tim.tm_year = bcd2int(DR.bits.year) + 100;    // year since 1900

	return TimeUtil::tm_2_t(&tim);
}

template<bool use_lse>
bool RtcModule<use_lse>::writeTime(time_t t)
{
	struct tm tim;

	TimeUtil::t_2_tm(t, &tim);

	RTC_TR_Struct TR;
	RTC_DR_Struct DR;

	TR.bits.sec  = int2bcd(tim.tm_sec);
	TR.bits.min  = int2bcd(tim.tm_min);
	TR.bits.hrs  = int2bcd(tim.tm_hour);

	DR.bits.dow  = tim.tm_wday + 1;
	DR.bits.day  = int2bcd(tim.tm_mday);
	DR.bits.mon  = int2bcd(tim.tm_mon + 1);
	DR.bits.year = int2bcd(tim.tm_year % 100);

	disableWP();

	bool ret = enterInitMode();
	if (ret)
	{
		RTC->TR = TR.word;
		RTC->DR = DR.word;
	}
	leaveInitMode();
	if (ret && !(RTC->CR & RTC_CR_BYPSHAD))
		ret = waitSync();
	enableWP();
	return ret;
}

#endif // RTC_STM32F4XX_H_INCLUDED
