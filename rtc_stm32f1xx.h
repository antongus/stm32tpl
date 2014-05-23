/**
*  @file rtc_stm32f1xx.h
*  STM32F1xx RTC driver
*
*  Copyright (c) 2010-2013 by Anton Gusev aka AHTOXA
*
*/

#ifndef RTC_STM32F1XX_H_INCLUDED
#define RTC_STM32F1XX_H_INCLUDED

#include "stm32.h"

template<bool use_lse = true>
class RtcModule
{
private:
	uint32_t resetFlags_;
	enum { magicKey_ = 0x1970 };

	/**
	 * wait until APB1 registers became synchronized with internal RTC registers
	 */
	void WaitSync(void)
	{
		RTC->CRL &= ~RTC_CRL_RSF;
		while (!(RTC->CRL & RTC_CRL_RSF)) ;
	}

	/**
	 * wait until all previous write operations finished
	 */
	void WaitReady(void)
	{
		while (!(RTC->CRL & RTC_CRL_RTOFF)) ;
	}

	inline time_t ReadCounter(void)
	{
		return  ((uint32_t)RTC->CNTH << 16) | (uint32_t)RTC->CNTL;
	}

public:
	RtcModule(void);
	time_t ReadTime();
	bool WriteTime(time_t t);
	uint32_t ResetReason() { return resetFlags_; }
	uint8_t GetCorrection();
	void SetCorrection(uint8_t value);
};

template<bool use_lse>
RtcModule<use_lse>::RtcModule()
{
	// enable RTC-related clocks
	RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
	// remember reset reason
	resetFlags_ = RCC->CSR;
	// clear reset flags
	RCC->CSR |= RCC_CSR_RMVF;

	if (BKP->DR1 != magicKey_)
	{
		// disable backup domain write protection
		PWR->CR |= PWR_CR_DBP;

		// reset Backup Domain
		RCC->BDCR |= RCC_BDCR_BDRST;
		RCC->BDCR &= ~RCC_BDCR_BDRST;

		RCC->BDCR &= ~(RCC_BDCR_LSEON | RCC_BDCR_LSEBYP);

		if (use_lse)
		{
			// enable LSE
			RCC->BDCR |= RCC_BDCR_LSEON;
			// wait till LSE is ready
			while (!(RCC->BDCR & RCC_BDCR_LSERDY)) ;
			// select LSE as RTC clock source
			RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;
		}
		else
		{
			// enable LSI
			RCC->CSR |= RCC_CSR_LSION;
			// wait till LSI is ready
			while (!(RCC->CSR & RCC_CSR_LSIRDY)) ;
			// select LSI as RTC clock source
			RCC->BDCR |= RCC_BDCR_RTCSEL_LSI;
		}

		// enable RTC clock
		RCC->BDCR |= RCC_BDCR_RTCEN;

		// wait for RTC registers synchronization
		WaitSync();
		WaitReady();

		// set RTC prescaler: set RTC period to 1sec
		RTC->CRL |= RTC_CRL_CNF;	// enter config mode
		RTC->PRLH = 0;
		RTC->PRLL = 32767;
		RTC->CRL &= ~RTC_CRL_CNF;	// exit config mode

//		wait_ready();
		BKP->DR1 = magicKey_;

		// enable backup domain write protection
		PWR->CR &= ~PWR_CR_DBP;

		WriteTime(1351551600); // 2012-OCT-29 23:00
	}
	else
	{
		WaitSync();
	}

	// enable the RTC second interrupt
//		RTC->CRH |= RTC_CRH_SECIE;
//		WaitReady();
}

template<bool use_lse>
bool RtcModule<use_lse>::WriteTime(time_t value)
{
//	WaitReady();
	PWR->CR |= PWR_CR_DBP;
	RTC->CRL |= RTC_CRL_CNF;	// enter config mode
	RTC->CNTH = value >> 16;
	RTC->CNTL = value;
	RTC->CRL &= ~RTC_CRL_CNF;	// exit config mode
	WaitReady();
	PWR->CR &= ~PWR_CR_DBP;
	return true;
}

template<bool use_lse>
time_t RtcModule<use_lse>::ReadTime()
{
	time_t ret = ReadCounter();
	for (;;)
	{
		time_t ret1 = ReadCounter();
		if (ret1 == ret)
			return ret;
		ret = ret1;
	}
}

template<bool use_lse>
uint8_t RtcModule<use_lse>::GetCorrection()
{
	return BKP->RTCCR & 0x7F;
}

template<bool use_lse>
void RtcModule<use_lse>::SetCorrection(uint8_t value)
{
	PWR->CR |= PWR_CR_DBP;
	BKP->RTCCR = (BKP->RTCCR & ~0x7F) | (value & 0x7F);
	PWR->CR &= ~PWR_CR_DBP;
}

#endif // RTC_STM32F1XX_H_INCLUDED
