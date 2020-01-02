/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *  Visit https://github.com/antongus/stm32tpl for new versions
 *
 *  Copyright (c) 2011-2020 Anton B. Gusev aka AHTOXA
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
 *  file         : pwr_stm32l0xx.h
 *  description  : PWR module for STM32L0xx series.
 *
 */

#ifndef STM32TPL_PWR_STM32L0XX_H_INCLUDED
#define STM32TPL_PWR_STM32L0XX_H_INCLUDED

#include "stm32.h"

enum PowerRange
{
	POWER_RANGE_1_32MHZ,
	POWER_RANGE_2_16MHZ,
	POWER_RANGE_3_4MHZ
};

enum VoltageRange
{
	VOLTAGE_RANGE_1_32MHZ = 1,
	VOLTAGE_RANGE_2_16MHZ = 2,
	VOLTAGE_RANGE_3_4MHZ  = 3,
};

enum PowerMode
{
	POWER_MODE_RUN,
	POWER_MODE_LP_RUN,   //
	POWER_MODE_SLEEP,    // 1mA with all peripherals off
	POWER_MODE_LP_SLEEP,
	POWER_MODE_STOP_RTC,
	POWER_MODE_STOP,
	POWER_MODE_STANDBY_RTC,
	POWER_MODE_STANDBY,
};

struct SamplePowerProps
{
	static const bool UseHse = true;
	enum
	{
		QUARTZ_FREQ   =  8000000UL,
		RUN_FREQ      = 32000000UL,
	};
};

template <class props = SamplePowerProps>
class Power
{
public:
	static void EnableClocks()  { RCC->APB1ENR |= RCC_APB1ENR_PWREN; __DSB(); }
	static void DisableClocks() { RCC->APB1ENR &= ~RCC_APB1ENR_PWREN; }

	static void SetVoltageRange(int range)
	{
		// select voltage scaling range 1 (1.8 v, up to 32 MHz)
		PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_0;
	}

	static PowerRange GetPowerRange();
	static void SetPowerRange(PowerRange range);

	static PowerMode GetPowerMode();
	static void SetPowerMode(PowerMode mode);
	static bool GetDeepSleep() { return SCB->SCR & SCB_SCR_SLEEPDEEP_Msk; }
	static void SetDeepSleep(bool val)
	{
		if (val)
			SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
		else
			SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	}
	struct HSI
	{
		static void Enable()
		{
			RCC->CR |= RCC_CR_HSION;
			while (!(RCC->CR & RCC_CR_HSIRDY)) {}
		}
		static void Disable()   { RCC->CR &= ~RCC_CR_HSION; }
		static bool IsEnabled() { return RCC->CR & RCC_CR_HSION; }
		static void Select()    { RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI; }
	};

	struct HSE
	{
		static void Enable()
		{
			RCC->CR |= RCC_CR_HSEON;
			while (!(RCC->CR & RCC_CR_HSERDY)) {}
		}
		static void Disable()   { RCC->CR &= ~RCC_CR_HSEON; }
		static bool IsEnabled() { return RCC->CR & RCC_CR_HSEON; }
		static void Select()    { RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSE; }
	};

	struct PLL
	{
		static void Enable()
		{
			RCC->CR |= RCC_CR_PLLON;
			while (!(RCC->CR & RCC_CR_PLLRDY)) {}
		}
		static void Disable()   { RCC->CR &= ~RCC_CR_PLLON; }
		static bool IsEnabled() { return RCC->CR & RCC_CR_PLLON; }
		static void Select()    { RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL; }
	};

	static void EnterStopMode()
	{
		HSI::Enable();   // Enable HSI
		HSI::Select();   // switch to HSI
		PLL::Disable();  // disable PLL
	}

};



#endif // STM32TPL_PWR_STM32L0XX_H_INCLUDED
