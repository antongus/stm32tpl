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
 *  file         : rtc.h
 *  description  : RTC module class template for STM32 series.
 *
 */

#ifndef STM32TPL_RTC_H_INCLUDED
#define STM32TPL_RTC_H_INCLUDED

#include "stm32.h"

#include <ctime>

static const char monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31};

struct TimeUtil
{
	static bool IsLeapYear(uint32_t y) { return (y % 4) == 0; }
	static void CheckTime(struct tm *t)  __attribute__((noinline))
	{
	    if (t->tm_sec>59) t->tm_sec=59;
	    if (t->tm_min>59) t->tm_min=59;
	    if (t->tm_hour>23) t->tm_hour=23;
	    if (t->tm_wday>6) t->tm_wday=6;
	    if (t->tm_mday<1) t->tm_mday=1;
	    else if (t->tm_mday>31) t->tm_mday=31;
	    if (t->tm_mon>11) t->tm_mon=11;
	    if (t->tm_year<0) t->tm_year=0;
	}

	static struct tm *localtime(time_t t, struct tm * stm) __attribute__((noinline))
	{
		stm->tm_sec = t % 60;
		t /= 60;
		stm->tm_min = t % 60;
		t /= 60;
		stm->tm_hour = t % 24;
		t /= 24;
		stm->tm_wday = (t + 4) % 7;

		uint32_t year = 1970;
		time_t days = 0;

		while((days += (IsLeapYear(year) ? 366 : 365)) <= t)
			year++;

		stm->tm_year = year - 1900;

		days -= IsLeapYear(year) ? 366 : 365;
		t -= days;
		stm->tm_yday = t;

		for (int month = 0; month < 12; month++)
		{
			if (month == 1) // feb
				if (IsLeapYear(year))
					days = 29;
				else
					days = 28;
			else
				days = monthDays[month];

			if (t >= days)
				t -= days;
			else
			{
				stm->tm_mon = month;
				stm->tm_mday = t+1;
				break;
			}
		}
		return stm;
	}

	static time_t mktime(struct tm *t)   __attribute__((noinline))
	{
		int year, month, i;
		uint32_t seconds;

		CheckTime(t);

		year    = t->tm_year + 1900;
		month   = t->tm_mon;
		seconds = (uint32_t)(year - 1970) * (60*60*24UL*365);

		for (i = 1970; i < year; i++)
			if (IsLeapYear(i))
				seconds += 60*60*24UL;

		// add days for this year
		for (i = 0; i < month; i++)
			if (i == 1 && IsLeapYear(year))
				seconds += (uint32_t)60*60*24UL*29;
			else
				seconds += (uint32_t)60*60*24UL*monthDays[i];

		seconds += (uint32_t)(t->tm_mday-1) * 60*60*24UL;
		seconds += (uint32_t)t->tm_hour * 60*60UL;
		seconds += (uint32_t)t->tm_min * 60UL;
		seconds += (uint32_t)t->tm_sec;
		return seconds;
	}

	time_t date(time_t t)
	{
		struct tm stm;
		localtime(t, &stm);
		stm.tm_hour = 0;
		stm.tm_min = 0;
		stm.tm_sec = 0;
		return mktime(&stm);
	}


};

#if (defined STM32TPL_F2xxF4xx) || (defined STM32TPL_STM32L0XX) || (defined STM32TPL_STM32L1XX)
#  include "rtc_stm32f4xx.h"
#else
#  include "rtc_stm32f1xx.h"
#endif

typedef RtcModule<true> RtcModuleLSE;
typedef RtcModule<false> RtcModuleLSI;

#endif // STM32TPL_RTC_H_INCLUDED
