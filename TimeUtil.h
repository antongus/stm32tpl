/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *  Visit https://github.com/antongus/stm32tpl for new versions
 *
 *  Copyright (c) 2011-2025 Anton B. Gusev
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
 *  file         : TimeUtil.h
 *  description  : Time conversion functions for 32-bit time stamp
 *
 */

#pragma once

#include <ctime>

namespace TimeUtil {

static constexpr char monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31};

bool IsLeapYear(uint32_t y) { return (y % 4) == 0; }

void CheckTime(struct tm *t)
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

struct tm* localtime(uint32_t t, struct tm * stm)
{
	stm->tm_sec = t % 60;
	t /= 60;
	stm->tm_min = t % 60;
	t /= 60;
	stm->tm_hour = t % 24;
	t /= 24;
	stm->tm_wday = (t + 4) % 7;

	uint32_t year = 1970;
	uint32_t days = 0;

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

uint32_t mktime(struct tm *t)
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

uint32_t date(uint32_t t)
{
	struct tm stm;
	localtime(t, &stm);
	stm.tm_hour = 0;
	stm.tm_min = 0;
	stm.tm_sec = 0;
	return TimeUtil::mktime(&stm);
}

}  // namespace TimeUtil
