/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *  Visit https://github.com/antongus/stm32tpl for new versions
 *
 *  Copyright (c) 2011-2024 Anton B. Gusev
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
 *  @file         : stat.h
 *  @description  : simple statistics counter.
 *
 */

#pragma once

#include <stdint.h>
#include <limits.h>

namespace STM32TPL
{

class Stat
{
public:
	uint32_t Avg() const { return sum_/cnt_; }
	uint32_t Cnt() const { return cnt_; }
	uint32_t Max() const { return max_; }
	uint32_t Min() const { return min_; }
	void Add(uint32_t val)
	{
		if (min_ > val) min_ = val;
		if (max_ < val) max_ = val;
		sum_ += val;
		if (++cnt_ == UINT_MAX)
		{
			sum_ = sum_ / cnt_;
			cnt_ = 1;
		}
	}
	Stat& operator<< (uint32_t val) { Add(val); return *this; }
private:
	uint32_t min_ = UINT_MAX;
	uint32_t max_ = 0;
	uint32_t cnt_ = 0;
	uint64_t sum_ = 0;
};

} // namespace STM32TPL
