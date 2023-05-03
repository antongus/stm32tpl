/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *  Visit https://github.com/antongus/stm32tpl for new versions
 *
 *  Copyright (c) 2011-2023 Anton B. Gusev
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
 *  file         : filters_int.h
 *  description  : Some digital filters (no floating point)
 *
 */

#pragma once

#include <cstdint>
#include <array>
#include <algorithm>
#include <type_traits>

/**
 * Exponential filter with integer math.
 *
 *       (Yn-1 * (scale - coeff) + Xn * coeff)
 * Yn =  -------------------------------------
 *                   scale
 * (coeff should be in range 1..scale-1).
 *
 * types:
 *   ValueType - type of filter argument and result
 *   AccumType - type used in filter calculations. Must be wide enough to hold max(argument) * scale.
 */
template<typename ValueType = uint32_t, typename AccumType = uint32_t, AccumType scale = 1024>
class ExponentialFilterInt
{
public:
	ExponentialFilterInt(AccumType coeff = 256)
		: m_coeff(coeff)
	{}

    void setCoeff(AccumType coeff)
    {
        m_coeff = coeff;
        reset();
    }

    ValueType put(ValueType value)
	{
		m_raw = value;
		if (m_first)
		{
			m_first = false;
			m_result = value;
		}
		else
		{
			// accumulate filtered result (scaled)
			m_result = m_result - m_result * m_coeff / scale + m_raw * m_coeff + m_coeff/2;
		}
		return get();
	}
	ValueType get() { return m_result / scale; }
	ValueType raw() { return m_raw; }
	void reset() { m_first = true; }
private:
	bool m_first = true;
	ValueType m_raw;
	AccumType m_coeff;
	AccumType m_result;
};
