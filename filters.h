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
 *  file         : filters.h
 *  description  : Some digital filters
 *
 */

#ifndef STM32TPL_FILTERS_H_INCLUDED
#define STM32TPL_FILTERS_H_INCLUDED

#include <cstdint>
#include <array>
#include <algorithm>
#include <type_traits>

template<typename T = uint32_t>
class ExponentialFilter
{
public:
	ExponentialFilter(double coeff = 0.3)
		: coeff_(coeff)
	{}

    void SetCoeff(double coeff)
    {
        coeff_ = coeff;
        first_ = true;
    }

	void Put(T value)
	{
		raw_ = value;
		if (first_)
		{
			first_ = false;
			result_ = value;
		}
		else
		{
            result_ += coeff_ * ((double)value - result_);
		}
	}
	T Get() { return result_; }
	T Raw() { return raw_; }
	operator T() { return Get(); }
	void Reset() { first_ = true; }
private:
	bool first_ = true;
	T raw_;
	double coeff_;
	double result_;
};

template<typename T = uint32_t>
class ExponentialFilter2
{
public:
	ExponentialFilter2(double coeff = 0.3)
		: coeff_(coeff)
	{}

    void SetCoeff(double coeff)
    {
        coeff_ = coeff;
        first_ = true;
    }

    T Put(T value)
	{
		raw_ = value;
		if (first_)
		{
			first_ = false;
			result_ = value;
			middle_ = value;
		}
		else
		{
			middle_ += coeff_ * ((double)value - middle_);
            result_ += coeff_ * (middle_ - result_);
		}
		if constexpr(std::is_integral_v<T>)
			res_ = result_ + 0.5;
		else
			res_ = result_;
		return res_;
	}
	T Get() { return res_; }
	T Raw() { return raw_; }
	operator T() { return Get(); }
	void Reset() { first_ = true; }
private:
	bool first_ = true;
	T raw_;
	double coeff_;
	double middle_;
	double result_;
	T res_;
};

template<typename T, size_t size>
class MedianFilter
{
public:
	MedianFilter() {}

	T Put(T value);
	T Get() { return result_; }
	operator T() { return Get(); }
	void Reset() { first_ = true; }
private:
    typedef std::array<T, size> Buf;
    bool first_ = true;
	T result_;
	Buf history_;
	size_t put_ = 0;
};

template<typename T, size_t size>
T MedianFilter<T, size>::Put(T value)
{
	if (first_)
	{
		history_.fill(value);
		first_ = false;
		result_ = value;
	}
	else
	{
		if (++put_ >= size)
			put_ = 0;
		history_[put_] = value;
		Buf tmp(history_);
		std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
		result_ = tmp[tmp.size() / 2];
	}
	return result_;
}

/**
 * Median filter with adjustable window size (in range 3...size)
 */
template<typename T, size_t maximumSize>
class AdjustableMedianFilter
{
public:
	AdjustableMedianFilter() {}

	T Put(T value);
	T Get() { return result; }
	operator T() { return Get(); }
	void Reset() { first = true; }
	size_t GetWindowSize() const { return windowSize; }
	void SetWindowSize(size_t newSize) {
		if (newSize < minSize)
			newSize = minSize;
		else if (newSize > maxSize)
			newSize = maxSize;
		if (newSize != windowSize)
		{
			windowSize = newSize;
			Reset();
		}
	}

	static constexpr size_t minSize { 3 };
	static constexpr size_t maxSize { maximumSize };

	static_assert(minSize <= maxSize, "size should be >= 3");
private:
    using BufType = std::array<T, maxSize>;
    bool first { true };
    size_t windowSize { maxSize };
	T result;
	BufType history;
	BufType sortBuffer;
	size_t pos { 0 };
};

template<typename T, size_t size>
T AdjustableMedianFilter<T, size>::Put(T value)
{
	if (first)  // first measure
	{
		std::fill_n(std::begin(history), windowSize, value);
		first = false;
		result = value;
		pos = 0;
	}
	else
	{
		if (++pos >= windowSize)
			pos = 0;
		history[pos] = value;
		auto const head = std::begin(history);
		std::rotate_copy(head, head + 1, head + windowSize, std::begin(sortBuffer));

		auto const halfWindow = windowSize / 2;
		std::nth_element(std::begin(sortBuffer), std::begin(sortBuffer) + halfWindow, std::begin(sortBuffer) + windowSize);
		result = sortBuffer[halfWindow];
	}
	return result;
}

/**
 * Dummy filter. Does nothing.
 */
template<typename T>
class DummyFilter
{
public:
	DummyFilter() {}
	DummyFilter(double) {}
	T Put(T value) { return value; };
	void Reset() { }

};


#endif // STM32TPL_FILTERS_H_INCLUDED
