/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 20011-2015 Anton B. Gusev aka AHTOXA
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
		}
		else
		{
			middle_ += coeff_ * ((double)value - middle_);
            result_ += coeff_ * (middle_ - result_);
		}
		res_ = result_ + 0.5;
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

template<typename T>
class DummyFilter
{
public:
	DummyFilter() {}
	DummyFilter(double) {}
	T Put(T value) { return value; };
	void Reset() { }
private:
};


#endif // STM32TPL_FILTERS_H_INCLUDED
