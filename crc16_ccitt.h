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
 *  file         : crc16_ccitt.h
 *  description  : Class for CRC16-CCITT calculation (0x1021 polynomial)
 *
 * USAGE:
 *
 *   I. Add definition of static class member Crc16CcittCalculator::crc16CcittTable
 *   somewhere in *.cpp file :
 *
 *     template<uint16_t initialValue>
 *     constexpr STM32TPL::Crc16CcittTable STM32TPL::Crc16CcittCalculator<initialValue>::crc16CcittTable;
 *
 *   II. Use:
 *   uint16_t Calibration::CalcCrc(uint8_t const* buffer, size_t size)
 *   {
 *       STM32TPL::Crc16Ccitt crcCalculator;
 *       for (size_t i = 0; i < size; ++i)
 *           crcCalculator.Add(buffer[i]);
 *       return crcCalculator.Result();
 *   }
 *
 *
 */

#ifndef STM32TPL_CRC16CCITT_H_INCLUDED
#define STM32TPL_CRC16CCITT_H_INCLUDED

#include <cstdint>
#include <cstddef>

namespace STM32TPL
{

struct Crc16CcittTable
{
	uint16_t data[256];
};

namespace
{
/**
 *  This constexpr function computes crc16 table cell.
 *
 *  It is a meta-function implementing loop body of this function:
 *  void FillTable() {
 *      for (uint16_t pos = 0; pos < 256; ++pos) {
 *          uint16_t val = 0;
 *          for (uint8_t step = 0; step < 8; ++step) {
 *              val = (val ^ (pos << (8 + step))) & 0x8000 ? (val << 1) ^ 0x1021 : val << 1;
 *          }
 *          crc16_table[pos] = val;
 *      }
 *  }
 */
constexpr uint16_t ComputeCell(
		uint16_t pos,      // position in table (0..255)
		uint16_t val = 0,  // intermediate value
		int step = 0       // calculation step (recursion depth)
		)
{
	return step < 8 ?
			ComputeCell(pos, (val ^ (pos << (8 + step))) & 0x8000 ? (val << 1) ^ 0x1021 : val << 1, step+1) :
			val;
}

template<bool> struct TypeSelector { typedef Crc16CcittTable type; };
template<> struct TypeSelector<false> { };

template<typename ...T>
constexpr typename TypeSelector<sizeof...(T) == 256>::type ComputeTable(int, T... t)
{
	return Crc16CcittTable {{ t... }};
}

template<typename ...T>
constexpr typename TypeSelector<sizeof...(T) <= 255>::type ComputeTable(int n, T... t)
{
	return ComputeTable(n+1, t..., ComputeCell(n));
}

//constexpr Crc16CcittTable crc16CcittTable = ComputeTable(0);

}  // anonymous namespace


template <uint16_t initialValue = 0xFFFF>
class Crc16CcittCalculator
{
public:
	static const uint16_t initValue = initialValue;
	Crc16CcittCalculator(std::uint16_t init = initValue) { Reset(init); }
	void Reset(std::uint16_t init = initValue) { crc_ = init; }
	std::uint16_t Result() const { return crc_; }
	bool Valid() const { return Result() == 0; }
	static constexpr Crc16CcittTable crc16CcittTable = ComputeTable(0);
	void Add(uint8_t val)
	{
		std::uint16_t tmp = crc_;
		crc_ = (tmp << 8) ^ crc16CcittTable.data[(tmp >> 8) ^ val];
	}
	void Add(char *s) { while (*s) Add(*s++); }
	void Add(void const* buf, size_t size)
	{
		char const* ptr = reinterpret_cast<char const*>(buf);
		for (size_t i = 0; i < size; ++i)
			Add(ptr[i]);
	}

	/**
	 *  Calculate CRC16 of zero-terminated string.
	 *  Can be evaluated at compile-time (see static assert at the end of this file).
	 */
	constexpr static std::uint16_t
	Calc(const char* ptr, std::uint16_t acc = 0xFFFF)
	{
		return *ptr ?
				Calc(ptr+1, (acc << 8) ^ crc16CcittTable.data[(acc >> 8) ^ *ptr])
				: acc;
	}
private:
	std::uint16_t crc_;
};

typedef Crc16CcittCalculator<> Crc16Ccitt;

static_assert (
		Crc16Ccitt::Calc("123456789") == 0x29B1 , "CRC16-CCITT code of 123456789 should be 0x29B1"
);

} // namespace STM32TPL

#endif // STM32TPL_CRC16CCITT_H_INCLUDED
