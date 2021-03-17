/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *  Visit https://github.com/antongus/stm32tpl for new versions
 *
 *  Copyright (c) 2011-2021 Anton B. Gusev
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
 *  file         : HexCodec.h
 *  description  : Hexadecimal encoder and decoder - for data exchange using hex lines (with CRC).
 *
 *  USAGE :
 *    // send encoded data to UART:
 *    HexEncoder<80> line;
 *    line.putByte(10);
 *    line.putWord(someVar);
 *    line.putWord(line.crc());
 *    uart << line << "\r\n";
 *
 *    // decode received line:
 *
 */

#pragma once

#include <cstdint>
#include "crc16_ccitt.h"
#include "textbuf.h"

template<size_t bufSize>
class HexEncoder : public TextBuffer<bufSize>
{
public:
	HexEncoder() = default;

	uint16_t crc() const { return m_crc.Result(); }
	__attribute__((__noinline__))
	void putByte(uint8_t b)
	{
		m_crc.Add(b);
		this->PutChar(this->HexChar(b >> 4));
		this->PutChar(this->HexChar(b & 0x0F));
	};
	__attribute__((__noinline__))
	void putWord(uint16_t w)
	{
		putByte(w >> 8);
		putByte(w);
	}
	__attribute__((__noinline__))
	void putDword(uint32_t w)
	{
		putWord(w >> 16);
		putWord(w);
	}
	void reset()
	{
		TextBuffer<bufSize>::Reset();
		m_crc.Reset();
	}
private:
	STM32TPL::Crc16Ccitt m_crc {0xFFFF};
};


template<size_t maxSize>
class HexDecoder
{
public:
	HexDecoder() = default;

	__attribute__((__noinline__))
	void decode(char const* buf)
	{
		m_len = 0;
		for (size_t pos = 0; m_len < maxSize; pos += 2)
		{
			if (!isHexChar(buf[pos]) || !isHexChar(buf[pos+1]))
				break;
			m_data[m_len++] = decodeByte(buf, pos);
		}
	}
	int len() const { return m_len; }
	bool isValid() const
	{
		return m_len >= 2 && STM32TPL::Crc16Ccitt::Calc(m_data, m_len) == 0;
	}
	uint8_t* data() { return m_data; }

private:
	size_t m_len {0};
	uint8_t m_data[maxSize];

	static bool isHexChar(char ch) { return ((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'F')); }
	static uint8_t decodeByte(char const* buf, size_t pos)
	{
		uint8_t bhi = buf[pos];
		uint8_t blo = buf[pos+1];

		if (bhi > '9')
			bhi += 9;

		bhi <<= 4;

		if (blo > '9')
			blo += 9;

		return bhi | (blo & 0x0F);
	}
};

