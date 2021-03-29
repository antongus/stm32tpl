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

struct HexCodec
{
	/**
	 * Encode buffer to hex-string wit CRC16.
	 * @param data - pointer to data to encode
	 * @param dataSize - data length
	 * @param hex - pointer to buffer for encoded string. should be at least dataSize + 9 bytes!
	 */
	__attribute__((__noinline__))
	static void encode(uint8_t* data, size_t dataSize, uint8_t* hex)
	{
		STM32TPL::Crc16Ccitt crc {0xFFFF};
		auto putNibble = [&](char c){
			*hex++ = hexChar(c);
			*hex = 0;  // always zero-terminate output
		};
		for (size_t i = 0u; i < dataSize; ++i)
		{
			auto b = data[i];
			crc.Add(b);
			putNibble(b >> 4);
			putNibble(b);
		}
		auto w = crc.Result();
		putNibble(w >> 12);
		putNibble(w >> 8);
		putNibble(w >> 4);
		putNibble(w);
	}

	/**
	 * decode Hex-encoded string with CRC16
	 * @param hex - pointer to hex-encoded string
	 * @param buf - pointer to buffer for decoded data
	 * @param bufSize size of buffer
	 * @param decodedLength - length of decoded data
	 * @return true if decoded without errors and CRC is OK
	 */
	__attribute__((__noinline__))
	static bool decode(char const* hex, uint8_t* buf, size_t bufSize, size_t& decodedLength)
	{
		STM32TPL::Crc16Ccitt crc {0xFFFF};
		decodedLength = 0;
		size_t pos = 0u;
		for (; decodedLength < bufSize; ++decodedLength, pos += 2)
		{
			if (!isHexChar(hex[pos]) || !isHexChar(hex[pos+1]))
				return false;
			auto b = decodeByte(hex, pos);
			crc.Add(b);
			buf[decodedLength] = b;
		}
		if (!isHexChar(hex[pos]) || !isHexChar(hex[pos+1]))
			return false;
		crc.Add(decodeByte(hex, pos));
		pos += 2;
		if (!isHexChar(hex[pos]) || !isHexChar(hex[pos+1]))
			return false;
		crc.Add(decodeByte(hex, pos));
		return crc.Valid();
	}

	static bool isHexChar(char ch) { return ((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'F')); }

	static char hexChar(unsigned ch)
	{
		static constexpr char hexChars[] = "0123456789ABCDEF";
		return hexChars[ch & 0x0F];
	}

	static bool isCrcOk(const uint8_t* buf, size_t len) __attribute__((__noinline__))
	{
		STM32TPL::Crc16Ccitt crc {0xFFFF};
		crc.Add(buf, len);
		return crc.Valid();
	}

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

template<size_t bufSize>
class HexEncoder: public HexCodec
{
public:
	HexEncoder() = default;

	static_assert (bufSize > 0, "buf size should be > 0");

	uint16_t crc() const { return m_crc.Result(); }

	void putNibble(uint8_t b)  __attribute__((__noinline__))
	{
		if (m_pos < bufSize - 1)
		{
			m_buf[m_pos++] = hexChar(b);
			m_buf[m_pos] = 0;   // always zero-terminate output
		}
	};

	void putByte(uint8_t b) __attribute__((__noinline__))
	{
		m_crc.Add(b);
		putNibble(b >> 4);
		putNibble(b);
	};

	void putWord(uint16_t w) __attribute__((__noinline__))
	{
		putByte(w >> 8);
		putByte(w);
	}

	void putDword(uint32_t w) __attribute__((__noinline__))
	{
		putWord(w >> 16);
		putWord(w);
	}
	void reset()
	{
		m_pos = 0;
		m_crc.Reset();
	}
private:
	STM32TPL::Crc16Ccitt m_crc {0xFFFF};
	char m_buf[bufSize];
	size_t m_pos {0};
};

/// hex decoder with buffer
template<size_t maxSize>
class HexDecoder : public HexCodec
{
public:
	HexDecoder() = default;

	void decode(char const* buf) __attribute__((__noinline__))
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
		if (m_len < 2)
			return false;
		return isCrcOk(m_data, m_len);
	}
	uint8_t* data() { return m_data; }

private:
	size_t m_len {0};
	uint8_t m_data[maxSize];
};

