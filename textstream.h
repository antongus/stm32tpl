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
 *  file         : textstream.h
 *  description  : Text stream abstract class.
 *
 */

#ifndef STM32TPL_TEXTSTREAM_H_INCLUDED
#define STM32TPL_TEXTSTREAM_H_INCLUDED

#include <cstdint>
#include <cstdlib>
#include "ftoa.h"

/**
*  @brief Text stream abstract base class.
*/
class TextStream
{
public:
	// abstract methods ()
	virtual void PutChar(char ch) = 0;
	virtual int GetChar(int timeout = 0) = 0;
	virtual int Keypressed() = 0;
	virtual int CanSend() = 0;
	virtual int TxEmpty() = 0;

	static char HexChar(char ch)
	{
		static char const *const HexChars = "0123456789ABCDEF";
		return HexChars[ch&0x0F];
	}

	static void Reverse(char* begin, char* end)
	{
		while (end > begin)
		{
			char aux = *end;
			*end-- = *begin;
			*begin++ = aux;
		}
	}

	static char* Itoa(int64_t value, char* s, int base) __attribute__((__noinline__))
	{
		if (base < 2 || base > 16)
		{
			*s = '\0';
			return s;
		}

		auto sign = value < 0;
		char* ptr = s;

		do {
			*ptr++ = HexChar(abs(value % base));
			value /= base;
		} while (value);
		if (sign)
			*ptr++ = '-';

		*ptr-- = '\0';
		Reverse(s, ptr);
		return s;
	}

	char* Gets(char * s, int cnt, bool echo = true) __attribute__((__noinline__))
	{
		int count = 0;

		for (;;)
		{
			char c = GetChar();
			// make letter uppercase
			if (c >= 'a' && c <= 'z') c-=('a'-'A');
			switch(c)
			{
			case '\b': // backspace
				if (count)
				{
					if (echo)
					{
						PutChar('\b');
						PutChar(' ');
						PutChar('\b');
					}
					s--;
					count--;
				}
				break;

			case '\n':
			case '\r': // CR or LF
				if (echo)
				{
					PutChar('\r');
					PutChar('\n');
				}
				*s = 0;
				return s;

			default:
				if (count==cnt)
				{
					if (echo)
						PutChar(0x07); // make BEEP
				}
				else
				{
					*s++=c;
					count++;
					if (echo)
						PutChar(c);    // make echo
				}
				break;
			} //switch(c)
		} // for (;;)
		return s;
	}

	virtual void Puts(const char * s) __attribute__((__noinline__))
	{
		if (s)
			while (*s)
				PutChar(*s++);
	}

	virtual void SendBuffer(const void* buf, size_t size) __attribute__((__noinline__))
	{
		uint8_t const *s = reinterpret_cast<uint8_t const *>(buf);
		while(size--)
			PutChar(*s++);
	}

	virtual bool ReceiveBuffer(void* buf, size_t count, int timeout) __attribute__((__noinline__))
	{
		uint8_t *s = reinterpret_cast<uint8_t*>(buf);
		while(count--)
		{
			int ch = GetChar(timeout);
			if (ch == -1)
				return false;
			*s++ = ch;
		}
		return true;
	}

	void PutHex(uint8_t b) __attribute__((__noinline__))
	{
		PutChar(HexChar(b >> 4));
		PutChar(HexChar(b & 0x0F));
	}

	void PutHex(uint16_t w) __attribute__((__noinline__))
	{
		PutHex((uint8_t)(w >> 8));
		PutHex((uint8_t)w);
	}

	void PutHex(uint32_t w) __attribute__((__noinline__))
	{
		PutHex((uint16_t)(w >> 16));
		PutHex((uint16_t)w);
	}

	void PutHex(int i) { PutHex((uint32_t)i); }

	TextStream& operator<< (char value)
	{
		PutChar(value);
		return *this;
	}

	TextStream& operator<< (char const* value)
	{
		Puts(value);
		return *this;
	}

	TextStream& operator<< (double value) __attribute__((__noinline__))
	{
		char buf[20];
		Puts(ftoa(value, buf, -1));
		return *this;
	}

	TextStream& operator<< (int value) __attribute__((__noinline__))
	{
		char buf[20];
		Puts(Itoa(value, buf, 10));
		return *this;
	}

	TextStream& operator<< (uint16_t value) __attribute__((__noinline__))
	{
		char buf[20];
		Puts(Itoa(value, buf, 10));
		return *this;
	}

	TextStream& operator<< (uint32_t value) __attribute__((__noinline__))
	{
		char buf[20];
		Puts(Itoa(value, buf, 10));
		return *this;
	}

	TextStream& operator<< (size_t value)
	{
		return operator<< ((uint32_t)value);
	}

	TextStream& operator<< (void* value)
	{
		PutHex((uint32_t)value);
		return *this;
	}

	TextStream& operator<< (void const* value)
	{
		PutHex((uint32_t)value);
		return *this;
	}

	TextStream& operator<< (volatile void const* value)
	{
		PutHex((uint32_t)value);
		return *this;
	}

};

#endif // STM32TPL_TEXTSTREAM_H_INCLUDED
