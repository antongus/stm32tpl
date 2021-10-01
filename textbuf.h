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
 *  file         : textbuf.h
 *  description  : text output buffer.
 *  created on   : 05.06.2010
 *
 *  USAGE :
 *      TextBuffer<40> buffer;
 *      buffer << i << " + " << j << " == " << i+j << "\r\n";
 *      print(buffer);
 *
 */

#ifndef STM32TPL_TEXTBUF_H_INCLUDED
#define STM32TPL_TEXTBUF_H_INCLUDED

#include "textstream.h"

/**
 * class TextBuffer
 * output-only text stream (writes to internal buffer)
 */
template <size_t bufSize>
class TextBuffer: public TextStream
{
public:
	static const size_t SIZE = bufSize;
	TextBuffer()
		: len_(0) { buf_[0] = 0; }
	virtual void PutChar(char ch)
	{
		if (CanSend())
		{
			buf_[len_++] = ch;
			buf_[len_] = 0;
		}
	};
	virtual int CanSend() { return len_ < SIZE-1; }
	virtual int GetChar(int ) { return -1; }
	virtual int Keypressed() { return false; }
	virtual int TxEmpty() { return true; }
	int Len() const { return len_; }
	void Reset() { len_ = 0; buf_[0] = 0; }
	operator char*() { return buf_; }
private:
	char buf_[SIZE];
	size_t len_;
};

#endif // STM32TPL_TEXTBUF_H_INCLUDED
