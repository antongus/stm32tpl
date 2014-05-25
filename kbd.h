/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 2014 Anton B. Gusev aka AHTOXA
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
 *  file         : kbd.h
 *  description  : Keyboard class template.
 *
 */

#pragma once

#ifndef KBD_H_INCLUDED
#define KBD_H_INCLUDED

#include <scmRTOS.h>

enum { BUTTON_REPEAT = 0x80 };

template <int buf_size>
class Keyboard
{
private:
	// keyboard buffer
	OS::channel<char, buf_size> buf;

	// keyboard state machine
	typedef enum
	{
		ksIdle,
		ksDebounce,
		ksPause,
		ksRepeat
	}KbdState;

	static const int KB_DEBOUNCE_TME = 4;
	static const int KB_PAUSE_TME = 50;
	static const int KB_PAUSE_TME2 = 10;

	KbdState state;
	uint16_t debounce;
	uint16_t old_key;
	uint16_t ReadInput();
public:
	Keyboard():
		state(ksIdle),
		debounce(0),
		old_key(0)
	{}

	int Keypressed(void) { return buf.get_count(); }
	int GetChar(timeout_t timeout = 0) { char ch; return (buf.pop(ch, timeout)) ? ch :  -1; }

	void Loop()
	{
		uint16_t tmp = ReadInput();

		switch (state)
		{

		case ksDebounce:
			if (old_key != tmp)
				state = ksIdle;
			else if (!--debounce)
			{
				if (buf.get_free_size()) buf.push(tmp);
				debounce = KB_PAUSE_TME;
				state = ksPause;
			}
			break;

		case ksPause:
			if (old_key != tmp)
				state = ksIdle;
			else if (!--debounce)
			{
				if (buf.get_free_size()) buf.push(tmp | BUTTON_REPEAT);
				debounce = KB_PAUSE_TME2;
				state = ksRepeat;
			}
			break;

		case ksRepeat:
			if (old_key != tmp)
				state = ksIdle;
			else if (!--debounce)
			{
				if (buf.get_free_size()) buf.push(tmp | BUTTON_REPEAT);
				debounce = KB_PAUSE_TME2;
			}
			break;

		default:
			if (tmp)
			{
				state = ksDebounce;
				debounce = KB_DEBOUNCE_TME;
				old_key = tmp;
			}
			break;
		}	// switch (state)
	}
};

#endif // KBD_H_INCLUDED
