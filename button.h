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
 *  file         : button.h
 *  description  : single button class.
 *                 supports single, long and double press events
 *
 */

#ifndef STM32TPL_BUTTON_H_INCLUDED
#define STM32TPL_BUTTON_H_INCLUDED

#include "pin.h"
#include <scmRTOS.h>

struct DefaultSingleButtonProps
{
	using ButtonPin = Pin<'A', 1>;                      //!< pin
	static constexpr unsigned poolInterval     {10};    //!< pool interval (milliseconds)
	static constexpr unsigned debounceMs       {30};    //!< debounce time (milliseconds)
	static constexpr unsigned longPressMs     {900};	//!< long press time (milliseconds)
	static constexpr unsigned doubleClickMs   {500};	//!< double click time (milliseconds)
};

template <typename Props>
class SingleButton
{
public:
	SingleButton()
	{
		ButtonPin::Mode(INPUT);
	}

	/// possible button events
	enum class Event : uint8_t
	{
		None,
		Click,
		LongClick,
		DoubleClick,
	};

	void pool();
	Event getEvent(timeout_t timeout = 0);
	bool hasEvents() const { return events.get_count(); }

private:
	using ButtonPin = typename Props::ButtonPin;

	enum class ButtonState
	{
		Idle,        ///<! initial state (depressed)
		Debounce,    ///<! waiting for stable down state
		Pressed,     ///<! button is pressed (waiting for up or for long click)
		Depressed,   ///<! depressed (waiting for double click)
		Debounce2,   ///<! waiting for stable down state (for double click)
		WaitIdle,    ///<! waiting for up
		DebounceUp,  ///<! waiting for stable up state (not necessary)
		Doubled      ///<! double clicked.
	};

	static constexpr auto debounceTicks {Props::debounceMs/Props::poolInterval};
	static constexpr auto longPressTicks {Props::longPressMs/Props::poolInterval};
	static constexpr auto doubleClickTicks {Props::doubleClickMs/Props::poolInterval};

	OS::channel<Event, 4> events;
	ButtonState state {ButtonState::Idle};
	unsigned debounce {0};
	unsigned ticks {0};

	void pushEvent(Event e);
};

template<typename Props>
void SingleButton<Props>::pool()
{
	bool pressed = ButtonPin::Signalled();

	if (ticks)
		--ticks;

	switch(state)
	{
		// idle state - waiting for press
		case ButtonState::Idle :
			if (pressed)
			{
				ticks = debounceTicks;           // launch debounce counter
				state = ButtonState::Debounce;
			}
			break;

		// "debounce" state - wait for stable button state
		case ButtonState::Debounce :
			if (!pressed)                // state changed - idle again
			{
				state = ButtonState::Idle;
			}
			else if (!ticks)             // yes, button is stable
			{
				ticks = longPressTicks;  // launch long press counter
				state = ButtonState::Pressed;
			}
			break;

		// "pressed" state - wait for up
		case ButtonState::Pressed :
			if (!pressed)                 // button depressed
			{
				ticks = doubleClickTicks; // launch double press counter
				state = ButtonState::Depressed;
			}
			else if (!ticks)              // long press counter elapsed
			{
				state = ButtonState::WaitIdle; // wait for button up
				pushEvent(Event::LongClick);
			}
			break;

		// "depressed" state - wait for double press
		case ButtonState::Depressed :
			if (pressed)                        // button pressed again = double-click
			{
				ticks = debounceTicks;          // launch debounce timer
				state = ButtonState::Debounce2; // switch state to debounce
			}
			else if (!ticks)					// double-click timer elapsed => it was a single click
			{
				state = ButtonState::Idle;      // switch to idle state
				pushEvent(Event::Click);
			}
			break;

		// debounce for double-click state
		case ButtonState::Debounce2 :
			if (!pressed)
			{
				ticks = debounceTicks;          // re-launch debounce timer
				state = ButtonState::Depressed; // return to "depressed" state
			}
			else if (!ticks)					// button stable => double click
			{
				state = ButtonState::WaitIdle;
				pushEvent(Event::DoubleClick);
			}
			break;

		case ButtonState::WaitIdle :
			if (!pressed)
			{
				ticks = debounceTicks;
				state = ButtonState::DebounceUp;
			}
			break;

		case ButtonState::DebounceUp :
			if (pressed)
			{
				ticks = debounceTicks;
				state = ButtonState::WaitIdle;
			}
			else if (!ticks)     // Okay, go idle
			{
				state = ButtonState::Idle;
			}
			break;

		default :
			state = ButtonState::Idle;
			break;
	}

}

template<typename Props>
typename SingleButton<Props>::Event SingleButton<Props>::getEvent(timeout_t timeout)
{
	Event e;
	if (events.pop(e, timeout))
		return e;
	return Event::None;
}

template<typename Props>
void SingleButton<Props>::pushEvent(Event e)
{
	if (events.get_free_size())
		events.push(e);
}

#endif // STM32TPL_BUTTON_H_INCLUDED

