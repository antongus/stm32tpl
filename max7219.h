/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 2017 Anton B. Gusev aka AHTOXA
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
 *  file         : max7219.h
 *  description  : MAX7219 LED driver chain class.
 */

#ifndef STM32TPL_MAX7219_INCLUDED
#define STM32TPL_MAX7219_INCLUDED

#include <cstdint>
#include "pin.h"
#include "scmRTOS.h"

/**
 * Sample properties for Max7219Chain class template
 */
struct SampleMax7219ChainProps
{
	enum
	{
		CHIP_COUNT = 3,         ///<! chips in chain
		DEFAULT_BRIGHTNESS = 7, ///<! default brightness (used during initialization)
		LATCH_NOPS = 8,         ///<! NOP count during LATCH pin active
	};
	static void Init() {}       ///!< hardware initialization of serial connection to chips
	static void Latch() {}      ///!< latch data to MAX7219 chain
	static void ShiftWord(uint16_t) {}  ///!< shift one word of data to MAX7219 chain
};

/**
 * Max7219Chain class template
 */
template<typename props = SampleMax7219ChainProps>
class TMax7219Chain
{
public:
	enum
	{
		CHIP_COUNT = props::CHIP_COUNT,
		DIGITS_PER_CHIP = 8,
		DIGIT_COUNT = CHIP_COUNT * DIGITS_PER_CHIP,
		DEFAULT_BRIGHTNESS = props::DEFAULT_BRIGHTNESS,
	};

	void Init();
	void SetSleep(bool value)          { WriteAll(value ? 0x0C00 : 0x0C01); }
	void SetTest(bool value)           { WriteAll(value ? 0x0F01 : 0x0F00); }
	void SetDecodeMode(uint8_t flags)  { WriteAll(0x0900 | flags); }
	void SetUsedDigits(uint8_t value)  { WriteAll(0x0B00 | value); }
	void SetBrightness(uint8_t value)  { WriteAll(0x0A00 | (value & 0x0F)); }
	void Cls();
	void Test(int delay = 2);
	void SetDigit(unsigned digit, unsigned value);
private:
	void WriteAll(uint16_t w);
};

/**
 * Initialize MAX7219 chain.
 *  - turn off sleep
 *  - turn off test mode
 *  - set used digits to 0xFF (all used)
 *  - set decode mode to 0 (off)
 *  - set brightness to default value
 */
template<typename props>
void TMax7219Chain<props>::Init()
{
	props::Init();
	SetSleep(false);
	SetTest(false);
	SetUsedDigits(0xFF);
	SetDecodeMode(0x00);
	SetBrightness(DEFAULT_BRIGHTNESS);
}

/**
 * Write command to all chips
 * @param w - command to write
 */
template<typename props>
void TMax7219Chain<props>::WriteAll(uint16_t w)
{
	for (auto chipNo = 0u; chipNo < CHIP_COUNT; ++chipNo)
		props::ShiftWord(w);
	props::Latch();
}

/**
 * Turn off all LEDs
 */
template<typename props>
void TMax7219Chain<props>::Cls()
{
	for (uint16_t digit = 1; digit < 9; digit++)
		WriteAll(digit << 8);
}

/**
 * Test all LEDS
 * @param delay - delay between test steps (in milliseconds)
 */
template<typename props>
void TMax7219Chain<props>::Test(int delay)
{
	for (auto chipNo = 0u; chipNo < CHIP_COUNT; ++chipNo)
	{
		for (auto digit = 0u; digit < DIGITS_PER_CHIP; ++digit)
		{
			unsigned mask = 1u;
			for (auto bit = 0u; bit < 8; ++bit)
			{
				SetDigit(chipNo * DIGITS_PER_CHIP + digit, mask);
				mask <<= 1;
				mask |= 1;
				OS::sleep(delay);
			}
		}
		for (auto digit = 0u; digit <= DIGITS_PER_CHIP; ++digit)
			SetDigit(chipNo * DIGITS_PER_CHIP + digit, 0);
	}
}

template<typename props>
void TMax7219Chain<props>::SetDigit(unsigned digit, unsigned value)
{
	int targetChip = digit / DIGITS_PER_CHIP;
	uint16_t w = ((digit % DIGITS_PER_CHIP + 1) << 8) | value;


	for (int i = CHIP_COUNT - 1; i >= 0; i--)
		props::ShiftWord(i == targetChip ? w : 0x0000);

	props::Latch();
}


#endif // STM32TPL_MAX7219_INCLUDED

