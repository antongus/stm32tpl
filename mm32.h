/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *  Visit https://github.com/antongus/stm32tpl for new versions
 *
 *  Copyright (c) 2011-2023 Anton B. Gusev
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
 *  @file         : mm32.h
 *  @description  : Declares special type "chip", which contains MCU-specific info.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "ioregister.h"
#include "cortex-m3-regs.h"

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wregister"
#endif

#if (defined MM32F031)
#  include "CMSIS/mm32f031x6.h"
#else
#  error unsupported MM32 chip
#endif

/**
 * enum all supported chip types
 */
enum class ChipType
{
	Mm32F031,
};

/**
 *  Chip caps structure.
 *  Defines some chip capabilities based on chip type parameter.
 */
template<ChipType chipType> struct ChipCaps;

template<>
struct ChipCaps<ChipType::Mm32F031>
{
	static const uint32_t MAX_FREQ = 72000000;
	static const uint32_t APB1_FREQ = 36000000;
	static const uint32_t APB2_FREQ = 72000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFFF7E8;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFFF7E0;
};

template<ChipType chipType>
struct ChipInfo : public ChipCaps<chipType>
{
	static const ChipType type = chipType;
	typedef ChipCaps<chipType> caps;

	using caps::MAX_FREQ;
	using caps::APB1_FREQ;
	using caps::APB2_FREQ;
	using caps::DEVICE_ID_ADDR;
	using caps::FLASH_SIZE_ADDR;

	static IRegister<DEVICE_ID_ADDR + 0> serial1;
	static IRegister<DEVICE_ID_ADDR + 4> serial2;
	static IRegister<DEVICE_ID_ADDR + 8> serial3;
	static IRegister<FLASH_SIZE_ADDR, uint16_t> flashSize;

	// Cortex-M3 CPUID register
	static IRegister<0xE000ED00> CPUID;

	// Main Stack Pointer
	static RegisterMSP MSP;
	// Process Stack Pointer
	static RegisterPSP PSP;
	// Process Stack Pointer
	static RegisterCONTROL CONTROL;
};

/**
 * Define "chip" class based on selected chip.
 */

template<ChipType chipType> struct ChipInfo;

#if (defined MM32F031)
	using chip = ChipInfo<ChipType::Mm32F031>;
#else
#	error Chip type (MM32...) must be defined.
#endif

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

