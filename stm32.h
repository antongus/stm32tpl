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
 *  @file         : stm32.h
 *  @description  : Declares special type "chip", which contains MCU-specific info.
 *  created on    : 24.03.2010
 *
 */

#ifndef STM32TPL_STM32_H_INCLUDED
#define STM32TPL_STM32_H_INCLUDED

#include <cstdint>
#include "ioregister.h"
#include "cortex-m3-regs.h"

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wregister"
#endif

#if defined STM32F2XX
#  include "CMSIS/stm32f2xx.h"
#  define STM32TPL_F2xxF4xx
#elif defined STM32F4XX
#  include "CMSIS/stm32f4xx.h"
#  define STM32TPL_F2xxF4xx
#elif (defined STM32F40_41xxx) || (defined STM32F427_437xx) || (defined STM32F429_439xx) || (defined STM32F401xx)
#  define STM32F4XX
#  include "CMSIS/stm32f4xx.h"
#  define STM32TPL_F2xxF4xx
#elif (defined STM32L051xx) || (defined STM32L052xx) || (defined STM32L053xx) || (defined STM32L061xx) || (defined STM32L062xx) || (defined STM32L063xx)
#  define STM32TPL_STM32L0XX
#  include "CMSIS/stm32l0xx.h"
#elif (defined STM32L100xB) || (defined STM32L100xBA) || (defined STM32L100xC) || \
    (defined STM32L151xB) || (defined STM32L151xBA) || (defined STM32L151xC) || (defined STM32L151xCA) || (defined STM32L151xD) || (defined STM32L151xDX) || (defined STM32L151xE) || \
    (defined STM32L152xB) || (defined STM32L152xBA) || (defined STM32L152xC) || (defined STM32L152xCA) || (defined STM32L152xD) || (defined STM32L152xDX) || (defined STM32L152xE) || \
    (defined STM32L162xC) || (defined STM32L162xCA) || (defined STM32L162xD) || (defined STM32L162xDX) || (defined STM32L162xE)
#  define STM32TPL_STM32L1XX
#  include "CMSIS/stm32l1xx.h"
#elif (defined STM32F030x6) || (defined STM32F030x8) || (defined STM32F030xC) || (defined STM32F031x6) || \
    (defined STM32F038xx) || (defined STM32F042x6) || (defined STM32F048x6) || (defined STM32F051x8) || \
    (defined STM32F058xx) || (defined STM32F070x6) || (defined STM32F070xB) || (defined STM32F071xB) || \
    (defined STM32F072xB) || (defined STM32F078xx) || (defined STM32F091xC) || (defined STM32F098xx)
#  define STM32TPL_STM32F0XX
#  include "CMSIS/stm32f0xx.h"
#elif (defined STM32F756xx) || (defined STM32F746xx) || (defined STM32F745xx) || (defined STM32F765xx) || (defined STM32F767xx) \
|| (defined STM32F769xx) || (defined STM32F777xx) || (defined STM32F779xx) || (defined STM32F722xx) || (defined STM32F723xx) \
|| (defined STM32F732xx) || (defined STM32F733xx)
#  define STM32TPL_STM32F7XX
#  include "CMSIS/stm32f7xx.h"
#else
#  define STM32TPL_STM32F1XX
#  include "CMSIS/stm32f10x.h"
#endif


#if (defined STM32TPL_STM32L0XX) || (defined STM32TPL_STM32L1XX) || (defined STM32TPL_STM32F0XX) || (defined STM32TPL_STM32F7XX)
typedef IRQn_Type IRQn;    // in STM32L0xx headers IRQn type was renamed to IRQn_Type.
#endif

/**
 * enum all supported chip types
 */
enum ChipType
{
	stm32F10X_LD,      ///< Low density devices
	stm32F10X_MD,      ///< Medium density devices
	stm32F10X_HD,      ///< High density devices
	stm32F10X_CL,      ///< Connectivity line devices
	stm32F10X_LD_VL,   ///< Low density Value Line devices
	stm32F10X_MD_VL,   ///< Medium density Value Line devices
	stm32F10X_HD_VL,   ///< High density Value Line devices
	stm32F2XX,         ///< stm32F2xx chips
	stm32F4XX,         ///< stm32F4xx chips
	stm32F7XX,         ///< stm32F7xx chips
	stm32L0XX,         ///< stm32L0xx chips
	stm32F0XX,         ///< stm32F0xx chips
	stm32L1XX,         ///< stm32L1xx chips
};


/**
 *  Chip caps structure.
 *  Defines some chip capabilities based on chip type parameter.
 */
template<ChipType chipType> struct ChipCaps;

template<>
struct ChipCaps<stm32F10X_LD>
{
	static const uint32_t MAX_FREQ = 72000000;
	static const uint32_t APB1_FREQ = 36000000;
	static const uint32_t APB2_FREQ = 72000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFFF7E8;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFFF7E0;
};

template<>
struct ChipCaps<stm32F10X_MD>
{
	static const uint32_t MAX_FREQ = 72000000;
	static const uint32_t APB1_FREQ = 36000000;
	static const uint32_t APB2_FREQ = 72000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFFF7E8;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFFF7E0;
};

template<>
struct ChipCaps<stm32F10X_HD>
{
	static const uint32_t MAX_FREQ = 72000000;
	static const uint32_t APB1_FREQ = 36000000;
	static const uint32_t APB2_FREQ = 72000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFFF7E8;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFFF7E0;
};

template<>
struct ChipCaps<stm32F10X_CL>
{
	static const uint32_t MAX_FREQ = 72000000;
	static const uint32_t APB1_FREQ = 36000000;
	static const uint32_t APB2_FREQ = 72000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFFF7E8;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFFF7E0;
};

template<>
struct ChipCaps<stm32F10X_LD_VL>
{
	static const uint32_t MAX_FREQ = 24000000;
	static const uint32_t APB1_FREQ = 24000000;
	static const uint32_t APB2_FREQ = 24000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFFF7E8;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFFF7E0;
};

template<>
struct ChipCaps<stm32F10X_MD_VL>
{
	static const uint32_t MAX_FREQ = 24000000;
	static const uint32_t APB1_FREQ = 24000000;
	static const uint32_t APB2_FREQ = 24000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFFF7E8;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFFF7E0;
};

template<>
struct ChipCaps<stm32F10X_HD_VL>
{
	static const uint32_t MAX_FREQ = 24000000;
	static const uint32_t APB1_FREQ = 24000000;
	static const uint32_t APB2_FREQ = 24000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFFF7E8;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFFF7E0;
};

template<>
struct ChipCaps<stm32F2XX>
{
	static const uint32_t MAX_FREQ = 120000000;
	static const uint32_t APB1_FREQ = 30000000;
	static const uint32_t APB2_FREQ = 60000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFF7A10;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFF7A22;
};

template<>
struct ChipCaps<stm32F4XX>
{
	static const uint32_t MAX_FREQ = 168000000;
	static const uint32_t APB1_FREQ = 42000000;
	static const uint32_t APB2_FREQ = 84000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFF7A10;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFF7A22;
};

template<>
struct ChipCaps<stm32F7XX>
{
	static const uint32_t MAX_FREQ = 216000000;
	static const uint32_t APB1_FREQ = MAX_FREQ/4;
	static const uint32_t APB2_FREQ = MAX_FREQ/2;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFF7A10;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFF7A22;
};

template<>
struct ChipCaps<stm32L0XX>
{
	static const uint32_t MAX_FREQ  = 32000000;
	static const uint32_t APB1_FREQ = 32000000;
	static const uint32_t APB2_FREQ = 32000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFF7A10;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFF7A22;
};

template<>
struct ChipCaps<stm32F0XX>
{
	static const uint32_t MAX_FREQ  = 48000000;
	static const uint32_t APB1_FREQ = 48000000;
	static const uint32_t APB2_FREQ = 48000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFFF7AC;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFFF7CC;
};

template<>
struct ChipCaps<stm32L1XX>
{
	static const uint32_t MAX_FREQ  = 32000000;
	static const uint32_t APB1_FREQ = 32000000;
	static const uint32_t APB2_FREQ = 32000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FF800D0;  // for Cat.3, Cat.4, Cat.5 and Cat.6 devices
	static const uint32_t FLASH_SIZE_ADDR = 0x1FF800CC;  // for Cat.3, Cat.4, Cat.5 and Cat.6 devices
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

#if (defined STM32F10X_LD)
	typedef ChipInfo<stm32F10X_LD> chip;
#elif (defined STM32F10X_MD)
	typedef ChipInfo<stm32F10X_MD> chip;
#elif (defined STM32F10X_HD)
	typedef ChipInfo<stm32F10X_HD> chip;
#elif (defined STM32F10X_CL)
	typedef ChipInfo<stm32F10X_CL> chip;
#elif (defined STM32F10X_LD_VL)
	typedef ChipInfo<stm32F10X_LD_VL> chip;
#elif (defined STM32F10X_MD_VL)
	typedef ChipInfo<stm32F10X_MD_VL> chip;
#elif (defined STM32F2XX)
	typedef ChipInfo<stm32F2XX> chip;
#elif (defined STM32F4XX)
	typedef ChipInfo<stm32F4XX> chip;
#elif (defined STM32TPL_STM32F7XX)
	typedef ChipInfo<stm32F7XX> chip;
#elif (defined STM32TPL_STM32L0XX)
	typedef ChipInfo<stm32L0XX> chip;
#elif (defined STM32TPL_STM32L1XX)
	typedef ChipInfo<stm32L1XX> chip;
#elif (defined STM32TPL_STM32F0XX)
	typedef ChipInfo<stm32F0XX> chip;
#else
#	error Chip type (STM32FXXX_XX) must be defined.
#endif

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#endif // STM32TPL_STM32_H_INCLUDED
