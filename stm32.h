/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 2010-2014 Anton B. Gusev aka AHTOXA
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


#if defined STM32F2XX
#  include "CMSIS/stm32f2xx.h"
#  define F2xxF4xx
#elif defined STM32F4XX
#  include "CMSIS/stm32f4xx.h"
#  define F2xxF4xx
#elif (defined STM32F40_41xxx) || (defined STM32F427_437xx) || (defined STM32F429_439xx) || (defined STM32F401xx)
#  define STM32F4XX
#  include "CMSIS/stm32f4xx.h"
#  define F2xxF4xx
#elif (defined STM32L051xx) || (defined STM32L052xx) || (defined STM32L053xx) || (defined STM32L061xx) || (defined STM32L062xx) || (defined STM32L063xx)
#  define STM32L0XX
#  include "CMSIS/stm32l0xx.h"
#else
#  define STM32F1XX
#  include "CMSIS/stm32f10x.h"
#endif


#include "pin.h"

#if (defined STM32L0XX)
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
	stm32L0XX,         ///< stm32L0xx chips
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
	enum { HAVE_BUS = false };
	enum { SPI_COUNT = 1 };
	enum { USART_COUNT = 2 };
	enum { HAVE_USB = true };
	enum { HAVE_OTG = false };
};

template<>
struct ChipCaps<stm32F10X_MD>
{
	static const uint32_t MAX_FREQ = 72000000;
	static const uint32_t APB1_FREQ = 36000000;
	static const uint32_t APB2_FREQ = 72000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFFF7E8;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFFF7E0;
	enum { HAVE_BUS = false };
	enum { SPI_COUNT = 2 };
	enum { USART_COUNT = 3 };
	enum { HAVE_USB = true };
	enum { HAVE_OTG = false };
};

template<>
struct ChipCaps<stm32F10X_HD>
{
	static const uint32_t MAX_FREQ = 72000000;
	static const uint32_t APB1_FREQ = 36000000;
	static const uint32_t APB2_FREQ = 72000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFFF7E8;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFFF7E0;
	enum { HAVE_BUS = true };
	enum { SPI_COUNT = 3 };
	enum { USART_COUNT = 5 };
	enum { HAVE_USB = true };
	enum { HAVE_OTG = false };
};

template<>
struct ChipCaps<stm32F10X_CL>
{
	static const uint32_t MAX_FREQ = 72000000;
	static const uint32_t APB1_FREQ = 36000000;
	static const uint32_t APB2_FREQ = 72000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFFF7E8;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFFF7E0;
	enum { HAVE_BUS = false };
	enum { SPI_COUNT = 3 };
	enum { USART_COUNT = 5 };
	enum { HAVE_USB = false };
	enum { HAVE_OTG = true };
};

template<>
struct ChipCaps<stm32F10X_LD_VL>
{
	static const uint32_t MAX_FREQ = 24000000;
	static const uint32_t APB1_FREQ = 24000000;
	static const uint32_t APB2_FREQ = 24000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFFF7E8;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFFF7E0;
	enum { HAVE_BUS = false };
	enum { SPI_COUNT = 1 };
	enum { USART_COUNT = 2 };
	enum { HAVE_USB = false };
	enum { HAVE_OTG = false };
};

template<>
struct ChipCaps<stm32F10X_MD_VL>
{
	static const uint32_t MAX_FREQ = 24000000;
	static const uint32_t APB1_FREQ = 24000000;
	static const uint32_t APB2_FREQ = 24000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFFF7E8;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFFF7E0;
	enum { HAVE_BUS = false };
	enum { SPI_COUNT = 2 };
	enum { USART_COUNT = 3 };
	enum { HAVE_USB = false };
	enum { HAVE_OTG = false };
};

template<>
struct ChipCaps<stm32F10X_HD_VL>
{
	static const uint32_t MAX_FREQ = 24000000;
	static const uint32_t APB1_FREQ = 24000000;
	static const uint32_t APB2_FREQ = 24000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFFF7E8;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFFF7E0;
	enum { HAVE_BUS = false };
	enum { SPI_COUNT = 2 };
	enum { USART_COUNT = 3 };
	enum { HAVE_USB = false };
	enum { HAVE_OTG = false };
};

template<>
struct ChipCaps<stm32F2XX>
{
	static const uint32_t MAX_FREQ = 120000000;
	static const uint32_t APB1_FREQ = 30000000;
	static const uint32_t APB2_FREQ = 60000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFF7A10;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFF7A22;
	enum { HAVE_BUS = true };
	enum { SPI_COUNT = 3 };
	enum { USART_COUNT = 6 };
	enum { HAVE_USB = true };
	enum { HAVE_OTG = true };
};

template<>
struct ChipCaps<stm32F4XX>
{
	static const uint32_t MAX_FREQ = 168000000;
	static const uint32_t APB1_FREQ = 42000000;
	static const uint32_t APB2_FREQ = 84000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFF7A10;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFF7A22;
	static const uint32_t DEVICE_ID = 0x413;
	enum { HAVE_BUS = true };
	enum { SPI_COUNT = 3 };
	enum { USART_COUNT = 6 };
	enum { HAVE_USB = true };
	enum { HAVE_OTG = true };
};

template<>
struct ChipCaps<stm32L0XX>
{
	static const uint32_t MAX_FREQ  = 32000000;
	static const uint32_t APB1_FREQ = 32000000;
	static const uint32_t APB2_FREQ = 32000000;
	static const uint32_t DEVICE_ID_ADDR = 0x1FFF7A10;
	static const uint32_t FLASH_SIZE_ADDR = 0x1FFF7A22;
	static const uint32_t DEVICE_ID = 0x413;
	enum { HAVE_BUS = false };
	enum { SPI_COUNT = 3 };
	enum { USART_COUNT = 2 };
	enum { HAVE_USB = true };
	enum { HAVE_OTG = false };
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
	using caps::HAVE_BUS;
	using caps::SPI_COUNT;
	using caps::USART_COUNT;
	using caps::HAVE_USB;
	using caps::HAVE_OTG;

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
#elif (defined STM32L0XX)
	typedef ChipInfo<stm32L0XX> chip;
#else
#	error Chip type (STM32FXXX_XX) must be defined.
#endif


#endif // STM32TPL_STM32_H_INCLUDED
