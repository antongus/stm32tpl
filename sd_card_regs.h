/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *  Visit https://github.com/antongus/stm32tpl for new versions
 *
 *  Copyright (c) 2011-2024 Anton B. Gusev
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
 *  file         : sd_card_regs.h
 *  description  : SD card data structures (not stm32-specific)
 */

#pragma once

/**
 * Card IDentification (CID) register.
 * Contains the card identification information used during the
 * card identification phase. Every individual
 * Read/Write (RW) card shall have a unique identification number.
 */
struct CIDRegister
{
	uint8_t		manufacturerId;
	uint16_t	oemId;
	char		productName[6];
	uint32_t	serial;
	void fill(uint32_t* data)
	{
		struct CIDRegisterRaw
		{
			uint8_t		CRC7;     //
			uint16_t	MDT;      // Manufacturing date
			uint32_t	PSN;      // Product serial number
			uint8_t		PRV;      // Product revision
			uint8_t		PNM[5];   // Product name
			uint16_t	OID;      // OEM/Application ID
			uint8_t		MID;      // Manufacturer ID
		}__attribute__ ((packed))
		*cid = (CIDRegisterRaw*)data;

		manufacturerId = cid->MID;
		oemId = cid->OID;

		for (auto i = 0; i < 5; ++i)
			productName[i] = cid->PNM[i];
		productName[5] = 0;

		serial = cid->PSN;
	}
};

/**
 * Card-Specific Data (CSD) register.
 * CSD contains card information: size, sector count, etc.
 * This structure is used to parse CSD register.
 */
struct CSDRegister
{
	union
	{
		uint32_t words[4];
		struct
		{
			uint32_t		LAST_BIT              :1;
			uint32_t		CRC7                  :7;     // CRC
			uint32_t		                      :2;
			uint32_t		FILE_FORMAT           :2;     // File format
			uint32_t		TMP_WRITE_PROTECT     :1;     // temporary write protection
			uint32_t		PERM_WRITE_PROTECT    :1;     // permanent write protection
			uint32_t		COPY                  :1;     // copy flag
			uint32_t		FILE_FORMAT_GRP       :1;     // File format group
			uint32_t		                      :5;
			uint32_t		WRITE_BL_PARTIAL      :1;     // partial blocks for write allowed
			uint32_t		WRITE_BL_LEN          :4;     // max. write data block length
			uint32_t		R2W_FACTOR            :3;     // write speed factor
			uint32_t		                      :2;
			uint32_t		WP_GRP_ENABLE         :1;     // write protect group enable
			uint32_t		WP_GRP_SIZE           :7;     // write protect group size
			uint32_t		SECTOR_SIZE           :7;     // erase sector size
			uint32_t		ERASE_BLK_EN          :1;     // erase single block enable
			uint32_t		C_SIZE_MULT           :3;     // device size multiplier
			uint32_t		VDD_W_CURR_MAX        :3;     // max. write current @VDD max
			uint32_t		VDD_W_CURR_MIN        :3;     // max. write current @VDD min
			uint32_t		VDD_R_CURR_MAX        :3;     // max. read current @VDD max
			uint32_t		VDD_R_CURR_MIN        :3;     // max. read current @VDD min
			uint32_t		C_SIZE                :12;    // device size
			uint32_t		                      :2;
			uint32_t		DSR_IMP               :1;     // DSR(Driver Stage Register) implemented
			uint32_t		READ_BLK_MISALIGN     :1;     // read block misalignment
			uint32_t		WRITE_BLK_MISALIGN    :1;     // write block misalignment (1 = crossing physical block boundaries is allowed)
			uint32_t		READ_BL_PARTIAL       :1;     // partial blocks read allowed (always 1 in SD cards)
			uint32_t		READ_BL_LEN           :4;     // max. read data block length (=2^READ_BL_LEN)
			uint32_t		CCC                   :12;    // supported Card Command Classes (bit mask)
			uint32_t		TRAN_SPEED            :8;     // max. data transfer rate. For SD cards always 0x32(25MHz), in HS mode - 0x5A(50MHz)
			uint32_t		NSAC                  :8;     // data read access-time-2 in CLK cycles (NSAC*100)
			uint32_t		TAAC                  :8;     // data read access-time-1
			uint32_t		                      :6;
			uint32_t		CSD_STRUCTURE         :2;
		}__attribute__ ((packed))
		CSD;

		struct
		{
			uint32_t		LAST_BIT              :1;
			uint32_t		CRC7                  :7;     // CRC
			uint32_t		                      :2;
			uint32_t		FILE_FORMAT           :2;     // File format
			uint32_t		TMP_WRITE_PROTECT     :1;     // temporary write protection
			uint32_t		PERM_WRITE_PROTECT    :1;     // permanent write protection
			uint32_t		COPY                  :1;     // copy flag
			uint32_t		FILE_FORMAT_GRP       :1;     // File format group
			uint32_t		                      :5;
			uint32_t		WRITE_BL_PARTIAL      :1;     // partial blocks for write allowed
			uint32_t		WRITE_BL_LEN          :4;     // max. write data block length
			uint32_t		R2W_FACTOR            :3;     // write speed factor
			uint32_t		                      :2;
			uint32_t		WP_GRP_ENABLE         :1;     // write protect group enable
			uint32_t		WP_GRP_SIZE           :7;     // write protect group size
			uint32_t		SECTOR_SIZE           :7;     // erase sector size
			uint32_t		ERASE_BLK_EN          :1;     // erase single block enable
			uint32_t		                      :1;
			uint32_t		C_SIZE                :22;    // device size
			uint32_t		                      :6;
			uint32_t		DSR_IMP               :1;     // DSR(Driver Stage Register) implemented
			uint32_t		READ_BLK_MISALIGN     :1;     // read block misalignment
			uint32_t		WRITE_BLK_MISALIGN    :1;     // write block misalignment (1 = crossing physical block boundaries is allowed)
			uint32_t		READ_BL_PARTIAL       :1;     // partial blocks read allowed (always 1 in SD cards)
			uint32_t		READ_BL_LEN           :4;     // max. read data block length (=2^READ_BL_LEN)
			uint32_t		CCC                   :12;    // supported Card Command Classes (bit mask)
			uint32_t		TRAN_SPEED            :8;     // max. data transfer rate. For SD cards always 0x32(25MHz), in HS mode - 0x5A(50MHz)
			uint32_t		NSAC                  :8;     // data read access-time-2 in CLK cycles (NSAC*100)
			uint32_t		TAAC                  :8;     // data read access-time-1
			uint32_t		                      :6;
			uint32_t		CSD_STRUCTURE         :2;
		}__attribute__ ((packed))
		CSD2;
	};

	CSDRegister() { }
	CSDRegister(uint32_t* data) { fill(data); }

	void fill(uint32_t *data)
	{
		words[0] = data[0];
		words[1] = data[1];
		words[2] = data[2];
		words[3] = data[3];
	}

	bool isV20() { return CSD.CSD_STRUCTURE; }
	uint32_t getSectorCount()
	{
		if (isV20())     // CSD Version 2.0
			return (CSD2.C_SIZE + 1) * 1024;

		// CSD Version 1.0
		uint32_t blockLen = 1UL << (CSD.READ_BL_LEN);
		uint32_t sizeMultiplier = 1UL << (CSD.C_SIZE_MULT + 2);
		uint32_t blocks = (CSD.C_SIZE + 1) * sizeMultiplier;
		return blocks * blockLen / 512;  // size / sector size
	}
	uint32_t getSize() { return getSectorCount() / 2; }   // size in K bytes
	uint32_t getBlockLen() { return isV20() ? 512 : 1UL << (CSD.READ_BL_LEN); }
};

/**
 * Card Status structure and flags.
 * Card status contains error and state information of a executed command.
 * This structure is used to parse R1 response from card.
 */
struct CardStatus
{
	static constexpr auto CARD_STATE_IDLE     {0u};
	static constexpr auto CARD_STATE_READY    {1u};
	static constexpr auto CARD_STATE_IDENT    {2u};
	static constexpr auto CARD_STATE_STANDBY  {3u};
	static constexpr auto CARD_STATE_TRAN     {4u};
	static constexpr auto CARD_STATE_DATA     {5u};
	static constexpr auto CARD_STATE_RCV      {6u};
	static constexpr auto CARD_STATE_PRG      {7u};
	static constexpr auto CARD_STATE_DIS      {8u};

	union
	{
		uint32_t R1;
		struct
		{
			uint32_t		                      :3;
			uint32_t		AKE_SEQ_ERROR         :1;    // 3 Error in the sequence of the authentication process
			uint32_t		                      :1;
			uint32_t		APP_CMD               :1;    // 5 The card will expect ACMD/command has been interpreted as ACMD
			uint32_t		                      :2;
			uint32_t		READY_FOR_DATA        :1;    // 8 Corresponds to buffer empty signaling on the bus
			uint32_t		CURRENT_STATE         :4;    // 9..12
			uint32_t		ERASE_RESET           :1;    // 13
			uint32_t		CARD_ECC_DISABLED     :1;    // 14
			uint32_t		WP_ERASE_SKIP         :1;    // 15
			uint32_t		CSD_OVERWRITE         :1;    // 16
			uint32_t		                      :2;
			uint32_t		ERROR                 :1;    // 19 A general or an unknown error occurred during the operation.
			uint32_t		CC_ERROR              :1;    // 20 Internal card controller error
			uint32_t		CARD_ECC_FAILED       :1;    // 21 Card internal ECC was applied but failed to correct the data.
			uint32_t		ILLEGAL_COMMAND       :1;    // 22
			uint32_t		COM_CRC_ERROR         :1;    // 23
			uint32_t		LOCK_UNLOCK_FAILED    :1;    // 24
			uint32_t		CARD_IS_LOCKED        :1;    // 25
			uint32_t		WP_VIOLATION          :1;    // 26
			uint32_t		ERASE_PARAM           :1;    // 27
			uint32_t		ERASE_SEQ_ERROR       :1;    // 28
			uint32_t		BLOCK_LEN_ERROR       :1;    // 29
			uint32_t		ADDRESS_ERROR         :1;    // 30
			uint32_t		OUT_OF_RANGE          :1;    // 31 The command's argument was out of the allowed range for this card.
		}__attribute__ ((packed))
		bits;
	};
	CardStatus(uint32_t resp = 0)
	{
		operator =(resp);
	}
	CardStatus(const CardStatus & cs)
	{
		R1 = cs.R1;
	}

    operator uint32_t() const
    {
        return R1;
    }

    CardStatus& operator= (const CardStatus& other)
	{
		R1 = other.R1;
		return *this;
	}

    CardStatus& operator= (const uint32_t t)
	{
		R1 = t;
		return *this;
	}

    static constexpr auto CS_AKE_SEQ_ERROR          {1UL << 3};    // 3 Error in the sequence of the authentication process
    static constexpr auto CS_APP_CMD                {1UL << 5};    // 5 The card will expect ACMD/command has been interpreted as ACMD
    static constexpr auto CS_READY_FOR_DATA         {1UL << 8};    // 8 Corresponds to buffer empty signaling on the bus
    static constexpr auto CS_CURRENT_STATE          {0xFUL << 9};  // 9..12
    static constexpr auto CS_ERASE_RESET            {1UL << 13};    // 13
    static constexpr auto CS_CARD_ECC_DISABLED      {1UL << 14};    // 14
    static constexpr auto CS_WP_ERASE_SKIP          {1UL << 15};    // 15
    static constexpr auto CS_CSD_OVERWRITE          {1UL << 16};    // 16
    static constexpr auto CS_ERROR                  {1UL << 19};    // 19 A general or an unknown error occurred during the operation.
    static constexpr auto CS_CC_ERROR               {1UL << 20};    // 20 Internal card controller error
    static constexpr auto CS_CARD_ECC_FAILED        {1UL << 21};    // 21 Card internal ECC was applied but failed to correct the data.
    static constexpr auto CS_ILLEGAL_COMMAND        {1UL << 22};    // 22
    static constexpr auto CS_COM_CRC_ERROR          {1UL << 23};    // 23
    static constexpr auto CS_LOCK_UNLOCK_FAILED     {1UL << 24};    // 24
    static constexpr auto CS_CARD_IS_LOCKED         {1UL << 25};    // 25
    static constexpr auto CS_WP_VIOLATION           {1UL << 26};    // 26
    static constexpr auto CS_ERASE_PARAM            {1UL << 27};    // 27
    static constexpr auto CS_ERASE_SEQ_ERROR        {1UL << 28};    // 28
    static constexpr auto CS_BLOCK_LEN_ERROR        {1UL << 29};    // 29
    static constexpr auto CS_ADDRESS_ERROR          {1UL << 30};    // 30
    static constexpr auto CS_OUT_OF_RANGE           {1UL << 31};    // 31 The command's argument was out of the allowed range for this card.
    static constexpr auto CS_ERROR_MASK             {0xFDFFE008U};     // all error bits
};

static_assert(sizeof(CardStatus) == sizeof(uint32_t), "wrong CardStatus size");

struct SCRRegister
{
	union
	{
		uint32_t words[2];
		struct
		{
			uint32_t		MANUFACTURER_DATA     :32;   // reserved for manufacturer usage
			uint32_t		CMD_SUPPORT           :2;    // Command Support bits
			uint32_t		                      :9;    //
			uint32_t		EX_SECURITY           :4;    // Extended Security Support
			uint32_t		SD_SPEC3              :1;    // Spec. Version 3.00 or higher
			uint32_t		SD_BUS_WIDTHS         :4;    // DAT Bus widths supported
			uint32_t		SD_SECURITY           :3;    // CPRM Security Support
			uint32_t		DATA_STAT_AFTER_ERASE :1;    // data_status_after erases
			uint32_t		SD_SPEC               :4;    // SD Memory Card - Spec. Version
			uint32_t		SCR_STRUCTURE         :4;    // SCR Structure
		}__attribute__ ((packed))
		bits;
	};

	enum
	{
		SD_SPEC_V10             = 0,
		SD_SPEC_V11             = 1,
		SD_SPEC_V2_V3           = 2,
		SD_SPEC3_V2             = 0,
		SD_SPEC3_V3             = 1,
		SD_SECURITY_NONE        = 0,
		SD_SECURITY_SDSC        = 2,
		SD_SECURITY_SDHC        = 3,
		SD_SECURITY_SDXC        = 4,
		SD_BUS_WIDTH_1BIT       = 1,
		SD_BUS_WIDTH_4BIT       = 4,
		CMD_SUPPORT_CMD20       = 1,
		CMD_SUPPORT_CMD23       = 2
	};
};

