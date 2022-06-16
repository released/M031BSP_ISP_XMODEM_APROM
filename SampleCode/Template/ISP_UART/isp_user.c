/**************************************************************************//**
 * @file     isp_user.c
 * @brief    ISP Command source file
 * @version  0x32
 * @date     14, June, 2017
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "string.h"
#include "isp_user.h"
#include "fmc_user.h"
#include "EEPROM_Emulate.h"  

#if 0
#define RSTSTS      RSTSRC
#define ISPCTL      ISPCON
#endif


volatile uint8_t bISPDataReady;
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t response_buff[64];
static uint8_t aprom_buf[FMC_FLASH_PAGE_SIZE];
#else
uint8_t response_buff[64] __attribute__((aligned(4)));
static uint8_t aprom_buf[FMC_FLASH_PAGE_SIZE] __attribute__((aligned(4)));
#endif

uint32_t bUpdateApromCmd;
uint32_t g_apromSize, g_dataFlashAddr, g_dataFlashSize;

void SystemReboot_CHIP_RST(void)
{
    while(!UART_IS_TX_EMPTY(UART1));
        
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();

    FMC_SET_LDROM_BOOT();

    // NVIC_SystemReset();
    // SYS_ResetCPU();  
    SYS_ResetChip();  
}

uint8_t read_magic_tag(void)
{
    uint8_t tag = 0;

    #if defined (ENABLE_EMULATE_EEPROM)
    
    Read_Data(EEP_MAGIC_TAG_ADDR , &tag);

    #endif

    #if defined (ENABLE_RTC)
    RTC_EnableSpareAccess();

    RTC->RWEN = RTC_WRITE_KEY;
    while(!(RTC->RWEN & RTC_RWEN_RWENF_Msk));
    
    tag =  RTC_READ_SPARE_REGISTER(0);

    #endif

    LDROM_DEBUG("Read MagicTag <0x%02X>\r\n", tag);
    
    return tag;
}

void write_magic_tag(uint8_t tag)
{
    #if defined (ENABLE_EMULATE_EEPROM)

    Write_Data(EEP_MAGIC_TAG_ADDR , tag);

	// /* Disable FMC ISP function */
	// FMC_Close();

	// /* Lock protected registers */
	// SYS_LockReg();

    #endif

    #if defined (ENABLE_RTC)    
    RTC_EnableSpareAccess();

    RTC->RWEN = RTC_WRITE_KEY;
    while(!(RTC->RWEN & RTC_RWEN_RWENF_Msk));
    
    RTC_WRITE_SPARE_REGISTER(0, tag);
    #endif

    LDROM_DEBUG("Write MagicTag <0x%02X>\r\n", tag);
}


__STATIC_INLINE uint16_t Checksum(unsigned char *buf, int len)
{
    int i;
    uint16_t c;

    for (c = 0, i = 0 ; i < len; i++)
    {
        c += buf[i];
    }

    return (c);
}

static uint16_t CalCheckSum(uint32_t start, uint32_t len)
{
    int i;
    register uint16_t lcksum = 0;

    for (i = 0; i < len; i += FMC_FLASH_PAGE_SIZE)
    {
        ReadData(start + i, start + i + FMC_FLASH_PAGE_SIZE, (uint32_t *)aprom_buf);

        if (len - i >= FMC_FLASH_PAGE_SIZE)
        {
            lcksum += Checksum(aprom_buf, FMC_FLASH_PAGE_SIZE);
        }
        else
        {
            lcksum += Checksum(aprom_buf, len - i);
        }
    }

    return lcksum;
}

int ParseCmd(unsigned char *buffer, uint8_t len)
{
    static uint32_t StartAddress, StartAddress_bak, TotalLen, TotalLen_bak, LastDataLen, g_packno = 1;
    uint8_t *response;
    uint16_t lcksum;
    uint32_t lcmd, srclen, /*i,*/ regcnf0, security;
    unsigned char *pSrc;
    static uint32_t	gcmd;
    response = response_buff;
    pSrc = buffer;
    srclen = len;
    lcmd = inpw(pSrc);
    outpw(response + 4, 0);
    pSrc += 8;
    srclen -= 8;
    ReadData(Config0, Config0 + 16, (uint32_t *)(response + 8)); //read config
    regcnf0 = *(uint32_t *)(response + 8);
    security = regcnf0 & 0x2;

    if (lcmd == CMD_SYNC_PACKNO)
    {
        g_packno = inpw(pSrc);
    }

    if ((lcmd) && (lcmd != CMD_RESEND_PACKET))
    {
        gcmd = lcmd;
    }

    if (lcmd == CMD_GET_FWVER)
    {
        response[8] = FW_VERSION;
    }
    else if (lcmd == CMD_GET_DEVICEID)
    {
        outpw(response + 8, SYS->PDID);
        goto out;
    }
    else if (lcmd == CMD_RUN_APROM || lcmd == CMD_RUN_LDROM || lcmd == CMD_RESET)
    {
		#if 1
        write_magic_tag(0xBB); 
        
        //
        // In order to verify the checksum in the application, 
        // do CHIP_RST to enter bootloader again.
        //
        LDROM_DEBUG("Perform CHIP_RST...\r\n");
        SystemReboot_CHIP_RST();
		#else
        outpw(&SYS->RSTSTS, 3);//clear bit

        /* Set BS */
        if (lcmd == CMD_RUN_APROM)
        {
            i = (FMC->ISPCTL & 0xFFFFFFFC);
        }
        else if (lcmd == CMD_RUN_LDROM)
        {
            i = (FMC->ISPCTL & 0xFFFFFFFC);
            i |= 0x00000002;
        }
        else
        {
            i = (FMC->ISPCTL & 0xFFFFFFFE);//ISP disable
        }

        outpw(&FMC->ISPCTL, i);
        outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));
		#endif

        /* Trap the CPU */
        while (1);
    }
    else if (lcmd == CMD_CONNECT)
    {
        g_packno = 1;
        goto out;
    }
    else if ((lcmd == CMD_UPDATE_APROM) || (lcmd == CMD_ERASE_ALL))
    {
        EraseAP(FMC_APROM_BASE, (g_apromSize < g_dataFlashAddr) ? g_apromSize : g_dataFlashAddr); // erase APROM // g_dataFlashAddr, g_apromSize

        if (lcmd == CMD_ERASE_ALL)   //erase data flash
        {
            EraseAP(g_dataFlashAddr, g_dataFlashSize);
            *(uint32_t *)(response + 8) = regcnf0 | 0x02;
            UpdateConfig((uint32_t *)(response + 8), NULL);
        }

        bUpdateApromCmd = TRUE;
    }
    else if (lcmd == CMD_GET_FLASHMODE)
    {
        //return 1: APROM, 2: LDROM
        outpw(response + 8, (FMC->ISPCTL & 0x2) ? 2 : 1);
    }

    if ((lcmd == CMD_UPDATE_APROM) || (lcmd == CMD_UPDATE_DATAFLASH))
    {
        if (lcmd == CMD_UPDATE_DATAFLASH)
        {
            StartAddress = g_dataFlashAddr;

            if (g_dataFlashSize)   //g_dataFlashAddr
            {
                EraseAP(g_dataFlashAddr, g_dataFlashSize);
            }
            else
            {
                goto out;
            }
        }
        else
        {
            StartAddress = 0;
        }

        //StartAddress = inpw(pSrc);
        TotalLen = inpw(pSrc + 4);
        pSrc += 8;
        srclen -= 8;
        StartAddress_bak = StartAddress;
        TotalLen_bak = TotalLen;
    }
    else if (lcmd == CMD_UPDATE_CONFIG)
    {
        if ((security == 0) && (!bUpdateApromCmd))   //security lock
        {
            goto out;
        }

        UpdateConfig((uint32_t *)(pSrc), (uint32_t *)(response + 8));
        GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
        goto out;
    }
    else if (lcmd == CMD_RESEND_PACKET)     //for APROM&Data flash only
    {
        uint32_t PageAddress;
        StartAddress -= LastDataLen;
        TotalLen += LastDataLen;
        PageAddress = StartAddress & (0x100000 - FMC_FLASH_PAGE_SIZE);

        if (PageAddress >= Config0)
        {
            goto out;
        }

        ReadData(PageAddress, StartAddress, (uint32_t *)aprom_buf);
        FMC_Erase_User(PageAddress);
        WriteData(PageAddress, StartAddress, (uint32_t *)aprom_buf);

        if ((StartAddress % FMC_FLASH_PAGE_SIZE) >= (FMC_FLASH_PAGE_SIZE - LastDataLen))
        {
            FMC_Erase_User(PageAddress + FMC_FLASH_PAGE_SIZE);
        }

        goto out;
    }

    if ((gcmd == CMD_UPDATE_APROM) || (gcmd == CMD_UPDATE_DATAFLASH))
    {
        if (TotalLen < srclen)
        {
            srclen = TotalLen;//prevent last package from over writing
        }

        TotalLen -= srclen;
        WriteData(StartAddress, StartAddress + srclen, (uint32_t *)pSrc); //WriteData(StartAddress, StartAddress + srclen, (uint32_t*)pSrc);
        memset(pSrc, 0, srclen);
        ReadData(StartAddress, StartAddress + srclen, (uint32_t *)pSrc);
        StartAddress += srclen;
        LastDataLen =  srclen;

        if (TotalLen == 0)
        {
            lcksum = CalCheckSum(StartAddress_bak, TotalLen_bak);
            outps(response + 8, lcksum);
        }
    }

out:
    lcksum = Checksum(buffer, len);
    outps(response, lcksum);
    ++g_packno;
    outpw(response + 4, g_packno);
    g_packno++;
    return 0;
}

