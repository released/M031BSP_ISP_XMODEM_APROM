/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  0x32
 * @date     14, June, 2017
 *
 * @note
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdarg.h>

#include "targetdev.h"
#include "uart_transfer.h"
#include "EEPROM_Emulate.h"  

#include "xmodem.h"

#define TIMEOUT_INTERVAL    	5   // sec
__IO uint32_t timeout_cnt = 0;


#define DEBUG_UART_PORT						    (UART1)
#define DEBUG_UART_PORT_IRQn				    (UART13_IRQn)
#define DEBUG_UART_IRQHandler				    (UART13_IRQHandler)


// TODO:
/*
	boot loader code range :

	target APROM size : 128K = 0x20 000

	LDROM : 4K

	LDROM extra storage in APROM start address : // 14K
	0x1C000 ~ 0x1F800

	for checksum storage start address :
	0x1C000-4 = 0x1BFFC   

	DATA FALSH in APROM start address : // 2K
	0x1F800 ~ 0x1FFFF
*/

#define DATA_FLASH_OFFSET  						(0x1F800)

#define DATA_FLASH_AMOUNT						(48)
#define DATA_FLASH_PAGE  						(4)     // M031 page : 0x200 (512)

// #define ENABLE_SW_CRC32

#if defined (ENABLE_SW_CRC32)
unsigned long table[256];
unsigned long state = 0xFFFFFFFF;
#define POLYNOMIAL 0xedb88320
void calculate_table(void)
{
    unsigned b = 0;
    unsigned long v = 0;
    int i = 0;

    for (b = 0; b < 256; ++b)
    {
        v = b;
        i = 8;
        for (; --i >= 0; )
            v = (v & 1) ? ((v >> 1) ^ POLYNOMIAL) : (v >> 1);
        table[b] = v;
    }
}

unsigned long UPDC32(unsigned char octet, unsigned long crc)
{
    // The original code had this as a #define
    return table[(crc ^ octet) & 0xFF] ^ (crc >> 8);
}

#endif

#if defined (ENABLE_EMULATE_EEPROM)
int set_data_flash_base(uint32_t u32DFBA)
{
    uint32_t   au32Config[2];
	
    /* Read User Configuration 0 & 1 */
    if (FMC_ReadConfig(au32Config, 2) < 0)
    {
        printf("\nRead User Config failed!\n");
        return -1;
    }

    /* Check if Data Flash is enabled (CONFIG0[0]) and is expected address (CONFIG1) */
    if ((!(au32Config[0] & 0x1)) && (au32Config[1] == u32DFBA))
        return 0;

    FMC_ENABLE_CFG_UPDATE();

    au32Config[0] &= ~0x1;         /* CONFIG0[0] = 0 (Enabled) / 1 (Disabled) */
    au32Config[1] = u32DFBA;

    /* Update User Configuration settings. */
    if (FMC_WriteConfig(au32Config, 2) < 0)
        return -1;

    FMC_ReadConfig(au32Config, 2);

    /* Check if Data Flash is enabled (CONFIG0[0]) and is expected address (CONFIG1) */
    if (((au32Config[0] & 0x01) == 1) || (au32Config[1] != u32DFBA))
    {
        printf("Error: Program Config Failed!\n");
        /* Disable FMC ISP function */
        FMC_Close();
        SYS_LockReg();
        return -1;
    }


    printf("\nSet Data Flash base as 0x%x.\n", u32DFBA);

    /* To check if all the debug messages are finished */
    while(!UART_IS_TX_EMPTY(DEBUG_UART_PORT));//while(!IsDebugFifoEmpty());

    /* Perform chip reset to make new User Config take effect */
    SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;
    return 0;
}

void Emulate_EEPROM(void)
{
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    if (set_data_flash_base(DATA_FLASH_OFFSET) < 0)
    {
        printf("Failed to set Data Flash base address!\r\n");
    }

	/* Test Init_EEPROM() */
	Init_EEPROM(DATA_FLASH_AMOUNT, DATA_FLASH_PAGE);
	Search_Valid_Page();	
}
#endif

//
// check_reset_source
//
uint8_t check_reset_source(void)
{
    uint8_t tag = 0;
    uint32_t src = SYS_GetResetSrc();

    SYS->RSTSTS |= 0x1FF;
    LDROM_DEBUG("Reset Source <0x%08X>\r\n", src);
   
    tag = read_magic_tag();
    
    if (src & SYS_RSTSTS_PORF_Msk) {
        SYS_ClearResetSrc(SYS_RSTSTS_PORF_Msk);
        
        if (tag == 0xA5) {
            write_magic_tag(0);
            LDROM_DEBUG("Enter BOOTLOADER from APPLICATION\r\n");
            return TRUE;
        } else if (tag == 0xBB) {
            write_magic_tag(0);
            LDROM_DEBUG("Upgrade finished...\r\n");
            return FALSE;
        } else {
            LDROM_DEBUG("Enter BOOTLOADER from POR\r\n");
            return FALSE;
        }
    } else if (src & SYS_RSTSTS_PINRF_Msk){
        SYS_ClearResetSrc(SYS_RSTSTS_PINRF_Msk);
        LDROM_DEBUG("Enter BOOTLOADER from nRESET pin\r\n");
        return FALSE;
    }
    
    LDROM_DEBUG("Enter BOOTLOADER from unhandle reset source\r\n");
    return FALSE;
}

uint32_t caculate_crc32_checksum(uint32_t start, uint32_t size)
{
    #if defined (ENABLE_SW_CRC32)
    volatile uint32_t addr, data;    

    LDROM_DEBUG("ENABLE_SW_CRC32\r\n",);
    calculate_table();
    state = 0xFFFFFFFF;    
    addr = start;

    for(addr = start; addr < (start+size) ; addr += 4){
        data = FMC_Read(addr);
        state = UPDC32(GET_BYTE0(data), state);
        state = UPDC32(GET_BYTE1(data), state);
        state = UPDC32(GET_BYTE2(data), state);
        state = UPDC32(GET_BYTE3(data), state); 
    }

    return ~state;   

    #else
    volatile uint32_t addr, data;

    LDROM_DEBUG("HW CRC32\r\n",);
    CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFF, CRC_WDATA_32);
    
    for(addr = start; addr < (start+size) ; addr += 4){
        data = FMC_Read(addr);
        CRC_WRITE_DATA(data);
    }
    return CRC_GetChecksum();
    #endif
}

uint8_t verify_application_chksum(void)
{
    uint32_t chksum_cal, chksum_app;
    
    LDROM_DEBUG("Verify Checksum\r\n");
    
    chksum_cal = caculate_crc32_checksum(0x00000000, (g_apromSize - 4));//(g_apromSize - FMC_FLASH_PAGE_SIZE)
    LDROM_DEBUG("Caculated .....<0x%08X>\r\n", chksum_cal);
    
    chksum_app = FMC_Read(g_apromSize - 4);    
    LDROM_DEBUG("In APROM ......<0x%08X>\r\n", chksum_app);
    
    if (chksum_cal == chksum_app) {
        LDROM_DEBUG("Verify ........<PASS>\r\n");
        return TRUE;
    } else {
        LDROM_DEBUG("Verify ........<FAIL>\r\n");
        return FALSE;
    }
}


void UARTx_Process(void)
{
	uint8_t res = 0;

	res = UART_READ(DEBUG_UART_PORT);

	if (res > 0x7F)
	{
		LDROM_DEBUG("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
	
			case '1':

				break;	

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
			
				break;		
			
		}
	}
}

void DEBUG_UART_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(DEBUG_UART_PORT, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(DEBUG_UART_PORT) == 0)
        {
			UARTx_Process();
        }
    }

    if(DEBUG_UART_PORT->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(DEBUG_UART_PORT, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }
}

void DEBUG_UART_Init(void)
{
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 baud rate */
    UART_Open(DEBUG_UART_PORT, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(DEBUG_UART_PORT, 20);

	DEBUG_UART_PORT->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	DEBUG_UART_PORT->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(DEBUG_UART_PORT, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(DEBUG_UART_PORT_IRQn);

	LDROM_DEBUG("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	LDROM_DEBUG("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	LDROM_DEBUG("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	LDROM_DEBUG("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	LDROM_DEBUG("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
}

void TMR1_IRQHandler(void)
{
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
        timeout_cnt++;
    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
//    TIMER_Start(TIMER1);
}


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);	

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);	

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    CLK_EnableModuleClock(UART1_MODULE);
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    // CLK_EnableModuleClock(RTC_MODULE);
    // CLK_SetModuleClock(RTC_MODULE, CLK_CLKSEL3_RTCSEL_LIRC,  NULL);

    CLK_EnableModuleClock(CRC_MODULE);
	
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD);
	
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    // uint32_t lcmd;
    int32_t i32Err;
    uint32_t fw_addr = 0;
    uint8_t i = 0;
    uint8_t buffer[16] = {0};

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Init UART to 115200-8n1 */

	DEBUG_UART_Init();
    ISP_UART_Init();
	TIMER1_Init();
    
    #if defined (ENABLE_EMULATE_EEPROM)
    Emulate_EEPROM();
    #endif

	CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    // Enable FMC and APROM update
    //
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();
    
    //
    // Get APROM and data flash information
    //
    g_apromSize = APROM_APPLICATION_SIZE;
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

    //
    // Stay in BOOTLOADER or jump to APPLICATION
    //
    if (!check_reset_source()) 
	{
        if (verify_application_chksum()) 
		{
            goto exit;
        } 
		else 
		{
            LDROM_DEBUG("Stay in BOOTLOADER (checksum invaild)\r\n");
        }
    } 
	else 
    {
        LDROM_DEBUG("Stay in BOOTLOADER (from APPLICATION)\r\n");
        
        //
        // start timer
        //
        LDROM_DEBUG("Time-out counter start....\r\n");
        TIMER_Start(TIMER1);
    }

    SYS_UnlockReg();
    FMC_Open();

    bUartDataReady = TRUE;

    LDROM_DEBUG("receive Xmodem data\r\n");
    while(!UART_IS_TX_EMPTY(DEBUG_UART_PORT));
 
	
    while (1)
    {
        if (bUartDataReady == TRUE)
        {
            goto _ISP;
        }

        if (timeout_cnt > TIMEOUT_INTERVAL) {
            LDROM_DEBUG("Time-out, perform CHIP_RST\r\n");
            SystemReboot_CHIP_RST();
        }

    }

_ISP:

    while (1)
    {
        if (bUartDataReady == TRUE)
        {
            i32Err = Xmodem(FMC_APROM_BASE);
        }

        if(i32Err < 0)
        {
            LDROM_DEBUG("Xmodem transfer fail!\r\n");
            while(1);
        }
        else
        {
            LDROM_DEBUG("Xomdem transfer done!\r\n");
            LDROM_DEBUG("Total trnasfer size is %d\r\n", i32Err);

            write_magic_tag(0xBB); 
            
            //
            // In order to verify the checksum in the application, 
            // do CHIP_RST to enter bootloader again.
            //
            LDROM_DEBUG("Perform CHIP_RST...\r\n");
            SystemReboot_CHIP_RST();

            /* Trap the CPU */
            while (1);
        }
    }


exit:
    fw_addr = APROM_FW_VER_ADDR;
    for (i = 0 ; i <16 ; i++)
    {
        buffer[i] = *(__IO uint8_t *)fw_addr;
        fw_addr++;
    }
    LDROM_DEBUG("Jump to <APPLICATION>,%s\r\n",buffer);
    while(!UART_IS_TX_EMPTY(DEBUG_UART_PORT));
    
    FMC_SetVectorPageAddr(0);               /* Set vector remap to APROM address 0x0      */
    FMC_SET_APROM_BOOT();                   /* Change boot source as APROM                */
    SYS->IPRST0 = SYS_IPRST0_CPURST_Msk;    /* Let CPU reset. Will boot from APROM.       */
    
    /* Trap the CPU */
    while (1);   

	
}
