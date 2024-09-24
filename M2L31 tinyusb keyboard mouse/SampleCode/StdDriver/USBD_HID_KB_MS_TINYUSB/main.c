/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 13 $
 * $Date: 18/07/18 3:24p $
 * @brief
 *           Demonstrate how to implement a TINYUSB keyboard and mouse device.
 *           It supports to use GPIO to simulate keyboard and mouse input.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
void hid_task(void);
//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

// Interface index depends on the order in configuration descriptor
enum {
  ITF_KEYBOARD = 0,
	ITF_MOUSE = 1,
};

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;


volatile uint32_t system_ticks = 0;
void SysTick_Handler (void)
{
  system_ticks++;
}

uint32_t board_millis(void)
{
  return system_ticks;
}

#define CRYSTAL_LESS        1    /* CRYSTAL_LESS must be 1 if USB clock source is HIRC */
#define TRIM_INIT           (SYS_BASE+0x10C)

void USBD_IRQHandler(void)
{
  tud_int_handler(0);
}

int IsDebugFifoEmpty(void);

void SYS_Init(void)
{
    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable HIRC48 clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRC48MEN_Msk);

    /* Waiting for HIRC48 clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);

    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_HIRC48M, CLK_CLKDIV0_USB(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();
    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Enable UART1 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable GPB module clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins */
    Uart0DefaultMPF();
		

}

void PowerDown()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    printf("Enter power down ...\n");
    while(!IsDebugFifoEmpty());

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WKEN_Msk);

    CLK_PowerDown();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if(CLK->PWRCTL & CLK_PWRCTL_PDEN_Msk)
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;

    printf("device wakeup!\n");

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);
    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_USBEN_Msk | SYS_USBPHY_SBO_Msk;

    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    SYS->GPA_MFP3 &= ~(SYS_GPA_MFP3_PA12MFP_Msk | SYS_GPA_MFP3_PA13MFP_Msk | SYS_GPA_MFP3_PA14MFP_Msk | SYS_GPA_MFP3_PA15MFP_Msk);
    SYS->GPA_MFP3 |= (SYS_GPA_MFP3_PA12MFP_USB_VBUS | SYS_GPA_MFP3_PA13MFP_USB_D_N | SYS_GPA_MFP3_PA14MFP_USB_D_P | SYS_GPA_MFP3_PA15MFP_USB_OTG_ID);

    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("|          NuMicro USB HID Keyboard Sample Code          |\n");
    printf("+--------------------------------------------------------+\n");
    printf("If PB.15 = 0, just report it is key 'a'.\n");

    /* Set PB.2 as Quasi-bidirectional mode */
    GPIO_SetMode(PB, BIT2, GPIO_MODE_QUASI);

    PB2 = 1;
		    /* Set PB.2 as Quasi-bidirectional mode */
    GPIO_SetMode(PB, BIT3, GPIO_MODE_QUASI);

    PB3 = 1;
#if CFG_TUSB_OS  == OPT_OS_NONE
  // 1ms tick timer
   SysTick_Config(48000000 / 1000);
#endif


#if CRYSTAL_LESS
    /* Backup default trim */
    u32TrimInit = M32(TRIM_INIT);
#endif

    // init device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);
		    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

    while(1)
    {
#if CRYSTAL_LESS
        /* Start USB trim if it is not enabled. */
        if((SYS->HIRCTCTL & SYS_HIRCTCTL_FREQSEL_Msk) != 1)
        {
            /* Start USB trim only when SOF */
            if(USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

                /* Re-enable crystal-less */
                SYS->HIRCTCTL = 0x01;
                SYS->HIRCTCTL |= SYS_HIRCTCTL_REFCKSEL_Msk | SYS_HIRCTCTL_BOUNDEN_Msk | (8 << SYS_HIRCTCTL_BOUNDARY_Pos);
            }
        }

        /* Disable USB Trim when error */
        if(SYS->HIRCTISTS & (SYS_HIRCTISTS_CLKERRIF_Msk | SYS_HIRCTISTS_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable crystal-less */
            SYS->HIRCTCTL = 0;

            /* Clear error flags */
            SYS->HIRCTISTS = SYS_HIRCTISTS_CLKERRIF_Msk | SYS_HIRCTISTS_TFAILIF_Msk;

            /* Clear SOF */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
        }
#endif
    tud_task(); // tinyusb device task

    hid_task();
    }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

void hid_task(void)
{
  // Poll every 10ms
  const uint32_t interval_ms = 10;
  static uint32_t start_ms = 0;

  if ( board_millis() - start_ms < interval_ms) return; // not enough time
  start_ms += interval_ms;

  //uint32_t const btn = board_button_read();

  // Remote wakeup
  if ( tud_suspended() && (PB15==0) )
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    tud_remote_wakeup();
  }

  /*------------- Keyboard -------------*/
  if ( tud_hid_n_ready(ITF_KEYBOARD) )
  {
    // use to avoid send multiple consecutive zero report for keyboard
    static bool has_key = false;

    if ( PB2==0 )
    {
      uint8_t keycode[6] = { 0 };
      keycode[0] = HID_KEY_A;

      tud_hid_n_keyboard_report(ITF_KEYBOARD, 0, 0, keycode);

      has_key = true;
    }else
    {
      // send empty key report if previously has key pressed
      if (has_key) tud_hid_n_keyboard_report(ITF_KEYBOARD, 0, 0, NULL);
      has_key = false;
    }
  }

  /*------------- Mouse -------------*/
  if ( tud_hid_n_ready(ITF_MOUSE) )
  {
    if ( PB3==0  )
    {
      int8_t const delta = 5;

      // no button, right + down, no scroll pan
      tud_hid_n_mouse_report(ITF_MOUSE, 0, 0x00, delta, delta, 0, 0);
    }
  }

}


// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) itf;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  // TODO set LED based on CAPLOCK, NUMLOCK etc...
  (void) itf;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) bufsize;
}



/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

