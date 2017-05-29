/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB(c) Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - pic24-dspic-pic32mm : v1.25
        Device            :  dsPIC33EP256GP502
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC16 1.26
        MPLAB             :  MPLAB X 3.45
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#include "mcc_generated_files/mcc.h"
#include <stdint.h>
#define FCY 70000000
#include <libpic30.h>
/*
                         Main application
 */

//#define N   4
#define N   64
#define WREG_X_SPACE    8   //W8
#define WREG_Y_SPACE    10  //W10
__eds__ uint16_t __attribute__((aligned(256),space(ymemory),eds)) h[N];
__eds__ uint16_t __attribute__((aligned(256),space(xmemory),eds)) x[N];
__eds__ uint16_t *W8_SAVE;
__eds__ uint16_t *W10_SAVE;


void InitCoefs(void)
{
h[0] = 0x0001;
h[1] = 0x0011;
h[2] = 0x0010;
h[3] = 0xfffb;
h[4] = 0xffe7;
h[5] = 0xffec;
h[6] = 0xfffe;
h[7] = 0x0001;
h[8] = 0xfff9;
h[9] = 0xfffa;
h[10] = 0x0000;
h[11] = 0xffff;
h[12] = 0x0007;
h[13] = 0x000e;
h[14] = 0xffec;
h[15] = 0xffc9;
h[16] = 0xffee;
h[17] = 0xffc5;
h[18] = 0xfe77;
h[19] = 0xfd6b;
h[20] = 0xffc4;
h[21] = 0x04c6;
h[22] = 0x05c7;
h[23] = 0xfeaf;
h[24] = 0xf655;
h[25] = 0xf6df;
h[26] = 0xfe1a;
h[27] = 0xff72;
h[28] = 0xf88b;
h[29] = 0xf85f;
h[30] = 0x0928;
h[31] = 0x1ccb;
h[32] = 0x1ccb;
h[33] = 0x0928;
h[34] = 0xf85f;
h[35] = 0xf88b;
h[36] = 0xff72;
h[37] = 0xfe1a;
h[38] = 0xf6df;
h[39] = 0xf655;
h[40] = 0xfeaf;
h[41] = 0x05c7;
h[42] = 0x04c6;
h[43] = 0xffc4;
h[44] = 0xfd6b;
h[45] = 0xfe77;
h[46] = 0xffc5;
h[47] = 0xffee;
h[48] = 0xffc9;
h[49] = 0xffec;
h[50] = 0x000e;
h[51] = 0x0007;
h[52] = 0xffff;
h[53] = 0x0000;
h[54] = 0xfffa;
h[55] = 0xfff9;
h[56] = 0x0001;
h[57] = 0xfffe;
h[58] = 0xffec;
h[59] = 0xffe7;
h[60] = 0xfffb;
h[61] = 0x0010;
h[62] = 0x0011;
h[63] = 0x0001;
}

int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    
    
    InitCoefs();
    
    XMODSRT = (__eds__ uint16_t*)x;
    XMODEND = (__eds__ uint16_t*)(x + N - 1);
    MODCONbits.XWM = WREG_X_SPACE;
    
    YMODSRT = (__eds__ uint16_t*)h;
    YMODEND = (__eds__ uint16_t*)(&h[N] - 1);
    MODCONbits.YWM = WREG_Y_SPACE;
    asm("NOP");
    asm("MOV #%0, W8"::"i"(x));
    asm("MOV #%0, W10"::"i"(h));    
    W8_SAVE = x;
    W10_SAVE = h;

    
    // When using interrupts, you need to set the Global Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalEnable();

    AD1CON1bits.SAMP = 1;

    
    LDAC_SetLow();
    SYNC_SetLow();
    
    __delay_ms(1);
        
    while ( 1 )
    {
        ClrWdt();
    }
    return -1;
}

//CODIGO SACADO:

/*    MODCONbits.XMODEN = 1;
    MODCONbits.YMODEN = 1;
    asm("NOP");
    asm("MOV #%0, W8"::"i"(x));
    asm("MOV #%0, W10"::"i"(h));

    //asm("MOV 0x0300, W0");
    //asm("MOV W0, [W10]");
    asm("CLR A, [W8], W4, [W10]-=2, W5");
    asm("REPEAT #%0"::"i"(N));
    asm("MAC W4*W5, A, [W8]+=2, W4, [W10]-=2, W5");
    asm("SAC A, #0, W4");
*/    
//    asm("MAC W4*W5, A, [W8]+=2, W4, [W10]-=2, W5");
//    func1();
/*    volatile register int result asm("A");
    volatile register int B asm("B");
    int *xmemory;
    int *ymemory;
    int xVal, yVal;
    
    result = __builtin_mac( result      , xVal  , yVal  ,
                            &xmemory    , &xVal , 2     ,
                            &ymemory    , &yVal , 2     ,
                            0,B);
*/    
   // asm volatile ("MOV #%0,w14"::"i"(15));
//    asm ("MAC ");
/*

        LDAC_SetLow();
        SYNC_SetLow();
        
        __delay_us(2);
        IFS0bits.SPI1IF = 0;
        SPI1BUF = DAC_CONF;

        __delay_us(2);
        SPI1BUF = DAC_VALUE & 0x00FF;

        __delay_us(2);
        IFS0bits.SPI1IF = 0;
        SPI1BUF = (DAC_VALUE>>8) & 0x00FF;


//        while( SPI1STATbits.SPITBF == 1 );
        __delay_us(1);
        SYNC_SetHigh();
*/        


/**
 End of File
 */