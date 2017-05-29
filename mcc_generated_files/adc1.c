
/**
  ADC1 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    adc1.c

  @Summary
    This is the generated header file for the ADC1 driver using MPLAB(c) Code Configurator

  @Description
    This header file provides APIs for driver for ADC1.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - pic24-dspic-pic32mm : v1.25
        Device            :  dsPIC33EP256GP502
        Driver Version    :  0.5
    The generated drivers are tested against the following:
        Compiler          :  XC16 1.26
        MPLAB 	          :  MPLAB X 3.45
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

/**
  Section: Included Files
*/

#include <xc.h>
#include "adc1.h"
#include "pin_manager.h"
/**
  Section: Data Type Definitions
*/

/* ADC Driver Hardware Instance Object

  @Summary
    Defines the object required for the maintenance of the hardware instance.

  @Description
    This defines the object required for the maintenance of the hardware
    instance. This object exists once per hardware instance of the peripheral.

 */
typedef struct
{
	uint8_t intSample;
}

ADC_OBJECT;

static ADC_OBJECT adc1_obj;

/**
  Section: Driver Interface
*/


void ADC1_Initialize (void)
{
    // ASAM enabled; ADDMABM disabled; ADSIDL disabled; DONE disabled; SIMSAM Sequential; FORM Fractional result, signed, left-justified; SAMP disabled; SSRC Internal counter ends sampling and starts conversion; AD12B 12-bit; ADON enabled; SSRCG disabled; 

   AD1CON1 = 0x87E4;

    // CSCNA disabled; VCFG0 AVDD; VCFG1 AVSS; ALTS disabled; BUFM disabled; SMPI 1; CHPS 1 Channel; 

   AD1CON2 = 0x0;

    // SAMC 31; ADRC FOSC/2; ADCS 19; 

   AD1CON3 = 0x1F13;

    // CH0SA AN0; CH0SB AN0; CH0NB AVSS; CH0NA AVSS; 

   AD1CHS0 = 0x0;

    // CSS25 disabled; CSS24 disabled; CSS31 disabled; CSS30 disabled; 

   AD1CSSH = 0x0;

    // CSS2 disabled; CSS1 disabled; CSS0 disabled; CSS5 disabled; CSS4 disabled; CSS3 disabled; 

   AD1CSSL = 0x0;

    // CH123SA disabled; CH123SB CH1=OA2/AN0,CH2=AN1,CH3=AN2; CH123NA disabled; CH123NB CH1=VREF-,CH2=VREF-,CH3=VREF-; 

   AD1CHS123 = 0x0;


   adc1_obj.intSample = AD1CON2bits.SMPI;
   
   // Enabling ADC1 interrupt.
   IEC0bits.AD1IE = 1;
}

#define N 64
uint16_t DAC_VALUE;
extern uint16_t x[N];
extern uint16_t h[N];
extern uint16_t *W8_SAVE;
extern uint16_t *W10_SAVE;
void __attribute__ ( ( __interrupt__ , auto_psv ) ) _AD1Interrupt ( void )
{
    LDAC_Toggle();  //just for debug
    
    SYNC_SetHigh();
    AD1CON1bits.SAMP = 1;
    
    //Digital Filter code goes here!!
    MODCONbits.XMODEN = 1;
    MODCONbits.YMODEN = 1;
    asm("NOP");
    asm("MOV %0, W8"::"r"(W8_SAVE));
    asm("MOV #%0, W10"::"i"(h));    //    asm("MOV %0, W10"::"r"(W10_SAVE));
    asm("MOV %0, W0"::"r"(ADC1BUF0));
//    asm("MOV 0x0300, W0");
    asm("MOV W0, [W8]");
    asm("CLR A, [W8]-=2, W4, [W10]+=2, W5");
    asm("REPEAT #%0"::"i"(N-1));
    asm("MAC W4*W5, A, [W8]-=2, W4, [W10]+=2, W5");
//    asm("MAC W4*W5, A, [W8]+=2, W4, [W10]+=2, W5");   //esta instruccion va con N-2
//    asm("SAC A, #0, W13");
    asm("MOVSAC B, [W8]+=4, W4, W13");  //Guarda ACC_A -> W13 y configura W8 para la proxima muestra que se adquiera
    asm("MOV  W8, %0":"=r"(W8_SAVE));   //salva puntero donde guardar proxima muestra del ADC
//    asm("MOV W10, %0":"=r"(W10_SAVE));//no hace falta guardar puntero de h[n] ya que no cambia


    //Convierte Q15 en Unsigned para el DAC
    asm("MOV #0x8000,W0");
    asm("XOR W0,W13,W13");
    
    asm("MOV W13,%0":"=r"(DAC_VALUE));
    MODCONbits.XMODEN = 0;
    MODCONbits.YMODEN = 0;
    asm("NOP");
    //End of digital dilter code

    //Send output sample to DAC
    SYNC_SetLow();
    while( SPI1STATbits.SPITBF == 1 );
    SPI1BUF = 0b00010000;
    while( SPI1STATbits.SPITBF == 1 );
    SPI1BUF = (DAC_VALUE>>8) & 0x00FF;
    while( SPI1STATbits.SPITBF == 1 );
    SPI1BUF = DAC_VALUE & 0x00FF;
    while( SPI1STATbits.SPITBF == 1 );

    
    // clear the ADC interrupt flag
    IFS0bits.AD1IF = false;
}



/**
  End of File
*/
