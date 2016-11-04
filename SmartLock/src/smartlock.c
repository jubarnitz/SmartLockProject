/***********************************************************************
* FILENAME : SmartLock.c
* AUTHOR: Justin Barnitz, Kevin Loeffler
* DATE: NOV 2016
* DESCRIPTION: LPC 1115 xpresso board. Recieves bluetooth commands to
* control servo motor, to lock/unlock door. 
*
* PIN LAYOUT: 
* 	Pin 2.0 => PWM for servo
*   Pin 2.1 => Red LED
*   Pin 2.2 => Green LED
***********************************************************************/
#ifdef __USE_CMSIS
#include "LPC11xx.h"
#endif
#include "driver_config.h"
#include "target_config.h"
#include "timer32.h"
#include "gpio.h"
#include "uart.h"
#include <cr_section_macros.h>
#include <stdio.h>



volatile uint32_t duty = 49;
volatile uint32_t counter = 0;
volatile uint32_t period = 300;		// Signal Period is 3ms (333Hz)
volatile uint32_t isLocked = 0;
volatile uint32_t isUART = 0;
volatile uint32_t turned = 0;

extern volatile uint32_t UARTCount;
extern volatile uint8_t UARTBuffer[BUFSIZE];

void TIMERInit()
{
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<9); //enable CT32B0 32-bit counter/timer 0 in System AHB clock control register

    LPC_IOCON->PIO1_5 &= ~0x07;	/*  Timer0_32 I/O config */
    LPC_IOCON->PIO1_5 |= 0x02;	/* Timer0_32 CAP0 */

  	// Clear the EMR
  	LPC_TMR32B0->EMR &= ~(0xFF<<4);
  	// Set External Match Control 0,1,2,3 to Toggle the corresponding External Match bit/output
  	LPC_TMR32B0->EMR |= ((0x3<<4)|(0x3<<6)|(0x3<<8)|(0x3<<10));

  	/*
  	The Match register values are continuously compared to the Timer Counter value. When
	the two values are equal, actions can be triggered automatically. The action possibilities
	are to generate an interrupt, reset the Timer Counter, or stop the timer. Actions are
	controlled by the settings in the MCR register.
	*/
  	//LPC_TMR32B0->MR0 = SystemCoreClock/1000-1; //ticks every 1 ms
  	LPC_TMR32B0->MR0 = SystemCoreClock/100000-1; //ticks every 1/100 ms

  	LPC_TMR32B0->TCR = 0x01; // start the timer(s)

  	
  	/* Bit 0 -> Interrupt on MR0: an interrupt is generated when MR0 matches the value in the TC */
  	/* Bit 1 -> Reset on MR0: the TC will be reset if MR0 matches it. */
  	
  	LPC_TMR32B0->MCR = 0x03;

  	NVIC_EnableIRQ(TIMER_32_0_IRQn);
  	NVIC_SetPriority(TIMER_32_0_IRQn, 0);

  	return;

}

void TIMER32_0_IRQHandler(void)
{
	counter++;

	if( counter == 1 )
	{
		// start off a new period, turn on signal
		GPIOSetValue(2, 0, 1);
	}
	else if( counter == (( duty * period ) / 100 ))
	{
		// end of duty cycle turn off signal
		GPIOSetValue(2, 0, 0);
	}
	else if( counter >= period )
	{
		counter = 0;
		turned++;
	}

	// clear the register
	LPC_TMR32B0->IR = (0x1<<0);

	return;
}

void Lock()
{
	//if(!isLocked)
	//{
			enable_timer32(0);
			turned = 0;
			duty = 25;
			counter = 0;
			while(turned < 100){}
			duty = 49;
			disable_timer32(0);
			GPIOSetValue(2, 2, 0); // turn off grn LED
			GPIOSetValue(2, 1, 1); // turn on red LED
			isLocked = 1;
	//}
}

void Unlock()
{
	//if(isLocked)
	//{
		enable_timer32(0);
		turned = 0;
		duty = 70;
		counter = 0;
		while(turned < 100){} //old 75 = about 90 deg
		duty = 49;
		disable_timer32(0);
		GPIOSetValue(2, 1, 0); // turn off red LED
		GPIOSetValue(2, 2, 1); // turn on grn LED
		isLocked = 0;
	//}
}

int main(void)
{
 	TIMERInit();
	GPIOInit();
	UARTInit(115200);
	GPIOSetDir(2, 0, 1); // output Pin 2 port 0 for servo control
	GPIOSetDir(2, 1, 1); // output pin 2.1 red LED for locked
	GPIOSetDir(2, 2, 1); // output pin 2.2 green LED for unlock
	disable_timer32(0);

	while(1)
	{
		if (UARTCount != 0)
		{
			LPC_UART->IER = IER_THRE | IER_RLS;			/* Disable RBR */
			UARTSend( (uint8_t *)UARTBuffer, UARTCount );
			if(UARTBuffer[0] == 'c')
			{
				Lock();
			}
			else if(UARTBuffer[0] == 'o')
			{
				Unlock();
			}
			UARTCount = 0;
			LPC_UART->IER = IER_THRE | IER_RLS | IER_RBR;	/* Re-enable RBR */
		}
	}
}
