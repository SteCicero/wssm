/* ###*B*###
 * ERIKA Enterprise - a tiny RTOS for small microcontrollers
 *
 * Copyright (C) 2002-2008  Evidence Srl
 *
 * This file is part of ERIKA Enterprise.
 *
 * ERIKA Enterprise is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation,
 * (with a special exception described below).
 *
 * Linking this code statically or dynamically with other modules is
 * making a combined work based on this code.  Thus, the terms and
 * conditions of the GNU General Public License cover the whole
 * combination.
 *
 * As a special exception, the copyright holders of this library give you
 * permission to link this code with independent modules to produce an
 * executable, regardless of the license terms of these independent
 * modules, and to copy and distribute the resulting executable under
 * terms of your choice, provided that you also meet, for each linked
 * independent module, the terms and conditions of the license of that
 * module.  An independent module is a module which is not derived from
 * or based on this library.  If you modify this code, you may extend
 * this exception to your version of the code, but you are not
 * obligated to do so.  If you do not wish to do so, delete this
 * exception statement from your version.
 *
 * ERIKA Enterprise is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License version 2 for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 2 along with ERIKA Enterprise; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301 USA.
 * ###*E*### */

#include <stdio.h>

#include "ee.h"
#include "ee_irq.h"

#include "console_serial.h"

#include "ConfigApp.h"
//#include "Common\Console.h"
#include "WirelessProtocols/P2P/P2P.h"
#include "Transceivers/Transceivers.h"
#include "Common/SymbolTime.h"
#include "Transceivers/Security.h"
#include "WirelessProtocols/MCHP_API.h"

_CONFIG1(JTAGEN_OFF & FWDTEN_OFF)
_CONFIG2(FNOSC_PRIPLL & POSCMOD_XT)



#ifndef MY_FIRST_SERIAL
#define MY_FIRST_SERIAL 0
#endif

#ifndef MY_FIRST_CONSOLE
#define MY_FIRST_CONSOLE 0
#endif

#define FIRST_MINIFLEX   0x01
#define SECOND_MINIFLEX  0x02

#define CONSOLE_OUT(msg)     \
	console_write(MY_FIRST_CONSOLE, (const EE_UINT8*) msg, strlen(msg))


#if ADDITIONAL_NODE_ID_SIZE > 0
    BYTE AdditionalNodeID[ADDITIONAL_NODE_ID_SIZE] = {FIRST_MINIFLEX};
#endif

console_descriptor_t *my_console_1;
EE_UINT16 TxNum, RxNum;
EE_UINT8 myChannel = 25;

EE_UINT8 ledb=0;
EE_UINT8 ledc=0;
EE_UINT8 time_out=50;
EE_UINT8 counter=0;


extern void PrintDec(EE_UINT8 to_print);
extern void PrintChar(EE_UINT8 to_print);

extern void mrf24j40_isr(void);


/* Program the Timer1 peripheral to raise interrupts */
void T1_program(void)
{
	T1CON = 0;		/* Stops the Timer1 and reset control reg	*/
	TMR1  = 0;		/* Clear contents of the timer register	*/
	PR1   = 0x3E80;		/* Load the Period register with the value 0x3E80
							to have a tick of 1 ms */
	IPC0bits.T1IP = 5;	/* Set Timer1 priority to 1		*/
	IFS0bits.T1IF = 0;	/* Clear the Timer1 interrupt status flag	*/
	IEC0bits.T1IE = 1;	/* Enable Timer1 interrupts		*/
	T1CONbits.TON = 1;	/* Start Timer1 with prescaler settings at 1:1
				* and clock source set to the internal
				* instruction cycle			*/
}

/* Clear the Timer1 interrupt status flag */
void T1_clear(void)
{
	IFS0bits.T1IF = 0;
}

/*
 * radio_isr is the callback function executed when
 * the radio module issues an interrupt.
 */
void radio_isr(void)
{
	ActivateTask(TaskInt);
}

int board_init(void)
{
	__builtin_write_OSCCONL(OSCCON & 0xbf);

	/* Assign UART1 input pins */
	RPINR18bits.U1RXR = 25; //U1RX on RP25
	/* Assign UART1 output pins */
	RPOR12bits.RP24R = 3; //U1TX on RP24

	/* Lock pin configuration registers */

	//__builtin_write_OSCCONL(OSCCON | 0x40);


	EE_miniflex_radio_init(&radio_isr);

	EE_leds_init();

	//EE_analog_init();
	//EE_battery_monitor_init();
	//EE_temperature_init();
	//EE_accelerometer_init();
	//EE_light_init();


	return 0;
}


/* This is an ISR Type 2 which is attached to the Timer 1 peripheral IRQ pin
 * The ISR simply calls CounterTick to implement the timing reference
 */
ISR2(_T1Interrupt)
{
	/* clear the interrupt source */
	T1_clear();

	/* count the interrupts, waking up expired alarms */
	CounterTick(myCounter);
}




TASK(TaskMiWiOP)
{
	if(  MiApp_MessageAvailable() )
	{
		ActivateTask(TaskRec);
		counter=0;
	}
	else
	{
		if(counter==time_out)
		{
			if(PORTC!=0)
			{
				LATC=PORTC>>1;
			}
			else
			{
				LATB=PORTB>>1;
			}
		}
		if(counter<time_out)
		{
			counter++;
		}
	}
}

TASK(TaskInt)
{
	mrf24j40_isr();
}

TASK(TaskRec)
{
	ledc=0;
	ledb=0;
	int i=0;
	_Bool approx=0;
	for(i=0;i<(rxMessage.PacketRSSI/25);i++)
	{
		if(ledb==255)
		{
			ledc=(ledc*2)+1;
		}
		else
		{
			ledb=(ledb*2)+1;
		}
		if((rxMessage.PacketRSSI%25)>12 && !approx)
		{
			i--;
			approx=1;
		}
	}
	LATB=(((EE_UINT16)ledb) << 8) | (PORTB&0x00FF);
	LATC=(((EE_UINT16)ledc) << 8) | (PORTC&0x00FF);

    MiApp_DiscardMessage();
}

TASK (TaskSend)
{
	char str[100];
	EE_UINT8 i;

	sprintf(str, "\r\nHello from Miniflex signal strenght indicator\r\nDeveloped by Stefano Cicero");

	MiApp_FlushTx();

	i = 0;
	while (str[i] != '\0')
			MiApp_WriteData(str[i++]);


	if (!MiApp_BroadcastPacket(FALSE))
	{
		GetResource(S0);
		//CONSOLE_OUT("\r\nTX ERROR");
		ReleaseResource(S0);
	}
	++TxNum;
}

int main(void)
{
	EE_UINT8 i = 0;

	/* Wait for PLL to lock */
	while(OSCCONbits.LOCK!=1);

	/* Program Timer 1 to raise interrupts */
	T1_program();

	/* Application Init */

	board_init();

	TRISBbits.TRISB8=0;
	TRISBbits.TRISB9=0;
	TRISBbits.TRISB10=0;
	TRISBbits.TRISB11=0;
	TRISBbits.TRISB12=0;
	TRISBbits.TRISB13=0;
	TRISBbits.TRISB14=0;
	TRISBbits.TRISB15=0;
	TRISCbits.TRISC8=0;
	TRISCbits.TRISC9=0;

	LATBbits.LATB8=0;
	LATBbits.LATB9=0;
	LATBbits.LATB10=0;
	LATBbits.LATB11=0;
	LATBbits.LATB12=0;
	LATBbits.LATB13=0;
	LATBbits.LATB14=0;
	LATBbits.LATB15=0;
	LATCbits.LATC8=0;
	LATCbits.LATC9=0;

	EE_led_sys_off();

    /*********************************************************************/
    /* Which protocol to use depends on the configuration in ConfigApp.h */
    /*********************************************************************/

    MiApp_ProtocolInit();

    if( MiApp_SetChannel(myChannel) == FALSE )
    {
        return 0;
    }

    /*******************************************************************/
    // Function MiApp_ConnectionMode defines the connection mode. The
    // possible connection modes are:
    //  ENABLE_ALL_CONN:    Enable all kinds of connection
    //  ENABLE_PREV_CONN:   Only allow connection already exists in
    //                      connection table
    //  ENABL_ACTIVE_SCAN_RSP:  Allow response to Active scan
    //  DISABLE_ALL_CONN:   Disable all connections.
    /*******************************************************************/
    MiApp_ConnectionMode(ENABLE_ALL_CONN);


    /*******************************************************************/
    // Function MiApp_EstablishConnection try to establish a new connection
    // with peer device.
    // The first parameter is the index to the active scan result, which is
    //      acquired by discovery process (active scan). If the value of the
    //      index is 0xFF, try to establish a connection with any peer.
    // The second parameter is the mode to establish connection, either direct
    //      or indirect. Direct mode means connection within the radio range;
    //      Indirect mode means connection may or may not in the radio range.
    /*******************************************************************/
    #ifdef ENABLE_HAND_SHAKE
        while( (i = MiApp_EstablishConnection(0xFF, CONN_MODE_DIRECT)) == 0xFF );
    #endif

    EE_led_sys_on();

	SetRelAlarm(AlarmProt, 50, 50);
	SetRelAlarm(AlarmSend, 500, 500);

	/* Forever loop: background activities (if any) should go here */
	for (;;);

	return 0;
}
