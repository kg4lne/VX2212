/****************************************************************************
 *
 * VX2212 Virtual X2212 EPROM Emulator - Version 1.00
 *
 * Copyright (C) 2006, Robert E Starr (KG4LNE)
 * 
 * This code is designed for use with the Atmel Mega8 processor and
 * was developed using the GCC WinAVR development tools.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The name of the author may not be used to endorse or promote
 *     products derived from this software without specific prior written
 *     permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT,  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
//#include <avr/signal.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/crc16.h>

#include "VX2212.h"
#include "async.h"

/****************************************************************************
 * Global Data
 ***************************************************************************/

uint8_t  s_cs1 = 0;             /* CS1 chip select int flag */
uint8_t  s_cs2 = 0;             /* CS2 chip select int flag */
uint8_t  s_frame[BANKSIZE];     /* serial rx frame data buffer */

/* X2212 EPROM Image Data Buffer */
uint8_t  s_bank[BANKSIZE * MAX_BANKS];

/* This table is used to mirror the data bit order to correct
 * for a hardware mistake. The 4-bit data buss order is reversed
 * from the actual data order. So, we mirror the bits in our
 * lookup table prior to storing EPROM data for speed.
 */

const uint8_t s_xbit[16] = {
    0x00,   // 00 : 0000 => 0000 (0x00)
    0x08,   // 01 : 0001 => 1000 (0x08)
    0x04,   // 02 : 0010 => 0100 (0x04)
    0x0C,   // 03 : 0011 => 1100 (0x0C)
    0x02,   // 04 : 0100 => 0010 (0x02)
    0x0A,   // 05 : 0101 => 1010 (0x0A)
    0x06,   // 06 : 0110 => 0110 (0x06)
    0x0E,   // 07 : 0111 => 1110 (0x0E)
    0x01,   // 08 : 1000 => 0001 (0x01)
    0x09,   // 09 : 1001 => 1001 (0x09)
    0x05,   // 10 : 1010 => 0101 (0x05)
    0x0D,   // 11 : 1011 => 1101 (0x0D)
    0x03,   // 12 : 1100 => 0011 (0x03)
    0x0B,   // 13 : 1101 => 1011 (0x0B)
    0x07,   // 14 : 1110 => 0111 (0x07)
    0x0F    // 15 : 1111 => 1111 (0x0F)
};

/* Do not remove! */
const char copyright[] PROGMEM = {
    "VX2212 v1.00, Copyright (C) 2007-2009, KG4LNE"
};


/****************************************************************************
 * MAIN - Program entry at startup.
 ***************************************************************************/

int main(void)
{
    int      c;
    int      stat;
    uint8_t  mode;
    uint32_t count = 0;
    register uint8_t addr;
    register uint16_t bankoff;

    /* Setup default I/O pins and assignments */
    init_io();

    /* Initialize the USART serial port driver */
    asi_init();

    asi_putc('<');

    /* Blink the LED 1 or 2 times at power up to 
     * indicate the MODE jumper JP2 select state.
     */

    mode = (PINB & _BV(B_MODE)) ? 0 : 1;
    
    blink_led(mode+1, 150);

    /* Read EPROM image into RAM */
    eeprom_read_block(s_bank, 0, BANKSIZE * MAX_BANKS);

    /* Force a radio CPU reset */
    reset_radio();

    /* Final device init, startup interrupts */
    start_ints();

    /*
     * Enter the main X2212 EPROM emulation loop.
     */

    bankoff = (mode) ? BANKSIZE : 0;

    for (;;)
    {
        /* TEST FOR /CS ACTIVE (low) */

        //if (s_cs1)
        if ((PIND & _BV(B_CS)) == 0)
        {
            /* EPROM READ /CS ACTIVE STATE */

            PORTB |= _BV(B_LED);

            /* read the address data lines */
            addr = (PINC & 0x3F) + (PINB << 6);

            /* output the data to the buss */
            PORTD = s_bank[addr+bankoff] << 4;
            //PORTD = s_bank[addr] << 4;
        }
        else
        {
            /* LED blink state counter */
            if (++count == 250000)
            {
                count = 0;
                PORTB ^= _BV(B_LED);
            }

            /* RS-232 SERIAL PORT PROCESSING */

            /* Any character received? */
            if ((c = asi_getc()) != '.')
                continue;

            /* LED off while we read from serial */
            PORTB &= ~(_BV(B_LED));

            /* Got a period, echo back STX to begin packet rx */
            asi_putc(STX);

            /* Attempt to read a data/command packet */

            do {

                /* attempt to read a packet */
                stat = rx_packet(); 

            } while (stat == 0);
        }
    }

    return 0;
}

/* Attempt to receive a serial command packet from the host (GE-Flash).
 * If we get a valid packet then decode the command and process it.
 */

int rx_packet(void)
{
    int         c;
    int         stat = -1;
    uint8_t     flgcmd;
    uint8_t     banknum;
    uint16_t    lsb;
    uint16_t    msb;
    uint16_t    len;
    uint16_t    i;
    uint16_t    crc;
    uint16_t    crc_packet;

    crc = 0;

    /* Read the SOH first (wait up to 5 seconds) */
    if ((c = asi_tgetc(500)) == -1)
        return 1;
    if (c != SOH)
        return 2;

    // Read 'len_msb' byte
    if ((c = asi_tgetc(100)) == -1)
        return 3;
    crc = _crc_xmodem_update(crc, c);
    msb = (uint16_t)c;

    // Read 'len_lsb' byte
    if ((c = asi_tgetc(100)) == -1)
        return 4;
    crc = _crc_xmodem_update(crc, c);
    lsb = (uint16_t)c;
    len = (msb << 8) | lsb;

    // Read 'flgs/cmd' byte
    if ((c = asi_tgetc(100)) == -1)
        return 5;
    crc = _crc_xmodem_update(crc, c);
    flgcmd = (uint16_t)c;

    // Read 'bank#' byte state
    if ((c = asi_tgetc(100)) == -1)
        return 6;
    crc = _crc_xmodem_update(crc, c);
    banknum = (uint16_t)c & 0x1;

    // frame length validation
    if (len > 256)
        return 7;

    // Read any data for length specified
    for (i=0; i < len; i++)
    {
        // Read a byte of data from the stream
        if ((c = asi_tgetc(100)) == -1)
            return 8;
        // Update CRC with the data byte
        crc = _crc_xmodem_update(crc, c);
        // Store it in the rx frame buffer
        s_frame[i] = c;
    }

    /* final CRC calculation */
    crc = _crc_xmodem_update(_crc_xmodem_update(crc,0),0);

    // Read CRC MSB byte
    if ((msb = asi_tgetc(100)) == -1)
        return 9;

    if ((lsb = asi_tgetc(100)) == -1)
        return 10;

    // Get the packet CRC value
    crc_packet = (msb << 8) | lsb;

    // Check if the CRC's match
    if (crc != crc_packet)
    {
        asi_putc(NAK);
        asi_putc(msb);
        asi_putc(lsb);
        // We return 0 to indicate continue reading
        // another packet after a NAK. 
        return 0;
    }

    /* VALID FRAME RECEIVED - DECODE THE COMMAND */

    stat = 0;

    switch(flgcmd & 0x0F)
    {
        case VXCMD_PING:
            break;

        case VXCMD_WRITE_BANK:
            /* Patch downloaded data to correct for mirrored data lines */
            for (i=0; i < BANKSIZE; i++)
                s_frame[i] = s_xbit[s_frame[i] & 0x0f];

            /* Copy from temp rx buffer to active emulation buffer */
            memcpy(&s_bank[banknum * BANKSIZE], s_frame, BANKSIZE);

            /* Test to see if this is the last packet or MORE are to follow */
            if (flgcmd & F_VXCMD_MORE)
            {
                /* More packets will follow this one.. */
                /* Tell host we stored the bank */
                asi_putc(ACK);
                asi_putc(msb);
                asi_putc(lsb);
            }
            else
            {
                /* Save the new data to our EPROM */
                eeprom_write_block(s_bank, 0, BANKSIZE * MAX_BANKS);
                /* Tell host we stored the bank */
                asi_putc(ACK);
                asi_putc(msb);
                asi_putc(lsb);
                /* Blink the LED 2 times upon completion */
                blink_led(2, 75);
                /* Reset the radio cpu */
                reset_radio();
                /* exit read packet loop */
                stat = -1;
            }
            break;

        case VXCMD_READ_BANK:
            /* Copy from active emulation buffer to tx buffer */
            memcpy(s_frame, &s_bank[banknum * BANKSIZE], BANKSIZE);

            /* Un-mirror/copy data bits */
            for (i=0; i < BANKSIZE; i++)
                s_frame[i] = s_xbit[s_frame[i]];

            /* Send the packet */
            tx_packet(VXCMD_READ_BANK, banknum, s_frame, BANKSIZE);
            break;

        case VXCMD_GET_ACTIVE_BANK:
            break;

        case VXCMD_SET_ACTIVE_BANK:
            break;

        default:
            break;
    }

    return stat;
}


int tx_packet(uint8_t flgcmd, uint8_t bank, uint8_t* data, uint16_t datasize)
{
    int      i;
    uint16_t lsb;
    uint16_t msb;
    uint16_t crc = 0;

    /* Send the SOH first (0) */
    asi_putc(SOH);

    /* Send the length MSB/LSB bytes */
    msb = (uint16_t)datasize >> 4;;
    lsb = (uint16_t)datasize & 0xff;

    /* Send CRC MSB byte (1) */
    crc = _crc_xmodem_update(crc, msb);
    asi_putc(msb);

    /* Send CRC LSB byte (2) */
    crc = _crc_xmodem_update(crc, lsb);
    asi_putc(lsb);

    /* Send the FLG/CMD byte (3) */
    crc = _crc_xmodem_update(flgcmd, lsb);
    asi_putc(flgcmd);

    /* Send the BANK# byte (4) */
    crc = _crc_xmodem_update(flgcmd, bank);
    asi_putc(bank);

    /* Send any DATA payload */
    if (data && datasize)
    {
        for (i=0; i < datasize; i++)
        {
            crc = _crc_xmodem_update(crc, data[i]);
            asi_putc(data[i]);
        }
    }

    /* Send the CRC MSB */
    asi_putc((int)(crc >> 4) & 0xff);

    /* Send the CRC LSB */
    asi_putc((int)crc & 0xff);

    return 0;
}

/* Issue a reset pulse to the radio CPU */

void reset_radio(void)
{
    PORTB |= _BV(B_RRST);
    /* ~100us */
    _delay_loop_2(5*100);
    PORTB &= ~(_BV(B_RRST));
    /* ~100us */
    //_delay_loop_2(5*100);
}

/* Blink the LED for count and rate duration */

void blink_led(uint16_t count, uint16_t rate)
{
    uint16_t i;
    /* Blink the n times at ms delay rate */
    for (i=0; i < count; i++)
    {
        PORTB |= _BV(B_LED);
        delay_ms(rate);
        PORTB &= ~(_BV(B_LED));
        delay_ms(rate);
    }
}

/****************************************************************************
 * Initialize the VX2212 I/O ports
 ***************************************************************************/

void init_io(void)
{
    /* PORT-C
     *
     * PCO  = A0        (in)
     * PC1  = A1        (in)
     * PC2  = A2        (in)
     * PC3  = A3        (in)
     * PC4  = A4        (in)
     * PC5  = A5        (in)
     * ADC6 = /WE       (in)
     * ADC7 = /STORE    (in)
     */

    /* all pins low */
    PORTC = 0x00;
    /* all pins are input */
    DDRC  = 0x00;

    /* PORT-D
     *
     * PDO  = RXD       (in)
     * PD1  = TXD       (out)
     * PD2  = /CS       (in)
     * PD3  = /RECALL   (in)
     * PD4  = D4        (in/out)
     * PD5  = D3        (in/out)
     * PD6  = D2        (in/out)
     * PD7  = D1        (in/out)
     */

    /* all pins low */
    PORTD = 0x00;
    /* all pins are input for now */
    DDRD  = 0x00;
    /* PD1=output, PD4-PD7=output */
    DDRD  = _BV(PD1) | _BV(PD4) | _BV(PD5) | _BV(PD6) | _BV(PD7);

    /* PORT-B
     *
     * PBO  = A6        (in)
     * PB1  = A7        (in)
     * PB2  = RRST      (out)
     * PB3  = RTS       (out)
     * PB4  = LED       (out)
     * PB5  = MODE      (in)
     */

    /* all pins low */
    PORTB = 0x00;
    /* all pins are input for now */
    DDRB  = 0x00;
    /* PB2-PB4=output, PB0, PB1 & PB5 are input */
    DDRB  = _BV(PB2) | _BV(PB3) | _BV(PB4);
}

/* Here we startup any interrupts or devices needed */

void start_ints(void)
{
    cli();

#if 0
    uint16_t cmatch;
    cmatch = (uint16_t)((F_CPU / TICK_RATE_HZ) / 1024UL);
    /* no compare/pwm/capture mode */
    TCCR1A = 0;
    /* setup this timer's prescaler 1024 */
    TCCR1B = 128;   //1024;
    /* Setup the timer starting value */
    TCNT1L = (uint8_t)cmatch;
    TCNT1H = (uint8_t)(cmatch >> 8);
    /* enable the timer/counter1 overflow interrupt in the T/C int mask register */
    timer_enable_int(_BV(TOIE1));
#endif

	// Enable the external interrupt for INT0/PD2
    MCUCR   = (1<<ISC00);       // Set interrupt on fall and on rise
    GICR	= (1<<INT0);		// Enable interrupt for interrupt0

    sei();
}

/****************************************************************************
 * General Purpose Functions
 ***************************************************************************/

void delay_ms(uint16_t ms)
{
    uint16_t d;

    for (d=0; d < ms; d++) {
        /* 16-bit count - 4 cycles/loop */
        _delay_loop_2((uint16_t)(F_CPU / 4000UL));
    }
}

/* ~21 cycles per microsecond */
#define CYCLES_PER_US       ((F_CPU+500000UL)/1000000UL)

void delay_us(uint16_t us) 
{
    uint16_t dcnt;
    dcnt = (us+3) / 3 * CYCLES_PER_US;
    /* 16-bit count - 4 cycles/loop */
    _delay_loop_2(dcnt / 4);
}

/****************************************************************************
 * Change PORT direction based on condition of INT0 Pin (/CS).
 ***************************************************************************/

SIGNAL(SIG_INTERRUPT0)
{
    if (PIND & 0x04)        // Chip select line high (not enabled)
    {
        DDRD &= ~0xF0;      // Tri-state the outputs!
        s_cs1 = 0;        // chip select CS1/ inactive
    }
    else                    // Chip select line low (enabled)
    {
        DDRD |= 0xF0;       // Drive the outputs
        s_cs1 = 1;        // chip select CS1/ active
    }
}

#if 0
SIGNAL(SIG_INTERRUPT1)
{
    if (PIND & 0x04)        // Chip select line high (not enabled)
    {
        DDRD &= ~0xF0;      // Tri-state the outputs!
        s_cs2 = 0;        // chip select CS1/ inactive
    }
    else                    // Chip select line low (enabled)
    {
        DDRD |= 0xF0;       // Drive the outputs
        s_cs2 = 1;       // chip select CS1/ active
    }
}
#endif

/****************************************************************************
 *
 ***************************************************************************/

#define TICK_RATE_HZ    60UL

SIGNAL(SIG_OVERFLOW0)
{
    uint16_t cmatch;

    cmatch = (uint16_t)((F_CPU / TICK_RATE_HZ) / 1024UL);

    /* Setup the timer starting value */
    TCNT1L = (uint8_t)cmatch;
    TCNT1H = (uint8_t)(cmatch >> 8);

//    PORTB |= _BV(B_LED);
}

/*
 // controller ATmega8
> CLI(); //disable all interrupts
>
> // desired value: 1Sec
>
>  TCNT1  = 0xF0BE; //setup
> //start Timer (prescale:1024 input from internal clk)
> TCCR1B = 0x05;
>
> //timer interrupt sources (TOIE1: Timer/Counter0 Overflow Interrupt
> Enable)
> TIMSK  = 0x04;
>
> SEI(); //re-enable interrupts
>
> The interrupt:
>
> #pragma interrupt_handler timer1_ovf_isr:7
> void timer1_ovf_isr(void)
> {
>  WDR();
>  TCNT1  = 0xF0BE; //reload counter low value
> }
>
*/

/* End-Of-File */


