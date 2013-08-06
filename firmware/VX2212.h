/****************************************************************************
 *
 * VX2212 Virtaul X2212 EPROM Emulator
 *
 * Copyright (C) 2006, Robert E Starr (KG4LNE)
 *
 * This code was developed for the VX2212 virtual EPROM emulator.
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

#if !defined(__AVR_ATmega8__)
#error "must build for ATMega8 - check processor type define"
#endif

/* X2212 Memory Size */

#define MAX_BANKS       2       

#define BANKSIZE        256     /* 256 x 4 bits */

/****************************************************************************
 * Macro's
 ***************************************************************************/

/* NOP instruction macro */
#define nop()                   __asm__ __volatile__ ("nop" ::)

/* Save processor flags and disable interrupts */
#define ENTER_CRITICAL()        __asm__ __volatile__( \
                                    "in __tmp_reg__, __SREG__"  "\n\t" \
                                    "cli"                       "\n\t" \
                                    "push __tmp_reg__"          "\n\t" \
                                    :: )

/* Restore saved processor flags and reenables the interrupts */
#define EXIT_CRITICAL()         __asm__ __volatile__( \
                                    "pop __tmp_reg__"           "\n\t" \
                                    "out __SREG__, __tmp_reg__" "\n\t" \
                                    :: )

/****************************************************************************
 * VX2212 Hardware I/O Definitions (refer to schematic)
 ***************************************************************************/

#define B_A0        PCO
#define B_A1        PC1
#define B_A2        PC2
#define B_A3        PC3
#define B_A4        PC4
#define B_A5        PC5
#define B_WE        ADC6
#define B_STORE     ADC7

#define B_RXD       PDO
#define B_TXD       PD1
#define B_CS        PD2
#define B_RECALL    PD3
#define B_D4        PD4
#define B_D3        PD5
#define B_D2        PD6
#define B_D1        PD7

#define B_A6        PBO
#define B_A7        PB1
#define B_RRST      PB2
#define B_RTS       PB3
#define B_LED       PB4
#define B_MODE      PB5

/****************************************************************************
 * Serial Port Definitions
 ***************************************************************************/

/////////////////////////////////////////////////////////////////////////////
// VX2212 RS-232 Serial Interface Structures and Contants
//                                                    
//                    +------------------------------+   byte
//                +-- |             SOH              |    0
//                |   +------------------------------+
//    Preamble ---+   |       Data Length (msb)      |    1
//      Part      |   +------------------------------+
//                +-- |       Data Length (lsb)      |    2
//                    +---+---+---+---+--------------+
//                +-- | M | 0 | 0 | 0 |   Command    |    3
//      Header ---+   +---+---+---+---+--------------+
//       Part     +-- |         Bank Number          |    4
//                    +------------------------------+
//                +-- |              .               |    n
//                |   |              .               |    .
//        Bank ---+   |             Data             |    .
//        Data    |   |              .               |    .
//                +-- |              .               |    .
//                    +------------------------------+
//                    |          CRC (msb)           |  datalen+5
//                    +------------------------------+
//                    |          CRC (lsb)           |  datalen+6
//                    +------------------------------+
// 
//  Frame Data Description:
// 
//      * SOH:          Start of frame header identifier.
// 
//      * Data length:  Length of 'Data' in bytes (must be 0, 256 or 512)
// 
//      * Flags:        M=MORE, 0, 0, 0
//
//      * Command:      0=Ping, 1=WriteBank, 2=ReadBank, etc...
// 
//      * Bank#:        Bank number (0=Bank-0, 1=Bank-1)
//       
//      * Data:         Banks data bytes (if data-length nonzero)
//       
//      * CRC value:    Xmodem CRC-16 (msb, lsb)
//

// VX2212 Packet Command Codes
#define VXCMD_PING              0x00        // ping cmd, echo back and ACK
#define VXCMD_WRITE_BANK        0x01        // write bank specified by bank# field 
#define VXCMD_READ_BANK         0x02        // read bank specified by bank# field
#define VXCMD_GET_ACTIVE_BANK   0x03        // get radio current active bank
#define VXCMD_SET_ACTIVE_BANK   0x04        // set radio current active bank

#define VXCMD_MASK(c)           (c & 0xFF)  // lower 4 bits are cmd bits

// command byte flag bits (upper four bits)
#define F_VXCMD_RES0            0x10        // reserved flag bit 0
#define F_VXCMD_RES1            0x20        // reserved flag bit 1
#define F_VXCMD_RES2            0x40        // reserver flag bit 2
#define F_VXCMD_MORE            0x80        // more bank data frames will follow

/****************************************************************************
 * Function Prototypes
 ***************************************************************************/

/*-- VX2212.c --*/

int main(void);

int rx_packet(void);
int tx_packet(uint8_t flgcmd, uint8_t bank, uint8_t* data, uint16_t datasize);

void blink_led(uint16_t count, uint16_t rate);
void recall_bank(uint16_t banknum);
void store_bank(uint16_t banknum);
void reset_radio(void);

void delay_ms(uint16_t ms);
void delay_us(uint16_t us);

void init_io(void);
void start_ints(void);

/* End-Of-File */
