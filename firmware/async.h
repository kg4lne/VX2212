/****************************************************************************
 *
 * Micro-RC Repeater Controller
 *
 * Copyright (C) 2005-2006, Robert E. Starr, Jr. (KG4LNE)
 *
 * USART Support Functions
 *
 ***************************************************************************/

/* Useful ASCII Codes */

#define SOH     0x01
#define STX     0x02
#define ENQ     0x05
#define ACK     0x06
#define NAK     0x15
#define SYN     0x16
#define ESC     0x1B

/*
 * Asynchronous USART Structure
 */

#define BAUD_RATE           9600UL

#define ASYNC_TX_BUFSIZE    16
#define ASYNC_RX_BUFSIZE    32

typedef struct _ASYNC {
    struct  _rx {
        uint8_t buf[ASYNC_RX_BUFSIZE];
        size_t  head;       /* rx queue head index */
        size_t  tail;       /* rx queue tail index */
        size_t  count;      /* rx queue size       */
    } rx;
    struct _tx {
        uint8_t buf[ASYNC_TX_BUFSIZE];
        size_t  head;       /* tx queue head index */
        size_t  tail;       /* tx queue tail index */
        size_t  count;      /* tx queue size       */
    } tx;
    struct _stat {
        uint8_t cntl;       /* interrupt cntl bits */
        uint8_t flags;      /* async status bits   */
        uint8_t line;       /* line status bits    */
        uint8_t modem;      /* modem status bits   */
    } stat;
} ASYNC;

/* interrupt status flags for 'stat.cntl' */
#define C_TXING             0x01        /* tx ISR in progress       */
#define C_RXING             0x02        /* rx ISR in progress       */

/* async buffer status flags for 'stat.flags */
#define A_RX_UNDERFLOW      0x01        /* receive buffer empty     */
#define A_RX_OVERFLOW       0x02        /* receive buffer overrun   */
#define A_TX_UNDERFLOW      0x04        /* transmit buffer empty    */
#define A_TX_OVERFLOW       0x08        /* transmit buffer overrun  */

/* USART0 line status flags for 'stat.line */
#define L_FRAMING_ERROR     (1<<FE)     /* framing error occured    */
#if defined(__AVR_ATmega8__)
#define L_PARITY_ERROR      (1<<PE)     /* parity error occured     */
#else
#define L_PARITY_ERROR      (1<<UPE)    /* parity error occured     */
#endif
#define L_DATA_OVERRUN      (1<<DOR)    /* overrun error occured    */

/* modem status bits and macros */
#define M_DELTA_CTS         0x01        /* change in clear to send  */
#define M_DELTA_DSR         0x02        /* change in data set ready */
#define M_RING_TRAIL        0x04        /* trailing edge ring ind.  */
#define M_DELTA_CD          0x08        /* change in carrier detect */
#define M_CTS               0x10        /* modem clear to send      */
#define M_DSR               0x20        /* modem data set ready     */
#define M_RING              0x40        /* modem ring indicator     */
#define M_CD                0x80        /* modem carrier detect     */

/*
 * Function Prototypes
 */

void asi_init(void);

int asi_getc(void);
int asi_read(char *buf, size_t buflen);
int asi_write(char *buf, size_t buflen);
int asi_tgetc(int timeout);
int asi_tgets(int timeout, char buf[]);

int asi_putc(char c);
int asi_puts(char* p);

/* End-Of-File */
