/*
 * ts7200.h - definitions describing the ts7200 peripheral registers
 *
 * Specific to the TS-7200 ARM evaluation board
 *
 */

#define	TIMER1_BASE	0x80810000
#define	TIMER2_BASE	0x80810020
#define	TIMER3_BASE	0x80810080

#define	LDR_OFFSET	0x00000000	// 16/32 bits, RW
#define	VAL_OFFSET	0x00000004	// 16/32 bits, RO
#define CRTL_OFFSET	0x00000008	// 3 bits, RW
	#define	ENABLE_MASK	0x00000080
	#define	MODE_MASK	0x00000040
	#define	CLKSEL_MASK	0x00000008
#define CLR_OFFSET	0x0000000c	// no data, WO


#define LED_ADDRESS	0x80840020
	#define LED_NONE	0x0
	#define LED_GREEN	0x1
	#define LED_RED		0x2
	#define LED_BOTH	0x3

#define COM1	0
#define COM2	1

#define IRDA_BASE	0x808b0000
#define UART1_BASE	0x808c0000
#define UART2_BASE	0x808d0000

// All the below registers for UART1
// First nine registers (up to Ox28) for UART 2

#define UART_DATA_OFFSET	0x0	// low 8 bits
	#define DATA_MASK	0xff
#define UART_RSR_OFFSET		0x4	// low 4 bits
	#define FE_MASK		0x1
	#define PE_MASK		0x2
	#define BE_MASK		0x4
	#define OE_MASK		0x8
#define UART_LCRH_OFFSET	0x8	// low 7 bits
	#define BRK_MASK	0x1
	#define PEN_MASK	0x2	// parity enable
	#define EPS_MASK	0x4	// even parity
	#define STP2_MASK	0x8	// 2 stop bits
	#define FEN_MASK	0x10	// fifo
	#define WLEN_MASK	0x60	// word length 8 bits
#define UART_LCRM_OFFSET	0xc	// low 8 bits
	#define BRDH_MASK	0xff	// MSB of baud rate divisor
#define UART_LCRL_OFFSET	0x10	// low 8 bits
	#define BRDL_MASK	0xff	// LSB of baud rate divisor
#define UART_CTLR_OFFSET	0x14	// low 8 bits
	#define UARTEN_MASK	0x1
	#define MSIEN_MASK	0x8	// modem status int
	#define RIEN_MASK	0x10	// receive int
	#define TIEN_MASK	0x20	// transmit int
	#define RTIEN_MASK	0x40	// receive timeout int
	#define LBEN_MASK	0x80	// loopback 
#define UART_FLAG_OFFSET	0x18	// low 8 bits
	#define CTS_MASK	0x1
	#define DCD_MASK	0x2
	#define DSR_MASK	0x4
	#define TXBUSY_MASK	0x8
	#define RXFE_MASK	0x10	// Receive buffer empty
	#define TXFF_MASK	0x20	// Transmit buffer full
	#define RXFF_MASK	0x40	// Receive buffer full
	#define TXFE_MASK	0x80	// Transmit buffer empty
#define UART_INTR_OFFSET	0x1c
  #define UART_MIS_MASK  0x1
  #define UART_RIS_MASK  0x2
  #define UART_TIS_MASK  0x4
  #define UART_RTIS_MASK 0x8
#define UART_DMAR_OFFSET	0x28

// Specific to UART1

#define UART_MDMCTL_OFFSET	0x100
#define UART_MDMSTS_OFFSET	0x104
#define UART_HDLCCTL_OFFSET	0x20c
#define UART_HDLCAMV_OFFSET	0x210
#define UART_HDLCAM_OFFSET	0x214
#define UART_HDLCRIB_OFFSET	0x218
#define UART_HDLCSTS_OFFSET	0x21c


/* HWI Registers */
// Source: http://www.cgl.uwaterloo.ca/~wmcowan/teaching/cs452/pdf/ep93xx-user-guide.pdf
#define TIMER3_BIT_ON           (1 << 19)
#define UART1_COMBINED_BIT_ON   (1 << 20)

#define VIC1_BASE 0x800B0000
#define VIC2_BASE 0x800C0000

#define VICX_IRQ_STATUS_OFFSET      0x0
#define VICX_FIQ_STATUS_OFFSET      0x4
#define VICX_RAW_INTR_OFFSET        0x8

#define VICX_INT_SELECT_OFFSET      0xC
  #define TIMER3_IRQ_MASK           ~TIMER3_BIT_ON
  #define TIMER3_FIQ_MASK           TIMER3_BIT_ON
  #define UART1_COMBINED_IRQ_MASK   ~UART1_COMBINED_BIT_ON
  #define UART1_COMBINED_FIQ_MASK   UART1_COMBINED_BIT_ON

#define VICX_INT_ENABLE_OFFSET      0x10
  #define TIMER3_INT_ON             TIMER3_BIT_ON
  #define UART1_COMBINED_INT_ON     UART1_COMBINED_BIT_ON

#define VICX_INT_ENCLEAR_OFFSET     0x14
  #define CLEAR_ALL                 0xFFFFFFFF 
  #define CLEAR_TIMER3              TIMER3_BIT_ON
  #define CLEAR_UART1_COMBINED      UART1_COMBINED_BIT_ON

