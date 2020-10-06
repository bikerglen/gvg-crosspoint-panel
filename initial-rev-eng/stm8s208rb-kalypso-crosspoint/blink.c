// 17 SEG B  1 o o  2 SEG G 23
// 21 SEG E  3 o o  4 DIG 1  4
// 25 DIG 2  5 o o  6 DIG 3  3
//  1 DIG 4  7 o o  8 DIG 5 26
//  2 DIG 6  9 o o 10 DIG 7 27
// 15 DP    11 o o 12 SEG D 18
// 20 SEG C 13 o o 14 SEG A 16
// 22 SEG F 15 o o 16 N/C    -
//
//  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
//  B  G  E  1  2  3  4  5  6  7 DP  D  C  A  F

#include <stdint.h>
#include <stdio.h>

#include "numbers.c"
#include "letters.c"
#include "font.c"

#define TEST_WORKING_STUFF

#define GPIO_PIN_0 0x01
#define GPIO_PIN_1 0x02
#define GPIO_PIN_2 0x04
#define GPIO_PIN_3 0x08
#define GPIO_PIN_4 0x10
#define GPIO_PIN_5 0x20
#define GPIO_PIN_6 0x40
#define GPIO_PIN_7 0x80

#define CLK_DIVR	(*(volatile uint8_t *)0x50c6)
#define CLK_PCKENR1	(*(volatile uint8_t *)0x50c7)

#define TIM1_CR1    (*(volatile uint8_t *)0x5250)
#define TIM1_IER    (*(volatile uint8_t *)0x5254)
#define TIM1_SR1    (*(volatile uint8_t *)0x5255)
#define TIM1_CNTRH  (*(volatile uint8_t *)0x525e)
#define TIM1_CNTRL  (*(volatile uint8_t *)0x525f)
#define TIM1_PSCRH  (*(volatile uint8_t *)0x5260)
#define TIM1_PSCRL  (*(volatile uint8_t *)0x5261)
#define TIM1_ARRH   (*(volatile uint8_t *)0x5262)
#define TIM1_ARRL   (*(volatile uint8_t *)0x5263)

#define PA_ODR (*(volatile uint8_t *)0x5000)
#define PA_IDR (*(volatile uint8_t *)0x5001)
#define PA_DDR (*(volatile uint8_t *)0x5002)
#define PA_CR1 (*(volatile uint8_t *)0x5003)
#define PA_CR2 (*(volatile uint8_t *)0x5004)

#define PB_ODR (*(volatile uint8_t *)0x5005)
#define PB_IDR (*(volatile uint8_t *)0x5006)
#define PB_DDR (*(volatile uint8_t *)0x5007)
#define PB_CR1 (*(volatile uint8_t *)0x5008)
#define PB_CR2 (*(volatile uint8_t *)0x5009)

#define PC_ODR (*(volatile uint8_t *)0x500a)
#define PC_IDR (*(volatile uint8_t *)0x500b)
#define PC_DDR (*(volatile uint8_t *)0x500c)
#define PC_CR1 (*(volatile uint8_t *)0x500d)
#define PC_CR2 (*(volatile uint8_t *)0x500e)

#define PD_ODR (*(volatile uint8_t *)0x500f)
#define PD_IDR (*(volatile uint8_t *)0x5010)
#define PD_DDR (*(volatile uint8_t *)0x5011)
#define PD_CR1 (*(volatile uint8_t *)0x5012)
#define PD_CR2 (*(volatile uint8_t *)0x5013)

#define PE_ODR (*(volatile uint8_t *)0x5014)
#define PE_IDR (*(volatile uint8_t *)0x5015)
#define PE_DDR (*(volatile uint8_t *)0x5016)
#define PE_CR1 (*(volatile uint8_t *)0x5017)
#define PE_CR2 (*(volatile uint8_t *)0x5018)

#define PG_ODR (*(volatile uint8_t *)0x501e)
#define PG_IDR (*(volatile uint8_t *)0x501f)
#define PG_DDR (*(volatile uint8_t *)0x5020)
#define PG_CR1 (*(volatile uint8_t *)0x5021)
#define PG_CR2 (*(volatile uint8_t *)0x5022)

#define USART1_SR	(*(volatile uint8_t *)0x5230)
#define USART1_DR	(*(volatile uint8_t *)0x5231)
#define USART1_BRR1	(*(volatile uint8_t *)0x5232)
#define USART1_BRR2	(*(volatile uint8_t *)0x5233)
#define USART1_CR2	(*(volatile uint8_t *)0x5235)
#define USART1_CR3	(*(volatile uint8_t *)0x5236)

#define USART_CR2_TEN (1 << 3)
#define USART_CR3_STOP2 (1 << 5)
#define USART_CR3_STOP1 (1 << 4)
#define USART_SR_TXE (1 << 7)

int putchar (int c);
uint16_t lfsr (void);
void SendPacket (uint8_t addr, uint8_t length, uint8_t *data);
void Write (uint8_t addr, uint8_t data);
uint8_t Read (uint8_t addr);
void SetTallyLight (uint8_t tally);
void ClearTallyLight (uint8_t tally);
void SetBacklights (uint8_t redtop, uint8_t redbot, uint8_t grntop, uint8_t grnbot);

void SetDisplay (uint8_t window, uint8_t *image);
void FastWrite (uint8_t addr, uint8_t data);
void DisplayText (uint8_t window, uint8_t row, char *text);

volatile uint8_t flag50 = 0;
uint16_t lfsr_state = 0x0001;

#define TXSOP   0x80
#define TXEOP   0xfe

#define DISP_CMD  0x20
#define DISP_DATA 0x28

static const uint8_t image[128] = {
	0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00,

	0x00, 0x00, 0x06, 0x07, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x83, 0xFF, 0xFF, 0xFE, 0x00, 0x00,
	0x00, 0xF0, 0xFC, 0xFE, 0x8F, 0x87, 0xC3, 0xC1, 0xC1, 0xC1, 0xC1, 0x83, 0x00, 0x00, 0x00, 0x00,

	0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0x78, 0x3C, 0x1E, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x3F, 0xFF, 0xFF, 0xC1, 0x80, 0x00, 0x00, 0x00, 0x80, 0xC3, 0xFF, 0xFF, 0x3E, 0x00, 0x00,

	0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00
};


void main (void)
{
	uint8_t led_timer = 0;
	uint8_t tally = 63;
	uint8_t buffer[64];

	// set clock to internal 16 MHz oscillator
	CLK_DIVR = 0x00;

	// enable peripheral clocks
	CLK_PCKENR1 = 0xff;

	// set UART1_TX on PA5 as a push-pull output
	PA_DDR |= GPIO_PIN_5;
	PA_CR1 |= GPIO_PIN_5;

	// configure USART1
	USART1_CR2 = USART_CR2_TEN; // Allow TX and RX
	USART1_CR3 &= ~(USART_CR3_STOP1 | USART_CR3_STOP2); // 1 stop bit
	USART1_BRR2 = 0x03; USART1_BRR1 = 0x68; // 9600 baud

	// configure port b as push-pull outputs
	PB_ODR = 0xFF; // all high
    PB_DDR = 0xFF; // all outputs
	PB_CR1 = 0xFF; // all totem pole
	PB_CR2 = 0x00; // all normal speed

	// configure port c as push-pull outputs, bit 5 is user led
	// PC1 is /RD, PC2 is /WR
	PC_ODR = 0xDF; // all high except led
    PC_DDR = 0xFF; // all outputs
	PC_CR1 = 0xFF; // all totem pole
	PC_CR2 = 0x00; // all normal speed

	// configure port d as push-pull outputs
	PD_ODR = 0xFF; // all high
	PD_DDR = 0xFF; // all outputs
	PD_CR1 = 0xFF; // all totem pole
	PD_CR2 = 0x00; // all normal speed

    // configure timer prescaler for divide by 16000 operation for a 1 kHz clock
    TIM1_PSCRH = 0x3e;
    TIM1_PSCRL = 0x7f;

    // configure auto reload register to 13 to create an update event at 50 Hz
    TIM1_ARRH = 0x00;
    TIM1_ARRL = 0x13;

    // enable timer
    TIM1_CR1 = 0x01;

    // enable timer update event interrupt
    TIM1_IER = 0x01;

    // enable global interrupts
    __asm
        rim
    __endasm;

	// need to so some sort of write at beginning to clear fault light, boot pic
	PD_ODR &= ~0x80;
	__asm
		nop
		nop
		nop
		nop
		nop
	__endasm;
	Write (0x00, 0x00);
	__asm
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PD_ODR |= 0x80;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;

	// enable, clear, and turn on lcd backlights
/*
    SetBacklights (0x00, 0x00, 0x00, 0x00);
	buffer[0]  = 0xa4;
	buffer[1]  = 0xfe;
	SendPacket (0x00, 0x02, buffer);
*/

	// set contrast to something reasonable
	buffer[0] = 0xA4;
	buffer[1] = 0x02;
	buffer[2] = 0x83;
	buffer[3] = 0x21;
	buffer[4] = 0xA4;
	buffer[5] = 0x02;
	buffer[6] = 0xC3;
	buffer[7] = 0x21;
	buffer[8] = 0xFE;
	SendPacket (0x00, 0x09, buffer);

/*
	// enable other stuff possibly (?)
	buffer[0]  = 0xa2;
	buffer[1]  = 0xfe;
	SendPacket (0x00, 0x02, buffer);

	// enable other stuff possibly (?)
	buffer[0]  = 0xa3;
	buffer[1]  = 0xfe;
	SendPacket (0x00, 0x02, buffer);

	// enable other stuff possibly (?)
	buffer[0]  = 0xa5;
	buffer[1]  = 0xfe;
	SendPacket (0x00, 0x02, buffer);
*/

/*
	// no idea
	buffer[0] = 0xE5;
	buffer[1] = 0x03;
	buffer[2] = 0x13;
	buffer[3] = 0x50;
	buffer[4] = 0x01;
	buffer[5] = 0xA4;
	buffer[6] = 0x02;
	buffer[7] = 0x82;
	buffer[8] = 0x20;
	buffer[9] = 0xA4;
	buffer[10] = 0x02;
	buffer[11] = 0xC2;
	buffer[12] = 0x20;
	buffer[13] = 0xFE;
	SendPacket (0x00, 0x0E, buffer);
*/

	// more strange init
	buffer[ 0] = 0x81;
	buffer[ 1] = 0x00;
	buffer[ 2] = 0xFE;
	SendPacket (0x00, 0x03, buffer);

	buffer[ 0] = 0xA7;
	buffer[ 1] = 0x00;
	buffer[ 2] = 0xA5;
	buffer[ 3] = 0x00;
	buffer[ 4] = 0xA4;
	buffer[ 5] = 0x02;
	buffer[ 6] = 0x02;
	buffer[ 7] = 0xA2;
	buffer[ 8] = 0xA4;
	buffer[ 9] = 0x02;

	buffer[10] = 0x42;
	buffer[11] = 0xA2;
	buffer[12] = 0xA4;
	buffer[13] = 0x02;
	buffer[14] = 0x02;
	buffer[15] = 0xA2;
	buffer[16] = 0xA4;
	buffer[17] = 0x02;
	buffer[18] = 0x42;
	buffer[19] = 0xA2;

	buffer[20] = 0xA4;
	buffer[21] = 0x02;
	buffer[22] = 0x02;
	buffer[23] = 0xA2;
	buffer[24] = 0xA4;
	buffer[25] = 0x02;
	buffer[26] = 0x42;
	buffer[27] = 0xA2;
	buffer[28] = 0xA4;
	buffer[29] = 0x02;

	buffer[30] = 0x02;
	buffer[31] = 0xA2;
	buffer[32] = 0xA4;
	buffer[33] = 0x02;
	buffer[34] = 0x42;
	buffer[35] = 0xA2;
	buffer[36] = 0xA4;
	buffer[37] = 0x02;
	buffer[38] = 0x02;
	buffer[39] = 0xA2;

	buffer[40] = 0xA4;
	buffer[41] = 0x02;
	buffer[42] = 0x42;
	buffer[43] = 0xA2;
	buffer[44] = 0xFE;

	SendPacket (0x00, 0x2D, buffer);

	buffer[ 0] = 0xA4; // something in this packet gets rid of the vertical rolling effect
	buffer[ 1] = 0x02;
	buffer[ 2] = 0x02;
	buffer[ 3] = 0xA2;
	buffer[ 4] = 0xA4;
	buffer[ 5] = 0x02;
	buffer[ 6] = 0x42;
	buffer[ 7] = 0xA2;
	buffer[ 8] = 0xA4;
	buffer[ 9] = 0x02;

	buffer[10] = 0x02;
	buffer[11] = 0xA2;
	buffer[12] = 0xA4;
	buffer[13] = 0x02;
	buffer[14] = 0x42;
	buffer[15] = 0xA2;
	buffer[16] = 0xA4;
	buffer[17] = 0x02;
	buffer[18] = 0x02;
	buffer[19] = 0xA2;

	buffer[20] = 0xA4;
	buffer[21] = 0x02;
	buffer[22] = 0x42;
	buffer[23] = 0xA2;
	buffer[24] = 0xA4;
	buffer[25] = 0x02;
	buffer[26] = 0x02;
	buffer[27] = 0xA2;
	buffer[28] = 0xA4;
	buffer[29] = 0x02;
	buffer[30] = 0x42;
	buffer[31] = 0xA2;

	buffer[32] = 0xA6;
	buffer[33] = 0x02;
	buffer[34] = 0x13;
	buffer[35] = 0x00; // was 1 -- a '1' will "preset" a tally light to dim

	buffer[36] = 0xA6;
	buffer[37] = 0x02;
	buffer[38] = 0x1B;
	buffer[39] = 0x00; // was 1 -- a '1' will "preset" a tally light to dim

	buffer[40] = 0xFE;

	SendPacket (0x00, 0x29, buffer);

	buffer[ 0] = 0xE5; // this packet sets the contrast
	buffer[ 1] = 0x03;
	buffer[ 2] = 0x13;
	buffer[ 3] = 0x50;
	buffer[ 4] = 0x01;
	buffer[ 5] = 0xA4;
	buffer[ 6] = 0x02;
	buffer[ 7] = 0x82;
	buffer[ 8] = 0x20;
	buffer[ 9] = 0xA4;
	buffer[10] = 0x02;
	buffer[11] = 0xC2;
	buffer[12] = 0x20;
	buffer[13] = 0xFE;
	SendPacket (0x00, 0x0E, buffer);


	// create pulse on LCD-11 to reset display (?)
	Write (0x18, 0x03);
	Write (0x20, 0xc0);
	Write (0x20, 0x00);
	Write (0x20, 0xc0);

	// set backlighting to amber
    SetBacklights (0xff, 0xff, 0xff, 0xff);

	// initialize lcd display controllers

	// select display 0
	Write (0x18, 0x00);

	// write command registers
	Write (DISP_CMD, 0xAE);			// display off
	Write (DISP_CMD, 0x40);			// page address
	Write (DISP_CMD, 0xA1);			// scan direction = reversed
	Write (DISP_CMD, 0xA6);			// inverse display = normal
	Write (DISP_CMD, 0xA4);			// all pixel on = normal
	Write (DISP_CMD, 0xA3);			// bias = 1/7
	Write (DISP_CMD, 0xC8);			// com direction = reversed
	Write (DISP_CMD, 0x2D);			// power control = VB, VF
	Write (DISP_CMD, 0xAC);
	Write (DISP_CMD, 0xAF);			// display on

	// select display 1
	Write (0x18, 0x01);

	// write command registers
	Write (DISP_CMD, 0xAE);
	Write (DISP_CMD, 0x40);
	Write (DISP_CMD, 0xA1);
	Write (DISP_CMD, 0xA6);
	Write (DISP_CMD, 0xA4);
	Write (DISP_CMD, 0xA2);
	Write (DISP_CMD, 0x28);
	Write (DISP_CMD, 0xAC);
	Write (DISP_CMD, 0xAF);

	// select display 2
	Write (0x18, 0x02);

	// write command registers
	Write (DISP_CMD, 0xAE);
	Write (DISP_CMD, 0x40);
	Write (DISP_CMD, 0xA1);
	Write (DISP_CMD, 0xA6);
	Write (DISP_CMD, 0xA4);
	Write (DISP_CMD, 0xA2);
	Write (DISP_CMD, 0x28);
	Write (DISP_CMD, 0xAC);
	Write (DISP_CMD, 0xAF);

	SetDisplay (0, SPACE);
	SetDisplay (1, SPACE);
	SetDisplay (2, SPACE);
	SetDisplay (3, SPACE);
	SetDisplay (4, SPACE);
	SetDisplay (5, SPACE);
	SetDisplay (6, SPACE);
	SetDisplay (7, SPACE);

	// main loop
	for (;;) {
		if (flag50) {
			flag50 = 0;

			if (led_timer == 0) {
				// turn off user 1 led
				PC_ODR &= ~GPIO_PIN_5;
			} else if (led_timer == 10) {
				// turn on user 1 led
				PC_ODR |= GPIO_PIN_5;
			}
			led_timer++;
			if (led_timer == 25) {
				led_timer = 0;
			}

			// advance tally lights
			if ((led_timer == 0) || (led_timer == 12)) {
				ClearTallyLight (tally);
				tally = (tally + 1) & 0x1F;
				SetTallyLight (tally);
			}

			if ((led_timer == 12) && ((tally & 0x1f) == 0)) {
				SetDisplay (0, A);
				SetDisplay (1, W);
				SetDisplay (2, E);
				SetDisplay (3, S);
				SetDisplay (4, O);
				SetDisplay (5, M);
				SetDisplay (6, E);
				SetDisplay (7, SPACE);
			} else if ((led_timer == 12) && ((tally & 0x1f) == 8)) {
				SetDisplay (0, SPACE);
				SetDisplay (1, SPACE);
				SetDisplay (2, S);
				SetDisplay (3, A);
				SetDisplay (4, U);
				SetDisplay (5, C);
				SetDisplay (6, E);
				SetDisplay (7, EXCLAM);

				DisplayText (0, 0, "ABCDE");
				DisplayText (0, 1, "12345");
				DisplayText (0, 2, "54321");
				DisplayText (0, 3, "EDCBA");
			} else if ((led_timer == 12) && ((tally & 0x1f) == 16)) {
				SetDisplay (0, EXCLAM);
				SetDisplay (1, D);
				SetDisplay (2, A);
				SetDisplay (3, N);
				SetDisplay (4, G);
				SetDisplay (5, E);
				SetDisplay (6, R);
				SetDisplay (7, EXCLAM);
			} else if ((led_timer == 12) && ((tally & 0x1f) == 24)) {
				SetDisplay (0, Z);
				SetDisplay (1, O);
				SetDisplay (2, M);
				SetDisplay (3, B);
				SetDisplay (4, I);
				SetDisplay (5, E);
				SetDisplay (6, S);
				SetDisplay (7, EXCLAM);
			}

			// alternate red and green backlights in different columns
			// if (led_timer == 0) {
				// SetBacklights (0xaa, 0x55, 0x55, 0xaa);
				// SetBacklights (0xff, 0xff, 0x00, 0x00);
			// } else if (led_timer == 12) {
				// SetBacklights (0xcc, 0x33, 0x33, 0xcc);
				// SetBacklights (0x00, 0x00, 0xff, 0xff);
			// }
			
/*
			// read PIC status
			for (i = 0; i < 9; i++) {
				a = Read (0x00);
				printf ("%02x ", a);
			}
			printf ("\n\r");
*/

/*
			// create pulse on LCD-5
			Write (0x18, 0x00);
			Write (0x20, 0xc0);

			// create pulse on LCD-7
			Write (0x18, 0x01);
			Write (0x20, 0xc0);	// any write to this address but 0xc0 keeps rogue pulse off LCD-11

			// create pulse on LCD-9
			Write (0x18, 0x02);
			Write (0x20, 0xc0);	// any write to this address but 0xc0 keeps rogue pulse off LCD-11

			// create pulse on LCD-11
			Write (0x18, 0x03);
			Write (0x20, 0xc0);
			Write (0x20, 0x00);
			Write (0x20, 0xc0);
*/
		}
	}
}

void SendPacket (uint8_t addr, uint8_t length, uint8_t *data)
{
	int i;
	uint8_t cksum = 0;

	printf ("Send: ");
	Write (addr, TXSOP); cksum += TXSOP;
	Write (addr, length); cksum += length;
	for (i = 0; i < length; i++) {
		Write (addr, data[i]); cksum += data[i];
	}
	Write (addr, cksum);
	printf ("\n\r");
}

void SetBacklights (uint8_t redtop, uint8_t redbot, uint8_t grntop, uint8_t grnbot)
{
    int i;
    uint8_t data;

	Write (0x20, 0xc0); // put d7,d6 back in a safe state before switching to backlight
	Write (0x18, 0x03); // <=== this appears critical for backlights to change (0x03)

    for (i = 0; i < 8; i++) {
        data = 0;
        data |= (redtop & 0x80) ? 8 : 0;
        data |= (redbot & 0x80) ? 4 : 0;
        data |= (grntop & 0x80) ? 2 : 0;
        data |= (grnbot & 0x80) ? 1 : 0;
        Write (0x20, 0xc0 | data);
        Write (0x20, 0x40 | data);
        Write (0x20, 0xc0 | data);
        redtop <<= 1;
        redbot <<= 1;
        grntop <<= 1;
        grnbot <<= 1;
    }
	// create high pulse on A6275 latch
    Write (0x20, 0xc0);
    Write (0x20, 0x80);
    Write (0x20, 0xc0);
}

void SetTallyLight (uint8_t tally)
{
	uint8_t buffer[5];

	buffer[0]  = 0xa6;
	buffer[1]  = 0x00;
	buffer[2]  = tally;
	buffer[3]  = 0x07;
	buffer[4]  = 0xfe;
	SendPacket (0x00, 0x05, buffer);
}

void ClearTallyLight (uint8_t tally)
{
	uint8_t buffer[5];

	buffer[0]  = 0xa6;
	buffer[1]  = 0x00;
	buffer[2]  = tally;
	buffer[3]  = 0x00;
	buffer[4]  = 0xfe;
	SendPacket (0x00, 0x05, buffer);
}

void TIM1_overflow_Handler() __interrupt(11)
{
    // clear interrupt
    TIM1_SR1 &= ~1;

	// set flag
	flag50 = 1;

    // toggle user 1 led on PC1
    // PC_ODR ^= GPIO_PIN_1;
}


int putchar (int c)
{
	while (!(USART1_SR & USART_SR_TXE));
	USART1_DR = c;
	return (c);
}


uint16_t lfsr (void)
{
    uint16_t bit;

    bit = ((lfsr_state >> 0) ^ (lfsr_state >> 2) ^ (lfsr_state >> 3) ^ (lfsr_state >> 5)) /* & 1u */;
    lfsr_state = (lfsr_state >> 1) | (bit << 15);

    return lfsr_state;
}


void Write (uint8_t addr, uint8_t data)
{
	printf (".");
	// printf ("%02x ", data);

	PB_DDR = 0xff;
	PD_ODR = (PD_ODR & 0x80) | ((addr & 0x3f) << 1) | 0x01;
	PB_ODR = data;
	PC_ODR &= ~GPIO_PIN_2; // set write low
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PD_ODR &= ~GPIO_PIN_0; // set cs low
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PD_ODR |= GPIO_PIN_0; // set cs high
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PC_ODR |= GPIO_PIN_2; // set write high
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
}

uint8_t Read (uint8_t addr)
{
	uint8_t data;

	PB_DDR = 0x00;
	PD_ODR = (addr << 1) | 0x81;		// set activity (FPGA 7) and CS (FPGA 0) high
	PD_ODR &= ~GPIO_PIN_0; // set cs low
	__asm
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PC_ODR &= ~GPIO_PIN_1; // set read low
	__asm
		nop
		nop
		nop
		nop
		nop
	__endasm;
	data = PB_IDR;
	__asm
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PC_ODR |=  GPIO_PIN_1; // set read high
	__asm
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PD_ODR |= GPIO_PIN_0; // set cs high
	__asm
		nop
		nop
		nop
		nop
		nop
	__endasm;

	return data;
}

static const uint8_t lcd_cmds[8][13] = {
	{ 0, 0xb0, 0x11, 0x00, 0xb1, 0x11, 0x00, 0xb2, 0x11, 0x00, 0xb3, 0x11, 0x00 },
	{ 0, 0xb0, 0x13, 0x00, 0xb1, 0x13, 0x00, 0xb2, 0x13, 0x00, 0xb3, 0x13, 0x00 },
	{ 0, 0xb0, 0x15, 0x00, 0xb1, 0x15, 0x00, 0xb2, 0x15, 0x00, 0xb3, 0x15, 0x00 },
	{ 1, 0xb0, 0x10, 0x05, 0xb1, 0x10, 0x05, 0xb2, 0x10, 0x05, 0xb3, 0x10, 0x05 },
	{ 1, 0xb0, 0x12, 0x05, 0xb1, 0x12, 0x05, 0xb2, 0x12, 0x05, 0xb3, 0x12, 0x05 },
	{ 1, 0xb0, 0x14, 0x05, 0xb1, 0x14, 0x05, 0xb2, 0x14, 0x05, 0xb3, 0x14, 0x05 },
	{ 2, 0xb0, 0x10, 0x05, 0xb1, 0x10, 0x05, 0xb2, 0x10, 0x05, 0xb3, 0x10, 0x05 },
	{ 2, 0xb0, 0x12, 0x05, 0xb1, 0x12, 0x05, 0xb2, 0x12, 0x05, 0xb3, 0x12, 0x05 }
};


void DisplayText (uint8_t window, uint8_t row, char *text)
{
	uint8_t i, j;

	// pick window
	FastWrite (0x18, lcd_cmds[window][0]);

	// pick row
	FastWrite (DISP_CMD, lcd_cmds[window][3*row+1]);
	FastWrite (DISP_CMD, lcd_cmds[window][3*row+2]);
	FastWrite (DISP_CMD, lcd_cmds[window][3*row+3]);

	// write text data
	FastWrite (DISP_DATA, 0x00);
	for (i = 0; i < 5; i++) {
		for (j = 0; j < 5; j++) {
			FastWrite (DISP_DATA, font[text[i]-32][j]); 
		}
		FastWrite (DISP_DATA, 0x00);
	}
	FastWrite (DISP_DATA, 0x00);
}

void SetDisplay (uint8_t window, uint8_t *image)
{
	uint8_t i, j;

	FastWrite (0x18, lcd_cmds[window][0]);

	for (i = 0; i < 4; i++) {
		FastWrite (DISP_CMD, lcd_cmds[window][3*i+1]);
		FastWrite (DISP_CMD, lcd_cmds[window][3*i+2]);
		FastWrite (DISP_CMD, lcd_cmds[window][3*i+3]);
		for (j = 0; j < 32; j++) {
			FastWrite (DISP_DATA, *image++);
		}
	}
}


void FastWrite (uint8_t addr, uint8_t data)
{
	PB_DDR = 0xff;
	PD_ODR = (PD_ODR & 0x80) | ((addr & 0x3f) << 1) | 0x01;
	PB_ODR = data;
	PC_ODR &= ~GPIO_PIN_2; // set write low
	__asm
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PD_ODR &= ~GPIO_PIN_0; // set cs low
	__asm
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PD_ODR |= GPIO_PIN_0; // set cs high
	__asm
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PC_ODR |= GPIO_PIN_2; // set write high
	__asm
		nop
		nop
		nop
		nop
		nop
	__endasm;
}
