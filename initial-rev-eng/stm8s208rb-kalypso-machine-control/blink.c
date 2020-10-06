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
void SetBacklights (uint8_t redtop, uint8_t redbot, uint8_t grntop, uint8_t grnbot);

volatile uint8_t flag50 = 0;
uint16_t lfsr_state = 0x0001;

#define TXSOP   0x80
#define TXEOP   0xfe

void main (void)
{
	uint8_t length = 1;
	uint8_t j = 0;
	uint8_t i = 0;
	uint8_t addr = 0;
	uint8_t data = 0;
	uint8_t led_timer = 0;
	uint8_t buffer[32];
	uint8_t command_code = 0xa1;

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

	// need to so some sort of write at beginning to clear fault light
	PD_ODR &= ~0x80;
	Write (0x00, 0x00);
	PD_ODR |= 0x80;

	// write bikerglen to display
	buffer[0]  = 0xa1;
	buffer[1]  = 0x00;
	buffer[2]  = 'B';
	buffer[3]  = 'I';
	buffer[4]  = 'K';
	buffer[5]  = 'E';
	buffer[6]  = 'R';
	buffer[7]  = 'G';
	buffer[8]  = 'L';
	buffer[9]  = 'E';
	buffer[10] = 'N';
	buffer[11] = '!';
	buffer[12] = '!';
	buffer[13] = '!';
	buffer[14] = 0xfe;
	length = 15;
	SendPacket (0x00, length, buffer);

	// turn on tally light 0
	buffer[0]  = 0xa6;
	buffer[1]  = 0x00;
	buffer[2]  = 0x00; // 0x00, 0x0F, 0x10
	buffer[3]  = 0x01;
	buffer[4]  = 0xfe;
	SendPacket (0x00, 0x05, buffer);

	// turn on tally light 15
	buffer[0]  = 0xa6;
	buffer[1]  = 0x00;
	buffer[2]  = 0x0F; // 0x00, 0x0F, 0x10
	buffer[3]  = 0x01;
	buffer[4]  = 0xfe;
	SendPacket (0x00, 0x05, buffer);

	// turn on tally light 31
	buffer[0]  = 0xa6;
	buffer[1]  = 0x00;
	buffer[2]  = 0x10; // 0x00, 0x0F, 0x10
	buffer[3]  = 0x01;
	buffer[4]  = 0xfe;
	SendPacket (0x00, 0x05, buffer);

/*
	// turn on tally light X - REW
	buffer[0]  = 0xa6;
	buffer[1]  = 0x00;
	buffer[2]  = 0x0a; // 0x00, 0x0F, 0x10
	buffer[3]  = 0xff;
	buffer[4]  = 0xfe;
	SendPacket (0x00, 0x05, buffer);

	// turn on tally light X - STOP
	buffer[0]  = 0xa6;
	buffer[1]  = 0x00;
	buffer[2]  = 0x0b; // 0x00, 0x0F, 0x10
	buffer[3]  = 0xff;
	buffer[4]  = 0xfe;
	SendPacket (0x00, 0x05, buffer);

	// turn on tally light X - FWD
	buffer[0]  = 0xa6;
	buffer[1]  = 0x00;
	buffer[2]  = 0x0c; // 0x00, 0x0F, 0x10
	buffer[3]  = 0xff;
	buffer[4]  = 0xfe;
	SendPacket (0x00, 0x05, buffer);

	// turn on tally light X - MARK
	buffer[0]  = 0xa6;
	buffer[1]  = 0x00;
	buffer[2]  = 0x0d; // 0x00, 0x0F, 0x10
	buffer[3]  = 0xff;
	buffer[4]  = 0xfe;
	SendPacket (0x00, 0x05, buffer);

	// turn on tally light X - CUE/LOAD
	buffer[0]  = 0xa6; // command code
	buffer[1]  = 0x00; // who knows
	buffer[2]  = 0x0e; // light number
	buffer[3]  = 0xff; // brightness
	buffer[4]  = 0xfe; // EOP
	SendPacket (0x00, 0x05, buffer);
*/

	command_code = 0xa6;
	length = 2;
	addr = 0;
	data = 0;

	// enable, clear, and turn on lcd backlights
    SetBacklights (0x00, 0x00, 0x00, 0x00);
	buffer[0]  = 0xa4;
	buffer[1]  = 0xfe;
	SendPacket (0x00, 0x02, buffer);

	// turn on a few of the backlights
    SetBacklights (0xaa, 0x55, 0x55, 0xaa);

	buffer[0]  = 0xa2;
	buffer[1]  = 0xfe;
	SendPacket (0x00, 0x02, buffer);

	buffer[0]  = 0xa3;
	buffer[1]  = 0xfe;
	SendPacket (0x00, 0x02, buffer);

	buffer[0]  = 0xa5;
	buffer[1]  = 0xfe;
	SendPacket (0x00, 0x02, buffer);

	// create pulse on LCD-11
	Write (0x18, 0x03);
	Write (0x20, 0xc0);
	Write (0x20, 0x00);
	Write (0x20, 0xc0);

/*
#define LCD_ADDR    0x10
#define LCD_RSTB    0x80
#define LCD_CSB     0x40
#define LCD_DATCMD  0x20
#define LCD_CLK     0x10

	// reset display
	Write (LCD_ADDR, LCD_CSB);
	Write (LCD_ADDR, LCD_RSTB | LCD_CSB);

	// write a command
	Write (LCD_ADDR, LCD_RSTB);
	data = 0xAF;
	for (i = 0; i < 8; i++) {
		Write (LCD_ADDR, LCD_RSTB | ((data & 0x80) ? 0xf : 0x0));
		Write (LCD_ADDR, LCD_RSTB | ((data & 0x80) ? 0xf : 0x0) | LCD_CLK);
		Write (LCD_ADDR, LCD_RSTB | ((data & 0x80) ? 0xf : 0x0));
		data = data << 1;
	}
	Write (LCD_ADDR, LCD_RSTB | LCD_CSB);

	// write some data
	for (j = 0; j < 128; j++) {
		Write (LCD_ADDR, LCD_RSTB | LCD_DATCMD);
		data = 0xFF;
		for (i = 0; i < 8; i++) {
			Write (LCD_ADDR, LCD_RSTB | LCD_DATCMD | ((data & 0x80) ? 0xf : 0x0));
			Write (LCD_ADDR, LCD_RSTB | LCD_DATCMD | ((data & 0x80) ? 0xf : 0x0) | LCD_CLK);
			Write (LCD_ADDR, LCD_RSTB | LCD_DATCMD | ((data & 0x80) ? 0xf : 0x0));
			data = data << 1;
		}
		Write (LCD_ADDR, LCD_RSTB | LCD_DATCMD | LCD_CSB);
	}
*/

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

#ifdef TEST_WORKING_STUFF
			if (led_timer == 0) {
				// turn off tally light 1
				buffer[0]  = 0xa6;
				buffer[1]  = 0x00;
				buffer[2]  = 0x0a; // 0x00, 0x0F, 0x10
				buffer[3]  = 0x00; // 0x00, 0x0F, 0x10
				buffer[4]  = 0xfe;
				SendPacket (0x00, 0x05, buffer);

				// turn on tally light 0
				buffer[0]  = 0xa6; // command code
				buffer[1]  = 0x00; // who knows
				buffer[2]  = 0x0c; // light number
				buffer[3]  = 0xff; // brightness
				buffer[4]  = 0xfe; // EOP
				SendPacket (0x00, 0x05, buffer);
			}

			if (led_timer == 12) {
				// turn off tally light 0
				buffer[0]  = 0xa6;
				buffer[1]  = 0x00;
				buffer[2]  = 0x0c; // 0x00, 0x0F, 0x10
				buffer[3]  = 0x00; // 0x00, 0x0F, 0x10
				buffer[4]  = 0xfe;
				SendPacket (0x00, 0x05, buffer);

				// turn on tally light 1
				buffer[0]  = 0xa6;
				buffer[1]  = 0x00;
				buffer[2]  = 0x0a; // 0x00, 0x0F, 0x10
				buffer[3]  = 0xff; // 0x00, 0x0F, 0x10
				buffer[4]  = 0xfe;
				SendPacket (0x00, 0x05, buffer);
			}

			// alternate red and green backlights in different columns
			if (led_timer == 0) {
				SetBacklights (0xaa, 0x55, 0x55, 0xaa);
			} else if (led_timer == 12) {
				SetBacklights (0xcc, 0x33, 0x33, 0xcc);
			}
#endif

/*
			if (data & 1) {
				Write (0x30, 0xff);
			} else {
				Write (0x30, 0x00);
			}
			data++;
*/

/*
			if (led_timer == 0) {
				printf ("%d ", addr++);
				Write (0x18, (lfsr() & 0xfc) | 0x00);
				for (i = 0; i < 4; i++) {
					Write (0x20, lfsr());
				}
				Write (0x18, 0x00 | 0x00);
				for (i = 0; i < 4; i++) {
					Write (0x20, lfsr());
				}
				printf ("\n\r");
			}

			if (addr == 67) {
				while (1) {
				}
			}
*/

/*
			// create pulse on LCD-5
			Write (0x18, 0x00);
			Write (0x20, 0xc0);

			// create pulse on LCD-7
			Write (0x18, 0x01);
			Write (0x20, 0xc0);	// any write to this address but 0x80 keeps rogue pulse off LCD-11

			// create pulse on LCD-9
			Write (0x18, 0x02);
			Write (0x20, 0xc0);	// any write to this address but 0x80 keeps rogue pulse off LCD-11

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
	printf ("%02x ", data);

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
