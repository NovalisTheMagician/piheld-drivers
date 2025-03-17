#include "stm8.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define WRITE_MASK (1 << 7)

#define VERSION_MAJOR			0
#define VERSION_MINOR			4

#define SLAVE_ADDRESS			0x1F
#define INT_DURATION_MS			1

#define FIFO_SIZE				31

#define KEY_LIST_SIZE			10

#define KEY_POLL_TIME			1
#define KEY_HOLD_TIME			300

enum reg_id
{
	REG_ID_VER = 0x01, // fw version
	REG_ID_CFG = 0x02, // config
	REG_ID_INT = 0x03, // interrupt status
	REG_ID_KEY = 0x04, // key status
	REG_ID_BKL = 0x05, // backlight
	REG_ID_DEB = 0x06, // debounce cfg
	REG_ID_FRQ = 0x07, // poll freq cfg
	REG_ID_RST = 0x08, // reset
	REG_ID_FIF = 0x09, // fifo

	REG_ID_LAST,
};

#define CFG_OVERFLOW_ON		(unsigned char)(1 << 0)
#define CFG_OVERFLOW_INT	(unsigned char)(1 << 1)
#define CFG_PANIC_INT		(unsigned char)(1 << 2)

#define INT_OVERFLOW		(unsigned char)(1 << 0)
#define INT_KEY				(unsigned char)(1 << 1)
#define INT_PANIC			(unsigned char)(1 << 2)

#define KEY_COUNT_MASK		0x1F

#define VER_VAL				((VERSION_MAJOR << 4) | (VERSION_MINOR << 0))

#define NUM_OF_ROWS	4
#define NUM_OF_COLS	9

GPIO_t *int_port = GPIO_PE;
const uint8_t int_pin = PIN0;

GPIO_t *led_port = GPIO_PD;
const uint8_t led_pin = PIN3;

TIM2_t *bkl_tim = TIM2;

GPIO_t *lcdrst_port = GPIO_PD;
const uint8_t lcdrst_pin = PIN7;

GPIO_t *row_ports[NUM_OF_ROWS] = { GPIO_PA, GPIO_PA, GPIO_PA, GPIO_PA };
const uint8_t row_pins[NUM_OF_ROWS] = { PIN2, PIN3, PIN4, PIN5 };
GPIO_t *col_ports[NUM_OF_COLS] = { GPIO_PB, GPIO_PB, GPIO_PB, GPIO_PB, GPIO_PB, GPIO_PB, GPIO_PB, GPIO_PB, GPIO_PA };
const uint8_t col_pins[NUM_OF_COLS] = { PIN0, PIN1, PIN2, PIN3, PIN4, PIN5, PIN6, PIN7, PIN6 };

enum key_state
{
	KEY_STATE_RELEASED,
	KEY_STATE_PRESSED
};

struct fifo_item 
{
	uint8_t key;
	uint8_t state;
};

uint8_t regs[REG_ID_LAST];

uint8_t reg_get_value(enum reg_id reg)
{
	return regs[reg];
}

void reg_set_value(enum reg_id reg, uint8_t value)
{
	regs[reg] = value;
}

bool reg_is_bit_set(enum reg_id reg, uint8_t bit)
{
	return regs[reg] & bit;
}

void reg_set_bit(enum reg_id reg, uint8_t bit)
{
	regs[reg] |= bit;
}

void reg_clear_bit(enum reg_id reg, uint8_t bit)
{
	regs[reg] &= ~bit;
}

void reg_init(void)
{
	regs[REG_ID_CFG] = CFG_OVERFLOW_INT;
	regs[REG_ID_DEB] = 1;
	regs[REG_ID_FRQ] = 5;
	regs[REG_ID_BKL] = 0;
}

static struct 
{
	struct fifo_item fifo[FIFO_SIZE];
	uint8_t count;
	uint8_t read_idx;
	uint8_t write_idx;
} fifo;

uint8_t fifo_count(void)
{
	return fifo.count;
}

void fifo_flush(void)
{
	fifo.write_idx = 0;
	fifo.read_idx = 0;
	fifo.count = 0;
}

bool fifo_enqueue(struct fifo_item item)
{
	if (fifo.count >= FIFO_SIZE)
		return false;

	fifo.fifo[fifo.write_idx++] = item;

	fifo.write_idx %= FIFO_SIZE;
	++fifo.count;

	return true;
}

void fifo_enqueue_force(struct fifo_item item)
{
	if (fifo_enqueue(item))
		return;

	fifo.fifo[fifo.write_idx++] = item;
	fifo.write_idx %= FIFO_SIZE;

	fifo.read_idx++;
	fifo.read_idx %= FIFO_SIZE;
}

struct fifo_item fifo_dequeue(void)
{
	struct fifo_item item = { 0 };
	if (fifo.count == 0)
		return item;

	item = fifo.fifo[fifo.read_idx++];
	fifo.read_idx %= FIFO_SIZE;
	--fifo.count;

	return item;
}

volatile uint32_t systickovf_counter=0;			//systick overflow counter

void tim4_ovr_isr(void) __interrupt(23)
{
	TIM4->sr1 &=~(1<<0);						//clear the flag
	systickovf_counter+=0x100;					//increment systick overflow counter - 8bit timer counter
}

enum
{
	idle,
	rx_reg,
	rx_dat,
	tx_msb,
	tx_lsb
} i2cstate;
uint8_t i2creg;
struct fifo_item i2citem;

void backlight_update(void);

void i2c_isr(void) __interrupt(19)
{
	uint8_t reg;
	if(I2C->sr1 & I2C_SR1_ADDR) // address match
	{
		if(i2cstate == idle)
		{
			i2cstate = rx_reg;
			i2creg = 0;
		}
		reg = I2C->sr1;
		reg = I2C->sr3;
	}
	else if (I2C->sr1 & I2C_SR1_RXNE) // receive data
	{
		uint8_t data = I2C->dr;
		if(i2cstate == rx_reg)
		{
			i2creg = data & ~WRITE_MASK;
			if(data & WRITE_MASK) 	// write op
				i2cstate = rx_dat;
			else					// read op
				i2cstate = tx_msb;
		}
		else if(i2cstate == rx_dat)
		{
			switch (i2creg) {
			case REG_ID_CFG:
			case REG_ID_INT:
			case REG_ID_DEB:
			case REG_ID_FRQ:
			case REG_ID_BKL:
			{
				reg_set_value(i2creg, data);

				if ((i2creg == REG_ID_BKL))
					backlight_update();
				break;
			}
			case REG_ID_RST:
				WWDG->cr = 0x80; // do a software reset
				break;
			}
			i2cstate = idle;
		}
	}
	else if (I2C->sr1 & I2C_SR1_TXE) // transmit data
	{
		if(i2cstate == tx_msb)
		{
			switch(i2creg)
			{
			case REG_ID_VER:
				I2C->dr = VER_VAL;
				//i2cstate = idle;
				break;
			case REG_ID_KEY:
				I2C->dr = fifo_count();
				//i2cstate = idle;
				break;
			case REG_ID_FIF:
				i2citem = fifo_dequeue();
				I2C->dr = i2citem.state;
				//i2cstate = tx_lsb;
				break;
			default:
				I2C->dr = reg_get_value(i2creg);
				//i2cstate = idle;
				break;
			}
			i2cstate = tx_lsb;
		}
		else if(i2cstate == tx_lsb)
		{
			if(i2creg == REG_ID_FIF)
				I2C->dr = i2citem.key;
			else
				I2C->dr = 0x00;
			i2cstate = idle;
		}
		else
		{
			I2C->dr = 0x00;
			//  Some dummy data to send.
		}
	}
	else if(I2C->sr2 & I2C_SR2_AF)
	{
		I2C->sr2 &= ~I2C_SR2_AF;
		i2cstate = idle;
	}
	else if(I2C->sr1 & I2C_SR1_STOPF)
	{
		i2cstate = idle;
		I2C->cr2 &= ~I2C_CR2_STOP;
	}
	else // error occured, clear all status registers
	{
		reg = I2C->sr1;
		reg = I2C->sr2;
		reg = I2C->sr3;

		i2cstate = idle;
	}
}

void i2c_init(void)
{
	CLK->pckenr1 |= CLK_PCKENR1_I2C;
	I2C->cr1 &= ~I2C_CR1_PE;

	I2C->freqr = 16;
	I2C->ccrh &= ~I2C_CCRH_FS;
	I2C->ccrl = 0xa0;
	I2C->ccrh |= 0x00 & I2C_CCRH_CCR_MASK;

	I2C->oarh &= ~I2C_OARH_ADDMODE;
	I2C->oarh &= ~((0x00 & I2C_OARH_ADD_MASK) << I2C_OARH_ADD_SHIFT);
	I2C->oarl = SLAVE_ADDRESS << 1;
	I2C->oarh |= I2C_OARH_ADDCONF;

	//I2C->triser = 17;

	I2C->itr |= I2C_ITR_ITBUFEN;
	I2C->itr |= I2C_ITR_ITEVTEN;
	I2C->itr |= I2C_ITR_ITERREN;

	I2C->cr1 |= I2C_CR1_PE;

	I2C->cr2 |= I2C_CR2_ACK;

	i2cstate = idle;
}

void backlight_update(void)
{
	bkl_tim->ccr1h = 0;
	bkl_tim->ccr1l = reg_get_value(REG_ID_BKL);
}

void backlight_init(void)
{
	CLK->pckenr1 |= CLK_PCKENR1_TIM2;
	bkl_tim->ccmr1 = 0b01101000;
	bkl_tim->ccer1 = 1;
	bkl_tim->arrh = 0;
	bkl_tim->arrl = 0xff;
	bkl_tim->ccr1h = 0;
	bkl_tim->ccr1l = 255;
	bkl_tim->cr1 |= 1;
}

#define TIMx_PS			7						//prescaler = 2^TIMx_PS. [0..7]->[1x..128x]

uint32_t ticks(void) {
	uint32_t m;
	uint8_t f;

	//do a double read
	do {
		m = systickovf_counter;				//read the overflow counter
		f = TIM4->cntr;						//read the least significant 16-bits
	} while (m != systickovf_counter);		//gaurd against overflow

	return (m | f) << TIMx_PS;
}

void time_delay_ms(uint32_t delay)
{
	uint32_t start_time = ticks();

	delay *= (F_CPU / 1000);				//convert ms to ticks
	while (ticks() - start_time < delay) continue;	//wait for timer to expire
}

uint32_t time_uptime_ms(void)
{
	return systickovf_counter / (F_CPU / 1000);
}

void time_init(void)
{
	CLK->pckenr1 |= CLK_PCKENR1_TIM4;			//'1'=clock enabled, '0'=clock disabled
	
	//stop the timer
	TIM4->cr1 &=~(1<<0);						//stop the timer
	
	//set up the timer
	TIM4->cr1 = ((unsigned char)1<<7) |						//'1'->enable auto reload buffer
	  			((unsigned char)0<<5) |						//'0'->edge aligned. 1..3->center aligned
				((unsigned char)0<<4) |						//'0'->up counter, '1' downcounter
				((unsigned char)0<<3) |						//'0'->continuous mode, '1'->one pulse mode
				((unsigned char)0<<2) |						//'0'-> update enable source
				((unsigned char)0<<1) |						//'0'-> update enabled
				((unsigned char)0<<0);							//counter disabled
	//TIMx->CR2 = 0;							//default value
	//TIMx->SMCR = 0;							//default value
	//TIMx->ETR = 0;							//'0'->external trigger not inverted
	
	TIM4->pscr = TIMx_PS & 0x07;			//set up the prescaler to 0x07->128:1
	TIM4->cntr = 0; 						//TIMx->CNTRL = 0;			//reset the counter
	TIM4->arr = 0xff;						//load up the auto reload register - 0xff -> each cycle is 0x100
	
	TIM4->sr1&=~(1<<0);						//clear UIF
	TIM4->ier|= (1<<0);						//'1'->enable overflow interrupt, '0'->disable interrupt

	//re-enable the counter
	TIM4->cr1 |= (1<<0);	
}

void key_cb(uint8_t key, uint8_t state);

struct key
{
	uint8_t state;
	uint32_t hold_start_time;
};

struct 
{
	struct key curr_keys[NUM_OF_COLS * NUM_OF_ROWS];
	struct key prev_keys[NUM_OF_COLS * NUM_OF_ROWS];
	uint32_t last_process_time;
} keyboard;

void keyboard_process(void)
{
	//if ((time_uptime_ms() - keyboard.last_process_time) <= KEY_POLL_TIME)
		//return;

	bool doInt = false;

	for (uint32_t r = 0; r < NUM_OF_ROWS; ++r) {
		GPIO_t *rowPort = row_ports[r];
		uint8_t rowPin = row_pins[r];
		rowPort->ddr |= rowPin; // output pin
		rowPort->odr &= ~rowPin; // set to low

		for (uint32_t c = 0; c < NUM_OF_COLS; ++c) {
			GPIO_t *colPort = col_ports[c];
			uint8_t colPin = col_pins[c];
			bool pressed = (colPort->idr & colPin) == 0; //(port_pin_get_input_level(row_pins[r]) == 0);
			int32_t key_idx = (int32_t)(((r * NUM_OF_COLS) + c));

			keyboard.curr_keys[key_idx].state = pressed ? KEY_STATE_PRESSED : KEY_STATE_RELEASED;
			if(keyboard.prev_keys[key_idx].state != keyboard.curr_keys[key_idx].state)
			{
				//if((time_uptime_ms() - keyboard.prev_keys[key_idx].hold_start_time) > reg_get_value(REG_ID_DEB))
				{
					key_cb(key_idx+1, keyboard.curr_keys[key_idx].state);
					keyboard.curr_keys[key_idx].hold_start_time = time_uptime_ms();
					keyboard.prev_keys[key_idx].state = keyboard.curr_keys[key_idx].state;
					keyboard.prev_keys[key_idx].hold_start_time = keyboard.curr_keys[key_idx].hold_start_time;

					doInt = true;
				}
			}
		}

		rowPort->odr |= rowPin; // output high
		//port_pin_set_output_level(col_pins[c], 1);

		rowPort->ddr &= ~rowPin; // set to input

		//port_init.direction = PORT_PIN_DIR_INPUT;
		//port_init.input_pull = PORT_PIN_PULL_NONE;
		//port_pin_set_config(col_pins[c], &port_init);
	}

	if(doInt)
	{
		int_port->odr &= ~int_pin;
		time_delay_ms(INT_DURATION_MS);
		int_port->odr |= int_pin;
	}

	//memcpy(keyboard.prev_keys, keyboard.curr_keys, sizeof keyboard.curr_keys);

	keyboard.last_process_time = time_uptime_ms();
}

void keyboard_init(void)
{
	// Rows
	for (uint32_t i = 0; i < NUM_OF_ROWS; ++i)
	{
		GPIO_t *port = row_ports[i];
		uint8_t pin = row_pins[i];

		port->ddr &= ~pin; // set input
		port->cr1 &= ~pin; // no pull-up
	}

	// Cols
	for(uint32_t i = 0; i < NUM_OF_COLS; ++i)
	{
		GPIO_t *port = col_ports[i];
		uint8_t pin = col_pins[i];

		port->ddr &= ~pin; // set input
		port->cr1 |= pin; // set pull-up
	}

	memset(keyboard.curr_keys, 0, sizeof keyboard.curr_keys);
	memset(keyboard.prev_keys, 0, sizeof keyboard.prev_keys);
}

void led_on(void);
void led_off(void);

void key_cb(uint8_t key, uint8_t state)
{
	reg_set_bit(REG_ID_INT, INT_KEY);

	const struct fifo_item item = { key, state };
	if (!fifo_enqueue(item)) {
		if (reg_is_bit_set(REG_ID_CFG, CFG_OVERFLOW_INT)) {
			reg_set_bit(REG_ID_INT, INT_OVERFLOW);
		}

		if (reg_is_bit_set(REG_ID_CFG, CFG_OVERFLOW_ON))
			fifo_enqueue_force(item);
	}
}

void config_int_pin(void)
{
	int_port->ddr |= int_pin;
	int_port->odr |= int_pin;
}

void config_lcdrst_pin(void)
{
	lcdrst_port->ddr |= lcdrst_pin;
	//lcdrst_port->odr |= lcdrst_pin;
}

void config_led_pin(void)
{
	led_port->ddr |= led_pin;
	led_port->cr1 |= led_pin;
}

void led_on(void)
{
	led_port->odr |= led_pin;
}

void led_off(void)
{
	led_port->odr &= ~led_pin;
}

void main(void)
{
	CLK->ckdivr = 0x00;

	disableInterrupts();

	/*
	memset(GPIO_PA, 0, sizeof *GPIO_PA);
	memset(GPIO_PB, 0, sizeof *GPIO_PB);
	memset(GPIO_PC, 0, sizeof *GPIO_PC);
	memset(GPIO_PD, 0, sizeof *GPIO_PD);
	memset(GPIO_PE, 0, sizeof *GPIO_PE);
	memset(GPIO_PF, 0, sizeof *GPIO_PF);
	*/

	time_init();
	config_led_pin();
	config_int_pin();
	reg_init();
	backlight_init();
	keyboard_init();
	i2c_init();
	fifo_flush();

	enableInterrupts();

	backlight_update();

	//led_on();

	while (true) 
	{
		keyboard_process();
	}
}
