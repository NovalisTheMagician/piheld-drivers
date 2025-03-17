#pragma once

#define F_CPU 16000000

typedef struct
{
    volatile unsigned char odr;
    volatile unsigned char idr;
    volatile unsigned char ddr;
    volatile unsigned char cr1;
    volatile unsigned char cr2;
} GPIO_t;

#define GPIO_PA ((GPIO_t*)0x5000)
#define GPIO_PB ((GPIO_t*)0x5005)
#define GPIO_PC ((GPIO_t*)0x500A)
#define GPIO_PD ((GPIO_t*)0x500F)
#define GPIO_PE ((GPIO_t*)0x5014)
#define GPIO_PF ((GPIO_t*)0x5019)
#define GPIO_PG ((GPIO_t*)0x501E)
#define GPIO_PH ((GPIO_t*)0x5023)
#define GPIO_PI ((GPIO_t*)0x5028)

#define PIN0 ((unsigned char)1 << 0)
#define PIN1 ((unsigned char)1 << 1)
#define PIN2 ((unsigned char)1 << 2)
#define PIN3 ((unsigned char)1 << 3)
#define PIN4 ((unsigned char)1 << 4)
#define PIN5 ((unsigned char)1 << 5)
#define PIN6 ((unsigned char)1 << 6)
#define PIN7 ((unsigned char)1 << 7)

typedef struct
{
    volatile unsigned char cr1;
    volatile unsigned char cr2;
    volatile unsigned char ncr2;
    volatile unsigned char fpr;
    volatile unsigned char nfpr;
    volatile unsigned char iaspr;
    char unused0[2];
    volatile unsigned char pukr;
    char unused1;
    volatile unsigned char dukr;
} FLASH_t;

#define FLASH ((FLASH_t*)0x505A)

typedef struct
{
    volatile unsigned char cr1;
    volatile unsigned char cr2;
} ITC_t;

#define ITC ((ITC_t*)0x50A0)

typedef struct
{
    volatile unsigned char sr;
} RST_t;

#define RST ((RST_t*)0x50B3)

typedef struct
{
    volatile unsigned char ickr;
    volatile unsigned char eckr;
    char unused0;
    volatile unsigned char cmsr;
    volatile unsigned char swr;
    volatile unsigned char swcr;
    volatile unsigned char ckdivr;
    volatile unsigned char pckenr1;
    volatile unsigned char cssr;
    volatile unsigned char ccor;
    volatile unsigned char pckenr2;
    char unused1;
    volatile unsigned char hsitrimr;
    volatile unsigned char swimccr;
} CLK_t;

#define CLK ((CLK_t*)0x50C0)

#define CLK_PCKENR1_TIM1 (unsigned char)1<<7
#define CLK_PCKENR1_TIM3 (unsigned char)1<<6
#define CLK_PCKENR1_TIM2 (unsigned char)1<<5
#define CLK_PCKENR1_TIM4 (unsigned char)1<<4
#define CLK_PCKENR1_SPI  (unsigned char)1<<1
#define CLK_PCKENR1_I2C  (unsigned char)1<<0

#define CLK_PCKENR2_ADC  (unsigned char)1<<3

typedef struct
{
    volatile unsigned char cr;
    volatile unsigned char wr;
} WWDG_t;

#define WWDG ((WWDG_t*)0x50D1)

typedef struct
{
    volatile unsigned char kr;
    volatile unsigned char pr;
    volatile unsigned char rlr;
} IWDG_t;

#define IWDG ((IWDG_t*)0x50E0)

typedef struct
{
    volatile unsigned char csr1;
    volatile unsigned char apr;
    volatile unsigned char tbr;
} AWU_t;

#define AWU ((AWU_t*)0x50F0)

typedef struct
{
    volatile unsigned char csr;
} BEEP_t;

#define BEEP ((BEEP_t*)0x50F3)

typedef struct
{
    volatile unsigned char cr1;
    volatile unsigned char cr2;
    volatile unsigned char icr;
    volatile unsigned char sr;
    volatile unsigned char dr;
    volatile unsigned char crcpr;
    volatile unsigned char rxcrcr;
    volatile unsigned char txcrcr;
} SPI_t;

#define SPI ((SPI_t*)0x5200)

typedef struct
{
    volatile unsigned char cr1;
    volatile unsigned char cr2;
    volatile unsigned char freqr;
    volatile unsigned char oarl;
    volatile unsigned char oarh;
    char unused0;
    volatile unsigned char dr;
    volatile unsigned char sr1;
    volatile unsigned char sr2;
    volatile unsigned char sr3;
    volatile unsigned char itr;
    volatile unsigned char ccrl;
    volatile unsigned char ccrh;
    volatile unsigned char triser;
    volatile unsigned char pecr;
} I2C_t;

#define I2C ((I2C_t*)0x5210)

#define I2C_CR1_PE (1<<0)

#define I2C_CR2_ACK (1<<2)
#define I2C_CR2_STOP (1<<1)

#define I2C_CCRH_FS (1<<7)
#define I2C_CCRH_CCR_MASK (0b111)

#define I2C_OARH_ADDMODE (1<<7)
#define I2C_OARH_ADDCONF (1<<6)
#define I2C_OARH_ADD_MASK (0b11)
#define I2C_OARH_ADD_SHIFT 1

#define I2C_ITR_ITBUFEN (1<<2)
#define I2C_ITR_ITEVTEN (1<<1)
#define I2C_ITR_ITERREN (1<<0)

#define I2C_SR1_TXE (1<<7)
#define I2C_SR1_RXNE (1<<6)
#define I2C_SR1_STOPF (1<<4)
#define I2C_SR1_ADDR (1<<1)

#define I2C_SR2_AF (1<<2)

typedef struct
{
    volatile unsigned char cr1;
    volatile unsigned char cr2;
    volatile unsigned char smcr;
    volatile unsigned char etr;
    volatile unsigned char ier;
    volatile unsigned char sr1;
    volatile unsigned char sr2;
    volatile unsigned char egr;
    volatile unsigned char ccmr1;
    volatile unsigned char ccmr2;
    volatile unsigned char ccmr3;
    volatile unsigned char ccmr4;
    volatile unsigned char ccer1;
    volatile unsigned char ccer2;
    volatile unsigned char cntrh;
    volatile unsigned char cntrl;
    volatile unsigned char pscrh;
    volatile unsigned char pscrl;
    volatile unsigned char arrh;
    volatile unsigned char arrl;
    volatile unsigned char rcr;
    volatile unsigned char ccr1h;
    volatile unsigned char ccr1l;
    volatile unsigned char ccr2h;
    volatile unsigned char ccr2l;
    volatile unsigned char ccr3h;
    volatile unsigned char ccr3l;
    volatile unsigned char ccr4h;
    volatile unsigned char ccr4l;
    volatile unsigned char bkr;
    volatile unsigned char dtr;
    volatile unsigned char oisr;
} TIM1_t;

#define TIM1 ((TIM1_t*)0x5250)

typedef struct
{
    volatile unsigned char cr1;
    volatile unsigned char ier;
    volatile unsigned char sr1;
    volatile unsigned char sr2;
    volatile unsigned char egr;
    volatile unsigned char ccmr1;
    volatile unsigned char ccmr2;
    volatile unsigned char ccmr3;
    volatile unsigned char ccer1;
    volatile unsigned char ccer2;
    volatile unsigned char cntrh;
    volatile unsigned char cntrl;
    volatile unsigned char pscr;
    volatile unsigned char arrh;
    volatile unsigned char arrl;
    volatile unsigned char ccr1h;
    volatile unsigned char ccr1l;
    volatile unsigned char ccr2h;
    volatile unsigned char ccr2l;
    volatile unsigned char ccr3h;
    volatile unsigned char ccr3l;
} TIM2_t;

#define TIM2 ((TIM2_t*)0x5300)

typedef struct
{
    volatile unsigned char cr1;
    volatile unsigned char ier;
    volatile unsigned char sr1;
    volatile unsigned char sr2;
    volatile unsigned char egr;
    volatile unsigned char ccmr1;
    volatile unsigned char ccmr2;
    volatile unsigned char ccer1;
    volatile unsigned char cntrh;
    volatile unsigned char cntrl;
    volatile unsigned char pscr;
    volatile unsigned char arrh;
    volatile unsigned char arrl;
    volatile unsigned char ccr1h;
    volatile unsigned char ccr1l;
    volatile unsigned char ccr2h;
    volatile unsigned char ccr2l;
} TIM3_t;

#define TIM3 ((TIM3_t*)0x5320)

typedef struct
{
    volatile unsigned char cr1;
    volatile unsigned char ier;
    volatile unsigned char sr1;
    volatile unsigned char egr;
    volatile unsigned char cntr;
    volatile unsigned char pscr;
    volatile unsigned char arr;
} TIM4_t;

#define TIM4 ((TIM4_t*)0x5340)

typedef struct
{
    volatile unsigned char csr;
    volatile unsigned char cr1;
    volatile unsigned char cr2;
    volatile unsigned char cr3;
    volatile unsigned char drh;
    volatile unsigned char drl;
    volatile unsigned char tdrh;
    volatile unsigned char tdrl;
    volatile unsigned char htrh;
    volatile unsigned char htrl;
    volatile unsigned char ltrh;
    volatile unsigned char ltrl;
    volatile unsigned char awsrh;
    volatile unsigned char awsrl;
    volatile unsigned char awcrh;
    volatile unsigned char awcrl;
} ADC_t;

#define ADC ((ADC_t*)0x5400)

#define enableInterrupts()    {__asm__("rim\n");}  /* enable interrupts */
#define disableInterrupts()   {__asm__("sim\n");}  /* disable interrupts */
#define nop()                 {__asm__("nop\n");}  /* No Operation */
#define trap()                {__asm__("trap\n");} /* Trap (soft IT) */
#define wfi()                 {__asm__("wfi\n");}  /* Wait For Interrupt */
#define halt()                {__asm__("halt\n");} /* Halt */
