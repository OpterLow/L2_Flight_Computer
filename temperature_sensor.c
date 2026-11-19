/* This is code that outputs temperature in Celcius to an LCD Display */

#include <stdio.h>
#include <stdlib.h>
#include "stm32l4xx.h"

/* HTS221 temperature and humidity sensor */
#define HTS_ADDR       0x5F   /* 7-bit address */
#define HTS_WHO_AM_I   0x0F   /* expect 0xBC */
#define HTS_CTRL_REG1  0x20
#define HTS_DATA_OUT   0x28   /* H_L, H_H, T_L, T_H */
#define HTS_CAL_START  0x30   /* 16 bytes of factory calibration */
#define HTS_AUTO_INC   0x80   /* set MSB of the register address for multi-byte reads */

/* 74HC595 latch pin (RCLK): PA2 = Arduino D10 */
#define LATCH_PORT GPIOA
#define LATCH_PIN  2

static volatile uint32_t ticks;
void SysTick_Handler(void) { ticks++; }
static void delay_ms(uint32_t ms) { uint32_t s = ticks; while ((ticks - s) <= ms); }
static void short_delay(void) { for (volatile uint32_t i = 0; i < 100; i++); }

/* ---------- UART (blocking TX, for the serial terminal) ---------- */
static void uart_puts(const char *s) {
    while (*s) {
        while (!(USART1->ISR & USART_ISR_TXE_TXFNF));
        USART1->TDR = (uint8_t)*s++;
    }
}

/* ---------- I2C2 register-level driver (polling) ---------- */
static int i2c_wait(uint32_t mask) {
    uint32_t t = 200000;
    while (!(I2C2->ISR & mask)) {
        if (I2C2->ISR & I2C_ISR_NACKF) return -1;
        if (--t == 0) return -2;
    }
    return 0;
}

static void i2c_clear_err(void) {
    uint32_t t = 200000;
    while (!(I2C2->ISR & I2C_ISR_STOPF) && --t);
    I2C2->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
    I2C2->ISR |= I2C_ISR_TXE;
}

static int i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    I2C2->CR2 = ((uint32_t)addr << 1) | (2U << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_AUTOEND | I2C_CR2_START;
    if (i2c_wait(I2C_ISR_TXIS)) { i2c_clear_err(); return -1; }
    I2C2->TXDR = reg;
    if (i2c_wait(I2C_ISR_TXIS)) { i2c_clear_err(); return -1; }
    I2C2->TXDR = val;
    if (i2c_wait(I2C_ISR_STOPF)) { i2c_clear_err(); return -1; }
    I2C2->ICR = I2C_ICR_STOPCF;
    return 0;
}

static int i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
    I2C2->CR2 = ((uint32_t)addr << 1) | (1U << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
    if (i2c_wait(I2C_ISR_TXIS)) { i2c_clear_err(); return -1; }
    I2C2->TXDR = reg;
    if (i2c_wait(I2C_ISR_TC)) { i2c_clear_err(); return -1; }
    I2C2->CR2 = ((uint32_t)addr << 1) | I2C_CR2_RD_WRN
              | ((uint32_t)n << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | I2C_CR2_START;
    for (uint8_t i = 0; i < n; i++) {
        if (i2c_wait(I2C_ISR_RXNE)) { i2c_clear_err(); return -1; }
        buf[i] = (uint8_t)I2C2->RXDR;
    }
    if (i2c_wait(I2C_ISR_STOPF)) { i2c_clear_err(); return -1; }
    I2C2->ICR = I2C_ICR_STOPCF;
    return 0;
}

/* ---------- SPI1 -> 74HC595 -> LCD1602 (4-bit mode) ---------- */
static void shift_out(uint8_t b) {
    LATCH_PORT->BSRR = (1U << (LATCH_PIN + 16));
    while (!(SPI1->SR & SPI_SR_TXE));
    *(volatile uint8_t *)&SPI1->DR = b;
    while (SPI1->SR & SPI_SR_BSY);
    (void)*(volatile uint8_t *)&SPI1->DR;
    LATCH_PORT->BSRR = (1U << LATCH_PIN);
}

static void lcd_nibble(uint8_t n, uint8_t rs) {
    uint8_t b = (uint8_t)((n & 0x0F) << 4) | (rs ? 0x01 : 0x00);
    shift_out(b | 0x02);
    shift_out(b);
}

static void lcd_byte(uint8_t v, uint8_t rs) {
    lcd_nibble(v >> 4, rs);
    lcd_nibble(v & 0x0F, rs);
    short_delay();
}

static void lcd_init(void) {
    delay_ms(50);
    lcd_nibble(0x3, 0); delay_ms(5);
    lcd_nibble(0x3, 0); delay_ms(1);
    lcd_nibble(0x3, 0); delay_ms(1);
    lcd_nibble(0x2, 0); delay_ms(1);
    lcd_byte(0x28, 0);
    lcd_byte(0x08, 0);
    lcd_byte(0x01, 0); delay_ms(3);
    lcd_byte(0x06, 0);
    lcd_byte(0x0C, 0);
}

static void lcd_print(uint8_t row, const char *s) {
    lcd_byte(0x80 | (row ? 0x40 : 0x00), 0);
    for (int i = 0; i < 16; i++) lcd_byte(*s ? (uint8_t)*s++ : ' ', 1);
}

/* ---------- hardware init ---------- */
static void init_hw(void) {
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    RCC->APB2ENR  |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_SPI1EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;

    GPIOB->MODER &= ~((3U<<12)|(3U<<14)|(3U<<20)|(3U<<22));
    GPIOB->MODER |=  ((2U<<12)|(2U<<14)|(2U<<20)|(2U<<22));
    GPIOB->OTYPER |= (1U<<10)|(1U<<11);
    GPIOB->PUPDR  &= ~((3U<<20)|(3U<<22));
    GPIOB->PUPDR  |=  ((1U<<20)|(1U<<22));
    GPIOB->AFR[0] &= ~((0xFU<<24)|(0xFU<<28));
    GPIOB->AFR[0] |=  ((7U<<24)|(7U<<28));
    GPIOB->AFR[1] &= ~((0xFU<<8)|(0xFU<<12));
    GPIOB->AFR[1] |=  ((4U<<8)|(4U<<12));

    GPIOA->MODER &= ~((3U<<10)|(3U<<14)|(3U << (2*LATCH_PIN)));
    GPIOA->MODER |=  ((2U<<10)|(2U<<14)|(1U << (2*LATCH_PIN)));
    GPIOA->AFR[0] &= ~((0xFU<<20)|(0xFU<<28));
    GPIOA->AFR[0] |=  ((5U<<20)|(5U<<28));
    LATCH_PORT->BSRR = (1U << LATCH_PIN);

    USART1->BRR = (SystemCoreClock + (115200U / 2U)) / 115200U;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE;
    USART1->CR1 |= USART_CR1_UE;

    I2C2->CR1 &= ~I2C_CR1_PE;
    I2C2->TIMINGR = 0x00420F13;
    I2C2->CR1 |= I2C_CR1_PE;

    SPI1->CR1 = 0;
    SPI1->CR2 = (7U << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI
              | (1U << SPI_CR1_BR_Pos) | SPI_CR1_SPE;

    SysTick_Config(SystemCoreClock / 1000U);
}

/* ---------- main ---------- */
int main(void) {
    char msg[128], l1[24], l2[24];
    uint8_t id = 0, cal[16], raw[4];

    init_hw();
    uart_puts("\r\nHTS221 temperature and humidity over I2C2, LCD over SPI1\r\n");
    lcd_init();
    lcd_print(0, "HTS221 starting");

    if (i2c_read_regs(HTS_ADDR, HTS_WHO_AM_I, &id, 1) != 0 || id != 0xBC) {
        uart_puts("HTS221 not found at 0x5F\r\n");
        lcd_print(1, "Sensor not found");
        while (1);
    }
    snprintf(msg, sizeof msg, "WHO_AM_I = 0x%02X\r\n", (unsigned)id);
    uart_puts(msg);

    /* power on, block data update, 1 Hz output data rate */
    i2c_write_reg(HTS_ADDR, HTS_CTRL_REG1, 0x85);

    /* factory calibration constants (for linear interpolation) */
    if (i2c_read_regs(HTS_ADDR, HTS_CAL_START | HTS_AUTO_INC, cal, 16) != 0) {
        uart_puts("Calibration read failed\r\n");
        while (1);
    }
    int32_t H0_x2  = cal[0];
    int32_t H1_x2  = cal[1];
    int32_t T0_x8  = cal[2] | ((cal[5] & 0x03) << 8);
    int32_t T1_x8  = cal[3] | (((cal[5] >> 2) & 0x03) << 8);
    int32_t H0_out = (int16_t)(cal[6]  | (cal[7]  << 8));
    int32_t H1_out = (int16_t)(cal[10] | (cal[11] << 8));
    int32_t T0_out = (int16_t)(cal[12] | (cal[13] << 8));
    int32_t T1_out = (int16_t)(cal[14] | (cal[15] << 8));
    if (H1_out == H0_out || T1_out == T0_out) {
        uart_puts("Bad calibration data\r\n");
        while (1);
    }

    delay_ms(1500);   /* let the first conversion finish */

    while (1) {
        if (i2c_read_regs(HTS_ADDR, HTS_DATA_OUT | HTS_AUTO_INC, raw, 4) == 0) {
            int32_t h_out = (int16_t)(raw[0] | (raw[1] << 8));
            int32_t t_out = (int16_t)(raw[2] | (raw[3] << 8));

            /* temperature in hundredths of a degree C */
            int32_t dT = T1_out - T0_out;
            int64_t tn = (int64_t)T0_x8 * dT + (int64_t)(t_out - T0_out) * (T1_x8 - T0_x8);
            int32_t t_x100 = (int32_t)((tn * 100) / (8 * (int64_t)dT));

            /* humidity in tenths of a percent, limited to 0..100 */
            int32_t dH = H1_out - H0_out;
            int64_t hn = (int64_t)H0_x2 * dH + (int64_t)(h_out - H0_out) * (H1_x2 - H0_x2);
            int32_t rh_x10 = (int32_t)((hn * 10) / (2 * (int64_t)dH));
            if (rh_x10 < 0) rh_x10 = 0;
            if (rh_x10 > 1000) rh_x10 = 1000;

            int32_t f_x100 = (t_x100 * 9) / 5 + 3200;
            const char *ts = (t_x100 < 0) ? "-" : "";
            const char *fs = (f_x100 < 0) ? "-" : "";
            int32_t ta = abs(t_x100), fa = abs(f_x100);

            snprintf(l1, sizeof l1, "T=%s%ld.%02ld C", ts, (long)(ta / 100), (long)(ta % 100));
            snprintf(l2, sizeof l2, "RH=%ld.%ld %%", (long)(rh_x10 / 10), (long)(rh_x10 % 10));
            lcd_print(0, l1);
            lcd_print(1, l2);

            snprintf(msg, sizeof msg, "Temp: %s%ld.%02ld C (%s%ld.%02ld F) | Humidity: %ld.%ld %%RH\r\n",
                     ts, (long)(ta / 100), (long)(ta % 100),
                     fs, (long)(fa / 100), (long)(fa % 100),
                     (long)(rh_x10 / 10), (long)(rh_x10 % 10));
            uart_puts(msg);
        } else {
            uart_puts("I2C read error\r\n");
        }
        delay_ms(1000);
    }
}
