#include <stdint.h>

// CPU speed, in Hz.
#define F_CPU     2000000UL

// STM8S registers.
// GPIO
#define GPIOB_ODR *(volatile uint8_t *)(0x5005)
#define GPIOB_DDR *(volatile uint8_t *)(0x5007)
#define GPIOB_CR1 *(volatile uint8_t *)(0x5008)
#define GPIOC_ODR *(volatile uint8_t *)(0x500A)
#define GPIOC_DDR *(volatile uint8_t *)(0x500C)
#define GPIOC_CR1 *(volatile uint8_t *)(0x500D)
#define GPIOD_ODR *(volatile uint8_t *)(0x500F)
#define GPIOD_DDR *(volatile uint8_t *)(0x5011)
#define GPIOD_CR1 *(volatile uint8_t *)(0x5012)
// ADC
#define ADC1_CSR  *(volatile uint8_t *)(0x5400)
#define ADC1_CR1  *(volatile uint8_t *)(0x5401)
#define ADC1_CR2  *(volatile uint8_t *)(0x5402)
#define ADC1_DRH  *(volatile uint8_t *)(0x5404)
#define ADC1_DRL  *(volatile uint8_t *)(0x5405)
// I2C
#define I2C_CR1   *(volatile uint8_t *)(0x5210)
#define I2C_CR2   *(volatile uint8_t *)(0x5211)
#define I2C_FREQR *(volatile uint8_t *)(0x5212)
#define I2C_OARL  *(volatile uint8_t *)(0x5213)
#define I2C_OARH  *(volatile uint8_t *)(0x5214)
#define I2C_DR    *(volatile uint8_t *)(0x5216)
#define I2C_SR1   *(volatile uint8_t *)(0x5217)
#define I2C_SR2   *(volatile uint8_t *)(0x5218)
#define I2C_SR3   *(volatile uint8_t *)(0x5219)
#define I2C_CCRL  *(volatile uint8_t *)(0x521B)
#define I2C_CCRH  *(volatile uint8_t *)(0x521C)

// Peripheral definitions.
#define I2C_RXQ   (1)
#define I2C_TXQ   (0)

// Pin definitions.
#define CHARGED_PC  (6)
#define MEH_PC      (7)
#define LOW_PD      (3)
#define BATT_PD     (2)
#define SCL_PB      (4)
#define SDA_PB      (5)
// Global values.
#define DELAY_MS      (200)
#define SSD1306_BYTES ((128 * 64) / 8)
// (NiMH Battery; about 1.0V - 1.5V)
#define GOOD_CHARGE_VOLTAGE (1.325)
#define MEH_CHARGE_VOLTAGE  (1.175)
// (Lithium hybrid supercapacitor; about 1.8V - 2.7V)
//#define GOOD_CHARGE_VOLTAGE (2.4)
//#define MEH_CHARGE_VOLTAGE  (2.25)

// Is an OLED display connected?
// TODO: Detect with a timeout on initialization.
#define OLED_ATTACHED (1)
//#undef OLED_ATTACHED

// Declare some values before 'main'
uint32_t delay_i;
uint16_t adc_r;
float    batt_v;

// Framebuffer for the SSD1306.
// Since the STM8S003 only has 1KB of RAM, only store
// enough space for the middle 16 pixels, one row of large text.
#define OLED_FB_SIZE         ((128 * 16) / 8)
#define OLED_BLANK_ROWS_SIZE ((SSD1306_BYTES - OLED_FB_SIZE) / 2)
static volatile uint8_t oled_fb[OLED_FB_SIZE];

// (Thanks, lujji!)
uint16_t ADC_read() {
  uint8_t adcH, adcL;
  ADC1_CR1 |=  (0x01);
  while (!(ADC1_CSR & (0x80)));
  adcL = ADC1_DRL;
  adcH = ADC1_DRH;
  // (Clear the EOC flag manually)
  ADC1_CSR &= ~(0x80);
  return (adcL | (adcH << 8));
}

void i2c_init() {
  // Tell the I2C peripheral that the clock is 2MHz.
  I2C_FREQR |=  (0x02);
  // Ask for 100KHz "standard mode" timings.
  // 0x0A = 0d10, and speed = 2 * CCR * (1/FREQR).
  // So, 2 * 0x0A * (1/2MHz) = 0.00001 = (1/100KHz)
  I2C_CCRL  |=  (0x0A);
  // Set the usual 7-bit addressing mode.
  I2C_OARH  &= ~(0x80);
  // Enable the peripheral.
  I2C_CR1   |=  (0x01);
}

void i2c_set_addr(uint8_t addr) {
  // Set the 7-bit device address.
  // First, set the given address in the 'Data Register'.
  I2C_DR     =  (addr);
  // Wait for the 'address acknowledged' flag in SR1.
  while (!(I2C_SR1 & (0x02))) {};
  // Read SR3 to clear the 'Event #6' flag...?
  (void) I2C_SR3;
  // Make sure that 'ACK's are enabled.
  I2C_CR2   |=  (0x04);
}

void i2c_start() {
  // Set the start condition.
  I2C_CR2   |=  (0x01);
  // Wait for it to manifest.
  while (!(I2C_SR1 & (0x01))) {};
}

void i2c_stop() {
  // Set the stop condition.
  I2C_CR2   |=  (0x02);
  // Wait for it to manifest.
  while (I2C_SR3 & (0x01)) {};
}

void i2c_w(uint8_t dat) {
  // Set the 'Data Register'.
  I2C_DR     =  (dat);
  // Wait for the 'transmit data' buffer to be empty.
  while (!(I2C_SR1 & (0x80))) {};
}

void i2c_cmd_txq() {
  // Send '0x00' to indicate a command transmission.
  i2c_w(0x00);
}

void i2c_dat_txq() {
  // Send '0x40' to indicate a data transmission.
  i2c_w(0x40);
}

// (SSD1306 drawing)
// Draw the framebuffer to the display.
void ssd1306_draw_fb() {
  uint16_t s_iter;
  // Set the drawing window.
  i2c_start();
  i2c_set_addr(0x78 | I2C_TXQ);
  i2c_cmd_txq();
  i2c_w(0x21);
  i2c_w(0);
  i2c_w(127);
  i2c_w(0x20);
  i2c_w(0);
  i2c_w(7);
  i2c_stop();
  // Draw.
  i2c_start();
  i2c_set_addr(0x78 | I2C_TXQ);
  i2c_dat_txq();
  // Clear the top 24px
  for (s_iter = 0; s_iter < OLED_BLANK_ROWS_SIZE; ++s_iter) {
    i2c_w(0x00);
  }
  // Draw the framebuffer
  for (s_iter = 0; s_iter < OLED_FB_SIZE; ++s_iter) {
    i2c_w(oled_fb[s_iter]);
  }
  // Clear the bottom 24px
  for (s_iter = 0; s_iter < OLED_BLANK_ROWS_SIZE; ++s_iter) {
    i2c_w(0x00);
  }
  i2c_stop();
}

// Clear the display.
void ssd1306_clear() {
  uint16_t s_iter;
  // Set the drawing window.
  i2c_start();
  i2c_set_addr(0x78 | I2C_TXQ);
  i2c_cmd_txq();
  i2c_w(0x21);
  i2c_w(0);
  i2c_w(127);
  i2c_w(0x20);
  i2c_w(0);
  i2c_w(7);
  i2c_stop();
  // Draw.
  i2c_start();
  i2c_set_addr(0x78 | I2C_TXQ);
  i2c_dat_txq();
  for (s_iter = 0; s_iter < SSD1306_BYTES; ++s_iter) {
    i2c_w(0x00);
  }
  i2c_stop();
}

void ssd1306_clear_fb() {
  uint16_t s_iter;
  for (s_iter = 0; s_iter < OLED_FB_SIZE; ++s_iter) {
    oled_fb[s_iter] = 0x00;
  }
}

// Draw a pixel to the framebuffer.
void ssd1306_draw_px(uint8_t x, uint8_t y, int c) {
  if (c) {
    oled_fb[x + ((y/8) * 128)] |=  (1 << (y & 0x07));
  }
  else {
    oled_fb[x + ((y/8) * 128)] &= ~(1 << (y & 0x07));
  }
}

// Draw a single integer digit to the display.
void ssd1306_draw_digit(int x, int y, int d) {
  // Just use a simple 3x5 font for now.
  if (d == 0) {
    ssd1306_draw_px(x + 1, y,     1);
    ssd1306_draw_px(x,     y + 1, 1);
    ssd1306_draw_px(x,     y + 2, 1);
    ssd1306_draw_px(x,     y + 3, 1);
    ssd1306_draw_px(x + 1, y + 4, 1);
    ssd1306_draw_px(x + 2, y + 1, 1);
    ssd1306_draw_px(x + 2, y + 2, 1);
    ssd1306_draw_px(x + 2, y + 3, 1);
  }
  else if (d == 1) {
    ssd1306_draw_px(x + 1, y,     1);
    ssd1306_draw_px(x,     y + 1, 1);
    ssd1306_draw_px(x + 1, y + 1, 1);
    ssd1306_draw_px(x + 1, y + 2, 1);
    ssd1306_draw_px(x + 1, y + 3, 1);
    ssd1306_draw_px(x,     y + 4, 1);
    ssd1306_draw_px(x + 1, y + 4, 1);
    ssd1306_draw_px(x + 2, y + 4, 1);
  }
  else if (d == 2) {
    ssd1306_draw_px(x,     y,     1);
    ssd1306_draw_px(x + 1, y,     1);
    ssd1306_draw_px(x + 2, y,     1);
    ssd1306_draw_px(x + 2, y + 1, 1);
    ssd1306_draw_px(x,     y + 2, 1);
    ssd1306_draw_px(x + 1, y + 2, 1);
    ssd1306_draw_px(x + 2, y + 2, 1);
    ssd1306_draw_px(x,     y + 3, 1);
    ssd1306_draw_px(x,     y + 4, 1);
    ssd1306_draw_px(x + 1, y + 4, 1);
    ssd1306_draw_px(x + 2, y + 4, 1);
  }
  else if (d == 3) {
    ssd1306_draw_px(x,     y,     1);
    ssd1306_draw_px(x + 1, y,     1);
    ssd1306_draw_px(x + 2, y,     1);
    ssd1306_draw_px(x + 2, y + 1, 1);
    ssd1306_draw_px(x,     y + 2, 1);
    ssd1306_draw_px(x + 1, y + 2, 1);
    ssd1306_draw_px(x + 2, y + 2, 1);
    ssd1306_draw_px(x + 2, y + 3, 1);
    ssd1306_draw_px(x,     y + 4, 1);
    ssd1306_draw_px(x + 1, y + 4, 1);
    ssd1306_draw_px(x + 2, y + 4, 1);
  }
  else if (d == 4) {
    ssd1306_draw_px(x,     y,     1);
    ssd1306_draw_px(x + 2, y,     1);
    ssd1306_draw_px(x,     y + 1, 1);
    ssd1306_draw_px(x + 2, y + 1, 1);
    ssd1306_draw_px(x,     y + 2, 1);
    ssd1306_draw_px(x + 1, y + 2, 1);
    ssd1306_draw_px(x + 2, y + 2, 1);
    ssd1306_draw_px(x + 2, y + 3, 1);
    ssd1306_draw_px(x + 2, y + 4, 1);
  }
  else if (d == 5) {
    ssd1306_draw_px(x,     y,     1);
    ssd1306_draw_px(x + 1, y,     1);
    ssd1306_draw_px(x + 2, y,     1);
    ssd1306_draw_px(x,     y + 1, 1);
    ssd1306_draw_px(x,     y + 2, 1);
    ssd1306_draw_px(x + 1, y + 2, 1);
    ssd1306_draw_px(x + 2, y + 2, 1);
    ssd1306_draw_px(x + 2, y + 3, 1);
    ssd1306_draw_px(x,     y + 4, 1);
    ssd1306_draw_px(x + 1, y + 4, 1);
    ssd1306_draw_px(x + 2, y + 4, 1);
  }
  else if (d == 6) {
    ssd1306_draw_px(x,     y,     1);
    ssd1306_draw_px(x + 1, y,     1);
    ssd1306_draw_px(x + 2, y,     1);
    ssd1306_draw_px(x,     y + 1, 1);
    ssd1306_draw_px(x,     y + 2, 1);
    ssd1306_draw_px(x + 1, y + 2, 1);
    ssd1306_draw_px(x + 2, y + 2, 1);
    ssd1306_draw_px(x,     y + 3, 1);
    ssd1306_draw_px(x + 2, y + 3, 1);
    ssd1306_draw_px(x,     y + 4, 1);
    ssd1306_draw_px(x + 1, y + 4, 1);
    ssd1306_draw_px(x + 2, y + 4, 1);
  }
  else if (d == 7) {
    ssd1306_draw_px(x,     y,     1);
    ssd1306_draw_px(x + 1, y,     1);
    ssd1306_draw_px(x + 2, y,     1);
    ssd1306_draw_px(x,     y + 1, 1);
    ssd1306_draw_px(x + 2, y + 1, 1);
    ssd1306_draw_px(x + 2, y + 2, 1);
    ssd1306_draw_px(x + 2, y + 3, 1);
    ssd1306_draw_px(x + 2, y + 4, 1);
  }
  else if (d == 8) {
    ssd1306_draw_px(x,     y,     1);
    ssd1306_draw_px(x + 1, y,     1);
    ssd1306_draw_px(x + 2, y,     1);
    ssd1306_draw_px(x,     y + 1, 1);
    ssd1306_draw_px(x + 2, y + 1, 1);
    ssd1306_draw_px(x,     y + 2, 1);
    ssd1306_draw_px(x + 1, y + 2, 1);
    ssd1306_draw_px(x + 2, y + 2, 1);
    ssd1306_draw_px(x,     y + 3, 1);
    ssd1306_draw_px(x + 2, y + 3, 1);
    ssd1306_draw_px(x,     y + 4, 1);
    ssd1306_draw_px(x + 1, y + 4, 1);
    ssd1306_draw_px(x + 2, y + 4, 1);
  }
  else if (d == 9) {
    ssd1306_draw_px(x,     y,     1);
    ssd1306_draw_px(x + 1, y,     1);
    ssd1306_draw_px(x + 2, y,     1);
    ssd1306_draw_px(x,     y + 1, 1);
    ssd1306_draw_px(x + 2, y + 1, 1);
    ssd1306_draw_px(x,     y + 2, 1);
    ssd1306_draw_px(x + 1, y + 2, 1);
    ssd1306_draw_px(x + 2, y + 2, 1);
    ssd1306_draw_px(x + 2, y + 3, 1);
    ssd1306_draw_px(x,     y + 4, 1);
    ssd1306_draw_px(x + 1, y + 4, 1);
    ssd1306_draw_px(x + 2, y + 4, 1);
  }
}

// Draw a float as text to the display;
// Use an arbitrary two decimal points of precision.
void ssd1306_draw_float(int x, int y, float txt) {
  // Draw the pre-decimal point digits.
  ssd1306_draw_digit(x, y, (int)txt);
  // Draw the decimal point.
  ssd1306_draw_px(x + 5, y + 6, 1);
  // Draw the post-decimal point digits.
  ssd1306_draw_digit(x + 10, y, (int)(txt * 10.0)  % 10);
  ssd1306_draw_digit(x + 15, y, (int)(txt * 100.0) % 10);
  // Draw a 'V'
  ssd1306_draw_px(x + 20, y, 1);
  ssd1306_draw_px(x + 20, y + 1, 1);
  ssd1306_draw_px(x + 20, y + 2, 1);
  ssd1306_draw_px(x + 21, y + 3, 1);
  ssd1306_draw_px(x + 21, y + 4, 1);
  ssd1306_draw_px(x + 22, y + 2, 1);
  ssd1306_draw_px(x + 22, y + 1, 1);
  ssd1306_draw_px(x + 22, y, 1);
}

void main() {
  // Set led pins on ports C and D to output mode.
  GPIOC_DDR |=  (1 << CHARGED_PC |
                 1 << MEH_PC);
  GPIOD_DDR |=  (1 << LOW_PD);
  // Set LED pins to push-pull mode.
  GPIOC_CR1 |=  (1 << CHARGED_PC |
                 1 << MEH_PC);
  GPIOD_CR1 |=  (1 << LOW_PD);

  #ifdef OLED_ATTACHED
    // Set I2C pins on port B to open-drain output mode.
    GPIOB_DDR |=  (1 << SCL_PB |
                   1 << SDA_PB);
    GPIOB_CR1 &= ~(1 << SCL_PB |
                   1 << SDA_PB);
  #endif

  // Set the LEDs all off.
  GPIOC_ODR &= ~(1 << CHARGED_PC);
  GPIOC_ODR &= ~(1 << MEH_PC);
  GPIOD_ODR &= ~(1 << LOW_PD);

  // Start up the ADC. (Pin D2 is channel 3)
  ADC1_CSR  &= ~(0xF0);
  ADC1_CSR  |=  (0x03);
  // Configure the ADC to store data in LSB format.
  ADC1_CR2  |=  (0x08);
  // Turn the ADC on.
  ADC1_CR1  |=  (0x01);

  #ifdef OLED_ATTACHED
    // Set up the I2C peripheral.
    i2c_init();

    // Send a simple SSD1306 initialization sequence.
    i2c_start();
    // Send the device address, and say we'll be transmitting.
    i2c_set_addr(0x78 | I2C_TXQ);
    // Send a byte to indicate 'data or command'.
    i2c_cmd_txq();

    // Send init commands; TODO, better documentation.
    // But this turns on the display with a 128x64-pixel window.
    i2c_w(0xD5);
    i2c_w(0x80);
    i2c_w(0xA8);
    i2c_w(0x3F);
    i2c_w(0xD3);
    i2c_w(0x00);
    i2c_w(0x40);
    i2c_w(0x8D);
    i2c_w(0x14);
    i2c_w(0x20);
    i2c_w(0x00);
    i2c_w(0xA1);
    i2c_w(0xC8);
    i2c_w(0xDA);
    i2c_w(0x12);
    i2c_w(0x81);
    i2c_w(0xCF);
    i2c_w(0xD9);
    i2c_w(0xF1);
    i2c_w(0xD8);
    i2c_w(0x40);
    i2c_w(0xA4);
    i2c_w(0xA6);
    i2c_w(0xAF);
    // (Done)
    i2c_stop();
  #endif
  
  // Main loop.
  while (1) {
    // Read the ADC value.
    adc_r = ADC_read();
    // Translate that into a voltage.
    batt_v = ((float)adc_r * 3.3) / 1024.0;
    // Compare the read voltage against target voltages.
    // TODO: Take a few ADC readings and average them?
    GPIOC_ODR &= ~(1 << CHARGED_PC);
    GPIOC_ODR &= ~(1 << MEH_PC);
    GPIOD_ODR &= ~(1 << LOW_PD);
    if (batt_v >= GOOD_CHARGE_VOLTAGE) {
      GPIOC_ODR |=  (1 << CHARGED_PC);
    }
    else if (batt_v >= MEH_CHARGE_VOLTAGE) {
      GPIOC_ODR |=  (1 << MEH_PC);
    }
    else {
      GPIOD_ODR |=  (1 << LOW_PD);
    }

    #ifdef OLED_ATTACHED
      // Clear the display.
      ssd1306_clear_fb();
      // Draw the voltage.
      ssd1306_draw_float(54, 3, batt_v);
      // Draw the framebuffer.
      ssd1306_draw_fb();
    #endif

    // Delay a bit.
    for (delay_i = 0; delay_i < (F_CPU / 18000) * DELAY_MS; ++delay_i) {
      __asm__("NOP");
    }
  }
}
