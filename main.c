#include <stdint.h>

// CPU speed, in Hz.
#define F_CPU     2000000UL

// STM8S registers.
// GPIO
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

// Pin definitions.
#define CHARGED_PC  (6)
#define MEH_PC      (7)
#define LOW_PD      (3)
#define BATT_PD     (2)
// Global values.
#define DELAY_MS    (200)
#define GOOD_CHARGE_VOLTAGE (1.4)
#define MEH_CHARGE_VOLTAGE  (1.25)

// Declare some values before 'main'
uint32_t delay_i;
uint16_t adc_r;
float    batt_v;

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

void main() {
  // Set led pins on ports C and D to output mode.
  GPIOC_DDR |=  (1 << CHARGED_PC |
                 1 << MEH_PC);
  GPIOD_DDR |=  (1 << LOW_PD);
  // Set LED pins to push-pull mode.
  GPIOC_CR1 |=  (1 << CHARGED_PC |
                 1 << MEH_PC);
  GPIOD_CR1 |=  (1 << LOW_PD);

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

    // Delay a bit.
    for (delay_i = 0; delay_i < (F_CPU / 18000) * DELAY_MS; ++delay_i) {
      __asm__("NOP");
    }
  }
}
