# Overview

This project uses the STM8S003F3 chip to monitor the charge voltage of an NiMH battery, and output to 'good/okay/low' charge indicator LEDs. They're used in a 'charge checker' which fits on a board that is about the size of a AAA battery.

It's just a quick first draft, but this design does seem to work, using a 3-pin step-up converter from Pololu:

https://oshpark.com/shared_projects/r9QXtg9v

https://www.pololu.com/product/2563

I haven't tested the I2C peripheral yet, but the 4 pins along the bottom are for an SSD1306 OLED display, to display the actual Analog-Digital Converter reading.

Thanks to 'lujji' for a great overview of bare-metal programming on these chips, and some core registers/peripherals/etc:

https://lujji.github.io/blog/bare-metal-programming-stm8/

It's hard to justify using a microcontroller like the STM8 - there is no GCC toolchain available for it, and the small 20-pin chips have no analog reference pin. Since a switching DC/DC converter is used to generate 3.3V from a single NiMH cell in this application, that is not ideal.

But they are simple, cheap, and I had some lying around.

# Running it

Build with SDCC:

`sdcc -lstm8 -mstm8 --out-fmt-ihx --std-sdcc11 main.c`

Flash with the 'stm8flash' project and a generic STLink/v2 module:

`stm8flash -c stlinkv2 -p stm8s003f3 -w main.ihx`

If the chip is write-protected and the flash operation times out, try resetting it:

`echo "00 00 ff 00 ff 00 ff 00 ff 00 ff" | xxd -r -p > defaults.bin`

`stm8flash -c stlinkv2 -p stm8s003f3 -s opt -w defaults.bin`

From: https://github.com/vdudouyt/stm8flash/issues/38

I haven't figured out on-chip debugging with OpenOCD yet.
