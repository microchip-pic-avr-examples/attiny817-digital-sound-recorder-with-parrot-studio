<a href="https://www.microchip.com" rel="nofollow"><img src="images/microchip.png" alt="MCHP" width="300"/></a>

# ATtiny817 Digital Sound Recorder using DAC with parrot

This example demonstrates a digital sound recorder using ADC for sampling and DAC for playback. Samples timed at a defined frequency, controlled by a timer and event system. SPI used to store on a dataflash, driver included. This example uses START peripheral drivers.

The example is explained in more details in the application note [AN2547](http://ww1.microchip.com/downloads/en/AppNotes/00002547A.pdf).

## Related Documentation

- [AN2547 -  Digital Sound Recorder using DAC](http://ww1.microchip.com/downloads/en/AppNotes/00002547A.pdf)
- [Parrot (Voice Recorder) with ATtiny817 Hardware Users Guide](http://ww1.microchip.com/downloads/en/DeviceDoc/40001916A.pdf)
- [ATtiny817 Product Page](https://www.microchip.com/wwwproducts/en/ATtiny817)

## Software Used

- [Atmel Studio 7.0.2397 or later](https://www.microchip.com/mplab/avr-support/atmel-studio-7)
- ATtiny_DFP 1.5.315 or later
- AVR/GNU C Compiler 5.4.0 (built into studio)

## Hardware Used

-   [AVR Parrot](https://www.microchip.com/developmenttools/ProductDetails/ATAVRPARROT)
-   [Atmel-ICE debugger](https://www.microchip.com/DevelopmentTools/ProductDetails/ATATMEL-ICE)

## Setup

- Connect usb power
- Connect a debugger of your choice

## Operation

1. Download the zip file or clone the example to get the source code.
2. Open `AVR42777Parrot.atsln` in Atmel Studio.
3. Connect the usb power and the debugger with your computer. 
4. In your menu bar in Atmel Studio go to `Debug->Start Without Debugging` or press `CTRL + ALT + F5`.
5. Use the buttons on the board to start recording, erase or start playback.

## Conclusion

We have here shown how you can use a ATtiny817 as a digital sound recorder with the parrot board.