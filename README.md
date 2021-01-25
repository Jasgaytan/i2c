# Library for I2C

Program that allows handling three I2C devices, which are a memory (24LC512), a temperature sensor (MLX90614), and a temperature and relative humidity sensor (SHT3x). Was developed with launchpad MSP430FR4133, making it easy to use the push buttons that include it (P1.2 and P2.6). When a push button is pressed the value corresponding to one sensor is displayed by the LCD screen, the same if the other button is pressed, and when the two push buttons are pressed the value is saved in the memory and is indicated on the screen. If the memory fails also this is indicated on the LCD screen,

This library is not optimal, so can be optime it. Feel free to do it.

### Developed in:
<p>
<img width="30" height="30" src="https://raw.githubusercontent.com/jesu95/jesu95/main/img/c-original.svg">
<img width="30" height="30" src="https://raw.githubusercontent.com/jesu95/jesu95/main/img/msp430.png">
<img width="30" height="30" src="https://raw.githubusercontent.com/jesu95/jesu95/main/img/ccs.png">
</p>

### Authors:

* [Matías López](https://github.com/matiflp/)
* [Jesús López](https://github.com/jesu95/)
