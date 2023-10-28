## Overview

An emulator for the ISIS antenna deployer system

## Repository Organization

* msp430: msp430 based antenna emulator 
* stm32: stm32 based antenna emulator 

## Implementation Details

### MSP430
The msp430 simulates both the i2c behavior and deployment system using leds and switches. Works for the part numbers msp430f6659 and msp430f5529.

### STM32
The stm32 simulates i2c behavior only, using constants to emulate sensor readings. It was designed for the part number stm32f103c8

### For platform specific details look at the respective README files



            


            
