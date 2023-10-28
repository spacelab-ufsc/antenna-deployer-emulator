# Antenna Deployer Emulator - STM32 Version

## Prerequisites 

### General Dependencies:

#### Hardware

- stm32f103c8 development kit (blue pill)
- ST-LINK/V2

#### Software
- gcc-arm-none-eabi
- stlink-tools

Platform-specific installation:

#### Ubuntu / Debian

```
$ sudo apt install stlink-tools gcc-arm-none-eabi

```
## Usage

### Hardware Requirements

Its quite simple to prepare the emulator, the only connections required are 3v3 and Gnd pins for powering,
besides the i2c related pins, SCL and SDA, which are mapped to PB6 and PB7, respectively. Also, i2c needs a 
pull-up resistor in both SCL and SDA lines, meaning that if your module isn't providing then you will need to add it aswell.


### Software Especifications

After installing the dependencies and preparing the hardware its needed to build and flash the stm32,
for that you can do as follows:

To build the emulator and libopencm3 used as a hal, you can just run:

```
$ make                                                                           
```

Finally, to flash to microcontroller its needed to connect the stm32 to your machine with the ST-LINK and run the command:

```
$ make flash  
```


