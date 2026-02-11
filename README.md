> Man's dearest possession is life. It is given to him but once, and he must live it so as to feel no torturing regrets for wasted years, never know the burning shame of a mean and petty past. 

# FlightController
A simple flight controller for fixed-wing plane.

# How to install and run the project
This section provides a list of prerequisites for this project as well as instructions on running it.
##Prerequisites
- GNU ARM GCC
```bash
sudo apt install gcc-arm-none-eabi
```
- Texane stlink to flash
```bash
Installation guide: https://github.com/texane/stlink
```
- FreeRTOS
```bash
Download source from: https://github.com/freelamb/stm32f10x_makefile_freertos/tree/master?tab=readme-ov-file
```

##Running
- Build the code 
```bash
make
```
- Flash the code to MCU
```bash
make flash
```
- Erase MCU flash
```bash
make erase
```

# Roadmap
- [x] Write guiding readme document
- [x] Add startup file
- [ ] Write blink led app to test the project's functionality

