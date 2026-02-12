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
- Texane stlink to flash, installation guide (v1.7.0 for Ubuntu 20.04): 
```bash
https://github.com/texane/stlink
```
- FreeRTOS, download source from:
```bash
https://github.com/freelamb/stm32f10x_makefile_freertos/tree/master?tab=readme-ov-file
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

# Common errors on Ubuntu 20.04 dev enviroment
1. st-flash    
st-flash: symbol lookup error: st-flash: undefined symbol: stlink_fwrite_option_bytes_32bit   
How to fix: 
```bash
which st-flash
rm -rf st-flash bin folder
```
Install st-flash from beginning: 
```bash
it clone https://github.com/stlink-org/stlink.git
cd stlink
git checkout v1.7.0
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make -j$(nproc)
sudo make install
sudo ldconfig
```
2. 



