> Man's dearest possession is life. It is given to him but once, and he must live it so as to feel no torturing regrets for wasted years, never know the burning shame of a mean and petty past. 

# FlightController
A simple flight controller for fixed-wing plane.
![alt text](https://github.com/atomvn/FlightController/blob/develop/asset/su27.jpg)

# How to install and run the project
This section provides a list of prerequisites for this project as well as instructions on running it.    
## Prerequisites   
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

## Running
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
## Hardware mapping
![alt text](https://github.com/atomvn/FlightController/blob/develop/asset/hardware_mapping.jpg)

# Roadmap
- [x] Ability to communicate with MPU6050
- [x] Read MCRE7v2
- [x] Apply Kalman filter to estimate angle from MPU6050
- [x] Control brushless and servo motor
- [x] Implement PID algorithm for outer loop (angle error correction)
- [x] Implement PID algorithm for inner loop (angular rate error correction)
- [x] Implement normal flight mode besides balancing mode
- [ ] Add GPS: change status to pending because neo 7m gps cannt be used indoor
- [x] Implement auto takeoff feature
- [ ] Implement return to home feature

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



