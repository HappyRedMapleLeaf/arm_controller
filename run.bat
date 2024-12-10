@echo off
cd Release
make all || exit /b
cd ..
usbipd attach --wsl --busid 2-2
wsl -e openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program /mnt/c/Users/evanl/STM32CubeIDE/goofing/arm/Release/arm.elf verify reset exit"