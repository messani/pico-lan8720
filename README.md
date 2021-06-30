# pico-lan8720

This was only an experiment so code is not very clean.

This is an attempt to use RPi Pico with LAN8720. This project was inspired by https://github.com/sandeepmistry/pico-rmii-ethernet. Drawbacks of pico-rmii-ethernet are that it underclocks RPi Pico to 50MHz and ethernet only works at 10MBit. I wanted to use 100MBit ethernet and not to underclock my RPi Pico, so I decided to go another way. 

This project requires overclocking RPi Pico to at least 250MHz. According to various sources the RPi Pico should work well overclocked to this freqnency but I cannot guarantee it to you - use it at your own risk.

## Build

Clone this repository and init submodules

git submodule init
git submodule update

Use cmake or VisualStudio Code to build the project.

## How does it work

Because RPi Pico runs at higher speed (250MHz) and TX and RX loop executes in 4 cycles so it is possible to catch changes of the clock and capture 50MHz signal by PIO program.

### RX

There is a little hack inspired by pico_rmii_ethernet to detect preamble and start frame delimiter (SFD) on RX0 and RX1 pins. After SFD, data are captured while RXEN is HI and written through DMA to memory. When RXEN gets LO, interrupt is called and length of the packet is calculated by write cursor of DMA. Interrupt handler should run as short as possible so packet processing is passed to event loop.

### TX

Packet data are constructed in TX fifo including preamble, SFD and CRC (https://en.wikipedia.org/wiki/Ethernet_frame). TX PIO program waits in loop while there are no data in TX fifo. When data are in fifo, PIO program detects falling edge and pushes bits to TX0/TX1 and sets TXEN to HI. When there are no data in TX fifo, TX0/TX1 and TXEN signals go LO.

## No guarantee

I did not have a logic analyzer so this this code was created by method of trial and error. I tested it by ping (1ms interval) and by connecting to http server. There can be bugs in the code.
