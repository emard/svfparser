# SVF parser

Unifinished project about writing from scratch 1-pass SVF parser 
that in creates binary stream in RAM, immediately DMA'ble to JTAG.
Current source is for linux with simple libc dependency and such
should be easily portable to ESP32 arduino.
Data stream is coming from the network or SD card in packets (blocks).
Each packet is passed to the parser which keeps internal state and
bitbangs data to JTAG and frees RAM for new data.

[SVF Format spec](http://www.jtagtest.com/pdf/svf_specification.pdf)

# TODO

    [x] fill binary data ready for bitbanging
    [ ] support incomplete byte lengths 
    [ ] implement bitbanging
    [ ] output to xsvf
    [ ] output splitted commands
    [ ] overrun not reported: 29 bit length, 31 bit content
