# SVF parser

Writing from scratch 1-pass SVF parser that minimizes its memory
requirements (microcontroller friendly). Data stream is coming
from the network in packets. Each packet is passed to the parser
which retains internal state and bitbangs data to JTAG.

