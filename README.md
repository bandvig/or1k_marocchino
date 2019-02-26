# *or1k_marocchino* - OpenRISC processor IP core based on Tomasulo algorithm

## The Basics

This repository contains an OpenRISC 1000 compliant processor IP core.
The core:
 1) Implements a variant of Tomasulo algorithm
 2) Supports single and double precision floating point instructions
 3) Has got separate CPU and Wishbone clocks assuming that
    CPU clock could be great or equal to Wishbone one but they
    must be aligned
 4) Supports atomic instructions (load linked / store conditional)
 5) Supports data bus snoop and shadow registers set for multicore systems

It is written in Verilog HDL.

This repository only contains the IP source code and some documentation. For
a verification environment, please see other projects.

## License

This project is licensed under the Open Hardware Description License (OHDL). For
details please see the [LICENSE](./LICENSE) file or http://juliusbaxter.net/ohdl/
