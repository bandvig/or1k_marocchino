#!/bin/sh

export PATH=$HOME/tools/bin:$PATH

verilator --lint-only rtl/verilog/*.v rtl/verilog/pfpu_marocchino/*.v +incdir+rtl/verilog
