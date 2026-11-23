# ============================================================================
# 3-SIPP-ESR Top-Level Makefile
# ============================================================================

SHELL := /usr/bin/env bash
PYTHON ?= python3

.PHONY: all sim cosim rvfi zcb trig debug priv axi rtos rtos-suite arch test bench coremark dhrystone fpga clean distclean help

all: sim

sim:
	./build.sh sim

bench: coremark dhrystone

coremark:
	cd coremark && ./run_coremark.sh

dhrystone:
	cd dhrystone && ./run_dhrystone.sh

cosim:
	./build.sh cosim

rvfi:
	./build.sh rvfi

zcb:
	./build.sh zcb

trig:
	./build.sh trig

priv:
	./build.sh priv

axi:
	./build.sh axi

rtos:
	./build.sh rtos

rtos-suite:
	./build.sh rtos-suite

arch:
	$(MAKE) -C arch-test test

test: sim arch

fpga:
	./build.sh fpga

clean:
	./build.sh clean
	$(MAKE) -C arch-test clean

distclean: clean
	$(MAKE) -C arch-test distclean

help:
	@echo "3-SIPP-ESR Build System"
	@echo "Targets:"
	@echo "  make sim        - Run baseline smoke test in Verilator"
	@echo "  make bench      - Run CoreMark and Dhrystone 2.1 benchmarks"
	@echo "  make coremark   - Run CoreMark benchmark"
	@echo "  make dhrystone  - Run Dhrystone 2.1 benchmark"
	@echo "  make arch       - Clone and run official RISC-V Architectural Compliance suite"
	@echo "  make test       - Run smoke test followed by architectural tests"
	@echo "  make rtos       - Build and run FreeRTOS demo"
	@echo "  make rtos-suite - Run full 5-stage RTOS verification suite"
	@echo "  make cosim      - Co-simulate against golden model"
	@echo "  make rvfi       - Run RISC-V Formal Interface check"
	@echo "  make zcb        - Run directed Zcb extension tests"
	@echo "  make trig       - Run Sdtrig hardware trigger tests"
	@echo "  make clean      - Remove build and simulation artifacts"
