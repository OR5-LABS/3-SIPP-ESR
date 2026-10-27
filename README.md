<div align="center">

# 3-SIPP-ESR

**Fast, Lightweight 3-Stage 32-bit RISC-V Processor Core**

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-MIT-yellow" alt="License: MIT"/></a>
  <a href="https://www.linkedin.com/company/open-risc-v/"><img src="https://img.shields.io/badge/LinkedIn-Follow-0077B5?logo=linkedin&logoColor=white" alt="LinkedIn Follow"/></a>
  <a href="https://or5.org"><img src="https://img.shields.io/badge/Visit-or5.org-purple?logo=google-chrome&logoColor=white" alt="Visit or5.org"/></a>
</p>

</div>

---

**3-SIPP-ESR** is a fast, lightweight 32-bit RISC-V processor core written in SystemVerilog. It uses a 3-stage pipeline with branch prediction to achieve high performance while staying small, efficient, and easy to integrate.

---


![diagram](diagram.svg)

## Overview

| | |
| :--- | :--- |
| **ISA** | `RV32IMACB_Zicsr_Zcb_Zbc_Zifencei` |
| **Pipeline** | 3-stage in-order (Fetch / Execute / Memory-Writeback) |
| **Branch Prediction** | 256-entry gshare + 64-entry BTB + RAS |
| **Bus Interfaces** | Native + AXI4-Lite master |
| **Debug** | RISC-V External Debug via JTAG (debug spec 0.13) |
| **RTOS** | Verified FreeRTOS port |
| **Arch Compliance** | RISC-V International official test suite |
| **Target FPGA** | Artix-7 100T @ 25 MHz (Digilent Arty A7) |

---

## Features

- Results are forwarded directly from write-back to execute, so there is no load-use stall — only multiplies, divides, and hardware-resolved misaligned accesses stall the pipeline
- Ships with `M` (multiply/divide), `A` (atomics), `C` (compressed), `B` (`Zba`/`Zbb`/`Zbs`/`Zbc`), `Zcb`, and `Zifencei`
- Optional `SECURE` build adds Machine + User privilege modes, an 8-region PMP with Smepmp, and user-level trap delegation
- Two hardware breakpoint/watchpoint triggers (`Sdtrig`, `mcontrol6`): one for instruction addresses and one for load/store addresses
- Unaligned loads and stores are handled in hardware — they complete normally without trapping
- Full RISC-V external debug support over JTAG: halt, inspect registers and memory, resume, and single-step
- Runs FreeRTOS with preemptive multitasking; interrupt latency is 63 cycles and deterministic
- Plain synthesizable SystemVerilog with no proprietary IP, tested on Artix-7 and compatible with standard FPGA toolflows

---

## Resource Utilization

Synthesized and implemented with **AMD Vivado 2023.2** targeting the **Xilinx Artix-7 100T** (`xc7a100tcsg324-1`) on the Digilent Arty A7 evaluation board.

| Configuration | Slice LUTs | Registers (FF) | DSP48E1 | BRAM (36Kb) |
| :--- | :---: | :---: | :---: | :---: |
| Core Only (Standalone OOC, M-mode) | 7,418 | 2,092 | 12 | 0 |
| Core Only (Secure M+U mode) | 8,647 | 2,993 | 12 | 0 |
| Full Evaluation SoC (Arty A7 Top) | 6,646 (10.5%) | 2,147 (1.7%) | 12 (5.0%) | 8 (5.9%) |

*Full Evaluation SoC includes the processor core, CLINT timer, UART controller, GPIOs, and 32 KB dedicated true dual-port Block RAM.*

Enable the secure configuration: `SECURE` RTL parameter or compile-time `-DTHREE_SIPP_ES_SECURE`.

---

## Benchmark Metrics

Measured in Verilator simulation (100 MHz equivalent clock).

| Benchmark | Result |
| :--- | :--- |
| CoreMark (100 iterations) | **2.92 CoreMark/MHz** |
| Dhrystone 2.1 (2,000,000 runs) | **1.26 DMIPS/MHz** |

```bash
cd coremark  && ./run_coremark.sh
cd dhrystone && ./run_dhrystone.sh
```

Detailed results, compiler flags, and methodology: **[benchmarks.md](benchmarks.md)**

---

## Requirements

| Tool | Version |
| :--- | :--- |
| Verilator | 5.0 or later |
| Python | 3.10 or later |
| RISC-V GCC | `riscv-none-elf-*` or `riscv64-unknown-elf-*` (needed for RTOS, benchmarks, arch-test) |
| Vivado | 2023.2 (optional, FPGA implementation only) |

Set `RISCV_TC=/path/to/toolchain/bin` to point at a non-`PATH` toolchain.

---

## Build & Test

A single top-level **Makefile** covers the most common workflows. For running individual workflows use `build.sh`.

### Makefile targets

```bash
make              # compile RTL + run self-checking smoke test
make arch         # RISC-V Architectural Compliance suite — clones upstream on first run
make rtos         # FreeRTOS preemptive demo + negative control
make rtos-suite   # full 5-stage RTOS verification battery
make clean        # remove simulation and build artifacts
make distclean    # also removes the cloned riscv-arch-test upstream
make help         # list all targets
```

### All `build.sh` actions

```bash
./build.sh            # smoke test
./build.sh cosim      # co-simulate against the golden RV32IM ISA model
./build.sh rvfi       # RISC-V Formal Interface trace self-check
./build.sh zcb        # directed Zcb tests + negative control
./build.sh trig       # Sdtrig hardware trigger tests
./build.sh debug      # JTAG DTM + Debug Module
./build.sh priv       # SECURE config: M/U privilege, PMP, Smepmp, triggers
./build.sh axi        # AXI4-Lite master bridge BFM test
./build.sh fpga       # FPGA SoC simulation (UART banner + LED blink)
./build.sh arch       # RISC-V Architectural Compliance suite
./build.sh rtos       # FreeRTOS demo
./build.sh rtos-suite # full 5-stage RTOS verification suite
./build.sh clean
./run_tests.sh        # all of the above (except RTOS/benchmarks), checked against tests/expected.txt
```

Every action exits non-zero if the testbench reports `FAIL`, `TIMEOUT`, `MISMATCH`, or does not print its `PASS` verdict.

### What each test checks

| Test | What it covers |
| :--- | :--- |
| `sim` | ALU operations, multiply/divide, branches, load/store, CSR access, and trap handling |
| `cosim` | Every retired instruction compared against a golden RV32IM ISA model |
| `rvfi` | RISC-V Formal Interface trace invariants checked over the smoke program |
| `zcb` | All Zcb compressed instructions, plus a negative control that must fail |
| `trig` | Hardware breakpoint and watchpoint triggers — both hit and miss cases |
| `debug` | JTAG DTM and Debug Module: halt, register and memory access, resume, single-step |
| `priv` | 24 SECURE-mode tests covering U-mode traps, PMP faults, Smepmp lock-down, and `mtvec` |
| `axi` | AXI4-Lite master bridge against a slave BFM, including SLVERR handling |
| `fpga` | FPGA SoC simulation: UART startup banner and LED blink |
| `arch` | Official RISC-V Architectural Compliance Suite — full results in [tests.md](tests.md) |

Full pass/fail results and per-test details are in **[tests.md](tests.md)**.

---

## RISC-V Architectural Compliance

Verified against the official **RISC-V International Architectural Compliance Test Suite** (`riscv-arch-test`). The upstream test suite is fetched automatically on first run — nothing to install manually.

```bash
make arch                                           # recommended — clones & runs in one step
./build.sh arch                                     # or via build.sh

# Run individual suites or a single test:
python3 arch-test/run_arch_test.py --suite I        # RV32I base integer
python3 arch-test/run_arch_test.py --suite M        # Multiply & divide
python3 arch-test/run_arch_test.py --suite C        # Compressed
python3 arch-test/run_arch_test.py --suite Zifencei # Instruction-fetch fence
python3 arch-test/run_arch_test.py --suite privilege # Traps & misalignment
python3 arch-test/run_arch_test.py --test jal-01    # single test by name
```

| Suite | Tests |
| :--- | :---: |
| RV32I — Base Integer | 38 |
| RV32M — Multiply & Divide | 8 |
| RV32C — Compressed Instructions | 27 |
| Zifencei — Instruction-Fetch Fence | 1 |
| Privilege — Traps, `ecall`/`ebreak`, Misalignment | 16 |
| **Total** | **90** |

The `arch-test/` directory contains only the target port (`target/three_sipp_es/`) and the test runner (`run_arch_test.py`). The upstream RISC-V test suite is cloned on demand and is gitignored.

---

## FreeRTOS & RTOS Verification

FreeRTOS port verified across 5 engineering stages in Verilator simulation.

```bash
make rtos           # quickstart demo + negative control
make rtos-suite     # full 5-stage battery
# or:
./build.sh rtos
./build.sh rtos-suite
```

| Stage | What is Verified |
| :--- | :--- |
| 1 — Baseline Demo | Preemptive tick, queue FIFO ordering, semaphores, and a negative control that confirms preemption is load-bearing |
| 2 — Smoke Suite | Multi-tasking, preemption, register state preservation across context switches, heap and stack safety |
| 3 — Interrupts | CLINT timer, machine software interrupt, ISR task notifications, and 10 competing interrupts under tick load |
| 4 — Latency | Cycle-accurate measurement of interrupt latency (63 cycles, zero jitter) and scheduler latency (316 cycles) |
| 5 — Stress & RTA | Three-task workload with mutex contention; positive run shows zero deadline misses, overloaded negative control reports 3 |

Detailed results and reproduction guide: [`rtos/README.md`](rtos/README.md)

---

## Repository Layout

```
3-SIPP-ESR/
├── Makefile                    top-level build entry point
├── build.sh                    fine-grained build & test driver
├── run_tests.sh                regression runner (checks against tests/expected.txt)
├── rtl/
│   ├── three_sipp_es_core.sv   3-stage pipeline, predictor, forwarding
│   ├── three_sipp_es_soc.sv    minimal SoC (IMEM, DRAM, CLINT, UART, tohost)
│   ├── three_sipp_es_debug.sv  JTAG DTM + RISC-V Debug Module
│   ├── three_sipp_es_axi_lite.sv  AXI4-Lite master bridge
│   ├── three_sipp_es_uart.sv   UART peripheral
│   └── common/                 shared datapath leaf cells (ALU, MulDiv, RegFile,
│                               CSR, RVC, ImmGen, Branch, Decode, PMP)
├── arch-test/
│   ├── Makefile                clone-on-demand + run targets
│   ├── run_arch_test.py        multi-threaded compliance test runner
│   └── target/three_sipp_es/  target port (linker script, entry, signature macros)
├── tb/                         testbenches (smoke, RVFI, debug, priv, AXI, arch)
├── rtos/                       FreeRTOS port (kernel, BSP, verification apps)
├── coremark/                   CoreMark bare-metal port + run scripts
├── dhrystone/                  Dhrystone 2.1 bare-metal port + run scripts
├── fpga/                       FPGA SoC wrapper + Arty A7 XDC
├── programs/                   test-program builders
├── sw/                         assembly programs & bring-up firmware
├── tools/                      golden ISA model + co-simulation driver
├── docs/                       MkDocs documentation site
└── tests/                      regression golden output (expected.txt)
```

---

## License

Released under the [MIT License](LICENSE).
