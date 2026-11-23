<div align="center">

# 3-SIPP-ESR

<p align="center">
  <img src="https://img.shields.io/badge/ISA-RV32IMACB-green" alt="ISA"/>
  <img src="https://img.shields.io/badge/Extensions-Zcb_Zbc_Zicsr-blue" alt="Extensions"/>
  <img src="https://img.shields.io/badge/Architecture-3--Stage%20Pipelined-blue" alt="Architecture"/>
  <img src="https://img.shields.io/badge/License-MIT-green" alt="License"/>
  <a href="https://www.linkedin.com/company/open-risc-v/"><img src="https://img.shields.io/badge/LinkedIn-Follow-0077B5" alt="LinkedIn Follow"/></a>
  <a href="https://or5.org"><img src="https://img.shields.io/badge/Visit-or5.org-brightgreen?logo=google-chrome" alt="Visit or5.org"/></a>
</p>

</div>

A fast, lightweight, high-IPC **RV32IMACB** 3-stage pipelined processor core in synthesizable SystemVerilog. Built around an in-order pipeline with zero load-use penalty, a 256-entry gshare branch predictor with 64-entry BTB, hardware misaligned memory support, Smepmp security, dual hardware watchpoint triggers (`Sdtrig`), and a verified FreeRTOS port, 3-SIPP-ESR targets hardened real-time embedded systems and high-throughput IoT edge platforms.

<!-- Architectural diagram on hold per project roadmap -->

---

## Overview

| | |
| :--- | :--- |
| **ISA** | `RV32IMACB_Zicsr_Zcb_Zbc_Zifencei` |
| **Pipeline** | 3-stage in-order (`Fetch` → `Execute` → `Memory-Writeback`) |
| **Branch Prediction** | 256-entry gshare + 64-entry BTB + Return Address Stack (RAS) |
| **Forwarding & Hazards**| Direct WB→EX forwarding with zero load-use stalls |
| **Security & Privilege**| Machine (M) and User (U) privilege modes, 8-region PMP with Smepmp |
| **Hardware Triggers** | Dual hardware breakpoint/watchpoint triggers (`Sdtrig`, `mcontrol6`) |
| **Bus Interfaces** | Native memory port + AXI4-Lite master bridge |
| **Debug** | RISC-V External Debug via JTAG (debug spec 0.13, DTM + DM) |
| **RTOS Support** | Validated FreeRTOS port with 63-cycle deterministic interrupt latency |
| **Arch Compliance** | Official RISC-V International Architectural Compliance Suite (`riscv-arch-test`) |
| **Target FPGA** | AMD Xilinx Artix-7 100T (Digilent Arty A7) |

---

## Features

- **Zero Load-Use Penalty:** Results are forwarded directly from writeback to execute, eliminating load-use stalls. Multiplies, divides, and hardware-resolved misaligned accesses are the only pipeline stalls.
- **Comprehensive Standard Extensions:** Complete support for `M` (hardware multiply/divide), `A` (atomic memory operations), `C` (compressed), `B` (Bitmanip `Zba`/`Zbb`/`Zbs`/`Zbc`), `Zcb`, and `Zifencei`.
- **Two-Tier Trust Model (`SECURE` Build):** Optional `SECURE` configuration adds Machine + User privilege rings, an 8-region PMP unit with Smepmp rules, and user-level trap delegation.
- **Hardware Breakpoint / Watchpoint Triggers:** Dual `Sdtrig` (`mcontrol6`) triggers: one for instruction address matching and one for load/store data address monitoring.
- **Hardware-Resolved Misalignment:** Unaligned memory accesses complete natively in hardware across two memory beats without raising software traps.
- **RISC-V External Debug over JTAG:** JTAG Debug Transport Module (DTM) and Debug Module (0.13 spec) supporting halt, register and memory inspection, resume, and single-step.
- **Hardened FreeRTOS Preemptive Multi-Tasking:** Verified FreeRTOS kernel with deterministic 63-cycle interrupt latency and 316-cycle task switch time.
- **Clean Synthesizable RTL:** Pure SystemVerilog design with no proprietary IP, tested and verified across Icarus Verilog, Verilator, and Vivado.

---

## Resource Utilization

Synthesized and implemented with **AMD Vivado 2023.2** targeting the **AMD Xilinx Artix-7 100T** (`xc7a100tcsg324-1`) on the Digilent Arty A7 evaluation board.

| Configuration | Slice LUTs | Registers (FF) | DSP48E1 | BRAM (36Kb) | Target Platform |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Core Only (Standalone OOC, M-mode)** | 7,418 | 2,092 | 12 | 0 | — |
| **Core Only (Secure M+U mode)** | 8,647 | 2,993 | 12 | 0 | — |
| **Full Evaluation SoC (Arty A7 Top)** | 6,646 | 2,147 | 12 | 8 | Digilent Arty A7 |

*Full Evaluation SoC includes the core, CLINT timer, UART controller, GPIOs, and 32 KB dedicated true dual-port Block RAM.*

---

## Benchmark Metrics

Measured in cycle-accurate Verilator simulation compiled with `riscv-none-elf-gcc -O2 -march=rv32imacb_zicsr_zcb_zbc -mabi=ilp32`.

| Benchmark | Workload | Status | Result / Metric |
| :--- | :--- | :---: | :--- |
| **CoreMark 1.0** | 100 iterations | **PASS** | **2.92 CoreMark/MHz** (CRCs Validated) |
| **Dhrystone 2.1** | 2,000,000 runs | **PASS** | **1.26 DMIPS/MHz** (222,222 Dhrystones/sec @ 100 MHz) |

```bash
# Run CoreMark simulation via Verilator
cd coremark && ./run_coremark.sh

# Run Dhrystone simulation via Verilator
cd dhrystone && ./run_dhrystone.sh
```

Detailed results, compiler flags, and methodology: **[benchmarks.md](benchmarks.md)**.

---

## Requirements

| Tool | Version | Purpose |
| :--- | :--- | :--- |
| **Verilator** | 5.018 or later | Cycle-accurate RTL simulation for benchmarks & verification |
| **Icarus Verilog** | 12.0 or later | Core simulation (`iverilog` / `vvp`) |
| **Python** | 3.10 or later | Test-program builders, co-simulation, report generation |
| **RISC-V GCC** | 13.0 or later (`riscv-none-elf-*` or `riscv64-unknown-elf-*`) | Compiling RTOS firmware, test suites, and benchmarks |
| **Vivado** | 2023.2 *(optional)* | FPGA synthesis and implementation for Arty A7 |

Set `RISCV_TC=/path/to/toolchain/bin` to point at a non-`PATH` toolchain.

---

## Build & Test

A single top-level **Makefile** covers standard workflows. For fine-grained simulation control, every action is also accessible via `build.sh`.

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
./build.sh sim        # smoke test
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
./build.sh clean      # remove build artifacts
./run_tests.sh        # full regression battery checked against tests/expected.txt
```

### What each test covers

| Test | Target | What it covers |
| :--- | :--- | :--- |
| `sim` | Smoke test | ALU operations, multiply/divide, branches, load/store, CSR access, and traps |
| `cosim` | Co-simulation | Every retired instruction compared against a golden RV32IM ISA model |
| `rvfi` | Formal trace | RISC-V Formal Interface trace invariants checked over the smoke program |
| `zcb` | Zcb extension | All Zcb compressed instructions, plus a negative control |
| `trig` | Triggers | Hardware breakpoint and watchpoint triggers (`Sdtrig`) — hit and miss cases |
| `debug` | Debug Module | JTAG DTM and Debug Module: halt, register/memory access, resume, single-step |
| `priv` | Privilege | 24 SECURE-mode tests covering U-mode traps, PMP faults, and Smepmp |
| `axi` | AXI4-Lite bridge | AXI4-Lite master bridge against slave BFM, including SLVERR handling |
| `fpga` | SoC simulation | Board-level SoC simulation: UART startup banner and LED blink |
| `arch` | Architecture | Official RISC-V Architectural Compliance Suite (`riscv-arch-test`) |

Full pass/fail results and per-test details are in **[tests.md](tests.md)**.

---

## RISC-V Architectural Compliance

Verified against the official **RISC-V International Architectural Compliance Test Suite** (`riscv-arch-test`):

### Official RISC-V Architecture Tests (`riscv-arch-test`)

```bash
make arch                                           # clones & runs in one step
# or run individual suites:
python3 arch-test/run_arch_test.py --suite I        # RV32I base integer
python3 arch-test/run_arch_test.py --suite M        # Multiply & divide
python3 arch-test/run_arch_test.py --suite C        # Compressed
python3 arch-test/run_arch_test.py --suite Zifencei # Instruction-fetch fence
python3 arch-test/run_arch_test.py --suite privilege # Traps & misalignment
```

| Suite | Tests | Result |
| :--- | :---: | :---: |
| RV32I — Base Integer | 38 | **38/38 PASS** |
| RV32M — Multiply & Divide | 8 | **8/8 PASS** |
| RV32C — Compressed Instructions | 27 | **27/27 PASS** |
| Zifencei — Instruction-Fetch Fence | 1 | **1/1 PASS** |
| Privilege — Traps, `ecall`/`ebreak`, Misalignment | 16 | **16/16 PASS** |
| **Total** | **90** | **90/90 PASS (100% Passing)** |

---

## FreeRTOS & RTOS Verification

FreeRTOS port verified across 5 engineering stages in cycle-accurate Verilator simulation:

| Stage | What is Verified | Result |
| :--- | :--- | :---: |
| **1 — Baseline Demo** | Preemptive tick, queue FIFO ordering, semaphores, preemption negative control | **PASS** |
| **2 — Smoke Suite** | Multi-tasking, preemption, register state preservation across switches, heap/stack safety | **PASS** |
| **3 — Interrupts** | CLINT timer, software interrupt, ISR notifications, 10 competing interrupts under load | **PASS** |
| **4 — Latency** | Interrupt latency (63 cycles, zero jitter) and scheduler latency (316 cycles) | **PASS** |
| **5 — Stress & RTA** | Three-task workload with mutex contention; 0 deadline misses on nominal run | **PASS** |

Detailed results and reproduction guide: [`rtos/README.md`](rtos/README.md).

---

## Repository Layout

```
3-SIPP-ESR/
├── Makefile                    top-level build entry point
├── README.md                   core overview, specifications, and documentation
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
│   └── target/three_sipp_es/   target port (linker script, entry, signature macros)
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

Released under the [MIT License](LICENSE) — free for commercial, research, and academic use.
