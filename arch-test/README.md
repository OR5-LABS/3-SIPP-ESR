# 3-SIPP-ESR architecture tests

Runs the official RISC-V Architecture Test suite (riscv-arch-test 2.7.4, legacy
framework with in-repo reference signatures).

    ./build.sh arch        # or: make arch

The suite is cloned on demand (`git clone --branch 2.7.4`) into the gitignored
`arch-test/riscv-arch-test`; a local checkout can be linked there instead. The
suite and the signature compare are used unmodified; `target/` and
`tb/tb_three_sipp_es_arch.sv` are core-specific.

Suites run: I, M, C, privilege, Zifencei (90 tests, none excluded).
A (atomics) and B/Zb* have no suite in riscv-arch-test 2.7.4 (not covered by
this suite version).

Deviation: the core performs misaligned loads and stores in hardware, so
misalign-lw/lh/lhu/sh/sw-01 are compared against the no-trap signatures in
`HW_MISALIGNED_GOLDEN` (run_arch_test.py). Use `--no-hw-misaligned` to require
upstream's trap-based references.
