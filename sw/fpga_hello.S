# ============================================================================
# fpga_hello.S — 3-SIPP-ESR FPGA bring-up demo (bare-metal, RV32IMC).
#
# Prints a banner over the UART (0x1000_0000) and blinks the 8 user LEDs
# (0x2000_1000) in a loop. Self-contained: no .data/.bss, stack at top of the
# unified RAM. Links at 0x0 (core reset vector). Build: ../sw/build_fpga_hello.sh
#
# UART register map (three_sipp_es_uart):
#   0x10000000 R: status (bit0 = tx_busy)   W: tx data byte
# ============================================================================
    .section .text.init
    .globl _start
_start:
    li   sp, 0x00003FF0        # stack top (top of 16 KB RAM)
    li   s0, 0x10000000        # UART base
    li   s1, 0x20001000        # LED register
    li   s2, 0x00000001        # LED pattern

main_loop:
    la   a0, msg
    jal  ra, puts
    sw   s2, 0(s1)             # drive LEDs
    slli s2, s2, 1             # walking-1 pattern
    bnez s2, no_wrap
    li   s2, 0x00000001
no_wrap:
    li   t0, 0x00020000        # crude delay
delay:
    addi t0, t0, -1
    bnez t0, delay
    j    main_loop

# puts(a0 = char*) — poll-and-send each byte until NUL
puts:
    mv   t1, a0
puts_l:
    lbu  t2, 0(t1)
    beqz t2, puts_done
puts_wait:
    lw   t3, 0(s0)             # UART status
    andi t3, t3, 1             # tx_busy?
    bnez t3, puts_wait
    sw   t2, 0(s0)             # send byte
    addi t1, t1, 1
    j    puts_l
puts_done:
    ret

    .section .rodata
msg:
    .asciz "Hello from 3-SIPP-ESR on FPGA!\r\n"
