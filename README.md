# CIRQ — Circuit Instruction & Register Qualification Language
## Version 2.0 — Fully Independent HDL

CIRQ is its own hardware description language. **Not Verilog. Not VHDL.**
Zero syntax borrowed from either. CIRQ compiles directly to its own binary
format (`.cqbit`) and programs FPGAs natively.

---

## The Full Pipeline

```
  you write:   cpu.cirq
                  ↓
  compiler:    cirqc cpu.cirq -o cpu.cqbit
                  ↓
  binary:      cpu.cqbit  ← CIRQ's own format, not Verilog
                  ↓
  simulate:    cqsim cpu.cqbit --rom prog.hex --cycles 100
                  ↓
  inspect:     cqdump cpu.cqbit --all
                  ↓
  flash FPGA:  cqprog flash cpu.cqbit --board ice40hx8k
                  ↓
               Real CPU running on real FPGA hardware
```

---

## Build All Tools

```bash
gcc -O2 -o compiler/cirqc  compiler/cirqc.c
gcc -O2 -o tools/cqdump    tools/cqdump.c
gcc -O2 -o tools/cqsim     tools/cqsim.c
gcc -O2 -o tools/cqprog    tools/cqprog.c
```

---

## CIRQ Syntax

```cirq
chip MyCPU {
    clock clk : 50mhz          // system clock
    reset rst : active_high    // synchronous reset

    reg R0 : 8 = 0x00          // 8-bit register, reset=0
    reg PC : 8 = 0x00

    ram IMEM : 256 x 8 = rom("program.hex")
    ram DMEM : 256 x 8

    const OP_ADD : 8 = 0x04
    state FETCH  = 0b00

    on clock.rise {            // fires every rising clock edge
        if rst == 1 {
            R0 <= 0x00         // non-blocking: real flip-flop input
            PC <= 0x00
        }
        else {
            when IR == OP_ADD {
                R0 <= R0 + R1
                PC <= PC + 1
            }
        }
    }

    out port debug_R0 : 8 = R0   // physical FPGA pin
}
```

---

## Keywords (all unique to CIRQ)

| Keyword | Meaning |
|---|---|
| `chip` | Hardware module — one chip per file |
| `reg` | D flip-flop register |
| `wire` | Combinational signal |
| `bus` | Multi-bit bus |
| `ram` | Block RAM (synchronous) |
| `clock` | System clock declaration |
| `reset` | Reset signal |
| `state` | Named state constant |
| `const` | Named constant value |
| `logic` | Combinational (no clock) gate block |
| `on clock.rise` | Clocked always block |
| `when X == Y` | Hardware MUX / conditional path |
| `if X == Y` | Same as `when` |
| `<=` | Non-blocking register assignment (flip-flop D input) |
| `out port` | Physical FPGA output pin |
| `in port` | Physical FPGA input pin |

---

## .cqbit Binary Format

CIRQ's own binary. Not Verilog. Not EDIF. Not bitstream directly —
it's the compiled, board-independent gate representation.

```
┌─────────────────────────────────────────────┐
│ Header                                      │
│   magic:   "CQBT"  (4 bytes)                │
│   version: u16                              │
│   n_cells: u32                              │
│   n_nets:  u32                              │
│   n_ports: u16                              │
├─────────────────────────────────────────────┤
│ Cell section  (16 bytes/cell)               │
│   type(u8) width(u8) a(u16) b(u16)          │
│   out(u16) sel(u16) lit(u32) flags(u8)      │
├─────────────────────────────────────────────┤
│ Net name table  (null-terminated strings)   │
├─────────────────────────────────────────────┤
│ Port table                                  │
├─────────────────────────────────────────────┤
│ Symbol table  (for debugger/simulator)      │
└─────────────────────────────────────────────┘
```

## Gate Primitives in .cqbit

| Code | Name | Meaning |
|---|---|---|
| `0x01` | DFF | D flip-flop (every `reg`) |
| `0x02` | BRAM | Block RAM (every `ram`) |
| `0x10` | ADD | Adder |
| `0x11` | SUB | Subtractor |
| `0x12` | AND | AND gate |
| `0x13` | OR | OR gate |
| `0x14` | XOR | XOR gate |
| `0x15` | NOT | Inverter |
| `0x16` | SHL | Shift left |
| `0x17` | SHR | Shift right |
| `0x20` | MUX | Multiplexer (every `when`) |
| `0x21` | EQ | Equality comparator |
| `0x30` | WIRE | Wire assignment |
| `0x31` | CONST | Constant value |
| `0x40` | MEM_RD | Memory read |
| `0x41` | MEM_WR | Memory write |
| `0x50` | OUT_PORT | FPGA output pin |
| `0x51` | IN_PORT | FPGA input pin |

---

## Tools

### cirqc — Compiler
```bash
./cirqc cpu.cirq -o cpu.cqbit        # compile
./cirqc cpu.cirq --dump              # compile + show cell listing
./cirqc cpu.cirq --tokens            # dump token list
```

### cqdump — Binary Inspector
```bash
./cqdump cpu.cqbit                   # header + stats
./cqdump cpu.cqbit --cells           # full cell listing
./cqdump cpu.cqbit --nets            # all nets/signals
./cqdump cpu.cqbit --ports           # physical I/O pins
./cqdump cpu.cqbit --syms            # symbol table
./cqdump cpu.cqbit --all             # everything
```

### cqsim — Simulator
```bash
./cqsim cpu.cqbit                             # simulate 100 cycles
./cqsim cpu.cqbit --cycles 500                # custom cycle count
./cqsim cpu.cqbit --rom program.hex --trace   # load ROM, trace every cycle
./cqsim cpu.cqbit --regs                      # dump register values at end
```

### cqprog — FPGA Programmer
```bash
./cqprog info  cpu.cqbit                      # resource estimate + board compat
./cqprog flash cpu.cqbit --board ice40hx8k    # flash to iCE40 HX8K
./cqprog flash cpu.cqbit --board ecp5-25f     # flash to ECP5
./cqprog flash cpu.cqbit --board artix7-35t   # flash to Artix-7
./cqprog flash cpu.cqbit --dry                # dry run (no USB needed)
./cqprog check                                # scan for connected boards
```

Supported boards: `ice40hx8k`, `ice40up5k`, `ecp5-25f`, `ecp5-85f`,
`artix7-35t`, `artix7-100t`, `cyclone10`

---

## Example CPU — SimpleCPU

See `examples/cpu.cirq` — a complete 8-bit CPU with:
- 7 registers (R0, R1, PC, IR, MAR, FLAGS, cpu_phase)
- 2 block RAMs (IMEM 256×8, DMEM 256×8)
- 19 opcodes (LOAD, STORE, MOVE, ADD, SUB, AND, OR, XOR, NOT, SHL, SHR,
              CLEAR, INC, DEC, JMP, JZ, JC, NOP, HALT)
- 3-phase FSM (FETCH → DECODE → EXECUTE)
- Compiles to 235 gate cells, uses 6% of iCE40 HX8K

```bash
# Compile
./compiler/cirqc examples/cpu.cirq -o examples/cpu.cqbit

# Simulate
./tools/cqsim examples/cpu.cqbit --cycles 50 --regs

# Inspect binary
./tools/cqdump examples/cpu.cqbit --stats --ports

# Flash to iCE40 HX8K
./tools/cqprog flash examples/cpu.cqbit --board ice40hx8k
```
