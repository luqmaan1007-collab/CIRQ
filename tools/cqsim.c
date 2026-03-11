/*
 * cqsim.c — CIRQ Simulator
 *
 * Loads a .cqbit file and simulates the hardware it describes.
 * No Verilog. No VHDL. Pure CIRQ execution.
 *
 * Simulation model:
 *   - Net values are 32-bit integers (supports up to 32-bit buses)
 *   - Each clock cycle: evaluate all combinational cells first,
 *     then latch all DFF outputs simultaneously (like real hardware)
 *   - RAM is modelled as a flat byte array
 *   - Output ports are printed every N cycles
 *
 * Build:  gcc -O2 -o cqsim cqsim.c
 * Usage:  ./cqsim cpu.cqbit
 *         ./cqsim cpu.cqbit --cycles 1000
 *         ./cqsim cpu.cqbit --trace
 *         ./cqsim cpu.cqbit --rom program.hex
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define MAX_NETS    4096
#define MAX_CELLS   4096
#define MAX_SYMS    1024
#define MAX_IDENT   128
#define RAM_SIZE    256

/* ── Cell type enum (mirrors compiler) ──────────────────────────── */
typedef enum {
    CQ_DFF=0x01, CQ_BRAM=0x02,
    CQ_ADD=0x10, CQ_SUB=0x11, CQ_AND=0x12, CQ_OR=0x13,  CQ_XOR=0x14,
    CQ_NOT=0x15, CQ_SHL=0x16, CQ_SHR=0x17, CQ_INC=0x18, CQ_DEC=0x19,
    CQ_MUX=0x20, CQ_EQ=0x21,  CQ_NEQ=0x22, CQ_LT=0x23,  CQ_GT=0x24,
    CQ_WIRE=0x30, CQ_CONST=0x31,
    CQ_MEM_RD=0x40, CQ_MEM_WR=0x41,
    CQ_OUT_PORT=0x50, CQ_IN_PORT=0x51
} CQType;

/* ── Cell ────────────────────────────────────────────────────────── */
typedef struct {
    uint8_t  type, width;
    uint16_t a, b, out, sel;
    uint32_t lit;
    uint8_t  flags, pad;
    char     name[MAX_IDENT];
} Cell;

/* ── Net ─────────────────────────────────────────────────────────── */
typedef struct {
    char     name[MAX_IDENT];
    uint8_t  width;
} Net;

/* ── Symbol ──────────────────────────────────────────────────────── */
typedef struct {
    char     name[MAX_IDENT];
    uint8_t  kind, width;
    uint16_t net_id;
    uint32_t reset;
} Sym;

/* ── Port ────────────────────────────────────────────────────────── */
typedef struct {
    char     name[MAX_IDENT];
    uint8_t  dir, width;
    uint16_t net_id;
} Port;

/* ── Simulator State ─────────────────────────────────────────────── */
typedef struct {
    /* Design */
    char    chip_name[MAX_IDENT];
    char    clock_name[MAX_IDENT];
    char    reset_name[MAX_IDENT];

    Cell    cells[MAX_CELLS]; int n_cells;
    Net     nets[MAX_NETS];   int n_nets;
    Sym     syms[MAX_SYMS];   int n_syms;
    Port    ports[64];        int n_ports;

    /* Simulation state */
    uint32_t net_val[MAX_NETS];     /* current value of each net         */
    uint32_t net_next[MAX_NETS];    /* next-cycle value (for DFFs)       */
    uint8_t  ram[16][RAM_SIZE];     /* up to 16 RAM blocks, 256 bytes ea */
    int      n_ram;                 /* how many RAMs declared             */
    char     ram_name[16][MAX_IDENT];

    /* Clock */
    uint64_t cycle;
    int      reset_active;
} Sim;

/* ── .cqbit reader ───────────────────────────────────────────────── */
static uint8_t  ru8 (FILE *f) { uint8_t  v; fread(&v,1,1,f); return v; }
static uint16_t ru16(FILE *f) { uint8_t b[2]; fread(b,1,2,f); return (uint16_t)(b[0]|(b[1]<<8)); }
static uint32_t ru32(FILE *f) { uint8_t b[4]; fread(b,1,4,f); return b[0]|(b[1]<<8)|(b[2]<<16)|(b[3]<<24); }
static void     rstr(FILE *f, char *buf, int max) {
    int i=0; char ch;
    while ((ch=(char)fgetc(f)) && i<max-1) buf[i++]=ch; buf[i]='\0';
}

static int load_cqbit(const char *path, Sim *s) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return 0; }

    char magic[5]={0}; fread(magic,1,4,f);
    if (strcmp(magic,"CQBT")!=0) { fprintf(stderr,"Bad magic\n"); fclose(f); return 0; }

    /* header */
    ru16(f); /* version */
    uint16_t clen = ru16(f);
    uint32_t nc   = ru32(f);
    uint32_t nn   = ru32(f);
    uint16_t np   = ru16(f);
    ru16(f); ru32(f); ru32(f);

    fread(s->chip_name,1,clen,f); s->chip_name[clen]='\0';
    rstr(f, s->clock_name, MAX_IDENT);
    rstr(f, s->reset_name, MAX_IDENT);

    /* cells */
    s->n_cells = (int)nc;
    for (int i=0;i<s->n_cells;i++) {
        s->cells[i].type  = ru8(f);
        s->cells[i].width = ru8(f);
        s->cells[i].a     = ru16(f);
        s->cells[i].b     = ru16(f);
        s->cells[i].out   = ru16(f);
        s->cells[i].sel   = ru16(f);
        s->cells[i].lit   = ru32(f);
        s->cells[i].flags = ru8(f);
        s->cells[i].pad   = ru8(f);
    }

    /* nets */
    s->n_nets = (int)ru16(f);
    for (int i=0;i<s->n_nets;i++) {
        s->nets[i].width = ru8(f);
        rstr(f, s->nets[i].name, MAX_IDENT);
    }

    /* ports */
    s->n_ports = (int)ru16(f);
    for (int i=0;i<s->n_ports;i++) {
        s->ports[i].dir    = ru8(f);
        s->ports[i].width  = ru8(f);
        s->ports[i].net_id = ru16(f);
        rstr(f, s->ports[i].name, MAX_IDENT);
    }

    /* syms */
    s->n_syms = (int)ru16(f);
    for (int i=0;i<s->n_syms;i++) {
        s->syms[i].kind   = ru8(f);
        s->syms[i].width  = ru8(f);
        s->syms[i].net_id = ru16(f);
        s->syms[i].reset  = ru32(f);
        rstr(f, s->syms[i].name, MAX_IDENT);
    }

    fclose(f);
    return 1;
}

/* ── Bit-width masking ───────────────────────────────────────────── */
static uint32_t mask(int width) {
    if (width >= 32) return 0xFFFFFFFF;
    return (1u << width) - 1u;
}

/* ── Find RAM index by name ──────────────────────────────────────── */
static int find_ram(Sim *s, const char *name) {
    for (int i = 0; i < s->n_ram; i++)
        if (strcmp(s->ram_name[i], name) == 0) return i;
    return -1;
}

/* ── Load hex program into RAM ───────────────────────────────────── */
static void load_hex(Sim *s, int ram_idx, const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "Warning: could not open ROM file '%s'\n", path); return; }
    int addr = 0; unsigned int val;
    while (fscanf(f, "%x", &val) == 1 && addr < RAM_SIZE)
        s->ram[ram_idx][addr++] = (uint8_t)val;
    fclose(f);
    printf("  Loaded %d bytes from %s into RAM[%d]\n", addr, path, ram_idx);
}

/* ── Apply reset values ──────────────────────────────────────────── */
static void apply_reset(Sim *s) {
    /* reset all nets to 0 */
    memset(s->net_val,  0, sizeof(s->net_val));
    memset(s->net_next, 0, sizeof(s->net_next));

    /* apply per-symbol reset values */
    for (int i = 0; i < s->n_syms; i++) {
        Sym *sym = &s->syms[i];
        if (sym->kind == 0 /* SYM_REG */) {
            s->net_val[sym->net_id] = sym->reset & mask(sym->width);
        }
    }
    /* CQ_CONST cells set their net on first eval */
}

/* ── Evaluate one cell ───────────────────────────────────────────── */
static void eval_cell(Sim *s, int ci) {
    Cell *c   = &s->cells[ci];
    uint32_t a = s->net_val[c->a];
    uint32_t b = s->net_val[c->b];
    uint32_t w = mask(c->width ? c->width : 8);
    uint32_t r = 0;

    switch (c->type) {
    case CQ_CONST:   r = c->lit & w; s->net_val[c->out] = r; return;
    case CQ_WIRE:    r = a & w;      break;
    case CQ_ADD:     r = (a + b) & w; break;
    case CQ_SUB:     r = (a - b) & w; break;
    case CQ_AND:     r = (a & b) & w; break;
    case CQ_OR:      r = (a | b) & w; break;
    case CQ_XOR:     r = (a ^ b) & w; break;
    case CQ_NOT:     r = (~a)    & w; break;
    case CQ_SHL:     r = (a << 1)& w; break;
    case CQ_SHR:     r = (a >> 1)& w; break;
    case CQ_INC:     r = (a + 1) & w; break;
    case CQ_DEC:     r = (a - 1) & w; break;
    case CQ_EQ:      r = (a == b) ? 1 : 0; break;
    case CQ_NEQ:     r = (a != b) ? 1 : 0; break;
    case CQ_LT:      r = (a <  b) ? 1 : 0; break;
    case CQ_GT:      r = (a >  b) ? 1 : 0; break;
    case CQ_MUX:     r = s->net_val[c->sel] ? b : a; break;

    case CQ_MEM_RD: {
        /* find which RAM this cell references by cell name */
        int ri = find_ram(s, c->name);
        if (ri >= 0) {
            uint32_t addr = a & 0xFF;
            r = s->ram[ri][addr] & w;
        }
        break;
    }
    case CQ_MEM_WR: {
        /* memory writes are handled in the clocked phase */
        return;
    }

    case CQ_DFF:     /* DFF outputs are latched, not combinationally driven */
        return;
    case CQ_BRAM:    return;
    case CQ_OUT_PORT: s->net_val[c->out] = a & w; return;
    default: return;
    }

    s->net_val[c->out] = r;
}

/* ── Clocked phase: latch DFFs and apply memory writes ──────────── */
static void clock_edge(Sim *s) {
    /* Evaluate all WIRE cells that feed DFF _next inputs */
    for (int i = 0; i < s->n_cells; i++) {
        Cell *c = &s->cells[i];
        if (c->type == CQ_WIRE)
            s->net_val[c->out] = s->net_val[c->a];
    }

    /* Latch DFF: Q ← _next */
    for (int i = 0; i < s->n_syms; i++) {
        Sym *sym = &s->syms[i];
        if (sym->kind != 0 /* not REG */) continue;
        char nxt[MAX_IDENT];
        snprintf(nxt, MAX_IDENT, "%s_next", sym->name);
        /* find _next net */
        for (int j = 0; j < s->n_nets; j++) {
            if (strcmp(s->nets[j].name, nxt) == 0) {
                s->net_val[sym->net_id] = s->net_val[j] & mask(sym->width);
                break;
            }
        }
    }

    /* Apply memory writes */
    for (int i = 0; i < s->n_cells; i++) {
        Cell *c = &s->cells[i];
        if (c->type != CQ_MEM_WR) continue;
        int ri = find_ram(s, c->name);
        if (ri >= 0) {
            uint32_t addr = s->net_val[c->a] & 0xFF;
            uint32_t data = s->net_val[c->b] & 0xFF;
            s->ram[ri][addr] = (uint8_t)data;
        }
    }
}

/* ── Full combinational evaluation pass ─────────────────────────── */
static void eval_all(Sim *s) {
    /* Multiple passes to propagate through combinational chains */
    for (int pass = 0; pass < 4; pass++) {
        for (int i = 0; i < s->n_cells; i++) {
            Cell *c = &s->cells[i];
            if (c->type != CQ_DFF && c->type != CQ_BRAM)
                eval_cell(s, i);
        }
    }
}

/* ── Print port values ───────────────────────────────────────────── */
static void print_ports(Sim *s) {
    printf("  Cycle %-6llu  │", (unsigned long long)s->cycle);
    for (int i = 0; i < s->n_ports; i++) {
        Port *p = &s->ports[i];
        uint32_t val = s->net_val[p->net_id];
        printf("  %s=0x%0*X", p->name, (p->width+3)/4, val & mask(p->width));
    }
    printf("\n");
}

/* ── Find net by name (for debugging) ───────────────────────────── */
static int find_net(Sim *s, const char *name) {
    for (int i = 0; i < s->n_nets; i++)
        if (strcmp(s->nets[i].name, name) == 0) return i;
    return -1;
}

static void print_registers(Sim *s) {
    printf("  Registers:\n");
    for (int i = 0; i < s->n_syms; i++) {
        Sym *sym = &s->syms[i];
        if (sym->kind == 0 /* REG */) {
            uint32_t val = s->net_val[sym->net_id] & mask(sym->width);
            printf("    %-12s = 0x%0*X  (%u)\n",
                   sym->name, (sym->width+3)/4, val, val);
        }
    }
}

/* ── Main ────────────────────────────────────────────────────────── */
int main(int argc, char **argv) {
    if (argc < 2) {
        printf("cqsim — CIRQ Hardware Simulator\n\n");
        printf("Usage: cqsim <file.cqbit> [options]\n\n");
        printf("Options:\n");
        printf("  --cycles N      Run for N clock cycles (default: 100)\n");
        printf("  --trace         Print port values every cycle\n");
        printf("  --every N       Print every N cycles (default: 10)\n");
        printf("  --rom file.hex  Load hex file into first RAM\n");
        printf("  --regs          Print register values after simulation\n\n");
        printf("Example:\n");
        printf("  cqsim cpu.cqbit --rom program.hex --cycles 50 --trace\n");
        return 1;
    }

    const char *infile  = argv[1];
    int  max_cycles     = 100;
    int  trace          = 0;
    int  print_every    = 10;
    int  show_regs      = 0;
    const char *rom_file = NULL;

    for (int i=2; i<argc; i++) {
        if (strcmp(argv[i],"--cycles")==0 && i+1<argc) max_cycles = atoi(argv[++i]);
        if (strcmp(argv[i],"--trace")==0)  trace      = 1;
        if (strcmp(argv[i],"--every")==0 && i+1<argc)  print_every = atoi(argv[++i]);
        if (strcmp(argv[i],"--rom")==0 && i+1<argc)    rom_file   = argv[++i];
        if (strcmp(argv[i],"--regs")==0)   show_regs  = 1;
    }

    Sim *s = calloc(1, sizeof(Sim));
    if (!s) { fprintf(stderr,"OOM\n"); return 1; }

    printf("╔══════════════════════════════════════════════╗\n");
    printf("║  CIRQ Simulator  —  cqsim v1.0              ║\n");
    printf("╚══════════════════════════════════════════════╝\n\n");
    printf("  Loading: %s\n", infile);

    if (!load_cqbit(infile, s)) { free(s); return 1; }

    printf("  Chip:   %s\n", s->chip_name);
    printf("  Cells:  %d   Nets: %d   Ports: %d\n\n",
           s->n_cells, s->n_nets, s->n_ports);

    /* Register all BRAMs */
    for (int i = 0; i < s->n_cells; i++) {
        Cell *c = &s->cells[i];
        if (c->type == CQ_BRAM && c->name[0]) {
            int already = find_ram(s, c->name);
            if (already < 0 && s->n_ram < 16) {
                strncpy(s->ram_name[s->n_ram], c->name, MAX_IDENT-1);
                s->n_ram++;
                printf("  RAM registered: %s\n", c->name);
            }
        }
    }

    /* Load ROM if provided */
    if (rom_file) {
        if (s->n_ram > 0) load_hex(s, 0, rom_file);
        else printf("  Warning: no RAM declared, ignoring --rom\n");
    }

    /* Apply reset for first 2 cycles */
    printf("\n  Applying reset...\n");
    apply_reset(s);
    s->reset_active = 1;

    /* Set reset net value */
    int rst_net = find_net(s, s->reset_name);
    if (rst_net >= 0) s->net_val[rst_net] = 1;

    printf("  Running simulation for %d cycles...\n\n", max_cycles);
    printf("  %-8s │  Ports\n", "Cycle");
    printf("  %s\n", "─────────────────────────────────────────────");

    /* ── Simulation loop ── */
    for (int cyc = 0; cyc < max_cycles; cyc++) {
        s->cycle = (uint64_t)cyc;

        /* Deassert reset after 2 cycles */
        if (cyc == 2) {
            s->reset_active = 0;
            if (rst_net >= 0) s->net_val[rst_net] = 0;
            printf("  Reset deasserted at cycle %d\n\n", cyc);
        }

        /* Evaluate combinational logic */
        eval_all(s);

        /* Print output */
        if (trace || (cyc % print_every == 0))
            print_ports(s);

        /* Check halted signal */
        for (int i = 0; i < s->n_ports; i++) {
            if (strcmp(s->ports[i].name, "halted") == 0) {
                if (s->net_val[s->ports[i].net_id] == 1 && !s->reset_active) {
                    printf("\n  ⏹  HALT detected at cycle %llu\n",
                           (unsigned long long)s->cycle);
                    goto done;
                }
            }
        }

        /* Rising clock edge: latch DFFs */
        clock_edge(s);
    }

done:
    printf("\n  Simulation complete. %llu cycles run.\n\n",
           (unsigned long long)s->cycle);

    if (show_regs) {
        print_registers(s);
        printf("\n");
    }

    /* Print RAM contents if small */
    for (int ri = 0; ri < s->n_ram; ri++) {
        int nonzero = 0;
        for (int j = 0; j < 32; j++) if (s->ram[ri][j]) nonzero++;
        if (nonzero > 0) {
            printf("  RAM '%s' (first 32 bytes):\n  ", s->ram_name[ri]);
            for (int j = 0; j < 32; j++) printf("%02X ", s->ram[ri][j]);
            printf("\n\n");
        }
    }

    free(s);
    return 0;
}
