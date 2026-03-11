/*
 * cqdump.c — CIRQ Binary Inspector
 *
 * Reads a .cqbit file and prints a human-readable disassembly.
 * Like objdump, but for CIRQ hardware binaries.
 *
 * Build:  gcc -O2 -o cqdump cqdump.c
 * Usage:  ./cqdump cpu.cqbit
 *         ./cqdump cpu.cqbit --cells
 *         ./cqdump cpu.cqbit --nets
 *         ./cqdump cpu.cqbit --ports
 *         ./cqdump cpu.cqbit --all
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* ── .cqbit reader helpers ───────────────────────────────────────── */
static uint8_t  ru8 (FILE *f) { uint8_t  v; fread(&v,1,1,f); return v; }
static uint16_t ru16(FILE *f) { uint8_t b[2]; fread(b,1,2,f); return (uint16_t)(b[0]|(b[1]<<8)); }
static uint32_t ru32(FILE *f) { uint8_t b[4]; fread(b,1,4,f); return (uint32_t)(b[0]|(b[1]<<8)|(b[2]<<16)|(b[3]<<24)); }
static void     rstr(FILE *f, char *buf, int max) {
    int i=0; char ch;
    while ((ch=(char)fgetc(f)) && i<max-1) buf[i++]=ch;
    buf[i]='\0';
}

/* ── Cell type names ─────────────────────────────────────────────── */
typedef enum {
    CQ_DFF=0x01,CQ_BRAM=0x02,
    CQ_ADD=0x10,CQ_SUB=0x11,CQ_AND=0x12,CQ_OR=0x13,CQ_XOR=0x14,
    CQ_NOT=0x15,CQ_SHL=0x16,CQ_SHR=0x17,CQ_INC=0x18,CQ_DEC=0x19,
    CQ_MUX=0x20,CQ_EQ=0x21,CQ_NEQ=0x22,CQ_LT=0x23,CQ_GT=0x24,
    CQ_WIRE=0x30,CQ_CONST=0x31,
    CQ_MEM_RD=0x40,CQ_MEM_WR=0x41,
    CQ_OUT_PORT=0x50,CQ_IN_PORT=0x51
} CQType;

static const char *ctype(uint8_t t) {
    switch(t){
    case CQ_DFF:     return "DFF     ";
    case CQ_BRAM:    return "BRAM    ";
    case CQ_ADD:     return "ADD     ";
    case CQ_SUB:     return "SUB     ";
    case CQ_AND:     return "AND     ";
    case CQ_OR:      return "OR      ";
    case CQ_XOR:     return "XOR     ";
    case CQ_NOT:     return "NOT     ";
    case CQ_SHL:     return "SHL     ";
    case CQ_SHR:     return "SHR     ";
    case CQ_INC:     return "INC     ";
    case CQ_DEC:     return "DEC     ";
    case CQ_MUX:     return "MUX     ";
    case CQ_EQ:      return "EQ      ";
    case CQ_NEQ:     return "NEQ     ";
    case CQ_LT:      return "LT      ";
    case CQ_GT:      return "GT      ";
    case CQ_WIRE:    return "WIRE    ";
    case CQ_CONST:   return "CONST   ";
    case CQ_MEM_RD:  return "MEM_RD  ";
    case CQ_MEM_WR:  return "MEM_WR  ";
    case CQ_OUT_PORT:return "OUT_PORT";
    case CQ_IN_PORT: return "IN_PORT ";
    default:         return "???     ";
    }
}

static const char *sym_kind(uint8_t k) {
    switch(k){ case 0:return "reg"; case 1:return "wire";
               case 2:return "ram"; case 3:return "const";
               case 4:return "state"; case 5:return "bus"; default:return "?"; }
}

/* ── Stored net/symbol name arrays ──────────────────────────────── */
#define MAX_NETS    4096
#define MAX_SYMS    1024
#define MAX_IDENT   128

typedef struct { char name[MAX_IDENT]; uint8_t width; } NetEntry;
typedef struct { char name[MAX_IDENT]; uint8_t kind; uint8_t width; uint16_t net_id; uint32_t reset; } SymEntry;
typedef struct { char name[MAX_IDENT]; uint8_t dir; uint8_t width; uint16_t net_id; } PortEntry;

typedef struct {
    /* header */
    char     chip_name[MAX_IDENT];
    char     clock_name[MAX_IDENT];
    char     reset_name[MAX_IDENT];
    uint16_t version;
    uint32_t n_cells;
    uint32_t n_nets;
    uint16_t n_ports;

    /* cells (raw) */
    struct { uint8_t type,width; uint16_t a,b,out,sel; uint32_t lit; uint8_t flags,pad; } *cells;

    /* nets, ports, syms */
    NetEntry  *nets;   uint32_t nets_count;
    PortEntry *ports;  uint16_t ports_count;
    SymEntry  *syms;   uint16_t syms_count;
} CQBit;

/* ── Load .cqbit from file ───────────────────────────────────────── */
static int load_cqbit(const char *path, CQBit *cq) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return 0; }

    /* Magic */
    char magic[5]={0};
    fread(magic, 1, 4, f);
    if (strcmp(magic, "CQBT") != 0) {
        fprintf(stderr, "Error: not a .cqbit file (bad magic: %.4s)\n", magic);
        fclose(f); return 0;
    }

    cq->version = ru16(f);
    uint16_t chip_len = ru16(f);
    cq->n_cells = ru32(f);
    cq->n_nets  = ru32(f);
    cq->n_ports = ru16(f);
    ru16(f); ru32(f); ru32(f); /* reserved */

    /* Chip name */
    fread(cq->chip_name, 1, chip_len, f);
    cq->chip_name[chip_len] = '\0';
    rstr(f, cq->clock_name, MAX_IDENT);
    rstr(f, cq->reset_name, MAX_IDENT);

    /* Cells */
    cq->cells = calloc(cq->n_cells, sizeof(*cq->cells));
    for (uint32_t i = 0; i < cq->n_cells; i++) {
        cq->cells[i].type  = ru8(f);
        cq->cells[i].width = ru8(f);
        cq->cells[i].a     = ru16(f);
        cq->cells[i].b     = ru16(f);
        cq->cells[i].out   = ru16(f);
        cq->cells[i].sel   = ru16(f);
        cq->cells[i].lit   = ru32(f);
        cq->cells[i].flags = ru8(f);
        cq->cells[i].pad   = ru8(f);
    }

    /* Nets */
    cq->nets_count = ru16(f);
    cq->nets = calloc(cq->nets_count, sizeof(*cq->nets));
    for (uint32_t i = 0; i < cq->nets_count; i++) {
        cq->nets[i].width = ru8(f);
        rstr(f, cq->nets[i].name, MAX_IDENT);
    }

    /* Ports */
    cq->ports_count = ru16(f);
    cq->ports = calloc(cq->ports_count, sizeof(*cq->ports));
    for (uint16_t i = 0; i < cq->ports_count; i++) {
        cq->ports[i].dir    = ru8(f);
        cq->ports[i].width  = ru8(f);
        cq->ports[i].net_id = ru16(f);
        rstr(f, cq->ports[i].name, MAX_IDENT);
    }

    /* Symbols */
    cq->syms_count = ru16(f);
    cq->syms = calloc(cq->syms_count, sizeof(*cq->syms));
    for (uint16_t i = 0; i < cq->syms_count; i++) {
        cq->syms[i].kind   = ru8(f);
        cq->syms[i].width  = ru8(f);
        cq->syms[i].net_id = ru16(f);
        cq->syms[i].reset  = ru32(f);
        rstr(f, cq->syms[i].name, MAX_IDENT);
    }

    /* End marker */
    char end[6]={0};
    fread(end, 1, 5, f);
    if (strcmp(end, "CQEND") != 0)
        fprintf(stderr, "Warning: missing CQEND marker (file may be truncated)\n");

    fclose(f);
    return 1;
}

/* ── Net name lookup ─────────────────────────────────────────────── */
static const char *net_name(CQBit *cq, uint16_t id) {
    if (id < cq->nets_count && cq->nets[id].name[0])
        return cq->nets[id].name;
    static char tmp[32];
    snprintf(tmp, sizeof(tmp), "net%d", id);
    return tmp;
}

/* ── Dump functions ──────────────────────────────────────────────── */
static void dump_header(CQBit *cq) {
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║  CIRQ Binary (.cqbit) — cqdump v1.0                 ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n\n");
    printf("  Chip:     %s\n", cq->chip_name);
    printf("  Version:  %d.%d\n", cq->version>>8, cq->version&0xFF);
    printf("  Clock:    %s\n", cq->clock_name);
    printf("  Reset:    %s\n", cq->reset_name);
    printf("  Cells:    %u\n", cq->n_cells);
    printf("  Nets:     %u\n", cq->nets_count);
    printf("  Ports:    %u\n", cq->ports_count);
    printf("  Symbols:  %u\n\n", cq->syms_count);
}

static void dump_cells(CQBit *cq) {
    printf("  CELLS (%u gate primitives):\n", cq->n_cells);
    printf("  %-5s %-10s %-4s  %-18s %-18s %-18s\n",
           "IDX", "TYPE", "W", "A (input)", "B (input)", "OUT (output)");
    printf("  %s\n", "─────────────────────────────────────────────────────────────────");

    for (uint32_t i = 0; i < cq->n_cells; i++) {
        uint8_t  tp = cq->cells[i].type;
        uint8_t  w  = cq->cells[i].width;
        uint16_t a  = cq->cells[i].a;
        uint16_t b  = cq->cells[i].b;
        uint16_t o  = cq->cells[i].out;
        uint16_t s  = cq->cells[i].sel;
        uint32_t lt = cq->cells[i].lit;

        if (tp == CQ_CONST) {
            printf("  [%3u] %s %2d   0x%08X                              → %-18s\n",
                   i, ctype(tp), w, lt, net_name(cq, o));
        } else if (tp == CQ_MUX) {
            printf("  [%3u] %s %2d   sel=%-14s %-18s %-18s\n",
                   i, ctype(tp), w,
                   net_name(cq,s), net_name(cq,a), net_name(cq,o));
        } else if (tp == CQ_NOT || tp == CQ_INC || tp == CQ_DEC ||
                   tp == CQ_SHL || tp == CQ_SHR || tp == CQ_OUT_PORT || tp == CQ_IN_PORT) {
            printf("  [%3u] %s %2d   %-18s                    → %-18s\n",
                   i, ctype(tp), w, net_name(cq,a), net_name(cq,o));
        } else if (tp == CQ_MEM_WR) {
            printf("  [%3u] %s %2d   addr=%-13s data=%-13s\n",
                   i, ctype(tp), w, net_name(cq,a), net_name(cq,b));
        } else if (tp == CQ_DFF) {
            printf("  [%3u] %s %2d   Q=%-16s reset=0x%X\n",
                   i, ctype(tp), w, net_name(cq,o), lt);
        } else {
            printf("  [%3u] %s %2d   %-18s %-18s → %-18s\n",
                   i, ctype(tp), w,
                   net_name(cq,a), net_name(cq,b), net_name(cq,o));
        }
    }
    printf("\n");
}

static void dump_nets(CQBit *cq) {
    printf("  NETS (%u signals):\n", cq->nets_count);
    printf("  %-6s %-4s  %s\n", "ID", "W", "NAME");
    printf("  %s\n", "─────────────────────────────");
    for (uint32_t i = 0; i < cq->nets_count; i++)
        printf("  [%4u] %2d   %s\n", i, cq->nets[i].width, cq->nets[i].name);
    printf("\n");
}

static void dump_ports(CQBit *cq) {
    printf("  PORTS (%u physical FPGA pins):\n", cq->ports_count);
    printf("  %-6s %-4s %-4s %-8s  %s\n", "ID", "DIR", "W", "NET", "NAME");
    printf("  %s\n", "──────────────────────────────────────");
    for (uint16_t i = 0; i < cq->ports_count; i++) {
        PortEntry *p = &cq->ports[i];
        printf("  [%4u] %-4s %2d   net%-5d  %s\n",
               i, p->dir ? "OUT" : "IN", p->width, p->net_id, p->name);
    }
    printf("\n");
}

static void dump_symbols(CQBit *cq) {
    printf("  SYMBOL TABLE (%u entries):\n", cq->syms_count);
    printf("  %-6s %-7s %-4s %-8s %-10s  %s\n","ID","KIND","W","NET","RESET","NAME");
    printf("  %s\n", "──────────────────────────────────────────────");
    for (uint16_t i = 0; i < cq->syms_count; i++) {
        SymEntry *s = &cq->syms[i];
        printf("  [%4u] %-7s %2d   net%-5d 0x%-8X  %s\n",
               i, sym_kind(s->kind), s->width, s->net_id, s->reset, s->name);
    }
    printf("\n");
}

/* ── Cell histogram ──────────────────────────────────────────────── */
static void dump_stats(CQBit *cq) {
    int counts[256] = {0};
    for (uint32_t i = 0; i < cq->n_cells; i++)
        counts[(uint8_t)cq->cells[i].type]++;

    printf("  GATE STATISTICS:\n");
    printf("  ─────────────────────────────\n");
    uint8_t types[] = {
        CQ_DFF,CQ_BRAM,CQ_ADD,CQ_SUB,CQ_AND,CQ_OR,CQ_XOR,
        CQ_NOT,CQ_SHL,CQ_SHR,CQ_INC,CQ_DEC,CQ_MUX,CQ_EQ,
        CQ_NEQ,CQ_LT,CQ_GT,CQ_WIRE,CQ_CONST,CQ_MEM_RD,CQ_MEM_WR,
        CQ_OUT_PORT,CQ_IN_PORT
    };
    for (int i = 0; i < (int)(sizeof(types)/sizeof(types[0])); i++) {
        if (counts[types[i]] > 0)
            printf("  %-10s  %4d\n", ctype(types[i]), counts[types[i]]);
    }
    printf("  ─────────────────────────────\n");
    printf("  TOTAL       %4u\n\n", cq->n_cells);
}

/* ── Main ────────────────────────────────────────────────────────── */
int main(int argc, char **argv) {
    if (argc < 2) {
        printf("cqdump — CIRQ Binary Inspector\n");
        printf("Usage: cqdump <file.cqbit> [--cells] [--nets] [--ports] [--syms] [--stats] [--all]\n");
        return 1;
    }

    int show_cells=0, show_nets=0, show_ports=0, show_syms=0, show_stats=0;
    for (int i=2; i<argc; i++) {
        if (strcmp(argv[i],"--cells")==0) show_cells=1;
        if (strcmp(argv[i],"--nets")==0)  show_nets=1;
        if (strcmp(argv[i],"--ports")==0) show_ports=1;
        if (strcmp(argv[i],"--syms")==0)  show_syms=1;
        if (strcmp(argv[i],"--stats")==0) show_stats=1;
        if (strcmp(argv[i],"--all")==0)   show_cells=show_nets=show_ports=show_syms=show_stats=1;
    }
    /* default: show header + stats + ports */
    if (!show_cells && !show_nets && !show_ports && !show_syms && !show_stats)
        show_stats = show_ports = 1;

    CQBit cq = {0};
    if (!load_cqbit(argv[1], &cq)) return 1;

    dump_header(&cq);
    if (show_stats)  dump_stats(&cq);
    if (show_nets)   dump_nets(&cq);
    if (show_cells)  dump_cells(&cq);
    if (show_ports)  dump_ports(&cq);
    if (show_syms)   dump_symbols(&cq);

    free(cq.cells); free(cq.nets); free(cq.ports); free(cq.syms);
    return 0;
}
