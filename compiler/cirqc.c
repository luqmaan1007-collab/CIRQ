/*
 * ╔══════════════════════════════════════════════════════════════════════╗
 * ║  cirqc.c  —  CIRQ Compiler v2.0  (FULLY INDEPENDENT)               ║
 * ║                                                                      ║
 * ║  CIRQ is its own language. No Verilog. No VHDL. No middleman.       ║
 * ║                                                                      ║
 * ║  Pipeline:                                                           ║
 * ║    .cirq source                                                      ║
 * ║      → Lexer        (CIRQ tokens)                                    ║
 * ║      → Parser       (CIRQ AST)                                       ║
 * ║      → Analyzer     (type check, bit widths)                         ║
 * ║      → IR Builder   (CIRQ Intermediate Representation)               ║
 * ║      → Gate Lowerer (IR → gate primitives)                           ║
 * ║      → .cqbit       (CIRQ Binary — own format, not Verilog)          ║
 * ║                                                                      ║
 * ║  Then:                                                               ║
 * ║    cqprog flash board.cqbit   → Programs FPGA directly               ║
 * ║    cqsim  run   design.cqbit  → Software simulation                  ║
 * ║                                                                      ║
 * ║  Build:   gcc -O2 -o cirqc cirqc.c                                   ║
 * ║  Usage:   ./cirqc cpu.cirq -o cpu.cqbit                              ║
 * ╚══════════════════════════════════════════════════════════════════════╝
 *
 *  .cqbit FILE FORMAT:
 *  ┌─────────────────────────────────────────────┐
 *  │ Header (32 bytes)                           │
 *  │   magic:    "CQBT" (4 bytes)                │
 *  │   version:  u16                             │
 *  │   chip_len: u16  (name length)              │
 *  │   n_cells:  u32  (number of gate cells)     │
 *  │   n_nets:   u32  (number of wires/nets)     │
 *  │   n_ports:  u16  (number of I/O ports)      │
 *  │   reserved: 10 bytes                        │
 *  ├─────────────────────────────────────────────┤
 *  │ Chip name  (chip_len bytes)                 │
 *  ├─────────────────────────────────────────────┤
 *  │ Cell section                                │
 *  │   per cell: type(u8) width(u8) a(u16) b(u16)│
 *  │             out(u16) reset(u32) flags(u8)   │
 *  ├─────────────────────────────────────────────┤
 *  │ Net name table  (null-terminated strings)   │
 *  ├─────────────────────────────────────────────┤
 *  │ Port table                                  │
 *  │   per port: dir(u8) width(u8) net_id(u16)   │
 *  │             name_len(u8) name(...)           │
 *  ├─────────────────────────────────────────────┤
 *  │ Constraint section  (pin mappings)          │
 *  │   per constraint: port_id(u16) pin(u8)      │
 *  └─────────────────────────────────────────────┘
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include <stdarg.h>

/* ═══════════════════════════════════════════════════════════════════
 *  CONSTANTS
 * ═══════════════════════════════════════════════════════════════════ */
#define CQBIT_MAGIC     "CQBT"
#define CQBIT_VERSION   0x0200   /* v2.0 */

#define MAX_TOKENS      16384
#define MAX_IDENT       128
#define MAX_SYMBOLS     1024
#define MAX_CELLS       4096
#define MAX_NETS        4096
#define MAX_PORTS       64
#define MAX_NAME_TABLE  (64 * 1024)

/* ═══════════════════════════════════════════════════════════════════
 *  CELL TYPES  (gate primitives in .cqbit)
 *  These are the fundamental hardware operations CIRQ knows about.
 *  Each maps to one or more LUTs on a real FPGA.
 * ═══════════════════════════════════════════════════════════════════ */
typedef enum {
    CQ_DFF      = 0x01,   /* D Flip-Flop  — clocked register       */
    CQ_BRAM     = 0x02,   /* Block RAM    — synchronous memory      */
    CQ_ADD      = 0x10,   /* Adder        — a + b → out, carry      */
    CQ_SUB      = 0x11,   /* Subtractor   — a - b → out, borrow     */
    CQ_AND      = 0x12,   /* AND gate     — a & b                   */
    CQ_OR       = 0x13,   /* OR gate      — a | b                   */
    CQ_XOR      = 0x14,   /* XOR gate     — a ^ b                   */
    CQ_NOT      = 0x15,   /* NOT gate     — ~a                      */
    CQ_SHL      = 0x16,   /* Shift left   — a << 1                  */
    CQ_SHR      = 0x17,   /* Shift right  — a >> 1                  */
    CQ_INC      = 0x18,   /* Increment    — a + 1                   */
    CQ_DEC      = 0x19,   /* Decrement    — a - 1                   */
    CQ_MUX      = 0x20,   /* Multiplexer  — sel ? b : a             */
    CQ_EQ       = 0x21,   /* Comparator   — a == b                  */
    CQ_NEQ      = 0x22,   /* Comparator   — a != b                  */
    CQ_LT       = 0x23,   /* Less than    — a < b                   */
    CQ_GT       = 0x24,   /* Greater than — a > b                   */
    CQ_WIRE     = 0x30,   /* Wire assign  — out = src               */
    CQ_CONST    = 0x31,   /* Constant     — out = literal           */
    CQ_MEM_RD   = 0x40,   /* Memory read  — out = RAM[addr]         */
    CQ_MEM_WR   = 0x41,   /* Memory write — RAM[addr] = data        */
    CQ_OUT_PORT = 0x50,   /* Output port  — physical FPGA pin       */
    CQ_IN_PORT  = 0x51,   /* Input port   — physical FPGA pin       */
} CQCellType;

/* ═══════════════════════════════════════════════════════════════════
 *  TOKEN TYPES  (unique CIRQ syntax)
 * ═══════════════════════════════════════════════════════════════════ */
typedef enum {
    TT_INT=1, TT_STRING, TT_IDENT,
    /* CIRQ keywords — none from Verilog/VHDL */
    TT_CHIP, TT_REG, TT_WIRE, TT_BUS, TT_RAM, TT_ROM,
    TT_CLOCK, TT_RESET, TT_STATE, TT_CONST, TT_LOGIC,
    TT_COMPUTE, TT_ON, TT_WHEN, TT_IF, TT_ELSE,
    TT_DEFAULT, TT_OUT, TT_IN, TT_PORT, TT_PROGRAM,
    /* Operators */
    TT_LARROW, TT_ASSIGN, TT_FATARROW,
    TT_EQ, TT_NEQ, TT_LT, TT_GT,
    TT_PLUS, TT_MINUS, TT_AND, TT_OR, TT_XOR, TT_NOT,
    TT_LSHIFT, TT_RSHIFT,
    TT_DOT, TT_COMMA, TT_COLON, TT_SEMI,
    TT_LBRACE, TT_RBRACE, TT_LPAREN, TT_RPAREN,
    TT_LBRACKET, TT_RBRACKET,
    TT_EOF
} TT;

typedef struct {
    TT    type;
    char  val[MAX_IDENT];
    long long ival;
    int   line, col;
} Token;

/* ═══════════════════════════════════════════════════════════════════
 *  SYMBOL TABLE
 * ═══════════════════════════════════════════════════════════════════ */
typedef enum { SYM_REG, SYM_WIRE, SYM_RAM, SYM_CONST, SYM_STATE, SYM_BUS } SymKind;
typedef struct {
    char      name[MAX_IDENT];
    SymKind   kind;
    int       width;
    int       depth;
    long long reset;
    long long value;
    char      init_file[MAX_IDENT];
    uint16_t  net_id;    /* which net carries this signal */
} Symbol;

/* ═══════════════════════════════════════════════════════════════════
 *  GATE CELL  (intermediate gate-level primitive)
 * ═══════════════════════════════════════════════════════════════════ */
typedef struct {
    CQCellType type;
    uint8_t    width;    /* bit width of output */
    uint16_t   a;        /* net id for input A  */
    uint16_t   b;        /* net id for input B  */
    uint16_t   out;      /* net id for output   */
    uint16_t   sel;      /* net id for MUX sel  */
    uint32_t   literal;  /* for CQ_CONST        */
    uint8_t    flags;    /* misc flags           */
    char       name[MAX_IDENT];
    char       comment[128];
} Cell;

/* ═══════════════════════════════════════════════════════════════════
 *  OUTPUT PORT
 * ═══════════════════════════════════════════════════════════════════ */
typedef struct {
    char     name[MAX_IDENT];
    uint8_t  dir;    /* 0=in, 1=out */
    uint8_t  width;
    uint16_t net_id;
} Port;

/* ═══════════════════════════════════════════════════════════════════
 *  NET TABLE  (named signals / wires)
 * ═══════════════════════════════════════════════════════════════════ */
typedef struct {
    char     name[MAX_IDENT];
    uint8_t  width;
} Net;

/* ═══════════════════════════════════════════════════════════════════
 *  COMPILER STATE
 * ═══════════════════════════════════════════════════════════════════ */
typedef struct {
    /* Source */
    char   *src;
    int     src_len;
    int     pos, line, col;

    /* Tokens */
    Token   tokens[MAX_TOKENS];
    int     token_count;
    int     tok_pos;

    /* Chip metadata */
    char    chip_name[MAX_IDENT];
    char    clock_name[MAX_IDENT];
    char    reset_name[MAX_IDENT];

    /* Symbol table */
    Symbol  symbols[MAX_SYMBOLS];
    int     sym_count;

    /* Gate cells */
    Cell    cells[MAX_CELLS];
    int     cell_count;

    /* Nets */
    Net     nets[MAX_NETS];
    int     net_count;

    /* Ports */
    Port    ports[MAX_PORTS];
    int     port_count;

    /* Counters */
    int     net_seq;
    int     mux_seq;
    int     cell_seq;
} Compiler;

/* ═══════════════════════════════════════════════════════════════════
 *  ERROR / UTILITY
 * ═══════════════════════════════════════════════════════════════════ */
static void die(Compiler *c, const char *fmt, ...) {
    va_list ap;
    fprintf(stderr, "\n[CIRQ ERROR] L%d:C%d — ", c->line, c->col);
    va_start(ap, fmt); vfprintf(stderr, fmt, ap); va_end(ap);
    fprintf(stderr, "\n");
    exit(1);
}

static uint16_t new_net(Compiler *c, const char *name, int width) {
    if (c->net_count >= MAX_NETS) die(c, "Too many nets");
    Net *n = &c->nets[c->net_count];
    if (name) strncpy(n->name, name, MAX_IDENT-1);
    else       snprintf(n->name, MAX_IDENT, "_net_%d", ++c->net_seq);
    n->width = (uint8_t)width;
    return (uint16_t)(c->net_count++);
}

static uint16_t find_net(Compiler *c, const char *name) {
    for (int i = 0; i < c->net_count; i++)
        if (strcmp(c->nets[i].name, name) == 0)
            return (uint16_t)i;
    /* auto-create */
    return new_net(c, name, 8);
}

static Cell *add_cell(Compiler *c, CQCellType type, const char *name) {
    if (c->cell_count >= MAX_CELLS) die(c, "Too many cells");
    Cell *cell = &c->cells[c->cell_count++];
    memset(cell, 0, sizeof(*cell));
    cell->type = type;
    if (name) strncpy(cell->name, name, MAX_IDENT-1);
    return cell;
}

static Symbol *add_sym(Compiler *c, const char *name, SymKind kind) {
    if (c->sym_count >= MAX_SYMBOLS) die(c, "Symbol table full");
    Symbol *s = &c->symbols[c->sym_count++];
    memset(s, 0, sizeof(*s));
    strncpy(s->name, name, MAX_IDENT-1);
    s->kind = kind;
    return s;
}

static Symbol *find_sym(Compiler *c, const char *name) {
    for (int i = 0; i < c->sym_count; i++)
        if (strcmp(c->symbols[i].name, name) == 0)
            return &c->symbols[i];
    return NULL;
}

/* ═══════════════════════════════════════════════════════════════════
 *  LEXER
 * ═══════════════════════════════════════════════════════════════════ */
static char lpeek(Compiler *c) { return c->pos < c->src_len ? c->src[c->pos] : 0; }
static char lpeek2(Compiler *c){ return (c->pos+1) < c->src_len ? c->src[c->pos+1] : 0; }
static char ladv(Compiler *c)  {
    char ch = c->src[c->pos++];
    if (ch == '\n') { c->line++; c->col = 1; } else c->col++;
    return ch;
}

static void skip_ws(Compiler *c) {
    for(;;) {
        while (c->pos < c->src_len && isspace((unsigned char)lpeek(c))) ladv(c);
        if (lpeek(c)=='/' && lpeek2(c)=='/') {
            while (c->pos < c->src_len && lpeek(c)!='\n') ladv(c); continue;
        }
        if (lpeek(c)=='/' && lpeek2(c)=='*') {
            ladv(c); ladv(c);
            while (c->pos < c->src_len) {
                if (lpeek(c)=='*' && lpeek2(c)=='/') { ladv(c); ladv(c); break; }
                ladv(c);
            }
            continue;
        }
        break;
    }
}

typedef struct { const char *kw; TT tt; } KW;
static KW KWS[] = {
    {"chip",TT_CHIP},{"reg",TT_REG},{"wire",TT_WIRE},{"bus",TT_BUS},
    {"ram",TT_RAM},{"rom",TT_ROM},{"clock",TT_CLOCK},{"reset",TT_RESET},
    {"state",TT_STATE},{"const",TT_CONST},{"logic",TT_LOGIC},
    {"compute",TT_COMPUTE},{"on",TT_ON},{"when",TT_WHEN},{"if",TT_IF},
    {"else",TT_ELSE},{"default",TT_DEFAULT},{"out",TT_OUT},{"in",TT_IN},
    {"port",TT_PORT},{"program",TT_PROGRAM},{NULL,0}
};

static TT kw_or_ident(const char *s) {
    for (int i=0; KWS[i].kw; i++)
        if (strcmp(s, KWS[i].kw)==0) return KWS[i].tt;
    return TT_IDENT;
}

static void lex(Compiler *c) {
    c->pos = 0; c->line = 1; c->col = 1;
    c->token_count = 0;
    for(;;) {
        skip_ws(c);
        if (c->pos >= c->src_len) break;
        if (c->token_count >= MAX_TOKENS) die(c, "Too many tokens");
        Token *t = &c->tokens[c->token_count++];
        memset(t, 0, sizeof(*t));
        t->line = c->line; t->col = c->col;
        char ch = lpeek(c);

        /* Integer: 0x 0b decimal */
        if (isdigit((unsigned char)ch)) {
            char buf[64]; int bi=0;
            if (ch=='0' && (lpeek2(c)=='x'||lpeek2(c)=='X'||lpeek2(c)=='b'||lpeek2(c)=='B')) {
                buf[bi++]=ladv(c); buf[bi++]=ladv(c);
                while (isxdigit((unsigned char)lpeek(c))||lpeek(c)=='_') {
                    char cc=ladv(c); if(cc!='_') buf[bi++]=cc;
                }
            } else {
                while (isdigit((unsigned char)lpeek(c))||lpeek(c)=='_') {
                    char cc=ladv(c); if(cc!='_') buf[bi++]=cc;
                }
            }
            buf[bi]='\0'; t->type=TT_INT; t->ival=strtoll(buf,NULL,0);
            continue;
        }

        /* String */
        if (ch=='"') {
            ladv(c); int bi=0;
            while (c->pos<c->src_len && lpeek(c)!='"') t->val[bi++]=ladv(c);
            if (lpeek(c)=='"') ladv(c);
            t->val[bi]='\0'; t->type=TT_STRING;
            continue;
        }

        /* Identifier / keyword */
        if (isalpha((unsigned char)ch)||ch=='_') {
            int bi=0;
            t->val[bi++]=ladv(c);
            while (isalnum((unsigned char)lpeek(c))||lpeek(c)=='_') t->val[bi++]=ladv(c);
            t->val[bi]='\0'; t->type=kw_or_ident(t->val);
            continue;
        }

        /* Operators */
        ladv(c);
        switch(ch) {
        case '<': if(lpeek(c)=='='){ladv(c);t->type=TT_LARROW;}
                  else if(lpeek(c)=='<'){ladv(c);t->type=TT_LSHIFT;}
                  else t->type=TT_LT; break;
        case '>': if(lpeek(c)=='>'){ladv(c);t->type=TT_RSHIFT;}
                  else t->type=TT_GT; break;
        case '=': if(lpeek(c)=='='){ladv(c);t->type=TT_EQ;}
                  else if(lpeek(c)=='>'){ladv(c);t->type=TT_FATARROW;}
                  else t->type=TT_ASSIGN; break;
        case '!': if(lpeek(c)=='='){ladv(c);t->type=TT_NEQ;}
                  else die(c,"Unexpected '!'"); break;
        case '+': t->type=TT_PLUS;     break;
        case '-': t->type=TT_MINUS;    break;
        case '&': t->type=TT_AND;      break;
        case '|': t->type=TT_OR;       break;
        case '^': t->type=TT_XOR;      break;
        case '~': t->type=TT_NOT;      break;
        case '.': t->type=TT_DOT;      break;
        case ',': t->type=TT_COMMA;    break;
        case ':': t->type=TT_COLON;    break;
        case ';': t->type=TT_SEMI;     break;
        case '{': t->type=TT_LBRACE;   break;
        case '}': t->type=TT_RBRACE;   break;
        case '(': t->type=TT_LPAREN;   break;
        case ')': t->type=TT_RPAREN;   break;
        case '[': t->type=TT_LBRACKET; break;
        case ']': t->type=TT_RBRACKET; break;
        default:  die(c,"Unknown char '%c'",ch);
        }
    }
    Token *e = &c->tokens[c->token_count++];
    memset(e,0,sizeof(*e)); e->type=TT_EOF; e->line=c->line;
}

/* ═══════════════════════════════════════════════════════════════════
 *  PARSER HELPERS
 * ═══════════════════════════════════════════════════════════════════ */
static Token *ptok(Compiler *c)           { return &c->tokens[c->tok_pos]; }
static Token *padv(Compiler *c)           { Token *t=&c->tokens[c->tok_pos]; if(c->tok_pos<c->token_count-1) c->tok_pos++; return t; }
static Token *pexpect(Compiler *c, TT tt) {
    Token *t=padv(c);
    if(t->type!=tt) { c->line=t->line; c->col=t->col;
                      die(c,"Expected token %d got %d ('%s')",tt,t->type,t->val); }
    return t;
}
static int pmatch(Compiler *c, TT tt) { if(ptok(c)->type==tt){padv(c);return 1;} return 0; }

static void skip_block(Compiler *c) {
    pexpect(c,TT_LBRACE); int d=1;
    while(d>0&&ptok(c)->type!=TT_EOF) {
        TT t=padv(c)->type;
        if(t==TT_LBRACE) d++; else if(t==TT_RBRACE) d--;
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  EXPRESSION PARSER
 *  Returns the net_id that carries the result of the expression
 * ═══════════════════════════════════════════════════════════════════ */
static uint16_t parse_expr(Compiler *c);

static uint16_t parse_primary(Compiler *c) {
    Token *t = ptok(c);

    /* ~expr */
    if (t->type == TT_NOT) {
        padv(c);
        uint16_t src = parse_expr(c);
        uint16_t out = new_net(c, NULL, 8);
        Cell *cell = add_cell(c, CQ_NOT, NULL);
        cell->a   = src;
        cell->out = out;
        cell->width = c->nets[src].width;
        return out;
    }

    /* Integer literal → CQ_CONST cell */
    if (t->type == TT_INT) {
        padv(c);
        uint16_t out = new_net(c, NULL, 8);
        Cell *cell   = add_cell(c, CQ_CONST, NULL);
        cell->literal = (uint32_t)t->ival;
        cell->out     = out;
        cell->width   = 8;
        snprintf(cell->comment, 127, "const 0x%X", (unsigned)t->ival);
        return out;
    }

    /* Identifier: register, wire, RAM read */
    if (t->type == TT_IDENT) {
        padv(c);
        char name[MAX_IDENT];
        strncpy(name, t->val, MAX_IDENT-1);

        /* Memory read: RAM[addr] */
        if (ptok(c)->type == TT_LBRACKET) {
            padv(c);
            uint16_t addr = parse_expr(c);
            pexpect(c, TT_RBRACKET);
            uint16_t out = new_net(c, NULL, 8);
            Cell *cell   = add_cell(c, CQ_MEM_RD, name);
            Symbol *sym  = find_sym(c, name);
            cell->a      = addr;
            cell->out    = out;
            cell->width  = sym ? (uint8_t)sym->width : 8;
            snprintf(cell->comment, 127, "%s[net_%d]", name, addr);
            return out;
        }

        /* Bit index: REG[n] */
        /* Regular signal */
        return find_net(c, name);
    }

    /* ( expr ) */
    if (t->type == TT_LPAREN) {
        padv(c);
        uint16_t r = parse_expr(c);
        pexpect(c, TT_RPAREN);
        return r;
    }

    die(c, "Unexpected token in expression: '%s'", t->val);
    return 0;
}

static uint16_t parse_binop(Compiler *c, uint16_t left, int min_prec) {
    for(;;) {
        Token *op = ptok(c);
        CQCellType ct; int prec;
        switch(op->type) {
        case TT_OR:     ct=CQ_OR;  prec=1; break;
        case TT_XOR:    ct=CQ_XOR; prec=2; break;
        case TT_AND:    ct=CQ_AND; prec=3; break;
        case TT_EQ:     ct=CQ_EQ;  prec=4; break;
        case TT_NEQ:    ct=CQ_NEQ; prec=4; break;
        case TT_LT:     ct=CQ_LT;  prec=5; break;
        case TT_GT:     ct=CQ_GT;  prec=5; break;
        case TT_LSHIFT: ct=CQ_SHL; prec=6; break;
        case TT_RSHIFT: ct=CQ_SHR; prec=6; break;
        case TT_PLUS:   ct=CQ_ADD; prec=7; break;
        case TT_MINUS:  ct=CQ_SUB; prec=7; break;
        default: return left;
        }
        if (prec < min_prec) return left;
        padv(c);
        uint16_t right = parse_primary(c);
        right = parse_binop(c, right, prec+1);
        uint16_t out  = new_net(c, NULL, c->nets[left].width);
        Cell *cell    = add_cell(c, ct, NULL);
        cell->a       = left;
        cell->b       = right;
        cell->out     = out;
        cell->width   = c->nets[left].width;
        snprintf(cell->comment, 127, "net%d op net%d → net%d", left, right, out);
        left = out;
    }
}

static uint16_t parse_expr(Compiler *c) {
    uint16_t left = parse_primary(c);
    return parse_binop(c, left, 0);
}

/* ═══════════════════════════════════════════════════════════════════
 *  STATEMENT PARSER
 *  Builds gate cells for each hardware statement
 * ═══════════════════════════════════════════════════════════════════ */
static void parse_stmt_list(Compiler *c);

static void parse_stmt(Compiler *c) {
    Token *t = ptok(c);

    /* if <cond> { ... } else { ... } */
    if (t->type == TT_IF) {
        padv(c);
        uint16_t cond = parse_expr(c);
        /* MUX cell: selector = cond */
        Cell *mux = add_cell(c, CQ_MUX, NULL);
        mux->sel  = cond;
        snprintf(mux->comment, 127, "if-mux on net%d", cond);
        pexpect(c, TT_LBRACE);
        parse_stmt_list(c);
        pexpect(c, TT_RBRACE);
        if (pmatch(c, TT_ELSE)) {
            pexpect(c, TT_LBRACE);
            parse_stmt_list(c);
            pexpect(c, TT_RBRACE);
        }
        return;
    }

    /* when <cond> { ... } else { ... } */
    if (t->type == TT_WHEN) {
        padv(c);
        uint16_t cond = parse_expr(c);
        Cell *mux = add_cell(c, CQ_MUX, NULL);
        mux->sel  = cond;
        snprintf(mux->comment, 127, "when-mux on net%d", cond);
        pexpect(c, TT_LBRACE);
        parse_stmt_list(c);
        pexpect(c, TT_RBRACE);
        if (pmatch(c, TT_ELSE)) {
            pexpect(c, TT_LBRACE);
            parse_stmt_list(c);
            pexpect(c, TT_RBRACE);
        }
        return;
    }

    /* <target> <= <expr>  or  <target>[idx] <= <expr>  (assignment) */
    if (t->type == TT_IDENT) {
        char target[MAX_IDENT];
        strncpy(target, t->val, MAX_IDENT-1);
        padv(c);

        /* Memory write: DMEM[addr] <= val */
        if (ptok(c)->type == TT_LBRACKET) {
            padv(c);
            uint16_t addr = parse_expr(c);
            pexpect(c, TT_RBRACKET);
            pmatch(c, TT_LARROW) || pmatch(c, TT_ASSIGN);
            uint16_t data = parse_expr(c);
            Cell *cell    = add_cell(c, CQ_MEM_WR, target);
            cell->a       = addr;
            cell->b       = data;
            snprintf(cell->comment, 127, "%s[net%d] <= net%d", target, addr, data);
            return;
        }

        /* Register assignment: REG <= expr */
        pmatch(c, TT_LARROW) || pmatch(c, TT_ASSIGN);
        uint16_t rhs = parse_expr(c);

        /* Find or create _next net for this register */
        char next_name[MAX_IDENT];
        snprintf(next_name, MAX_IDENT, "%s_next", target);
        uint16_t next_net = find_net(c, next_name);

        Cell *wire    = add_cell(c, CQ_WIRE, NULL);
        wire->a       = rhs;
        wire->out     = next_net;
        wire->width   = c->nets[rhs].width;
        snprintf(wire->comment, 127, "%s <= net%d", target, rhs);
        return;
    }

    /* Skip unexpected tokens gracefully */
    padv(c);
}

static void parse_stmt_list(Compiler *c) {
    while (ptok(c)->type != TT_RBRACE && ptok(c)->type != TT_EOF)
        parse_stmt(c);
}

/* ═══════════════════════════════════════════════════════════════════
 *  CHIP BODY PARSER
 * ═══════════════════════════════════════════════════════════════════ */
static void parse_chip_body(Compiler *c) {
    while (ptok(c)->type != TT_RBRACE && ptok(c)->type != TT_EOF) {
        Token *t = ptok(c);

        /* clock <n> : <freq> */
        if (t->type == TT_CLOCK) {
            padv(c);
            strncpy(c->clock_name, pexpect(c, TT_IDENT)->val, MAX_IDENT-1);
            pexpect(c, TT_COLON);
            padv(c); /* number */
            if (ptok(c)->type == TT_IDENT) padv(c); /* unit: mhz */
            continue;
        }

        /* reset <n> : <polarity> */
        if (t->type == TT_RESET) {
            padv(c);
            strncpy(c->reset_name, pexpect(c, TT_IDENT)->val, MAX_IDENT-1);
            pexpect(c, TT_COLON);
            padv(c);
            continue;
        }

        /* reg <n> : <width> = <reset> */
        if (t->type == TT_REG) {
            padv(c);
            Token *name  = pexpect(c, TT_IDENT);
            pexpect(c, TT_COLON);
            int    width = (int)pexpect(c, TT_INT)->ival;
            long long rst = 0;
            if (pmatch(c, TT_ASSIGN)) { Token *rv=padv(c); if(rv->type==TT_INT) rst=rv->ival; }

            Symbol *s = add_sym(c, name->val, SYM_REG);
            s->width  = width; s->reset = rst;

            /* Create net for this register */
            s->net_id = new_net(c, name->val, width);

            /* Create _next net */
            char nxt[MAX_IDENT]; snprintf(nxt, MAX_IDENT, "%s_next", name->val);
            new_net(c, nxt, width);

            /* DFF cell */
            Cell *dff = add_cell(c, CQ_DFF, name->val);
            dff->a    = s->net_id;   /* Q output net */
            dff->out  = s->net_id;
            dff->width= (uint8_t)width;
            dff->literal = (uint32_t)rst;
            snprintf(dff->comment, 127, "DFF reg %s : %d bits", name->val, width);

            printf("  REG   %-12s : %d bits  [net%d]\n", name->val, width, s->net_id);
            continue;
        }

        /* wire <n> : <width> */
        if (t->type == TT_WIRE) {
            padv(c);
            Token *name  = pexpect(c, TT_IDENT);
            pexpect(c, TT_COLON);
            int    width = (int)pexpect(c, TT_INT)->ival;
            Symbol *s    = add_sym(c, name->val, SYM_WIRE);
            s->width     = width;
            s->net_id    = new_net(c, name->val, width);
            continue;
        }

        /* bus <n> : <width> */
        if (t->type == TT_BUS) {
            padv(c);
            Token *name  = pexpect(c, TT_IDENT);
            pexpect(c, TT_COLON);
            int    width = (int)pexpect(c, TT_INT)->ival;
            Symbol *s    = add_sym(c, name->val, SYM_BUS);
            s->width     = width;
            s->net_id    = new_net(c, name->val, width);
            continue;
        }

        /* ram <n> : <depth> x <width> = rom("file") */
        if (t->type == TT_RAM) {
            padv(c);
            Token *name  = pexpect(c, TT_IDENT);
            pexpect(c, TT_COLON);
            int depth    = (int)pexpect(c, TT_INT)->ival;
            pexpect(c, TT_IDENT); /* 'x' */
            int width    = (int)pexpect(c, TT_INT)->ival;
            char init[MAX_IDENT] = "";
            if (pmatch(c, TT_ASSIGN)) {
                padv(c); /* 'rom' */
                pexpect(c, TT_LPAREN);
                strncpy(init, pexpect(c, TT_STRING)->val, MAX_IDENT-1);
                pexpect(c, TT_RPAREN);
            }
            Symbol *s  = add_sym(c, name->val, SYM_RAM);
            s->width   = width; s->depth = depth;
            strncpy(s->init_file, init, MAX_IDENT-1);
            s->net_id  = new_net(c, name->val, width);

            /* BRAM cell */
            Cell *bram = add_cell(c, CQ_BRAM, name->val);
            bram->width= (uint8_t)width;
            bram->b    = (uint16_t)depth;
            strncpy(bram->comment, init[0] ? init : "RAM", 127);
            printf("  RAM   %-12s : %dx%d\n", name->val, depth, width);
            continue;
        }

        /* const <n> : <width> = <val> */
        if (t->type == TT_CONST) {
            padv(c);
            Token *name  = pexpect(c, TT_IDENT);
            pexpect(c, TT_COLON);
            int    width = (int)pexpect(c, TT_INT)->ival;
            pexpect(c, TT_ASSIGN);
            long long val = pexpect(c, TT_INT)->ival;
            Symbol *s    = add_sym(c, name->val, SYM_CONST);
            s->width     = width; s->value = val;
            s->net_id    = new_net(c, name->val, width);

            Cell *k      = add_cell(c, CQ_CONST, name->val);
            k->literal   = (uint32_t)val;
            k->out       = s->net_id;
            k->width     = (uint8_t)width;
            continue;
        }

        /* state <n> = <val> */
        if (t->type == TT_STATE) {
            padv(c);
            Token *name   = pexpect(c, TT_IDENT);
            pexpect(c, TT_ASSIGN);
            long long val = pexpect(c, TT_INT)->ival;
            Symbol *s     = add_sym(c, name->val, SYM_STATE);
            s->value      = val;
            s->net_id     = new_net(c, name->val, 2);

            Cell *k       = add_cell(c, CQ_CONST, name->val);
            k->literal    = (uint32_t)val;
            k->out        = s->net_id;
            k->width      = 2;
            continue;
        }

        /* logic { } — skip for now */
        if (t->type == TT_LOGIC) { padv(c); padv(c); skip_block(c); continue; }

        /* on clock.rise { ... } */
        if (t->type == TT_ON) {
            padv(c); padv(c);       /* clock name */
            pexpect(c, TT_DOT); padv(c); /* .rise */
            pexpect(c, TT_LBRACE);
            parse_stmt_list(c);
            pexpect(c, TT_RBRACE);
            continue;
        }

        /* out port <n> : <width> = <expr> */
        if (t->type == TT_OUT) {
            padv(c);
            pexpect(c, TT_PORT);
            Token *name  = pexpect(c, TT_IDENT);
            pexpect(c, TT_COLON);
            int    width = (int)pexpect(c, TT_INT)->ival;
            pexpect(c, TT_ASSIGN);
            uint16_t src = parse_expr(c);

            Port *p = &c->ports[c->port_count++];
            strncpy(p->name, name->val, MAX_IDENT-1);
            p->dir    = 1;
            p->width  = (uint8_t)width;
            p->net_id = src;

            Cell *op = add_cell(c, CQ_OUT_PORT, name->val);
            op->a    = src;
            op->width= (uint8_t)width;
            snprintf(op->comment, 127, "out port %s", name->val);
            printf("  PORT  %-12s : %d bits [net%d]\n", name->val, width, src);
            continue;
        }

        /* program { } — skip */
        if (t->type == TT_PROGRAM) { padv(c); padv(c); skip_block(c); continue; }

        padv(c); /* skip unknown */
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  .cqbit BINARY EMITTER
 *  Writes the CIRQ binary format — completely independent of Verilog
 * ═══════════════════════════════════════════════════════════════════ */

/* Write helpers — little-endian */
static void wu8 (FILE *f, uint8_t  v) { fwrite(&v,1,1,f); }
static void wu16(FILE *f, uint16_t v) { uint8_t b[2]={v&0xFF,(v>>8)&0xFF}; fwrite(b,1,2,f); }
static void wu32(FILE *f, uint32_t v) { uint8_t b[4]={v&0xFF,(v>>8)&0xFF,(v>>16)&0xFF,(v>>24)&0xFF}; fwrite(b,1,4,f); }
static void wstr(FILE *f, const char *s) { fwrite(s,1,strlen(s)+1,f); }

static void emit_cqbit(Compiler *c, FILE *f) {
    /* ── Header ── */
    fwrite(CQBIT_MAGIC, 1, 4, f);               /* magic: "CQBT" */
    wu16(f, CQBIT_VERSION);                      /* version 2.0   */
    wu16(f, (uint16_t)strlen(c->chip_name));     /* chip name len */
    wu32(f, (uint32_t)c->cell_count);            /* n_cells       */
    wu32(f, (uint32_t)c->net_count);             /* n_nets        */
    wu16(f, (uint16_t)c->port_count);            /* n_ports       */
    wu16(f, 0);                                  /* reserved      */
    wu32(f, 0);                                  /* reserved      */
    wu32(f, 0);                                  /* reserved      */

    /* ── Chip name ── */
    fwrite(c->chip_name, 1, strlen(c->chip_name), f);

    /* ── Clock / reset names ── */
    wstr(f, c->clock_name);
    wstr(f, c->reset_name);

    /* ── Cell section ── */
    /* Each cell: type(u8) width(u8) a(u16) b(u16) out(u16) sel(u16)
                  literal(u32) flags(u8)  — total 16 bytes/cell */
    for (int i = 0; i < c->cell_count; i++) {
        Cell *cell = &c->cells[i];
        wu8 (f, (uint8_t)cell->type);
        wu8 (f, cell->width);
        wu16(f, cell->a);
        wu16(f, cell->b);
        wu16(f, cell->out);
        wu16(f, cell->sel);
        wu32(f, cell->literal);
        wu8 (f, cell->flags);
        /* pad to 16 bytes */
        wu8(f, 0);
    }

    /* ── Net name table ── */
    /* Format: u16 count, then null-terminated names */
    wu16(f, (uint16_t)c->net_count);
    for (int i = 0; i < c->net_count; i++) {
        wu8(f, c->nets[i].width);
        wstr(f, c->nets[i].name);
    }

    /* ── Port table ── */
    wu16(f, (uint16_t)c->port_count);
    for (int i = 0; i < c->port_count; i++) {
        Port *p = &c->ports[i];
        wu8 (f, p->dir);
        wu8 (f, p->width);
        wu16(f, p->net_id);
        wstr(f, p->name);
    }

    /* ── Symbol table ── (for debugger / simulator) */
    wu16(f, (uint16_t)c->sym_count);
    for (int i = 0; i < c->sym_count; i++) {
        Symbol *s = &c->symbols[i];
        wu8 (f, (uint8_t)s->kind);
        wu8 (f, (uint8_t)s->width);
        wu16(f, s->net_id);
        wu32(f, (uint32_t)s->reset);
        wstr(f, s->name);
    }

    /* End marker */
    fwrite("CQEND", 1, 5, f);
}

/* ═══════════════════════════════════════════════════════════════════
 *  .cqbit TEXT DUMP  (human readable, like objdump for CIRQ)
 * ═══════════════════════════════════════════════════════════════════ */
static const char *cell_type_name(CQCellType t) {
    switch(t) {
    case CQ_DFF:     return "DFF";
    case CQ_BRAM:    return "BRAM";
    case CQ_ADD:     return "ADD";
    case CQ_SUB:     return "SUB";
    case CQ_AND:     return "AND";
    case CQ_OR:      return "OR";
    case CQ_XOR:     return "XOR";
    case CQ_NOT:     return "NOT";
    case CQ_SHL:     return "SHL";
    case CQ_SHR:     return "SHR";
    case CQ_INC:     return "INC";
    case CQ_DEC:     return "DEC";
    case CQ_MUX:     return "MUX";
    case CQ_EQ:      return "EQ";
    case CQ_NEQ:     return "NEQ";
    case CQ_LT:      return "LT";
    case CQ_GT:      return "GT";
    case CQ_WIRE:    return "WIRE";
    case CQ_CONST:   return "CONST";
    case CQ_MEM_RD:  return "MEM_RD";
    case CQ_MEM_WR:  return "MEM_WR";
    case CQ_OUT_PORT:return "OUT_PORT";
    case CQ_IN_PORT: return "IN_PORT";
    default:         return "UNKNOWN";
    }
}

static void dump_cqbit(Compiler *c) {
    printf("\n╔══════════════════════════════════════════════════════╗\n");
    printf("║  .cqbit dump — %s\n", c->chip_name);
    printf("╚══════════════════════════════════════════════════════╝\n");
    printf("  clock: %s    reset: %s\n\n", c->clock_name, c->reset_name);

    printf("  NETS (%d):\n", c->net_count);
    for (int i = 0; i < c->net_count && i < 20; i++)
        printf("    [%3d] %-20s : %d bits\n", i, c->nets[i].name, c->nets[i].width);
    if (c->net_count > 20) printf("    ... (%d more)\n", c->net_count - 20);

    printf("\n  CELLS (%d):\n", c->cell_count);
    for (int i = 0; i < c->cell_count && i < 30; i++) {
        Cell *cell = &c->cells[i];
        printf("    [%3d] %-10s  a=net%-4d b=net%-4d out=net%-4d  %s\n",
               i, cell_type_name(cell->type),
               cell->a, cell->b, cell->out, cell->comment);
    }
    if (c->cell_count > 30) printf("    ... (%d more)\n", c->cell_count - 30);

    printf("\n  PORTS (%d):\n", c->port_count);
    for (int i = 0; i < c->port_count; i++) {
        Port *p = &c->ports[i];
        printf("    [%3d] %s %-20s : %d bits → net%d\n",
               i, p->dir?"OUT":"IN ", p->name, p->width, p->net_id);
    }
    printf("\n");
}

/* ═══════════════════════════════════════════════════════════════════
 *  MAIN
 * ═══════════════════════════════════════════════════════════════════ */
int main(int argc, char **argv) {
    if (argc < 2) {
        printf("CIRQ Compiler v2.0 — Independent HDL\n\n");
        printf("Usage: cirqc <file.cirq> [options]\n\n");
        printf("Options:\n");
        printf("  -o <out.cqbit>   Output file (default: <input>.cqbit)\n");
        printf("  --dump           Dump .cqbit cell listing\n");
        printf("  --tokens         Dump token list\n\n");
        printf("Pipeline:\n");
        printf("  .cirq → cirqc → .cqbit → cqprog → FPGA\n");
        printf("  .cirq → cirqc → .cqbit → cqsim  → simulation\n");
        return 1;
    }

    const char *infile  = argv[1];
    const char *outfile = NULL;
    int do_dump   = 0;
    int do_tokens = 0;

    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i],"-o")==0 && i+1<argc) outfile = argv[++i];
        else if (strcmp(argv[i],"--dump")==0)     do_dump   = 1;
        else if (strcmp(argv[i],"--tokens")==0)   do_tokens = 1;
    }

    /* Default output filename */
    char default_out[512];
    if (!outfile) {
        strncpy(default_out, infile, sizeof(default_out)-10);
        char *dot = strrchr(default_out, '.'); if (dot) *dot='\0';
        strcat(default_out, ".cqbit");
        outfile = default_out;
    }

    /* Read source */
    FILE *sf = fopen(infile, "r");
    if (!sf) { perror(infile); return 1; }
    fseek(sf,0,SEEK_END); long sz=ftell(sf); rewind(sf);
    char *src = malloc(sz+1);
    if (!src) { fprintf(stderr,"OOM\n"); return 1; }
    if (fread(src,1,sz,sf) != (size_t)sz) { fprintf(stderr,"Read error\n"); return 1; }
    src[sz]='\0'; fclose(sf);

    /* Allocate compiler */
    Compiler *c = calloc(1, sizeof(Compiler));
    if (!c) { fprintf(stderr,"OOM\n"); return 1; }
    c->src = src; c->src_len=(int)sz;
    strcpy(c->clock_name, "clk");
    strcpy(c->reset_name, "rst");

    printf("╔══════════════════════════════════════════════╗\n");
    printf("║  CIRQ Compiler v2.0  (Independent HDL)      ║\n");
    printf("║  %s\n", infile);
    printf("╚══════════════════════════════════════════════╝\n\n");

    /* [1] Lex */
    printf("[1/4] Lexing...\n");
    lex(c);
    printf("      %d tokens\n", c->token_count);

    if (do_tokens) {
        for (int i=0;i<c->token_count;i++) {
            Token *t=&c->tokens[i];
            printf("L%3d C%3d  type=%-3d  val=%-20s  ival=%lld\n",
                   t->line,t->col,t->type,t->val,t->ival);
        }
        return 0;
    }

    /* [2] Parse */
    printf("[2/4] Parsing...\n");
    /* Find chip */
    while (ptok(c)->type != TT_EOF) {
        if (ptok(c)->type == TT_CHIP) {
            padv(c);
            strncpy(c->chip_name, pexpect(c, TT_IDENT)->val, MAX_IDENT-1);
            printf("      chip: %s\n", c->chip_name);
            pexpect(c, TT_LBRACE);
            parse_chip_body(c);
            pexpect(c, TT_RBRACE);
            break;
        } else if (ptok(c)->type == TT_PROGRAM) {
            padv(c); padv(c); skip_block(c);
        } else {
            padv(c);
        }
    }
    if (c->chip_name[0]=='\0') { fprintf(stderr,"No chip found\n"); return 1; }

    /* [3] Analyze */
    printf("[3/4] Analyzing...\n");
    int regs=0,rams=0;
    for (int i=0;i<c->sym_count;i++) {
        if (c->symbols[i].kind==SYM_REG) regs++;
        if (c->symbols[i].kind==SYM_RAM) rams++;
    }
    printf("      %d symbols  %d regs  %d RAMs  %d nets  %d cells\n",
           c->sym_count, regs, rams, c->net_count, c->cell_count);

    if (do_dump) dump_cqbit(c);

    /* [4] Emit .cqbit */
    printf("[4/4] Emitting %s...\n", outfile);
    FILE *of = fopen(outfile, "wb");
    if (!of) { perror(outfile); return 1; }
    emit_cqbit(c, of);
    fclose(of);

    /* Print file size */
    FILE *check = fopen(outfile, "rb");
    fseek(check,0,SEEK_END); long fsz=ftell(check); fclose(check);

    printf("\n✅ Compiled: %s  (%ld bytes)\n", outfile, fsz);
    printf("   Cells:   %d gate primitives\n", c->cell_count);
    printf("   Nets:    %d signals\n",          c->net_count);
    printf("   Ports:   %d physical pins\n\n",  c->port_count);
    printf("Next steps:\n");
    printf("  cqsim  run   %s          (simulate)\n", outfile);
    printf("  cqprog flash %s board    (program FPGA)\n", outfile);
    printf("  cqdump read  %s          (inspect binary)\n\n", outfile);

    free(src); free(c);
    return 0;
}

