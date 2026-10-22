// Copyright (c) 2026 -- ULX3S Flute Linux program (TestRIG RVFI-DII)
//
// BDPI backing store for the RVFI-DII harness's data memory: a sparse 32-bit
// physical space in 4 KiB pages, allocated on first write (reads of untouched
// pages return zero).  Which addresses are RAM is decided on the BSV side
// (DII_AXI4_Mem.bsv / SoC_Map.bsv).  TestRIG requires memory to be zero at
// the start of every trace: dii_mem_clear() (called after each bridge-driven
// reset) releases every page written since the previous clear.

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define PAGE_BITS 12
#define NPAGES    (1u << (32 - PAGE_BITS))

static uint8_t  *pages[NPAGES];
static uint32_t  touched[NPAGES];
static uint32_t  n_touched = 0;

void dii_mem_clear(void)
{
    for (uint32_t i = 0; i < n_touched; i++) {
        free(pages[touched[i]]);
        pages[touched[i]] = NULL;
    }
    n_touched = 0;
}

uint64_t dii_mem_read64(uint64_t addr)
{
    uint64_t a = addr & ~7ULL, v = 0;
    if (a >> 32) return 0;
    uint8_t *p = pages[a >> PAGE_BITS];
    if (!p) return 0;
    uint32_t off = a & ((1u << PAGE_BITS) - 1);
    for (int i = 7; i >= 0; i--) v = (v << 8) | p[off + i];
    return v;
}

void dii_mem_write64(uint64_t addr, uint64_t data, uint8_t strb)
{
    uint64_t a = addr & ~7ULL;
    if ((a >> 32) || (strb == 0)) return;
    uint32_t pn = a >> PAGE_BITS;
    if (!pages[pn]) {
        pages[pn] = calloc(1, 1u << PAGE_BITS);
        touched[n_touched++] = pn;
    }
    uint32_t off = a & ((1u << PAGE_BITS) - 1);
    for (int i = 0; i < 8; i++)
        if (strb & (1u << i)) pages[pn][off + i] = (uint8_t)(data >> (8 * i));
}
