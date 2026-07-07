// LZ4 streaming ring loader for STM32U5: decodes a single LZ4 block (raw block
// format) from a compressed-input SRAM ring into a 64KB decoded-window ring,
// burst-programming 128B chunks as they complete. LZ4's max match distance is
// 65535, so the 64KB decoded window covers every back-reference. Byte-wise
// decode ~10cyc/B: silicon becomes the critical path.
#include <stdint.h>
#include <stddef.h>

#define NSSR  (*(volatile uint32_t *)0x40022020u)
#define NSCR  (*(volatile uint32_t *)0x40022028u)
#define BSY_WDW     0x00030000u
#define NSCR_STRT   (1u << 16)
#define NSCR_PG_BWR ((1u << 0) | (1u << 14))

typedef struct {
    uint32_t ring_base;              // +0   compressed-input ring
    uint32_t ring_mask;              // +4
    uint32_t table;                  // +8   n_pages x {erase_cr, dst}
    uint32_t total_out;              // +12
    volatile uint32_t status;        // +16  0xAA entry, 1 ok, 2 err, 3 mismatch
    volatile uint32_t dbg_produced;  // +20
    volatile uint32_t dbg_code;      // +24
    volatile uint32_t wr;            // +28  host byte counter (monotonic)
    volatile uint32_t rd;            // +32  stub byte counter (monotonic)
    volatile uint32_t total_in;      // +36
    uint32_t window;                 // +40  decoded-window ring base (64KB)
} params_t;
#define P ((volatile params_t *)0x20007000u)
#define DWT_CTRL   (*(volatile uint32_t *)0xe0001000u)
#define DWT_CYCCNT (*(volatile uint32_t *)0xe0001004u)
#define DBG_ERASE  (*(volatile uint32_t *)0x2000702cu)   // params +44
#define DBG_BURST  (*(volatile uint32_t *)0x20007030u)   // params +48

typedef struct { uint32_t cr, dst; } page_t;

// --- compressed input: byte-at-a-time with ring flow control -------------
static const uint8_t *g_ring;
static uint32_t g_mask, g_rd, g_wr_cache;

static inline uint8_t in_byte(void)
{
    while (g_rd == g_wr_cache) {
        g_wr_cache = P->wr;
        // input exhaustion with bytes still demanded = corrupt stream; the
        // host also watches status, so just spin (host will kill us on stall)
    }
    uint8_t b = g_ring[g_rd & g_mask];
    g_rd++;
    if ((g_rd & 1023u) == 0)
        P->rd = g_rd;    // publish occasionally (host uses it for flow control)
    return b;
}

void _start(void)
{
    P->status = 0xAA;
    DWT_CTRL |= 1;
    DBG_ERASE = 0; DBG_BURST = 0;
    g_ring = (const uint8_t *)P->ring_base;
    g_mask = P->ring_mask;
    g_rd = 0; g_wr_cache = 0;

    const page_t *table = (const page_t *)P->table;
    uint32_t total = P->total_out;
    uint8_t *win = (uint8_t *)P->window;      // 64KB decoded ring
    const uint32_t WMASK = 0xFFFFu;

    uint32_t out = 0;        // decoded bytes (monotonic)
    uint32_t burned = 0;     // bytes burst-programmed (monotonic, 128-aligned)
    uint32_t page_idx = 0;
    volatile uint32_t *dst = 0;

    // burn any complete 128B chunks the decoder has produced
    #define BURN_READY() do { \
        while (out - burned >= 128) { \
            if ((burned & 8191u) == 0) { \
                while (NSSR & BSY_WDW); \
                NSCR = 0; \
                NSCR = table[page_idx].cr; \
                NSCR = table[page_idx].cr | NSCR_STRT; \
                { uint32_t _c0 = DWT_CYCCNT; while (NSSR & BSY_WDW); DBG_ERASE += DWT_CYCCNT - _c0; } \
                NSCR = 0; \
                NSCR = NSCR_PG_BWR; \
                dst = (volatile uint32_t *)table[page_idx].dst; \
                page_idx++; \
            } \
            { uint32_t _c0 = DWT_CYCCNT; while (NSSR & BSY_WDW); DBG_BURST += DWT_CYCCNT - _c0; } \
            const uint32_t *w = (const uint32_t *)&win[burned & WMASK]; \
            for (int i = 0; i < 32; i++) \
                dst[i] = w[i]; \
            dst += 32; \
            burned += 128; \
        } \
    } while (0)

    // ---- LZ4 block decode (raw block format) ----
    while (out < total) {
        uint8_t token = in_byte();
        uint32_t lit = token >> 4;

        if (lit == 15) {
            uint8_t b;
            do { b = in_byte(); lit += b; } while (b == 255);
        }

        while (lit--) {
            win[out & WMASK] = in_byte();
            out++;
            if ((out & 127u) == 0) BURN_READY();
        }

        if (out >= total)
            break;   // last sequence is literals-only

        uint32_t off = in_byte();
        off |= (uint32_t)in_byte() << 8;
        if (off == 0) { P->dbg_code = 1; P->status = 2; goto halt; }

        uint32_t mlen = token & 15;
        if (mlen == 15) {
            uint8_t b;
            do { b = in_byte(); mlen += b; } while (b == 255);
        }
        mlen += 4;

        uint32_t src = out - off;   // window-relative (monotonic arithmetic)
        while (mlen--) {
            win[out & WMASK] = win[src & WMASK];
            out++; src++;
            if ((out & 127u) == 0) BURN_READY();
        }
    }

    BURN_READY();
    while (NSSR & BSY_WDW);
    NSCR = 0;
    P->rd = g_rd;
    P->dbg_produced = out;
    P->status = (out == total && burned == total) ? 1 : 3;
halt:
    for (;;) __asm volatile ("bkpt 0");
}
