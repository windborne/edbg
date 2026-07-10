/*
 * Copyright (c) 2026, WindBorne Systems
 * All rights reserved.
 *
 * Open reimplementation of the STM32U5 target driver (the original source was
 * lost; behavior recovered from the disassembly of the shipped binary and
 * verified against the STM32U5A5 CMSIS headers / RM0456). Loader/verify design
 * notes draw on stlink-org/stlink (BSD-3) and target_st_stm32g0.c.
 *
 * Improvements over the lost driver:
 *  - program(): one queued block write per 8 KB page instead of a USB flush +
 *    status poll per 16-byte quad-word (the flash stalls AHB while busy, the
 *    MEM-AP answers WAIT and the probe retries, so per-quad polling is
 *    redundant; sticky NSSR errors are checked once per page).
 *    Set EDBG_U5_SAFE_WRITES=1 to replicate the old per-quad behavior.
 *  - verify(): CRC32 computed on-target (tiny RAM stub feeding the hardware
 *    CRC peripheral) instead of a full readback; falls back to readback on any
 *    stub trouble, and on CRC mismatch reruns readback for a byte-accurate
 *    diagnostic. Set EDBG_U5_NO_STUB=1 to force readback verify.
 *  - erase/program honor OPTR.SWAP_BANK when computing the physical bank
 *    (the lost driver did not, which mistargets page erases on a swapped board).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*- Includes ----------------------------------------------------------------*/
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/time.h>

static long long prof_us(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (long long)tv.tv_sec * 1000000 + tv.tv_usec;
}
static long long prof_compress, prof_stage, prof_wait, prof_erase, prof_kick;
#define PROF(bucket, expr) do { long long _t0 = prof_us(); expr; prof_##bucket += prof_us() - _t0; } while (0)
#include "target.h"
#include "edbg.h"
#include "dap.h"
#include "lz4stub_bin.h"
#include "lz4.h"
#include "lz4hc.h"

/*- Definitions -------------------------------------------------------------*/
#define FLASH_ADDR             0x08000000
#define FLASH_PAGE_SIZE        8192
#define FLASH_QUAD_SIZE        16
#define READ_CHUNK_SIZE        32768    // words per dap_transfer batch: spans ~64 USB packets, exploits host pipelining

#define DHCSR                  0xe000edf0
#define DHCSR_DEBUGEN          (1 << 0)
#define DHCSR_HALT             (1 << 1)
#define DHCSR_MASKINTS         (1 << 3)
#define DHCSR_S_REGRDY         (1 << 16)
#define DHCSR_S_HALT           (1 << 17)
#define DHCSR_DBGKEY           (0xa05fu << 16)

#define DCRSR                  0xe000edf4
#define DCRSR_WRITE            (1 << 16)
#define DCRDR                  0xe000edf8

#define DEMCR                  0xe000edfc
#define DEMCR_VC_CORERESET     (1 << 0)

#define AIRCR                  0xe000ed0c
#define AIRCR_VECTKEY          (0x05fau << 16)
#define AIRCR_SYSRESETREQ      (1 << 2)

#define DBGMCU_IDCODE          0xe0044000
#define DBGMCU_CR              0xe0044004
#define DBGMCU_CR_DBG_LP       0x6      // DBG_STOP | DBG_STANDBY (matches the lost driver)

#define FLASH_NSKEYR           0x40022008
#define FLASH_OPTKEYR          0x40022010
#define FLASH_NSSR             0x40022020
#define FLASH_NSCR             0x40022028
#define FLASH_OPTR             0x40022040

#define FLASHSIZE_REG          0x0bfa07a0   // 16-bit size in KB
#define UID_REG                0x0bfa0700   // 96-bit unique ID

#define FLASH_KEY1             0x45670123
#define FLASH_KEY2             0xcdef89ab
#define FLASH_OPTKEY1          0x08192a3b
#define FLASH_OPTKEY2          0x4c5d6e7f

#define FLASH_NSSR_EOP         (1 << 0)
#define FLASH_NSSR_BSY         (1 << 16)
#define FLASH_NSSR_WDW         (1 << 17)
#define FLASH_NSSR_ALL_ERRORS  0x20fa       // OPERR|PROGERR|WRPERR|PGAERR|SIZERR|PGSERR|OPTWERR

#define FLASH_NSCR_PG          (1 << 0)
#define FLASH_NSCR_PER         (1 << 1)
#define FLASH_NSCR_MER1        (1 << 2)
#define FLASH_NSCR_PNB(x)      (((x) & 0xff) << 3)
#define FLASH_NSCR_BKER        (1 << 11)
#define FLASH_NSCR_BWR         (1 << 14)
#define FLASH_NSCR_MER2        (1 << 15)
#define FLASH_NSCR_STRT        (1 << 16)
#define FLASH_NSCR_LOCK        (1u << 31)

#define FLASH_ACR              0x40022000   // LATENCY[3:0]

#define RCC_CR                 0x46020c00   // MSISRDY = bit 2
#define RCC_CR_MSISRDY         (1 << 2)
#define RCC_ICSCR1             0x46020c08
#define RCC_ICSCR1_MSIRGSEL    (1 << 23)
#define RCC_ICSCR1_RANGE_16MHZ (2u << 28)   // MSISRANGE: range 2 = 16 MHz (reset is range 4 = 4 MHz)
#define RCC_ICSCR1_RANGE_48MHZ (0u << 28)   // MSISRANGE: range 0 = 48 MHz
#define PWR_VOSR               0x4602080c   // VOS[1:0]@16, VOSRDY@15
#define PWR_VOSR_VOS3          (1u << 16)   // range 3: up to 55 MHz
#define PWR_VOSR_VOSRDY        (1u << 15)

#define FLASH_OPTR_RDP_MASK    0x000000ff
#define FLASH_OPTR_SWAP_BANK   (1 << 20)
#define FLASH_OPTR_TZEN        (1u << 31)

#define DEVICE_ID_MASK         0x00000fff

#define SRAM_ADDR              0x20000000
#define STUB_SP                (SRAM_ADDR + 0x8000)
// Stub/buffer arena: 0x20000000..0x20004200 (+ a fault-frame just under STUB_SP).
// This range lies inside the app's .data/.bss (reinitialized every boot), so
// nothing persistent is disturbed: the b4 image places .noinit (persistent_info_t,
// survives warm resets) AFTER .bss, which ends far above (>0x20050000 on current
// images). If images ever shrink dramatically, re-check that .noinit stays clear.
#define WSTUB_ADDR             (SRAM_ADDR + 0x40)     // burst write stub (CRC stub sits at SRAM_ADDR)
#define VTOR_REG               0xe000ed08
#define SRAM_VTABLE            (SRAM_ADDR + 0x80)     // 128-aligned; all vectors -> the write stub's bkpt
#define WBUF_A                 (SRAM_ADDR + 0x100)    // page staging buffer A
#define WBUF_B                 (SRAM_ADDR + 0x2200)   // page staging buffer B
#define RINGSTUB_LOAD          (SRAM_ADDR + 0x4400)   // LZ4 ring-loader stub load address (text+bss)
#define RINGPARAMS             (SRAM_ADDR + 0x7000)   // ring loader params (KEEP >= 1KB above stub bss end; check nm __bss_end)
#define RINGTABLE              (SRAM_ADDR + 0x7100)   // page table: n x {erase_cr, dst}
#define RING_BASE              (SRAM_ADDR + 0x10000)  // 64KB compressed-stream ring
#define RING_SIZE              0x10000
#define LZ4_WINDOW             (SRAM_ADDR + 0x30000)  // 64KB decoded-window ring (covers LZ4's max match distance)

#define CRC_DR                 0x40023000
#define CRC_CR                 0x40023008
#define CRC_CR_RESET           (1 << 0)
#define CRC_POLY               0x04c11db7
#define RCC_AHB1ENR            0x46020c88
#define RCC_AHB1ENR_CRCEN      (1 << 12)

#define IWDG_KR                0x40003000
#define IWDG_KEY_FEED          0xaaaa

#define WAIT_DONE_LIMIT        100000       // dead-board cap on status polling

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  uint32_t  idcode;
  char      *family;
  char      *name;
} device_t;

/*- Variables ---------------------------------------------------------------*/
static device_t devices[] =
{
  { 0x481, "stm32u5", "STM32U59/U5Axx" },
  { 0x482, "stm32u5", "STM32U57/U58xx" },
  { 0x476, "stm32u5", "STM32U5F/U5Gxx" },
  { 0x455, "stm32u5", "STM32U53/U54xx" },
};

static device_t target_device;
static target_options_t target_options;
static uint32_t pages_per_bank;
static bool swap_bank;

/*- Prototypes --------------------------------------------------------------*/
static uint32_t crc32_stm32(uint32_t crc, const uint8_t *data, uint32_t size);
static bool crc_stub_run(uint32_t addr, uint32_t size, uint32_t *crc);
static bool wstub_prepare(void);
static bool wstub_kick(uint32_t src, uint32_t dst);
static bool wstub_wait(void);
static bool ringstub_prepare(void);
static int  ring_program(uint8_t *file_data, bool *skip, uint32_t start_page,
                         uint32_t number_of_pages, uint32_t base_addr);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void flash_wait_done(void)
{
  uint32_t sr = 0;
  int i;

  for (i = 0; i < WAIT_DONE_LIMIT; i++)
  {
    sr = dap_read_word(FLASH_NSSR);

    if (0 == (sr & (FLASH_NSSR_BSY | FLASH_NSSR_WDW)))
      break;
  }

  if (i == WAIT_DONE_LIMIT)
    error_exit("flash operation timeout. FLASH_NSSR = 0x%08x", sr);

  if (sr & FLASH_NSSR_ALL_ERRORS)
    error_exit("flash operation failed. FLASH_NSSR = 0x%08x", sr);
}

//-----------------------------------------------------------------------------
static void target_select(target_options_t *options)
{
  uint32_t idcode, flash_size, optr;
  bool locked;

  dap_disconnect();
  dap_connect(DAP_INTERFACE_SWD);
  dap_reset_pin(0);
  dap_reset_link();

  // Stop the core (reset-catch: halt out of SYSRESETREQ before any user code runs)
  dap_write_word(DHCSR, DHCSR_DBGKEY | DHCSR_DEBUGEN | DHCSR_HALT);
  dap_write_word(DEMCR, DEMCR_VC_CORERESET);
  dap_write_word(AIRCR, AIRCR_VECTKEY | AIRCR_SYSRESETREQ);

  dap_reset_pin(1);
  sleep_ms(10);

  // Keep the debug interface alive across low-power states
  dap_write_word(DBGMCU_CR, DBGMCU_CR_DBG_LP);

  idcode = dap_read_word(DBGMCU_IDCODE);

  for (int i = 0; i < ARRAY_SIZE(devices); i++)
  {
    if (devices[i].idcode != (idcode & DEVICE_ID_MASK))
      continue;

    verbose("Target: %s\n", devices[i].name);

    target_device = devices[i];
    target_options = *options;

    flash_size = dap_read_word(FLASHSIZE_REG) & 0xffff;

    if (0 == flash_size || 0xffff == flash_size)
      flash_size = 4096;   // blank/unreadable size register: assume 4 MB

    flash_size *= 1024;
    pages_per_bank = (flash_size / 2) / FLASH_PAGE_SIZE;

    target_check_options(&target_options, flash_size, FLASH_PAGE_SIZE);

    optr = dap_read_word(FLASH_OPTR);

    swap_bank = (0 != (optr & FLASH_OPTR_SWAP_BANK));

    verbose("OPTR: 0x%08x (RDP 0x%02x%s%s)\n", optr, optr & FLASH_OPTR_RDP_MASK,
        swap_bank ? ", SWAP_BANK" : "", (optr & FLASH_OPTR_TZEN) ? ", TZEN" : "");

    if (optr & FLASH_OPTR_TZEN)
      warning("OPTR.TZEN is set; only the non-secure flash interface is supported\n");

    locked = (0xaa != (optr & FLASH_OPTR_RDP_MASK));

    if (locked && !options->unlock)
      error_exit("target is locked, unlock is necessary");

    dap_write_word(FLASH_NSKEYR, FLASH_KEY1);
    dap_write_word(FLASH_NSKEYR, FLASH_KEY2);
    dap_write_word(FLASH_OPTKEYR, FLASH_OPTKEY1);
    dap_write_word(FLASH_OPTKEYR, FLASH_OPTKEY2);
    dap_write_word(FLASH_NSCR, 0);

    check(0 == (dap_read_word(FLASH_NSCR) & FLASH_NSCR_LOCK),
        "Failed to unlock the flash for write operation. Try to power cycle the target.");

    // Clear stale sticky flags (write-1-to-clear)
    dap_write_word(FLASH_NSSR, FLASH_NSSR_ALL_ERRORS | FLASH_NSSR_EOP);

    return;
  }

  error_exit("unknown target device (DBGMCU_IDCODE = 0x%08x)", idcode);
}

//-----------------------------------------------------------------------------
static void target_deselect(void)
{
  dap_write_word(DEMCR, 0);

  // Clear the debug control state before the release reset. A leftover C_MASKINTS
  // (set by the CRC stub harness) would otherwise leave the rebooted firmware
  // running with every interrupt masked -- alive but with no SysTick/scheduler/
  // console, which looks exactly like a dead board.
  //
  // ORDER MATTERS: on Cortex-M, C_MASKINTS is only writable while C_DEBUGEN=1 AND
  // C_HALT=1 (ARMv8-M DHCSR rules), so clearing all three bits in one write can
  // leave the mask set. Unmask FIRST (keep DEBUGEN+HALT, clear MASKINTS), THEN
  // release DEBUGEN/HALT in a second write.
  dap_write_word(DHCSR, DHCSR_DBGKEY | DHCSR_DEBUGEN | DHCSR_HALT);  // C_MASKINTS := 0, still halted
  dap_write_word(DHCSR, DHCSR_DBGKEY);                               // release C_DEBUGEN/C_HALT

  dap_write_word(AIRCR, AIRCR_VECTKEY | AIRCR_SYSRESETREQ);

  target_free_options(&target_options);
}

//-----------------------------------------------------------------------------
static void target_erase(void)
{
  dap_write_word(FLASH_NSCR, FLASH_NSCR_MER1 | FLASH_NSCR_MER2);
  dap_write_word(FLASH_NSCR, FLASH_NSCR_MER1 | FLASH_NSCR_MER2 | FLASH_NSCR_STRT);
  flash_wait_done();
  dap_write_word(FLASH_NSCR, 0);
}

//-----------------------------------------------------------------------------
static void target_lock(void)
{
  error_exit("target_lock() is not implemented yet");
}

//-----------------------------------------------------------------------------
static void target_unlock(void)
{
  error_exit("target_unlock() is not implemented yet");
}

//-----------------------------------------------------------------------------
static uint32_t erase_cr_for(uint32_t linear_page)
{
  bool high_half = (linear_page >= pages_per_bank);
  uint32_t pnb = high_half ? (linear_page - pages_per_bank) : linear_page;
  uint32_t cr = FLASH_NSCR_PER | FLASH_NSCR_PNB(pnb);

  if (high_half != swap_bank)
    cr |= FLASH_NSCR_BKER;

  return cr;
}

static void erase_page(uint32_t linear_page)
{
  // PNB/BKER address PHYSICAL banks. With OPTR.SWAP_BANK set, the low linear
  // half at 0x08000000 is physically bank 2, so the bank bit must be inverted
  // relative to the linear address (the lost driver got this wrong).
  bool high_half = (linear_page >= pages_per_bank);
  uint32_t pnb = high_half ? (linear_page - pages_per_bank) : linear_page;
  uint32_t cr = FLASH_NSCR_PER | FLASH_NSCR_PNB(pnb);

  if (high_half != swap_bank)   // == high_half XOR swap_bank
    cr |= FLASH_NSCR_BKER;

  dap_write_word(FLASH_NSCR, cr);
  dap_write_word(FLASH_NSCR, cr | FLASH_NSCR_STRT);
  flash_wait_done();
}

//-----------------------------------------------------------------------------
static void target_program(void)
{
  uint32_t addr = FLASH_ADDR + target_options.offset;
  uint32_t offs = 0;
  uint32_t start_page, number_of_pages;
  uint8_t *buf = target_options.file_data;
  uint32_t size = target_options.file_size;
  bool safe_writes = (NULL != getenv("EDBG_U5_SAFE_WRITES"));
  // Burst-write mode (BWR): flash programs 8 quad-words (128 B) per operation
  // instead of one, cutting tPROG per byte ~2-3x. Our write stream is naturally
  // burst-aligned (pages are 8192 B; the DAP layer re-TARs on 1 KB boundaries =
  // 8 bursts exactly), and a stalled feed just parks the AHB in WAIT like the
  // quad-word path. Experimental: enable with EDBG_U5_BURST=1.
  uint32_t pg_bits = FLASH_NSCR_PG | ((NULL != getenv("EDBG_U5_BURST")) ? FLASH_NSCR_BWR : 0);

  start_page = target_options.offset / FLASH_PAGE_SIZE;
  number_of_pages = (size + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;

  // Differential flash: CRC each target page on-chip (via the RAM stub, ~5-10 ms
  // per page) and skip erase+program for pages that already hold the new content.
  // A re-flash of an identical image touches nothing; a typical iteration only
  // rewrites the changed pages. Any skipped-but-stale page would still be caught
  // by the whole-image CRC verify (and its readback fallback). Skipped pages
  // print ',' instead of '.'. EDBG_U5_FULL_WRITE=1 disables the scan.
  // (file_data is 0xff-padded to the page-aligned options->size, so full-page
  // host CRCs are well defined for the final partial page.)
  bool *skip = buf_alloc(number_of_pages * sizeof(bool));
  memset(skip, 0, number_of_pages * sizeof(bool));

  if (NULL == getenv("EDBG_U5_FULL_WRITE") && NULL == getenv("EDBG_U5_NO_STUB"))
  {
    // Cold-image fast path: probe 3 spread pages first; if all differ, this is
    // a fresh image -- skip the remaining 60+ scan round trips (correctness
    // unaffected: unscanned pages are simply programmed).
    bool probably_cold = (number_of_pages >= 8);

    if (probably_cold)
    {
      uint32_t samples[3] = { 0, number_of_pages / 2, number_of_pages - 1 };

      for (int i = 0; i < 3; i++)
      {
        uint32_t pg = samples[i], crc_dev;

        if (crc_stub_run(addr + pg * FLASH_PAGE_SIZE, FLASH_PAGE_SIZE, &crc_dev) &&
            crc_dev == crc32_stm32(0xffffffff, &buf[pg * FLASH_PAGE_SIZE], FLASH_PAGE_SIZE))
        {
          probably_cold = false;   // something matches: do the full scan
          break;
        }
      }
    }
    else
      probably_cold = false;

    if (!probably_cold)
    {
      for (uint32_t page = 0; page < number_of_pages; page++)
      {
        uint32_t crc_dev, crc_host;

        if (!crc_stub_run(addr + page * FLASH_PAGE_SIZE, FLASH_PAGE_SIZE, &crc_dev))
        {
          warning("page-CRC stub did not run; programming all remaining pages\n");
          break;
        }

        crc_host = crc32_stm32(0xffffffff, &buf[page * FLASH_PAGE_SIZE], FLASH_PAGE_SIZE);
        skip[page] = (crc_dev == crc_host);
      }
    }
  }

  // Preferred path: streaming ring loader (one stub run for the whole image)
  if (!safe_writes && (NULL == getenv("EDBG_U5_NO_STUB")) && (NULL == getenv("EDBG_U5_NO_RING"))
      && ringstub_prepare())
  {
    if (ring_program(buf, skip, start_page, number_of_pages, FLASH_ADDR + target_options.offset))
    {
      for (uint32_t p = 0; p < number_of_pages; p++)
        verbose(skip[p] ? "," : ".");
      buf_free(skip);
      dap_write_word(FLASH_NSCR, 0);
      return;
    }
    warning("ring loader unavailable; using per-page path\n");
  }

  // Per-page write path: RAM-stub burst writer, double-buffered -- stage page
  // N+1 into SRAM (pure wire, no flash stalls) while the stub burst-programs
  // page N (pure tPROG); each hides under the other. Falls back to direct
  // writes when the stub can't run. EDBG_U5_SAFE_WRITES/EDBG_U5_NO_STUB opt out.
  bool stub_writes = !safe_writes && (NULL == getenv("EDBG_U5_NO_STUB")) && wstub_prepare();
  bool inflight = false;
  uint32_t bufsel = 0;

  for (uint32_t page = 0; page < number_of_pages; page++)
  {
    if (skip[page])
    {
      addr += FLASH_PAGE_SIZE;
      offs += FLASH_PAGE_SIZE;
      verbose(",");
      continue;
    }

    dap_write_word(IWDG_KR, IWDG_KEY_FEED);   // harmless if IWDG not running

    if (stub_writes)
    {
      uint32_t sram_buf = bufsel ? WBUF_B : WBUF_A;

      PROF(stage, dap_block_write(sram_buf, &buf[offs], FLASH_PAGE_SIZE));

      if (inflight)
      {
        long long _t0 = prof_us();
        if (!wstub_wait())
          error_exit("burst write stub stalled; rerun to repair (differential flash recovers fast)");
        flash_wait_done();
        prof_wait += prof_us() - _t0;
      }

      PROF(erase, {
        dap_write_word(FLASH_NSCR, 0);
        erase_page(start_page + page);
        dap_write_word(FLASH_NSCR, FLASH_NSCR_PG | FLASH_NSCR_BWR);
      });

      if (NULL != getenv("EDBG_DEBUG_STUBS"))
        warning("kick page=%u buf=%c addr=0x%08x\n",
            (unsigned)(start_page + page), bufsel ? 'B' : 'A', addr);

      long long _tk = prof_us();
      if (!wstub_kick(sram_buf, addr))
        error_exit("burst write stub kick failed; rerun to repair");
      prof_kick += prof_us() - _tk;
      inflight = true;
      bufsel ^= 1;

      addr += FLASH_PAGE_SIZE;
      offs += FLASH_PAGE_SIZE;
      verbose(".");
      continue;
    }

    erase_page(start_page + page);

    dap_write_word(FLASH_NSCR, pg_bits);

    if (safe_writes)
    {
      // Conservative path: flush + wait per 16-byte quad-word (lost-driver behavior)
      for (int i = 0; i < (FLASH_PAGE_SIZE / FLASH_QUAD_SIZE); i++)
      {
        dap_write_block(addr, &buf[offs], FLASH_QUAD_SIZE);
        addr += FLASH_QUAD_SIZE;
        offs += FLASH_QUAD_SIZE;

        while (dap_read_word(FLASH_NSSR) & (FLASH_NSSR_BSY | FLASH_NSSR_WDW));
      }
    }
    else
    {
      // Fast path: queue the whole page as back-to-back word writes. While a
      // quad-word is programming the flash stalls the AHB read/write, the
      // MEM-AP answers WAIT and the probe retries (edbg configures a 32768
      // retry budget), so no per-quad status polling is needed. Errors are
      // sticky in NSSR and checked below.
      dap_block_write(addr, &buf[offs], FLASH_PAGE_SIZE);
      addr += FLASH_PAGE_SIZE;
      offs += FLASH_PAGE_SIZE;
    }

    verbose(".");

    flash_wait_done();
  }

  if (inflight)
  {
    if (!wstub_wait())
      error_exit("burst write stub stalled; rerun to repair (differential flash recovers fast)");
    flash_wait_done();
  }

  if (NULL != getenv("EDBG_PROFILE"))
    warning("profile(ms): compress=%lld stage=%lld wait=%lld erase=%lld kick=%lld\n",
        prof_compress/1000, prof_stage/1000, prof_wait/1000, prof_erase/1000, prof_kick/1000);

  buf_free(skip);
  dap_write_word(FLASH_NSCR, 0);
}

//-----------------------------------------------------------------------------
// CRC-32/MPEG-2 as the U5 CRC peripheral computes it at reset defaults:
// poly 0x04C11DB7 MSB-first, init 0xFFFFFFFF, no reflection, no final xor,
// fed 32-bit words assembled little-endian from the byte stream.
static uint32_t crc32_stm32(uint32_t crc, const uint8_t *data, uint32_t size)
{
  for (uint32_t i = 0; i < size; i += 4)
  {
    uint32_t w = (uint32_t)data[i] | ((uint32_t)data[i+1] << 8) |
        ((uint32_t)data[i+2] << 16) | ((uint32_t)data[i+3] << 24);

    crc ^= w;

    for (int b = 0; b < 32; b++)
      crc = (crc & 0x80000000u) ? ((crc << 1) ^ CRC_POLY) : (crc << 1);
  }

  return crc;
}

//-----------------------------------------------------------------------------
static bool core_write_reg(int regno, uint32_t value)
{
  dap_write_word(DCRDR, value);
  dap_write_word(DCRSR, DCRSR_WRITE | regno);

  for (int i = 0; i < 1000; i++)
  {
    if (dap_read_word(DHCSR) & DHCSR_S_REGRDY)
      return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
// On-target CRC via a 12-byte RAM stub feeding the hardware CRC peripheral:
//   loop: ldr r3, [r0], #4 ; str r3, [r2] ; cmp r0, r1 ; bne loop ; bkpt #0
// r0 = addr, r1 = addr + size, r2 = CRC_DR. Prepared once per session; each
// run resets the CRC and hashes one range. Used both for verify and for the
// differential-program page scan. All helpers return false on stub trouble
// (callers fall back to plain readback / full programming).
static bool crc_stub_ready = false;

static bool crc_stub_prepare(void)
{
  static uint8_t stub[] = {
    0x50, 0xf8, 0x04, 0x3b,   // ldr   r3, [r0], #4
    0x13, 0x60,               // str   r3, [r2]
    0x88, 0x42,               // cmp   r0, r1
    0xfa, 0xd1,               // bne   loop
    0x00, 0xbe,               // bkpt  #0
  };

  if (crc_stub_ready)
    return true;

  // Enable the CRC peripheral clock from the host side
  dap_write_word(RCC_AHB1ENR, dap_read_word(RCC_AHB1ENR) | RCC_AHB1ENR_CRCEN);

  // Clock the core up for the stub: MSIS 4 -> 16 MHz (still within VOS4 limits)
  // with 1 flash wait state. ~4x faster page scans and image CRC. Any mistake
  // here degrades gracefully: the stub result self-checks against the host CRC
  // and every failure path falls back to readback. deselect()'s SYSRESETREQ
  // restores reset defaults. EDBG_U5_SLOW_STUB=1 skips the raise.
  if (NULL == getenv("EDBG_U5_SLOW_STUB"))
  {
    // 16 MHz (MSI range 2, 1 wait state): plenty since the LZ4 decoder costs
    // ~10 cycles/byte -- decode hides under the wire even at this clock, and
    // no VOS/regulator writes are needed (stays at the reset default range).
    // 48 MHz (VOS3 + 2WS + MSI range 0) was measured to buy only ~0.1-0.15s
    // on the scan/verify stubs and is not worth the extra target-state churn.
    dap_write_word(FLASH_ACR, (dap_read_word(FLASH_ACR) & ~0xf) | 1);
    dap_write_word(RCC_ICSCR1, (dap_read_word(RCC_ICSCR1) & ~(0xfu << 28)) |
        RCC_ICSCR1_MSIRGSEL | RCC_ICSCR1_RANGE_16MHZ);

    for (int i = 0; i < 100; i++)
    {
      if (dap_read_word(RCC_CR) & RCC_CR_MSISRDY)
        break;
    }
  }

  dap_write_block(SRAM_ADDR, stub, sizeof(stub));

  // Core is halted (reset-catch in select). Mask interrupts while halted and
  // load the run-invariant context. xPSR.T must be set (no valid vector table).
  dap_write_word(DHCSR, DHCSR_DBGKEY | DHCSR_DEBUGEN | DHCSR_HALT | DHCSR_MASKINTS);

  if (!core_write_reg(2, CRC_DR) ||
      !core_write_reg(13, STUB_SP) ||
      !core_write_reg(16, 0x01000000))
    return false;

  crc_stub_ready = true;
  return true;
}

static bool crc_stub_run(uint32_t addr, uint32_t size, uint32_t *crc)
{
  int timeout_ms;

  if (!crc_stub_prepare())
    return false;

  // Batch the whole invocation into one USB round trip: CRC reset, r0/r1/pc via
  // DCRDR/DCRSR, then the run. Skipping the per-register S_REGRDY poll is safe
  // here: a DCRSR reg write completes in a few core cycles, and the following
  // SWD word write cannot arrive sooner than several microseconds later.
  dap_write_word_req(CRC_CR, CRC_CR_RESET);
  dap_write_word_req(DCRDR, addr);
  dap_write_word_req(DCRSR, DCRSR_WRITE | 0);
  dap_write_word_req(DCRDR, addr + size);
  dap_write_word_req(DCRSR, DCRSR_WRITE | 1);
  dap_write_word_req(DCRDR, CRC_DR);
  dap_write_word_req(DCRSR, DCRSR_WRITE | 2);   // r2 is clobbered by the write stub (src_end) -- re-set every run
  dap_write_word_req(DCRDR, SRAM_ADDR);
  dap_write_word_req(DCRSR, DCRSR_WRITE | 15);
  dap_write_word_req(DHCSR, DHCSR_DBGKEY | DHCSR_DEBUGEN | DHCSR_MASKINTS);  // run (bkpt halts regardless)
  dap_transfer();

  // Stub at MSI 16 MHz: ~2.5 ms per 8 KB page (4x that if the clock raise was skipped)
  timeout_ms = 200 + (size / 1024) * 4;

  for (int t = 0; t < timeout_ms; t += 1)
  {
    if (dap_read_word(DHCSR) & DHCSR_S_HALT)
    {
      // Re-halt (belt and braces) and fetch the CRC in a single round trip
      dap_write_word_req(DHCSR, DHCSR_DBGKEY | DHCSR_DEBUGEN | DHCSR_HALT | DHCSR_MASKINTS);
      dap_read_word_req(CRC_DR);
      dap_transfer();
      *crc = dap_get_response(1);
      return true;
    }
  }

  // Stub never halted: force halt and report failure so the caller falls back
  dap_write_word(DHCSR, DHCSR_DBGKEY | DHCSR_DEBUGEN | DHCSR_HALT | DHCSR_MASKINTS);
  crc_stub_ready = false;
  return false;
}

//-----------------------------------------------------------------------------
// Burst page writer: a RAM stub copies a staged page from SRAM to flash with
// NSCR.PG|BWR set, words back-to-back at bus speed -- the in-spec way to feed
// 8-quad-word bursts (ST's HAL runs this exact loop with IRQs off; feeding
// bursts word-by-word over SWD leaves ~13us gaps and corrupts intermittently).
// The flash AHB-stalls the store while a burst programs, pacing the loop.
// Same shape as the CRC stub: r0=src, r1=dst, r2=src_end.
//   loop: ldr r3,[r0],#4 ; str r3,[r1],#4 ; cmp r0,r2 ; bne loop ; bkpt #0
// While the stub burns page N from one SRAM buffer, the host streams page N+1
// into the other (DAP->SRAM writes don't contend with the core->flash path).
static bool wstub_ready = false;

static bool wstub_prepare(void)
{
  // Assembled from (arm-none-eabi-as, verified by disassembly):
  //   ldr r3,=NSSR ; mov r5,#(BSY|WDW)
  //   outer: movs r6,#32
  //   inner: ldr r4,[r0],#4 ; str r4,[r1],#4 ; subs r6,#1 ; bne inner
  //   wait:  ldr r4,[r3] ; tst r4,r5 ; bne wait
  //   cmp r0,r2 ; bne outer ; bkpt #0 ; .word NSSR
  // 32 back-to-back word writes per 128B burst (bus-speed, the in-spec feed),
  // then wait BSY|WDW clear before the next burst -- the HAL's exact pattern.
  // Streaming words during BSY in burst mode raises PGSERR (bench-proven).
  static uint8_t stub[] = {
    0x07, 0x4b, 0x4f, 0xf4, 0x40, 0x35, 0x20, 0x26, 0x50, 0xf8, 0x04, 0x4b,
    0x41, 0xf8, 0x04, 0x4b, 0x01, 0x3e, 0xf9, 0xd1, 0x1c, 0x68, 0x2c, 0x42,
    0xfc, 0xd1, 0x90, 0x42, 0xf3, 0xd1, 0x00, 0xbe, 0x20, 0x20, 0x02, 0x40,
  };

  if (wstub_ready)
    return true;

  dap_write_block(WSTUB_ADDR, stub, sizeof(stub));

  // Minimal vector table in SRAM: any fault during a stub run lands on the
  // stub's bkpt and halts cleanly, instead of fetching vectors from (possibly
  // just-erased) flash and locking the core up beyond even a debug halt.
  {
    uint8_t vt[16 * 4];
    uint32_t bkpt = (WSTUB_ADDR + 0x1e) | 1;   // the stub's bkpt, thumb bit set

    for (int i = 0; i < 16; i++)
    {
      uint32_t v = (0 == i) ? STUB_SP : bkpt;
      vt[i*4+0] = v & 0xff; vt[i*4+1] = (v >> 8) & 0xff;
      vt[i*4+2] = (v >> 16) & 0xff; vt[i*4+3] = (v >> 24) & 0xff;
    }

    dap_write_block(SRAM_VTABLE, vt, sizeof(vt));
    dap_write_word(VTOR_REG, SRAM_VTABLE);
  }

  dap_write_word(DHCSR, DHCSR_DBGKEY | DHCSR_DEBUGEN | DHCSR_HALT | DHCSR_MASKINTS);

  if (!core_write_reg(13, STUB_SP) ||
      !core_write_reg(16, 0x01000000))
    return false;

  wstub_ready = true;
  return true;
}

// Kick the stub for one page (does not wait). NSCR.PG|BWR must already be set.
static bool wstub_kick(uint32_t src, uint32_t dst)
{
  if (!core_write_reg(0, src) ||
      !core_write_reg(2, src + FLASH_PAGE_SIZE) ||
      !core_write_reg(1, dst) ||
      !core_write_reg(15, WSTUB_ADDR))   // PC: else the core resumes at the previous stub's bkpt and no-ops
    return false;

  dap_write_word(DHCSR, DHCSR_DBGKEY | DHCSR_DEBUGEN | DHCSR_MASKINTS);
  return true;
}

static bool wstub_wait(void)
{
  for (int t = 0; t < 2000; t++)
  {
    if (dap_read_word(DHCSR) & DHCSR_S_HALT)
    {
      dap_write_word(DHCSR, DHCSR_DBGKEY | DHCSR_DEBUGEN | DHCSR_HALT | DHCSR_MASKINTS);
      return true;
    }
  }

  dap_write_word(DHCSR, DHCSR_DBGKEY | DHCSR_DEBUGEN | DHCSR_HALT | DHCSR_MASKINTS);

  // Post-mortem: where did the stub stop, and what does the flash say?
  {
    uint32_t r0, r1, sr;
    dap_write_word(DCRSR, 0);  for (int i = 0; i < 100 && !(dap_read_word(DHCSR) & DHCSR_S_REGRDY); i++);
    r0 = dap_read_word(DCRDR);
    dap_write_word(DCRSR, 1);  for (int i = 0; i < 100 && !(dap_read_word(DHCSR) & DHCSR_S_REGRDY); i++);
    r1 = dap_read_word(DCRDR);
    uint32_t pc, xpsr;
    dap_write_word(DCRSR, 15); for (int i = 0; i < 100 && !(dap_read_word(DHCSR) & DHCSR_S_REGRDY); i++);
    pc = dap_read_word(DCRDR);
    dap_write_word(DCRSR, 16); for (int i = 0; i < 100 && !(dap_read_word(DHCSR) & DHCSR_S_REGRDY); i++);
    xpsr = dap_read_word(DCRDR);
    sr = dap_read_word(FLASH_NSSR);
    warning("write stub timeout: src=0x%08x dst=0x%08x pc=0x%08x xpsr=0x%08x NSSR=0x%08x\n", r0, r1, pc, xpsr, sr);
  }

  return false;
}

//-----------------------------------------------------------------------------
// Load the LZ4 ring-loader stub into SRAM (once per session). It shares the
// burst-write primitives, so wstub_prepare() must run first.
static bool ringstub_ready = false;

static bool ringstub_prepare(void)
{
  if (ringstub_ready)
    return true;

  if (!wstub_prepare())
    return false;

  dap_write_block(RINGSTUB_LOAD, lz4stub_bin, (lz4stub_bin_len + 3) & ~3u);
  ringstub_ready = true;
  return true;
}

//-----------------------------------------------------------------------------
// Streaming ring loader: ONE stub run programs every non-skipped page. The
// whole image compresses as a single LZ4 stream (window stays warm
// across pages); the host streams it into a 64KB SRAM ring while the stub
// decodes, erases and burst-programs continuously. Wire time and silicon time
// finally run concurrently instead of interleaved.
static bool ring_kick(uint32_t total_out, uint32_t n_pages)
{
  (void)n_pages;
  dap_write_word_req(RINGPARAMS + 0,  RING_BASE);
  dap_write_word_req(RINGPARAMS + 4,  RING_SIZE - 1);
  dap_write_word_req(RINGPARAMS + 8,  RINGTABLE);
  dap_write_word_req(RINGPARAMS + 12, total_out);
  dap_write_word_req(RINGPARAMS + 16, 0);   // status
  dap_write_word_req(RINGPARAMS + 28, 0);   // wr
  dap_write_word_req(RINGPARAMS + 32, 0);   // rd
  dap_write_word_req(RINGPARAMS + 36, 0);   // total_in
  dap_write_word_req(RINGPARAMS + 40, LZ4_WINDOW);
  dap_write_word_req(DCRDR, STUB_SP);
  dap_write_word_req(DCRSR, DCRSR_WRITE | 13);
  dap_write_word_req(DCRDR, 0x01000000);
  dap_write_word_req(DCRSR, DCRSR_WRITE | 16);
  dap_write_word_req(DCRDR, lz4stub_entry);
  dap_write_word_req(DCRSR, DCRSR_WRITE | 15);
  dap_write_word_req(DHCSR, DHCSR_DBGKEY | DHCSR_DEBUGEN | DHCSR_MASKINTS);
  dap_transfer();
  return true;
}

// Returns 1 on verified success, 0 on any failure (caller falls back or exits)
static int ring_program(uint8_t *file_data, bool *skip, uint32_t start_page,
                        uint32_t number_of_pages, uint32_t base_addr)
{
  uint32_t n = 0;

  for (uint32_t p = 0; p < number_of_pages; p++)
    if (!skip[p])
      n++;

  if (0 == n)
    return 1;

  // Build the page table + the concatenated input for one compression stream
  uint8_t *table = buf_alloc(n * 8);
  uint8_t *stream_in = buf_alloc(n * FLASH_PAGE_SIZE);
  uint8_t *stream = buf_alloc(n * FLASH_PAGE_SIZE + (n * FLASH_PAGE_SIZE) / 2);
  uint32_t k = 0;

  for (uint32_t p = 0; p < number_of_pages; p++)
  {
    if (skip[p])
      continue;

    uint32_t cr  = erase_cr_for(start_page + p);
    uint32_t dst = base_addr + p * FLASH_PAGE_SIZE;

    memcpy(&table[k * 8],     &cr,  4);
    memcpy(&table[k * 8 + 4], &dst, 4);
    memcpy(&stream_in[k * FLASH_PAGE_SIZE], &file_data[p * FLASH_PAGE_SIZE], FLASH_PAGE_SIZE);
    k++;
  }

  long long _t0 = prof_us();
  int clen = LZ4_compress_HC((const char *)stream_in, (char *)stream,
      (int)(n * FLASH_PAGE_SIZE), (int)(n * FLASH_PAGE_SIZE + (n * FLASH_PAGE_SIZE) / 2), 9);
  long long _t_comp = prof_us() - _t0;

  if (clen <= 0 || (uint32_t)clen >= n * FLASH_PAGE_SIZE)
  {
    buf_free(table); buf_free(stream_in); buf_free(stream);
    return 0;   // incompressible: legacy path
  }

  dap_write_word(DEMCR, DEMCR_VC_CORERESET | (1u << 24));   // + TRCENA for the stub's DWT
  dap_write_block(RINGTABLE, table, (n * 8 + 3) & ~3u);
  ring_kick(n * FLASH_PAGE_SIZE, n);

  // Stream with flow control
  uint32_t wr = 0, rd_cache = 0, pos = 0, stall = 0;

  while (pos < (uint32_t)clen)
  {
    uint32_t space = RING_SIZE - (wr - rd_cache);

    if (space < 4096)
    {
      rd_cache = dap_read_word(RINGPARAMS + 32);
      space = RING_SIZE - (wr - rd_cache);

      if (space < 4096)
      {
        uint32_t st = dap_read_word(RINGPARAMS + 16);
        if (0xAA != st)   // stub died (or never entered)
        {
          warning("ring stub status %u mid-stream\n", st);
          buf_free(table); buf_free(stream_in); buf_free(stream);
          return 0;
        }
        if (++stall > 20000)
        {
          buf_free(table); buf_free(stream_in); buf_free(stream);
          return 0;
        }
        continue;
      }
    }

    uint32_t nbytes = (uint32_t)clen - pos;
    if (nbytes > 4096) nbytes = 4096;
    if (nbytes > space) nbytes = space;
    uint32_t to_wrap = RING_SIZE - (wr & (RING_SIZE - 1));
    if (nbytes > to_wrap) nbytes = to_wrap;

    // dap_block_write needs word-aligned length: only the final chunk may be
    // ragged; pad the staged bytes but advance wr by the true count.
    dap_block_write(RING_BASE + (wr & (RING_SIZE - 1)), &stream[pos], (nbytes + 3) & ~3u);
    wr += nbytes;
    pos += nbytes;
    dap_write_word(RINGPARAMS + 28, wr);
  }

  long long _t_stream = prof_us() - _t0 - _t_comp;
  dap_write_word(RINGPARAMS + 36, (uint32_t)clen);   // input complete

  // Wait for the stub to finish the tail (scale: ~30ms/page silicon)
  bool halted = false;
  for (int t = 0; t < 20000; t++)
  {
    if (dap_read_word(DHCSR) & DHCSR_S_HALT) { halted = true; break; }
  }

  uint32_t st = halted ? dap_read_word(RINGPARAMS + 16) : 0;

  if (NULL != getenv("EDBG_PROFILE"))
    warning("ring profile(ms): comp=%lld stream=%lld tail=%lld clen=%d/%u | stub(ms@16M): decode=%u erase=%u burst=%u\n",
        _t_comp/1000, _t_stream/1000, (prof_us() - _t0 - _t_comp - _t_stream)/1000,
        clen, n * FLASH_PAGE_SIZE,
        dap_read_word(RINGPARAMS + 0x28) / 16000, dap_read_word(RINGPARAMS + 44) / 16000,
        dap_read_word(RINGPARAMS + 48) / 16000);

  buf_free(table); buf_free(stream_in); buf_free(stream);

  if (!halted || 1 != st)
  {
    warning("ring loader: halted=%d status=%u produced=%u\n",
        halted, st, dap_read_word(RINGPARAMS + 20));
    if (!halted)
      dap_write_word(DHCSR, DHCSR_DBGKEY | DHCSR_DEBUGEN | DHCSR_HALT | DHCSR_MASKINTS);
    return 0;
  }

  flash_wait_done();
  dap_write_word(FLASH_NSCR, 0);
  return 1;
}

//-----------------------------------------------------------------------------
static void verify_readback(void)
{
  uint32_t addr = FLASH_ADDR + target_options.offset;
  uint32_t block_size;
  uint32_t offs = 0;
  uint8_t *bufb;
  uint8_t *bufa = target_options.file_data;
  uint32_t size = target_options.file_size;

  bufb = buf_alloc(READ_CHUNK_SIZE);

  while (size)
  {
    block_size = (size > READ_CHUNK_SIZE) ? READ_CHUNK_SIZE : size;

    dap_block_read(addr, bufb, (block_size + 3) & ~3u);

    for (int i = 0; i < (int)block_size; i++)
    {
      if (bufa[offs + i] != bufb[i])
      {
        verbose("\nat address 0x%x expected 0x%02x, read 0x%02x\n",
            addr + i, bufa[offs + i], bufb[i]);
        buf_free(bufb);
        error_exit("verification failed");
      }
    }

    addr += block_size;
    offs += block_size;
    size -= block_size;

    verbose(".");
  }

  buf_free(bufb);
}

//-----------------------------------------------------------------------------
static void target_verify(void)
{
  uint32_t addr = FLASH_ADDR + target_options.offset;
  uint32_t len = round_up(target_options.file_size, 4);
  uint32_t crc_dev, crc_exp;

  if (NULL != getenv("EDBG_U5_NO_STUB"))
  {
    verify_readback();
    return;
  }

  // file_data is padded to options->size with 0xff (matching erased flash), so
  // the expected CRC over the 4-byte-rounded length is well defined.
  crc_exp = crc32_stm32(0xffffffff, target_options.file_data, len);

  char *env = getenv("EDBG_U5_VERIFY_RETRIES");
  int retries = env ? atoi(env) : 4;
  if (retries < 1)
    retries = 1;

  for (int attempt = 0; attempt < retries; attempt++)
  {
    if (!crc_stub_run(addr, len, &crc_dev))
    {
      warning("CRC verify stub did not run; falling back to readback verify\n");
      verify_readback();
      return;
    }

    if (crc_dev == crc_exp)
    {
      verbose("CRC32 0x%08x ok\n", crc_dev);
      return;
    }

    if (attempt + 1 < retries)
    {
      // A silent in-band data glitch (ack=OK but wrong bits -- undetectable at
      // the transfer layer) corrupted a page; with compression one bad byte
      // corrupts a whole decoded page. The differential re-flash re-scans page
      // CRCs and reprograms ONLY the corrupted pages -- cheap and convergent --
      // then we re-verify. This is what makes a full flash over a marginal
      // cable reliable end to end (paired with latch-free transfer recovery).
      warning("verify CRC mismatch (target 0x%08x != 0x%08x); re-flashing changed pages [retry %d/%d]\n",
          crc_dev, crc_exp, attempt + 1, retries - 1);
      target_program();
      continue;
    }

    // Exhausted retries: readback for a byte-accurate diagnostic (error_exits).
    warning("CRC still mismatched after %d re-flash attempts; readback diagnostics\n", retries);
    verify_readback();
    return;
  }
}

//-----------------------------------------------------------------------------
static void target_read(void)
{
  uint32_t addr = FLASH_ADDR + target_options.offset;
  uint32_t offs = 0;
  uint32_t block_size;
  uint8_t *buf = target_options.file_data;
  uint32_t size = target_options.size;

  while (size)
  {
    block_size = (size > READ_CHUNK_SIZE) ? READ_CHUNK_SIZE : size;

    dap_block_read(addr, &buf[offs], block_size);

    addr += block_size;
    offs += block_size;
    size -= block_size;

    verbose(".");
  }

  save_file(target_options.name, buf, target_options.size);
}

//-----------------------------------------------------------------------------
static void target_identify(void)
{
  // Exact output format of the lost driver -- parsed by fw_deploy ("Core ID: ")
  uint32_t uid0 = dap_read_word(UID_REG + 0);
  uint32_t uid1 = dap_read_word(UID_REG + 4);
  uint32_t uid2 = dap_read_word(UID_REG + 8);

  message("Core ID: %08x%08x%08x\n", uid2, uid1, uid0);
}

//-----------------------------------------------------------------------------
static int target_fuse_read(int section, uint8_t *data)
{
  error_exit("target_fuse_read() is not implemented yet");
  (void)section;
  (void)data;
  return 0;
}

//-----------------------------------------------------------------------------
static void target_fuse_write(int section, uint8_t *data)
{
  error_exit("target_fuse_write() is not implemented yet");
  (void)section;
  (void)data;
}

//-----------------------------------------------------------------------------
static char *target_enumerate(int i)
{
  if (i < ARRAY_SIZE(devices))
    return devices[i].family;

  return NULL;
}

//-----------------------------------------------------------------------------
static char target_help[] =
  "Fuses:\n"
  "  not implemented for STM32U5\n";

//-----------------------------------------------------------------------------
target_ops_t target_st_stm32u5_ops =
{
  .select    = target_select,
  .deselect  = target_deselect,
  .erase     = target_erase,
  .lock      = target_lock,
  .unlock    = target_unlock,
  .program   = target_program,
  .verify    = target_verify,
  .read      = target_read,
  .identify  = target_identify,
  .fread     = target_fuse_read,
  .fwrite    = target_fuse_write,
  .enumerate = target_enumerate,
  .help      = target_help,
};
