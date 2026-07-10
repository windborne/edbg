# Debugging the STM32U5 driver

Field guide for when `edbg -t stm32u5` misbehaves. Everything here was learned
on real hardware (STM32U5A5 boards over a CMSIS-DAP probe); the timing numbers
are measured, not datasheet-copied.

## 30-second architecture map

A `-bpv` flash runs these phases, in order:

1. **Connect + reset-catch** — DP init (with ADIv5.2 dormant-to-SWD alert),
   `VC_CORERESET` + `SYSRESETREQ`, so the app (and its IWDG) never runs.
2. **Differential scan** — an on-chip CRC stub checksums each 8 KB page;
   matching pages are skipped. A 3-sample fast path skips the whole scan for
   fresh images (all 3 spread samples differ ⇒ program everything).
3. **Ring loader** (default write path) — the whole image is compressed as one
   LZ4-HC stream; the host streams it into a 64 KB SRAM ring while a single
   stub run decodes through a 64 KB decoded-window ring and self-drives NSCR
   through erase → 8-quad-word burst programming, page table in SRAM.
   Burst k programs while chunk k+1 decodes (ping-pong).
4. **Verify** — whole-image on-chip CRC; on mismatch, full readback compare
   (so a "verification failed" message means a *real* mismatch, with address).
5. **Deselect** — clears DHCSR debug state (a leftover `C_MASKINTS` survives
   reset and silently masks every interrupt in the app — boards look dead but
   run; this cost us an evening) and releases the core.

## Escape-hatch ladder

Each knob steps down to a simpler, slower, better-understood path. When
something is wrong, walk down until it works, then report the first rung that
fixed it — that localizes the bug.

| Env var | Effect |
|---|---|
| `EDBG_U5_NO_RING=1` | skip the LZ4 ring loader; use the per-page raw burst stub |
| `EDBG_U5_NO_STUB=1` | no RAM stubs at all: host-driven quad-word writes |
| `EDBG_U5_SAFE_WRITES=1` | most conservative write path |
| `EDBG_U5_FULL_WRITE=1` | disable the differential scan (program every page) |
| `EDBG_U5_SLOW_STUB=1` | skip the 16 MHz MSI raise (stubs run at reset 4 MHz) |
| `EDBG_BULK_WINDOW=N` | USB packets in flight (default 1 — see wire physics) |
| `EDBG_IDLE_CYCLES=N` | SWD idle cycles between transfers (default 8; raise on a noisy line) |
| `EDBG_SWD_CFG=N` | raw DAP_SWD_Configure byte (bit 2 = always-data-phase; helps a noisy line resync after a corrupted ACK). Bits 0-1 (turnaround) need matching target DLCR — leave 0. |
| `EDBG_PROFILE=1` | phase timing buckets printed after programming |
| `EDBG_DEBUG_TRANSIENTS=1` | log each retried bulk transfer |
| `EDBG_DEBUG_STUBS=1` | log each stub kick (page, buffer, address) |

## Error messages decoded

| Message | Meaning | First move |
|---|---|---|
| `verification failed` + address | flash content ≠ image after programming | rerun (differential repairs in ~1 s); if it repeats at the same address, suspect a stuck page — walk the escape ladder |
| `decode stub status 0` | ring stub never entered (bad PC/params, or clobbered stub) | check SRAM map below for overlap regressions |
| `decode stub status 2` | LZ4 decoder error mid-stream — staged stream corrupt or truncated | `EDBG_U5_NO_RING=1` (raw per-page path); if that fixes it, encoder/stub mismatch |
| `decode stub status 3` | decoded length ≠ image size | same as status 2 |
| `burst write stub stalled` | stub never halted (fault → lockup, or wedged bus) | DP may be wedged; see recovery below |
| `invalid response ... status = 7` | target didn't ACK on the wire | link-level: wedge, clock too high, or W≥2 storm |
| `invalid response ... status = 2/4` | WAIT/FAULT sticky | usually transient; retry layer handles it, `~` marks retries |
| `flash operation failed. FLASH_NSSR = 0x...` | erase/program error bits | decode against RM0456; `0x88`-class on old blob = SWAP_BANK bug (fixed here) |
| `is the board turned on?` suffix | generic no-ACK wrapper | power, mux, or wedge |

`rerun to repair` is literal: the differential scan makes any interrupted or
failed flash recoverable by simply re-running the same command — only broken
pages reprogram.

## SRAM map (the recurring landmine)

Everything the driver plants in SRAM1, current layout:

| Address | What |
|---|---|
| `0x20000000` | CRC scan stub (12 B) |
| `0x20000040` | raw burst-write stub (36 B) |
| `0x20000080` | SRAM vector table (all vectors → bkpt; VTOR points here so a faulting stub halts instead of locking up on erased-flash vectors) |
| `0x20000100` | raw-path staging buffer A (8 KB) |
| `0x20002200` | raw-path staging buffer B (8 KB) |
| `0x20004400` | LZ4 ring-loader stub load address (text+bss) |
| `0x20007000` | ring/stub params block (status, wr/rd counters, dbg slots) |
| `0x20007100` | ring page table (n × {erase_cr, dst}) |
| `0x20008000` | stub stack top (descends) |
| `0x20010000` | 64 KB compressed-input ring |
| `0x20030000` | 64 KB LZ4 decoded-window ring |

**The landmine (it bit four separate times):** a stub's `.bss` grows silently
(decoder state, FIFOs, instrumentation) and swallows the params block. The
host then writes params where the stub doesn't look, the stub reads garbage,
and you get `status 0` with perfect-looking uploads. **Any time a stub is
rebuilt, check `arm-none-eabi-nm stub.elf | grep __bss_end` against the params
address** — keep ≥1 KB headroom. The params address is hardcoded on *both*
sides (stub source and driver `#define`) and they must move together.

## Stub development recipe

Sources in `stub/`. Build (see `stub/README.md`):

```
arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -O2 -ffreestanding -nostdlib \
  -fno-builtin -Wl,-Ttext=0x20004400 -Wl,-e,_start -o stub.elf stub.c
arm-none-eabi-objcopy -O binary stub.elf stub.bin
xxd -i stub.bin > ../<name>_bin.h    # append: unsigned int <name>_entry = 0x<nm _start>;
```

Gotchas that each cost real time:
- `_start` is **not** necessarily at the bin's first byte — always export the
  entry from `nm` and kick PC to it, never to the load address.
- No crt0: `.bss` is whatever SRAM held; reset all state explicitly.
- The kick must set **SP and xPSR (T-bit!) every run**, not just PC — halted
  stubs never unwind, so SP creeps, and a prior fault leaves xPSR poisoned.
- `make` does **not** know about the generated `*_bin.h` headers — after
  regenerating one, build with `make -B` or you will test the old stub.
- Batched register writes (params + SP/xPSR/PC + DHCSR run in one transfer,
  no REGRDY polls) are proven safe — the CRC stub has used them throughout.
- Flash burst feeds must come from the core (back-to-back stores). Feeding
  bursts over SWD has ~13 µs inter-word gaps → PGSERR/corruption. Waiting
  BSY|WDW *before* the next feed (not after the last) lets decode run during
  tPROG — that overlap is worth ~0.6 s on a full image.

## Profiling

`EDBG_PROFILE=1` prints per-phase wall time. Ring-path buckets:
`comp` (host compression), `stream` (streaming loop wall — wire + any
stub-paced stalls), `tail` (stub drain after input complete), plus stub-side
DWT buckets when the stub is instrumented (host pre-sets `DEMCR.TRCENA`; stub
sets `DWT_CTRL.CYCCNTENA` and accumulates into spare params slots).

Reference numbers (b4 board, 16 MHz stub clock, defaults):

| | |
|---|---|
| release-size (344 KB) true cold | ~1.4 s |
| full 548 KB true cold | ~2.0 s |
| warm (image unchanged) | 0.2–0.5 s |
| wire cost | ~16–18 µs/word (probe PIO-over-bridge bound — not clock-limited) |
| page erase (tPER, measured) | ~1.6 ms |
| burst program | dominant silicon term (~11 ms/page effective) |
| LZ4-HC ratio on firmware | ~52 % |

**Profile before theorizing.** Every model of this system was wrong at least
once (the wire "asm speedup", the erase share, decode-vs-silicon ordering);
the profiler and DWT buckets were wrong zero times.

## Wire physics and W≥2 (pipelining)

- The wire runs ~16–18 µs/word regardless of requested clock: SAME70 PIO
  writes cross the AHB→APB bridge ~3×/bit. Neither `-c`, hand asm, nor
  asymmetric write/read delays change it measurably.
- Requesting >20 MHz (probe half-bit delay ≤6) breaks the *read* path
  instantly (probe-side sampling latency) — `status 7` at link init.
- `EDBG_BULK_WINDOW≥2` at 16 MHz storms on **writes only** (reads are immune):
  packet processing overlapping wire write activity → misread ACK → DP
  desync. Characterized software-only; suspected USBHS-DMA bus contention
  with ACK sampling. Stable operating point: **window 4 + `-c 12500`**
  (delay 12; delay ≤10 storms, 11 is marginal) — worth ~0.1 s, but requires
  updated (v7+) probe firmware, so it stays opt-in.
- A standalone reproducer for wire experiments lives in the bench notes
  (`blkstress.py`): raw-HID pipelined TransferBlock writes + content verify.
  If you rebuild one: reset-catch first (a live app's IWDG resets mid-test —
  looks exactly like a storm), and compare only length-prefixed response
  bytes (padding beyond the declared length is uninitialized).
- **Electrically noisy SWD line** (long/marginal cabling): sustained *writes*
  storm (`status 7` the moment real write traffic starts) far below the read
  clock ceiling — a corrupted ACK on a write desyncs the DP. Measured on one
  noisy port: reads/identify OK to 8 MHz, plain writes reliable only to 3 MHz.
  `EDBG_SWD_CFG=4` (always-data-phase) + `EDBG_IDLE_CYCLES=64` bought a clean
  4 MHz (6/6 full-image soak) where plain 4 MHz stormed; 5 MHz+ storms
  regardless. So: on a noisy line, drop the clock and add those two knobs
  before assuming a hardware fault. The real fix is quieter cabling.

## Long / AC-coupled (USB-C) cable — the bandpass trap

A different failure than the noisy line above, and one where the intuitive fix
is *backwards*. SWD carried over a long USB-C cable often rides the SuperSpeed
pairs, which have **AC-coupling capacitors** by spec. That makes the link a
**bandpass channel**, not the low-pass a long wire is assumed to be:

- **High-pass (the AC caps):** reads FAIL at low clock and are perfect high —
  bench: 0/6 connect at ≤1 MHz, 6/6 at ≥4 MHz. **Lowering the clock makes it
  worse**, the opposite of every other cable.
- **Low-pass (cable RC):** fast-toggling data corrupts at 16 MHz (0xAA/random
  100% wrong) while long runs stay clean.
- **Reflection notch:** a sharp dead band (bench: ~9 MHz) where every transfer
  fails, clean again just above and below.

Usable passband ≈ 3–8 MHz; **8 MHz measured the lowest glitch rate**. The
stm32u5 driver now **defaults to 8 MHz** whenever `-c` is not passed (16 MHz is
edbg's global default but measured **0/8** full flashes here vs **10/10** at
8 MHz), so `oc` and bare `edbg` are both safe out of the box; pass `-c` to
override. Diagnose with the read clock-response: if reads fail *low* and pass
*high*, it's AC-coupled — do NOT drop the clock below ~4 MHz.

### The actual root cause: a WDATAERR latch the recovery couldn't clear

The real reason block-4 boards "wouldn't flash over the long cable" was **not**
raw signal integrity — it was a latent bug in the DP recovery, exposed by the
cable. A write's data phase over the long link occasionally picks up a parity
glitch; the DP latches `CTRL/STAT.WDATAERR` (bit 7) and then **FAULTs every
subsequent write** until it is cleared with `ABORT.WDERRCLR` (bit 3). edbg's
`dap_reset_link` ABORT mask was the classic `STKCMPCLR|STKERRCLR|ORUNERRCLR`
(0x16) — **missing WDERRCLR** — so recovery re-synced the link but never cleared
WDATAERR, and one glitched write wedged the DP forever (`transfer failed after
N attempts … cable may be unusable`). It's cable-length dependent purely because
a short cable rarely sets WDATAERR, so the omission never bit. Fix: the ABORT now
clears **all** sticky flags incl. `WDERRCLR` (plus `DAPABORT` to drop any stuck
AP transaction). Confirmed on the bench: CTRL/STAT reads `0xf00000c0` (WDATAERR
set) after a glitch; ABORT=0x16 → FAULT, WDATAERR stays; ABORT=0x1e/0x1f → OK,
WDATAERR cleared.

On top of that base fix, the driver self-heals the rare in-band glitch without
any external unwedge: latch-free transfer retry (a single WDERRCLR-capable
`dap_reset_link` between tries), a self-healing verify (differential re-flash of
corrupted pages), and a top-level reconnect+re-run to outlast multi-second
bursts. Tunables: `EDBG_BLOCK_RETRIES`, `EDBG_FLASH_RETRIES`;
`EDBG_DEBUG_XFER=1` traces every transfer + dumps the failing batch. What's left
after all that is sustained EMI bursts — a hardware limit of the cable, not the
tool.

## DP wedge recovery

Symptoms: every command fails `status 7`, survives board power-cycle (supercap
back-powering keeps the DP state alive).

Recovery, in order:
1. `edbg` retry (it sends line reset + dormant alert on connect) — handles
   most transients.
2. `unwedge_dp.py` (octopice): line reset + ADIv5.2 dormant alert + ABORT
   with DAPABORT + CTRL/STAT powerup at a gentle clock.
3. OpenOCD init/exit at `adapter speed 1000` (stock cmsis-dap + stm32u5x
   cfgs) — the heavyweight unwedge.

## Compatibility notes

- `identify` output (`Core ID: ...`) is parsed by fw_deploy — treat the format
  as frozen.
- SWAP_BANK boards: `bker = high_half ^ swap_bank`. The old binary blob gets
  this wrong and cannot flash a swapped board at all (`FLASH_SR=0x88`).
  `select` prints OPTR so you can see RDP/SWAP_BANK/TZEN at a glance.
- Other targets (SAMD51 etc.) share none of this driver's write paths; the
  bulk-window default stays 1 for fleet probe compatibility.
